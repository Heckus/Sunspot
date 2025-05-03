# -*- coding: utf-8 -*-
"""
main.py

Main application script for the Pi Camera Stream & Record service.
Initializes hardware and camera managers, starts the web UI,
and runs the main control loop.

**Modification:** Main loop simplified as frame writing is handled by CameraManager's
                  dedicated recording thread. Loop timing can focus on capture rate.
**Modification 2 (User Request):** Confirmed removal of audio-related error checks.
**Modification 3 (User Request):** Added comments regarding potential multiprocessing setup for CV.
"""

import os
import time
import logging
import threading
import signal
import subprocess
import sys
import socket
# import multiprocessing # Import needed if CV multiprocessing is implemented later

# Import local modules
import config
from hardware_manager import HardwareManager
from camera_manager import CameraManager
from web_ui import setup_web_app, run_web_server, _app_state, _ui_lock

# --- Global Variables ---
shutdown_event = threading.Event()
flask_thread = None
main_loop_error_count = 0
MAX_MAIN_LOOP_ERRORS = 5
# cv_process = None # Placeholder for potential CV process
# cv_input_queue = None # Placeholder for potential CV frame queue
# cv_output_queue = None # Placeholder for potential CV result queue

def signal_handler(sig, frame):
    """Handles SIGINT and SIGTERM signals for graceful shutdown."""
    if shutdown_event.is_set():
        logging.warning("Shutdown already in progress, ignoring additional signal.")
        return
    logging.warning(f"Received signal {sig}. Initiating graceful shutdown...")
    shutdown_event.set()

def setup_logging():
    """Configures the logging system."""
    log_level = getattr(logging, config.LOG_LEVEL.upper(), logging.INFO)
    logging.basicConfig(level=log_level,
                        format=config.LOG_FORMAT,
                        datefmt=config.LOG_DATE_FORMAT,
                        stream=sys.stderr) # Force stream to stderr
    # Reduce log noise from web server and image libraries
    logging.getLogger("werkzeug").setLevel(logging.WARNING)
    logging.getLogger("PIL.PngImagePlugin").setLevel(logging.WARNING)
    logging.info("--- Logging configured ---")


def run_disable_script():
    """Executes the toggle.sh disable script for reboot."""
    # ... (unchanged) ...
    logging.warning("Reboot requested. Executing disable script...")
    toggle_script_path = config.TOGGLE_SCRIPT_PATH

    if not toggle_script_path or not os.path.exists(toggle_script_path):
        logging.error(f"Disable script path not found or invalid: '{toggle_script_path}'. Cannot execute.")
        return False
    if not os.access(toggle_script_path, os.X_OK):
        logging.error(f"Disable script '{toggle_script_path}' not executable! Run 'chmod +x'."); return False
    if os.geteuid() != 0:
         logging.error("Disable script requires root privileges (sudo). Cannot execute."); return False

    disable_command = ["sudo", toggle_script_path, "disable"]
    logging.info(f"Executing: {' '.join(disable_command)}")
    try:
        result = subprocess.run(disable_command, capture_output=True, text=True, timeout=10)
        logging.info(f"Disable script executed. Return code: {result.returncode}")
        logging.info(f"Stdout: {result.stdout}")
        logging.warning(f"Stderr: {result.stderr}")
        logging.warning("Disable script launched (includes reboot). Service will now exit.")
        time.sleep(5); return True # Allow time for script to potentially initiate reboot
    except subprocess.TimeoutExpired:
        logging.error("Disable script timed out. Reboot might still happen.")
        return False
    except Exception as e:
        logging.critical(f"!!! FAILED TO EXECUTE DISABLE SCRIPT: {e}", exc_info=True)
    return False


def main_loop(hw_manager, cam_manager):
    """
    The main operational loop of the application. Handles camera capture,
    passing frames to recorder, hardware checks, and reconfiguration requests.

    Args:
        hw_manager (HardwareManager): Instance of the hardware manager.
        cam_manager (CameraManager): Instance of the camera manager.
    """
    global _app_state, _ui_lock
    logging.info("--- Main loop started ---")
    consecutive_capture_errors = 0
    last_battery_check_time = time.monotonic()

    # Get initial primary camera FPS for loop timing
    try:
        cam_state = cam_manager.get_camera_state()
        # Use actual driver-reported FPS if available, else target, else fallback
        current_cam0_fps = cam_state.get('actual_cam0_fps') or cam_state.get('target_cam0_fps') or 30.0
        if not isinstance(current_cam0_fps, (int, float)) or current_cam0_fps <= 0:
             logging.warning(f"Invalid initial Cam0 FPS ({current_cam0_fps}), defaulting loop timing to 30fps.")
             current_cam0_fps = 30.0
        else:
             logging.info(f"Main loop using initial Cam0 FPS for timing: {current_cam0_fps:.2f}")
    except Exception:
         logging.exception("Error getting initial Cam0 FPS, defaulting loop timing.")
         current_cam0_fps = 30.0

    last_loop_log_time = 0

    while not shutdown_event.is_set():
        loop_start_time = time.monotonic()

        # --- Check for Cam0 Reconfiguration Request from Web UI ---
        requested_resolution_index = None
        with _ui_lock:
            if _app_state.get("reconfigure_resolution_index") is not None:
                requested_resolution_index = _app_state["reconfigure_resolution_index"]
                _app_state["reconfigure_resolution_index"] = None # Consume the request

        if requested_resolution_index is not None:
            logging.info(f"--- Processing Cam0 resolution change request to index {requested_resolution_index} ---")
            was_recording = cam_manager.is_recording

            if was_recording:
                logging.info("Stopping recording for reconfiguration...")
                cam_manager.stop_recording() # Stops video thread, releases writers, syncs
                time.sleep(0.5) # Give time for thread join and sync

            logging.info(f"Attempting to re-initialize cameras (Cam0 index {requested_resolution_index})...")
            if cam_manager.initialize_cameras(requested_resolution_index):
                logging.info("--- Reconfiguration successful ---")
                # Update loop timing FPS based on the new actual FPS
                try:
                    cam_state = cam_manager.get_camera_state()
                    new_fps = cam_state.get('actual_cam0_fps') or cam_state.get('target_cam0_fps') or 30.0
                    if not isinstance(new_fps, (int, float)) or new_fps <= 0: new_fps = 30.0
                    current_cam0_fps = new_fps
                    logging.info(f"Updated main loop timing target based on FPS: {current_cam0_fps:.2f}")
                except Exception:
                    logging.exception("Failed to update loop timing FPS after reconfig.")
                    # Keep old FPS on error

                if was_recording:
                    logging.info("Restarting recording after successful reconfiguration...")
                    time.sleep(1.0) # Give cameras time to settle
                    if not cam_manager.start_recording():
                        # Log only the camera manager's general error (no audio error)
                        logging.error(f"Failed to restart recording after reconfiguration! Error: {cam_manager.last_error}")
            else:
                # Reconfiguration failed
                logging.error(f"!!! Failed to reconfigure cameras (Cam0 index {requested_resolution_index}). Error: {cam_manager.last_error}. Attempting restore... !!!")
                if cam_manager.initialize_cameras(): # Try to restore previous state
                     logging.info("Successfully restored previous camera configuration.")
                     try: # Update loop timing FPS back
                         cam_state = cam_manager.get_camera_state()
                         old_fps = cam_state.get('actual_cam0_fps') or cam_state.get('target_cam0_fps') or 30.0
                         if not isinstance(old_fps, (int, float)) or old_fps <= 0: old_fps = 30.0
                         current_cam0_fps = old_fps
                         logging.info(f"Restored main loop timing target based on FPS: {current_cam0_fps:.2f}")
                     except Exception:
                         logging.exception("Failed to restore loop timing FPS after failed reconfig.")

                     if was_recording:
                         logging.warning("Attempting recording restart with restored configuration...")
                         time.sleep(1.0)
                         if not cam_manager.start_recording():
                             logging.error(f"Failed to restart recording after failed reconfig and restore. Error: {cam_manager.last_error}")
                else:
                    # Critical failure if restore also fails
                    logging.critical(f"!!! Failed to restore previous camera configuration after failed reconfig. Error: {cam_manager.last_error}. Signaling shutdown. !!!")
                    shutdown_event.set()
                    break # Exit main loop

            logging.info("--- Finished handling reconfiguration request ---")
            time.sleep(0.5) # Short pause after handling reconfig
            continue # Restart loop immediately


        # --- Capture and Process Frames ---
        # This captures frame(s), updates stream frame, updates raw frame for CV,
        # and puts frame into the recording queue if active.
        combined_frame_for_stream = cam_manager.capture_and_combine_frames()

        # Check if capture itself failed (main streaming frame is None)
        if combined_frame_for_stream is None and cam_manager.is_initialized0: # Check if cam0 should be working
            consecutive_capture_errors += 1
            logging.warning(f"Frame capture failed ({consecutive_capture_errors}/{config.MAX_CONSECUTIVE_CAPTURE_ERRORS}). Error: {cam_manager.last_error}")
            if consecutive_capture_errors >= config.MAX_CONSECUTIVE_CAPTURE_ERRORS:
                logging.error("Too many consecutive frame capture errors. Signaling shutdown.")
                shutdown_event.set()
                break # Exit main loop
            time.sleep(0.5) # Wait longer after capture error before next attempt
            continue # Skip rest of loop iteration
        else:
            # Reset error count if capture succeeds OR if cam0 wasn't initialized anyway
            if consecutive_capture_errors > 0 and combined_frame_for_stream is not None:
                logging.info(f"Frame capture recovered after {consecutive_capture_errors} errors.")
            consecutive_capture_errors = 0


        # --- Check Recording Triggers ---
        physical_switch_on = hw_manager.is_switch_pressed()
        with _ui_lock:
            digital_switch_on = _app_state.get("digital_recording_active", False)

        should_be_recording = physical_switch_on or digital_switch_on
        is_currently_recording = cam_manager.is_recording

        # --- Start/Stop Recording Logic ---
        if should_be_recording and not is_currently_recording:
            log_msg = "Physical switch ON" if physical_switch_on else ""
            if digital_switch_on: log_msg += (" / " if log_msg else "") + "Digital trigger ON"
            logging.info(f"Recording trigger active ({log_msg}) - initiating start.")
            if not cam_manager.start_recording(): # Starts video thread
                # Log only the camera manager's general error
                logging.error(f"Attempt to start recording failed. Error: {cam_manager.last_error}")

        elif not should_be_recording and is_currently_recording:
            logging.info("Recording trigger(s) OFF - initiating stop.")
            cam_manager.stop_recording() # Stops video thread, releases writers, syncs


        # --- Periodic Hardware Checks ---
        current_time = time.monotonic()
        if hw_manager.ina219_sensor and (current_time - last_battery_check_time > config.BATTERY_READ_INTERVAL):
             old_batt = hw_manager.battery_percentage
             hw_manager.read_battery_level() # Reads voltage and updates percentage
             new_batt = hw_manager.battery_percentage
             # Log only if level changed significantly or read failed after success
             if new_batt is not None and (old_batt is None or abs(new_batt - old_batt) > 1.0):
                 logging.info(f"Battery Level: {new_batt:.1f}%")
             elif new_batt is None and old_batt is not None:
                 logging.warning(f"Battery reading failed. Last Error: {hw_manager.last_error}")
             last_battery_check_time = current_time


        # --- Loop Delay ---
        # Base delay on primary camera's current target/actual FPS for capture responsiveness (Req 2 support)
        loop_duration = time.monotonic() - loop_start_time
        if current_cam0_fps > 0:
            target_loop_time = 1.0 / current_cam0_fps
        else:
            target_loop_time = 1.0 / 30.0 # Fallback if FPS is invalid

        # Calculate sleep time needed to approximate the target loop time
        sleep_time = max(0.001, target_loop_time - loop_duration)

        # Log loop timing info periodically for debugging
        if current_time - last_loop_log_time > 10.0:
             q_info = ""
             # Show recording queue size if recording
             if cam_manager.is_recording and cam_manager.recording_frame_queue:
                  q_info = f", RecQ: {cam_manager.recording_frame_queue.qsize()}"
             # Get measured FPS for logging
             measured_fps_str = f"{cam_manager.measured_cam0_fps_avg:.1f}" if cam_manager.measured_cam0_fps_avg else "N/A"
             logging.debug(f"MainLoop dur: {loop_duration:.4f}s, Sleep: {sleep_time:.4f}s (Target Cap FPS: {current_cam0_fps:.1f}, Measured: {measured_fps_str}{q_info})")
             last_loop_log_time = current_time

        time.sleep(sleep_time) # Sleep to maintain desired loop rate


    logging.info("--- Main loop finished ---")


def main():
    """Main function to initialize and run the application."""
    global flask_thread, main_loop_error_count
    # global cv_process, cv_input_queue, cv_output_queue # Uncomment if using multiprocessing

    setup_logging()
    logging.info("========================================================")
    logging.info("=== Starting Pi Camera Stream & Record Service ===")
    logging.info("========================================================")
    logging.info(f"Using Python: {sys.version}")
    logging.info(f"Script Directory: {config.SCRIPT_DIR}")
    logging.info(f"Toggle Script Path: {config.TOGGLE_SCRIPT_PATH}")
    if os.geteuid() == 0: logging.warning("Script is running with root privileges (sudo).")
    else: logging.warning("Script is NOT running with root privileges. Servo/Toggle script may fail.")

    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    hw_manager = None
    cam_manager = None

    while main_loop_error_count < MAX_MAIN_LOOP_ERRORS and not shutdown_event.is_set():
        try:
            logging.info(f"--- Initializing Managers (Attempt {main_loop_error_count + 1}) ---")

            # --- Multiprocessing Setup Placeholder (Requirement 5 Suggestion) ---
            # If using multiprocessing for CV, initialize queues and process here:
            # cv_input_queue = multiprocessing.Queue(maxsize=5) # Queue for frames to CV process
            # cv_output_queue = multiprocessing.Queue(maxsize=5) # Queue for results from CV process
            # cv_process = multiprocessing.Process(target=run_cv_process_loop, args=(cv_input_queue, cv_output_queue), name="CVProcess")
            # cv_process.daemon = True # Ensure CV process exits if main process exits
            # cv_process.start()
            # logging.info("Started separate CV processing process.")
            # ----------------------------------------------------------------

            # Initialize Hardware and Camera Managers
            hw_manager = HardwareManager()
            cam_manager = CameraManager()
            # If using multiprocessing, potentially pass cv_input_queue to CameraManager
            # e.g., cam_manager.set_cv_queue(cv_input_queue)

            # Initialize Cameras
            if not cam_manager.initialize_cameras():
                # Use general error from camera manager (no audio error)
                raise RuntimeError(f"Initial camera setup failed: {cam_manager.last_error}")

            # Setup Web UI (Flask)
            # If using multiprocessing, potentially pass cv_output_queue to web_ui setup
            # e.g., setup_web_app(cam_manager, hw_manager, shutdown_event, cv_output_queue)
            setup_web_app(cam_manager, hw_manager, shutdown_event)

            logging.info("Starting Flask web server thread...")
            flask_thread = threading.Thread(target=run_web_server, name="FlaskThread", daemon=True)
            flask_thread.start()
            time.sleep(2.0) # Give Flask time to start
            if not flask_thread.is_alive():
                raise RuntimeError("Flask web server thread failed to start.")
            logging.info("Flask thread started.")

            # Log access URLs
            try:
                hostname = socket.gethostname()
                local_ip = socket.gethostbyname(hostname + ".local") # Try .local first
                logging.info(f"--- System Running --- Host: {hostname}")
                logging.info(f"--- Access web interface at: http://{local_ip}:{config.WEB_PORT} or http://{hostname}.local:{config.WEB_PORT}")
            except socket.gaierror:
                 try:
                     # Fallback to finding IP via UDP connection to external host
                     s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.settimeout(1); s.connect(("8.8.8.8", 80)); local_ip = s.getsockname()[0]; s.close()
                     logging.info(f"--- System Running --- Host: {hostname}")
                     logging.info(f"--- Access web interface at: http://{local_ip}:{config.WEB_PORT}")
                 except Exception as ip_e:
                     logging.warning(f"Could not determine local IP address: {ip_e}")
                     logging.info(f"--- System Running --- Access web interface at: http://<YOUR_PI_IP>:{config.WEB_PORT}")

            # Start the main operational loop
            main_loop(hw_manager, cam_manager)

            # If main_loop exits without shutdown_event being set, it's an error
            if shutdown_event.is_set():
                 logging.info("Shutdown event received during main loop, exiting initialization loop.")
                 break
            else:
                 # This should ideally not happen if main_loop handles errors properly
                 raise RuntimeError("Main loop exited unexpectedly without shutdown signal.")


        except Exception as e:
            logging.exception(f"!!! Unhandled exception in main setup or loop: {e}")
            main_loop_error_count += 1
            logging.error(f"Main loop error count: {main_loop_error_count}/{MAX_MAIN_LOOP_ERRORS}")
            if main_loop_error_count >= MAX_MAIN_LOOP_ERRORS:
                logging.critical("Maximum main loop error count reached. Forcing shutdown.")
                shutdown_event.set() # Force shutdown
            elif not shutdown_event.is_set():
                # Attempt restart after a delay
                logging.warning("Attempting restart after 10 seconds...")
                # Ensure managers are cleaned up before retry
                if cam_manager: cam_manager.shutdown(); cam_manager = None
                if hw_manager: hw_manager.cleanup(); hw_manager = None
                if flask_thread and flask_thread.is_alive(): logging.warning("Flask thread still alive during restart sequence.")
                # --- Multiprocessing Cleanup Placeholder ---
                # if cv_process and cv_process.is_alive():
                #    logging.warning("Terminating CV process during restart sequence...")
                #    cv_process.terminate()
                #    cv_process.join(timeout=2.0)
                #    cv_process = None
                # -------------------------------------------
                time.sleep(10.0)

    # --- Final Cleanup ---
    logging.info("--- Initiating Final Cleanup Sequence ---")
    if not shutdown_event.is_set():
        logging.warning("Initiating cleanup without prior shutdown signal.")
        shutdown_event.set() # Ensure shutdown event is set for cleanup steps

    # --- Multiprocessing Cleanup Placeholder ---
    # Terminate CV process first if it exists
    # if cv_process and cv_process.is_alive():
    #    logging.info("Terminating CV process...")
    #    cv_process.terminate() # Send SIGTERM
    #    cv_process.join(timeout=3.0) # Wait for clean exit
    #    if cv_process.is_alive():
    #        logging.warning("CV process did not terminate cleanly, forcing kill.")
    #        cv_process.kill() # Force kill if necessary
    #        cv_process.join(timeout=1.0)
    #    cv_process = None
    #    logging.info("CV process stopped.")
    # elif cv_process: logging.info("CV process already stopped.")
    # else: logging.debug("No CV process to stop.")
    # -------------------------------------------


    # Shutdown Camera Manager (stops recording, closes cameras)
    if cam_manager:
        logging.info("Shutting down Camera Manager...")
        cam_manager.shutdown()
    else: logging.warning("Camera Manager instance not available during final cleanup.")

    # Cleanup Hardware Manager (closes GPIO, unexports PWM)
    if hw_manager:
        logging.info("Cleaning up Hardware Manager...")
        hw_manager.cleanup()
    else: logging.warning("Hardware Manager instance not available during final cleanup.")

    # Wait for Flask thread to exit
    if flask_thread and flask_thread.is_alive():
        logging.info("Waiting up to 3 seconds for Flask thread to exit..."); flask_thread.join(timeout=3.0)
        if flask_thread.is_alive(): logging.warning("Flask thread did not exit cleanly after join timeout.")
        else: logging.info("Flask thread exited.")
    elif flask_thread: logging.info("Flask thread already exited.")
    else: logging.warning("Flask thread instance not available during final cleanup.")

    # Check if reboot was requested via Web UI
    reboot_flag = False
    try:
        with _ui_lock: reboot_flag = _app_state.get("reboot_requested", False)
    except NameError: logging.warning("Could not check reboot flag: UI state potentially not initialized.")

    if reboot_flag:
        logging.info("Reboot flag is set, attempting to execute disable script.")
        run_disable_script() # Attempt to run toggle script for reboot
    else:
        logging.info("Reboot flag not set, performing normal exit.")

    logging.info("========================================================")
    logging.info("--- Pi Camera Stream & Record Service Stopped ---")
    logging.info("========================================================")
    logging.shutdown() # Flush logging handlers


if __name__ == "__main__":
    # --- Multiprocessing Start Method (Example - Place *before* main() call) ---
    # if os.name == 'posix': # Needed on Linux/macOS for certain start methods
    #    multiprocessing.set_start_method('fork', force=True) # Or 'spawn'
    # ------------------------------------------------------------------------
    main()

# --- CV Process Loop Function Placeholder ---
# def run_cv_process_loop(input_q, output_q):
#    """Target function for the separate CV process."""
#    logging.info("CV Process started.")
#    import config # CV Process needs its own config access
#    import cv_functions
#    import camera_manager # May need for shared enums/constants if not passed
#
#    while True:
#        try:
#            # Get frame from input queue (blocking with timeout?)
#            frame_data = input_q.get(timeout=1.0)
#            if frame_data is None: # Sentinel value to stop
#                logging.info("CV Process received stop sentinel.")
#                break
#
#            raw_frame, timestamp = frame_data # Assuming tuple is passed
#
#            # Process the frame
#            processed_output = cv_functions.process_frame(raw_frame)
#
#            # Put result into output queue (non-blocking?)
#            try:
#                 # Send back processed frame or specific results (e.g., dict of detections)
#                 output_q.put((processed_output, timestamp), block=False)
#            except queue.Full:
#                 logging.warning("CV Process: Output queue full. Dropping result.")
#
#        except queue.Empty:
#            # Timeout waiting for frame, just continue loop
#            continue
#        except Exception as e:
#            logging.exception(f"!!! Error in CV process loop: {e}")
#            time.sleep(0.5) # Avoid busy-looping on error
#
#    logging.info("CV Process finished.")
# -------------------------------------------