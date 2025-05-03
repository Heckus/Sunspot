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
**Modification 4 (User Request):** Removed reference to deleted recording_frame_queue in main_loop logging.
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
    hardware checks, and reconfiguration requests. Recording submission/timing
    handled by CameraManager.
    """
    global _app_state, _ui_lock
    logging.info("--- Main loop started ---")
    consecutive_capture_errors = 0
    last_battery_check_time = time.monotonic()

    # Get initial primary camera FPS for loop timing
    try:
        cam_state = cam_manager.get_camera_state()
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
            if was_recording: logging.info("Stopping recording for reconfiguration..."); cam_manager.stop_recording(); time.sleep(0.5)
            logging.info(f"Attempting to re-initialize cameras (Cam0 index {requested_resolution_index})...")
            if cam_manager.initialize_cameras(requested_resolution_index):
                logging.info("--- Reconfiguration successful ---")
                try:
                    cam_state = cam_manager.get_camera_state()
                    new_fps = cam_state.get('actual_cam0_fps') or cam_state.get('target_cam0_fps') or 30.0
                    if not isinstance(new_fps, (int, float)) or new_fps <= 0: new_fps = 30.0
                    current_cam0_fps = new_fps; logging.info(f"Updated loop timing FPS: {current_cam0_fps:.2f}")
                except Exception: logging.exception("Failed to update loop timing FPS after reconfig.")
                if was_recording:
                    logging.info("Restarting recording after successful reconfiguration..."); time.sleep(1.0)
                    if not cam_manager.start_recording(): logging.error(f"Failed restart recording! Error: {cam_manager.last_error}")
            else: # Reconfiguration failed
                logging.error(f"!!! Failed reconfig (index {requested_resolution_index}). Error: {cam_manager.last_error}. Attempting restore... !!!")
                if cam_manager.initialize_cameras(): # Try restore
                     logging.info("Restored previous camera configuration.")
                     try: # Update loop timing FPS back
                         cam_state = cam_manager.get_camera_state(); old_fps = cam_state.get('actual_cam0_fps') or cam_state.get('target_cam0_fps') or 30.0
                         if not isinstance(old_fps, (int, float)) or old_fps <= 0: old_fps = 30.0
                         current_cam0_fps = old_fps; logging.info(f"Restored loop timing FPS: {current_cam0_fps:.2f}")
                     except Exception: logging.exception("Failed to restore loop timing FPS.")
                     if was_recording:
                         logging.warning("Attempting recording restart with restored config..."); time.sleep(1.0)
                         if not cam_manager.start_recording(): logging.error(f"Failed restart recording after restore. Error: {cam_manager.last_error}")
                else: # Restore failed
                    logging.critical(f"!!! Failed restore after failed reconfig. Error: {cam_manager.last_error}. Signaling shutdown. !!!"); shutdown_event.set(); break
            logging.info("--- Finished handling reconfiguration request ---"); time.sleep(0.5); continue

        # --- Capture and Process Frames ---
        combined_frame_for_stream = cam_manager.capture_and_combine_frames()
        if combined_frame_for_stream is None and cam_manager.is_initialized0:
            consecutive_capture_errors += 1; logging.warning(f"Frame capture failed ({consecutive_capture_errors}/{config.MAX_CONSECUTIVE_CAPTURE_ERRORS}). Error: {cam_manager.last_error}")
            if consecutive_capture_errors >= config.MAX_CONSECUTIVE_CAPTURE_ERRORS: logging.error("Too many capture errors. Signaling shutdown."); shutdown_event.set(); break
            time.sleep(0.5); continue
        else:
            if consecutive_capture_errors > 0 and combined_frame_for_stream is not None: logging.info(f"Frame capture recovered after {consecutive_capture_errors} errors.")
            consecutive_capture_errors = 0

        # --- Check Recording Triggers ---
        physical_switch_on = hw_manager.is_switch_pressed()
        with _ui_lock: digital_switch_on = _app_state.get("digital_recording_active", False)
        should_be_recording = physical_switch_on or digital_switch_on
        is_currently_recording = cam_manager.is_recording

        # --- Start/Stop Recording Logic ---
        if should_be_recording and not is_currently_recording:
            log_msg = "Physical switch ON" if physical_switch_on else ""; log_msg += (" / " if log_msg else "") + "Digital trigger ON" if digital_switch_on else ""; logging.info(f"Recording trigger active ({log_msg}) - initiating start.")
            if not cam_manager.start_recording(): logging.error(f"Attempt to start recording failed. Error: {cam_manager.last_error}")
        elif not should_be_recording and is_currently_recording:
            logging.info("Recording trigger(s) OFF - initiating stop."); cam_manager.stop_recording()

        # --- Periodic Hardware Checks ---
        current_time = time.monotonic()
        if hw_manager.ina219_sensor and (current_time - last_battery_check_time > config.BATTERY_READ_INTERVAL):
             old_batt = hw_manager.battery_percentage; hw_manager.read_battery_level(); new_batt = hw_manager.battery_percentage
             if new_batt is not None and (old_batt is None or abs(new_batt - old_batt) > 1.0): logging.info(f"Battery Level: {new_batt:.1f}%")
             elif new_batt is None and old_batt is not None: logging.warning(f"Battery reading failed. Last Error: {hw_manager.last_error}")
             last_battery_check_time = current_time

        # --- Loop Delay ---
        loop_duration = time.monotonic() - loop_start_time
        target_loop_time = (1.0 / current_cam0_fps) if current_cam0_fps > 0 else (1.0 / 30.0)
        sleep_time = max(0.001, target_loop_time - loop_duration)

        # Log loop timing info periodically for debugging
        if current_time - last_loop_log_time > 10.0:
             # *** Removed check for cam_manager.recording_frame_queue ***
             measured_fps_str = f"{cam_manager.measured_cam0_fps_avg:.1f}" if cam_manager.measured_cam0_fps_avg else "N/A"
             logging.debug(f"MainLoop dur: {loop_duration:.4f}s, Sleep: {sleep_time:.4f}s (Target Cap FPS: {current_cam0_fps:.1f}, Measured: {measured_fps_str})")
             last_loop_log_time = current_time

        time.sleep(sleep_time)

    logging.info("--- Main loop finished ---")


def main():
    """Main function to initialize and run the application."""
    global flask_thread, main_loop_error_count
    # global cv_process, cv_input_queue, cv_output_queue # Uncomment if using multiprocessing

    setup_logging()
    logging.info("========================================================")
    logging.info("=== Starting Pi Camera Stream & Record Service ===")
    logging.info("========================================================")
    logging.info(f"Using Python: {sys.version}"); logging.info(f"Script Directory: {config.SCRIPT_DIR}"); logging.info(f"Toggle Script Path: {config.TOGGLE_SCRIPT_PATH}")
    if os.geteuid() == 0: logging.warning("Script is running with root privileges (sudo).")
    else: logging.warning("Script is NOT running with root privileges. Servo/Toggle script may fail.")
    signal.signal(signal.SIGINT, signal_handler); signal.signal(signal.SIGTERM, signal_handler)
    hw_manager = None; cam_manager = None

    while main_loop_error_count < MAX_MAIN_LOOP_ERRORS and not shutdown_event.is_set():
        try:
            logging.info(f"--- Initializing Managers (Attempt {main_loop_error_count + 1}) ---")
            # --- Multiprocessing Setup Placeholder ---
            # cv_input_queue = multiprocessing.Queue(maxsize=5)
            # cv_output_queue = multiprocessing.Queue(maxsize=5)
            # cv_process = multiprocessing.Process(...) etc.
            # -------------------------------------------
            hw_manager = HardwareManager(); cam_manager = CameraManager()
            if not cam_manager.initialize_cameras(): raise RuntimeError(f"Initial camera setup failed: {cam_manager.last_error}")
            setup_web_app(cam_manager, hw_manager, shutdown_event) # Pass cv_output_queue?
            logging.info("Starting Flask web server thread..."); flask_thread = threading.Thread(target=run_web_server, name="FlaskThread", daemon=True); flask_thread.start(); time.sleep(2.0)
            if not flask_thread.is_alive(): raise RuntimeError("Flask web server thread failed to start.")
            logging.info("Flask thread started.")
            try: # Log access URLs
                hostname = socket.gethostname(); local_ip = socket.gethostbyname(hostname + ".local")
                logging.info(f"--- System Running --- Host: {hostname}"); logging.info(f"--- Access web interface at: http://{local_ip}:{config.WEB_PORT} or http://{hostname}.local:{config.WEB_PORT}")
            except socket.gaierror:
                 try: s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.settimeout(1); s.connect(("8.8.8.8", 80)); local_ip = s.getsockname()[0]; s.close()
                 except Exception as ip_e: logging.warning(f"Could not determine local IP: {ip_e}"); local_ip="<YOUR_PI_IP>"
                 logging.info(f"--- System Running --- Host: {hostname}"); logging.info(f"--- Access web interface at: http://{local_ip}:{config.WEB_PORT}")
            main_loop(hw_manager, cam_manager)
            if shutdown_event.is_set(): logging.info("Shutdown event received, exiting init loop."); break
            else: raise RuntimeError("Main loop exited unexpectedly without shutdown signal.")
        except Exception as e:
            logging.exception(f"!!! Unhandled exception in main setup or loop: {e}"); main_loop_error_count += 1
            logging.error(f"Main loop error count: {main_loop_error_count}/{MAX_MAIN_LOOP_ERRORS}")
            if main_loop_error_count >= MAX_MAIN_LOOP_ERRORS: logging.critical("Max errors reached. Forcing shutdown."); shutdown_event.set()
            elif not shutdown_event.is_set():
                logging.warning("Attempting restart after 10 seconds...")
                if cam_manager: cam_manager.shutdown(); cam_manager = None
                if hw_manager: hw_manager.cleanup(); hw_manager = None
                if flask_thread and flask_thread.is_alive(): logging.warning("Flask thread alive during restart.")
                # --- MP Cleanup Placeholder ---
                # if cv_process and cv_process.is_alive(): cv_process.terminate()...
                # -----------------------------
                time.sleep(10.0)

    # --- Final Cleanup ---
    logging.info("--- Initiating Final Cleanup Sequence ---")
    if not shutdown_event.is_set(): logging.warning("Initiating cleanup without prior shutdown signal."); shutdown_event.set()
    # --- MP Cleanup Placeholder ---
    # if cv_process and cv_process.is_alive(): cv_process.terminate()...
    # -----------------------------
    if cam_manager: logging.info("Shutting down Camera Manager..."); cam_manager.shutdown()
    else: logging.warning("Camera Manager instance N/A during final cleanup.")
    if hw_manager: logging.info("Cleaning up Hardware Manager..."); hw_manager.cleanup()
    else: logging.warning("Hardware Manager instance N/A during final cleanup.")
    if flask_thread and flask_thread.is_alive(): logging.info("Waiting for Flask thread..."); flask_thread.join(timeout=3.0)
    if flask_thread and flask_thread.is_alive(): logging.warning("Flask thread did not exit cleanly.")
    elif flask_thread : logging.info("Flask thread exited.")
    else: logging.warning("Flask thread instance N/A during final cleanup.")
    reboot_flag = False; 
    try: 
        with _ui_lock: reboot_flag = _app_state.get("reboot_requested", False)
    except NameError: logging.warning("Could not check reboot flag.")
    
    if reboot_flag: logging.info("Reboot flag set, attempting disable script."); run_disable_script()
    else: logging.info("Reboot flag not set, performing normal exit.")
    logging.info("========================================================"); logging.info("--- Pi Camera Stream & Record Service Stopped ---"); logging.info("========================================================")
    logging.shutdown()


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