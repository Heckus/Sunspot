# -*- coding: utf-8 -*-
"""
main.py

Main application script for the Pi Camera Stream & Record service.
Initializes hardware and camera managers, starts the web UI,
and runs the main control loop.
Refactored for single-camera operation.
"""

import os
import time
import logging
import threading
import signal
import subprocess
import sys # Import sys for interpreter check
import socket

# Import local modules
import config
from hardware_manager import HardwareManager
from camera_manager import CameraManager
# Import shared state vars and setup functions from web_ui
from web_ui import setup_web_app, run_web_server, _app_state, _ui_lock

# --- Global Variables ---
shutdown_event = threading.Event()
flask_thread = None
main_loop_error_count = 0
MAX_MAIN_LOOP_ERRORS = 5 # Restart main loop up to 5 times on unexpected errors

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
                        # Force streaming to stderr to potentially bypass buffering issues
                        stream=sys.stderr)
    # Quieten noisy libraries
    logging.getLogger("werkzeug").setLevel(logging.WARNING)
    logging.getLogger("PIL.PngImagePlugin").setLevel(logging.WARNING)
    logging.getLogger("sounddevice").setLevel(logging.WARNING)
    logging.getLogger("soundfile").setLevel(logging.WARNING)
    logging.info("--- Logging configured ---")


def run_disable_script():
    """Executes the toggle.sh disable script for reboot."""
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
        # Use subprocess.run for simplicity, assuming script handles reboot quickly
        result = subprocess.run(disable_command, capture_output=True, text=True, timeout=10)
        logging.info(f"Disable script executed. Return code: {result.returncode}")
        logging.info(f"Stdout: {result.stdout}")
        logging.warning(f"Stderr: {result.stderr}") # Log stderr as warning
        logging.warning("Disable script launched (includes reboot). Service will now exit.")
        time.sleep(5) # Give time for shutdown/reboot process to take over
        return True
    except subprocess.TimeoutExpired:
        logging.error("Disable script timed out. Reboot might still happen.")
        return False # Indicate potential issue
    except Exception as e:
        logging.critical(f"!!! FAILED TO EXECUTE DISABLE SCRIPT: {e}", exc_info=True)
    return False


def main_loop(hw_manager, cam_manager):
    """
    The main operational loop of the application. Handles camera capture,
    recording logic, hardware checks, and reconfiguration requests.

    Args:
        hw_manager (HardwareManager): Instance of the hardware manager.
        cam_manager (CameraManager): Instance of the camera manager.
    """
    global _app_state, _ui_lock # Access shared state with UI
    logging.info("--- Main loop started ---")
    consecutive_capture_errors = 0
    last_battery_check_time = time.monotonic()
    current_fps = 30.0 # Default FPS for loop timing

    # Get initial camera FPS for loop timing
    try:
        # Use the ACTUAL FPS reported by the camera manager (refactored key)
        current_fps = cam_manager.get_camera_state().get('actual_fps', 30.0)
        if not isinstance(current_fps, (int, float)) or current_fps <= 0:
             logging.warning(f"Invalid initial Actual FPS ({current_fps}), defaulting loop timing to 30fps.")
             current_fps = 30.0
        else:
             logging.info(f"Main loop using initial Actual FPS for timing: {current_fps:.2f}")
    except Exception:
         logging.exception("Error getting initial Actual FPS, defaulting loop timing.")
         current_fps = 30.0

    last_loop_log_time = 0 # Track when loop timing was last logged

    while not shutdown_event.is_set():
        loop_start_time = time.monotonic()

        # --- Check for Reconfiguration Request from Web UI ---
        requested_resolution_index = None
        with _ui_lock:
            if _app_state.get("reconfigure_resolution_index") is not None:
                requested_resolution_index = _app_state["reconfigure_resolution_index"]
                _app_state["reconfigure_resolution_index"] = None # Consume the request

        if requested_resolution_index is not None:
            logging.info(f"--- Processing resolution change request to index {requested_resolution_index} ---")
            was_recording = cam_manager.is_recording # Check before stopping

            if was_recording:
                logging.info("Stopping recording for reconfiguration...")
                cam_manager.stop_recording() # Handles video and audio stop/mux
                time.sleep(0.5) # Allow time for files to close/mux

            logging.info(f"Attempting to re-initialize camera (index {requested_resolution_index})...")
            # Re-initialize the single camera (use refactored method)
            if cam_manager.initialize_camera(requested_resolution_index):
                logging.info("--- Reconfiguration successful ---")
                # Update loop timing FPS using the new ACTUAL FPS (use refactored key)
                try:
                    # Keep old FPS on error during state retrieval
                    current_fps = cam_manager.get_camera_state().get('actual_fps', current_fps)
                    if not isinstance(current_fps, (int, float)) or current_fps <= 0: current_fps = 30.0
                except Exception: pass # Keep old FPS on error
                logging.info(f"Updated loop timing target based on Actual FPS: {current_fps:.2f}")

                if was_recording:
                    logging.info("Restarting recording after successful reconfiguration...")
                    time.sleep(1.0) # Give camera time to settle
                    if not cam_manager.start_recording():
                        logging.error(f"Failed to restart recording after reconfiguration! Error: {cam_manager.last_error} / Audio: {cam_manager.audio_last_error}")
            else:
                logging.error(f"!!! Failed to reconfigure camera (index {requested_resolution_index}). Error: {cam_manager.last_error}. Attempting restore... !!!")
                # Try to re-initialize with the index *before* the failed attempt (use refactored method)
                if cam_manager.initialize_camera(): # Restore previous state (uses internal current_resolution_index)
                     logging.info("Successfully restored previous camera configuration.")
                     # Update loop timing FPS back to previous state's ACTUAL FPS (use refactored key)
                     try:
                         current_fps = cam_manager.get_camera_state().get('actual_fps', 30.0)
                         if not isinstance(current_fps, (int, float)) or current_fps <= 0: current_fps = 30.0
                     except Exception: pass
                     logging.info(f"Restored loop timing target based on Actual FPS: {current_fps:.2f}")
                     if was_recording:
                         logging.warning("Attempting recording restart with restored configuration...")
                         time.sleep(1.0)
                         if not cam_manager.start_recording():
                             logging.error(f"Failed to restart recording after failed reconfig and restore. Error: {cam_manager.last_error} / Audio: {cam_manager.audio_last_error}")
                else:
                    logging.critical(f"!!! Failed to restore previous camera configuration after failed reconfig. Error: {cam_manager.last_error}. Signaling shutdown. !!!")
                    shutdown_event.set()
                    break # Exit main loop

            logging.info("--- Finished handling reconfiguration request ---")
            time.sleep(0.5) # Allow settling time
            continue # Restart loop immediately after reconfig


        # --- Capture Frame ---
        # Use refactored method name
        frame = cam_manager.capture_frame()

        if frame is None:
            consecutive_capture_errors += 1
            logging.warning(f"Frame capture failed ({consecutive_capture_errors}/{config.MAX_CONSECUTIVE_CAPTURE_ERRORS}). Error: {cam_manager.last_error}")
            if consecutive_capture_errors >= config.MAX_CONSECUTIVE_CAPTURE_ERRORS:
                logging.error("Too many consecutive frame capture errors. Signaling shutdown.")
                shutdown_event.set()
                break
            time.sleep(0.5) # Wait longer after capture error
            continue # Skip rest of loop iteration
        else:
            # Reset error count on successful capture
            if consecutive_capture_errors > 0:
                logging.info(f"Frame capture recovered after {consecutive_capture_errors} errors.")
            consecutive_capture_errors = 0


        # --- Check Recording Triggers ---
        physical_switch_on = hw_manager.is_switch_pressed()
        with _ui_lock: # Read digital state under lock
            digital_switch_on = _app_state.get("digital_recording_active", False)

        should_be_recording = physical_switch_on or digital_switch_on
        is_currently_recording = cam_manager.is_recording

        # --- Start/Stop Recording Logic ---
        if should_be_recording and not is_currently_recording:
            log_msg = "Physical switch ON" if physical_switch_on else ""
            if digital_switch_on: log_msg += (" / " if log_msg else "") + "Digital trigger ON"
            logging.info(f"Recording trigger active ({log_msg}) - initiating start.")
            if not cam_manager.start_recording():
                logging.error(f"Attempt to start recording failed. Error: {cam_manager.last_error} / Audio: {cam_manager.audio_last_error}")
                # Optionally disable digital trigger on failure?
                # with _ui_lock: _app_state["digital_recording_active"] = False

        elif not should_be_recording and is_currently_recording:
            logging.info("Recording trigger(s) OFF - initiating stop.")
            cam_manager.stop_recording() # Handles video/audio stop and muxing


        # --- Periodic Hardware Checks ---
        current_time = time.monotonic()
        # Battery Check
        if hw_manager.ina219_sensor and (current_time - last_battery_check_time > config.BATTERY_READ_INTERVAL):
             # Read and log battery level periodically
             old_batt = hw_manager.battery_percentage
             hw_manager.read_battery_level() # Reads and updates internal state
             new_batt = hw_manager.battery_percentage
             if new_batt is not None and (old_batt is None or abs(new_batt - old_batt) > 1.0): # Log if changed significantly
                 logging.info(f"Battery Level: {new_batt:.1f}%")
             elif new_batt is None and old_batt is not None:
                 logging.warning(f"Battery reading failed. Last Error: {hw_manager.last_error}")

             last_battery_check_time = current_time
             # Add other periodic checks here if needed (e.g., disk space?)


        # --- Loop Delay ---
        # Base delay on camera's ACTUAL current FPS (use refactored variable)
        loop_duration = time.monotonic() - loop_start_time
        if current_fps > 0:
            target_loop_time = 1.0 / current_fps
        else:
            target_loop_time = 1.0 / 30.0 # Fallback if FPS is invalid
        sleep_time = max(0.001, target_loop_time - loop_duration) # Ensure minimal sleep

        # Log loop timing info periodically
        if current_time - last_loop_log_time > 10.0: # Log every 10 seconds
             logging.debug(f"Loop duration: {loop_duration:.4f}s, Sleep time: {sleep_time:.4f}s (Target FPS: {current_fps:.2f})")
             last_loop_log_time = current_time

        time.sleep(sleep_time)


    logging.info("--- Main loop finished ---")


def main():
    """Main function to initialize and run the application."""
    global flask_thread, main_loop_error_count

    # --- Setup Logging FIRST ---
    setup_logging()
    logging.info("========================================================")
    logging.info("====== Starting Pi Camera Stream & Record Service ======")
    logging.info("========================================================")
    logging.info(f"Using Python: {sys.version}")
    logging.info(f"Script Directory: {config.SCRIPT_DIR}")
    logging.info(f"Toggle Script Path: {config.TOGGLE_SCRIPT_PATH}")
    if os.geteuid() == 0: logging.warning("Script is running with root privileges (sudo).")
    else: logging.warning("Script is NOT running with root privileges. Servo/Toggle script may fail.")

    # --- Setup Signal Handlers ---
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    hw_manager = None
    cam_manager = None

    # --- Initialization Loop (Allows retries on failure) ---
    while main_loop_error_count < MAX_MAIN_LOOP_ERRORS and not shutdown_event.is_set():
        try:
            logging.info(f"--- Initializing Managers (Attempt {main_loop_error_count + 1}) ---")
            hw_manager = HardwareManager()
            cam_manager = CameraManager()
            # Initialize the single camera (use refactored method)
            if not cam_manager.initialize_camera():
                # Error message should be set in cam_manager.last_error
                raise RuntimeError(f"Initial camera setup failed: {cam_manager.last_error}")

            # --- Setup Web App ---
            # Pass manager instances and shutdown event to the web UI setup
            setup_web_app(cam_manager, hw_manager, shutdown_event)

            # --- Start Flask Thread ---
            logging.info("Starting Flask web server thread...")
            flask_thread = threading.Thread(target=run_web_server, name="FlaskThread", daemon=True)
            flask_thread.start()
            time.sleep(2.0) # Give Flask time to start
            if not flask_thread.is_alive():
                raise RuntimeError("Flask web server thread failed to start.")
            logging.info("Flask thread started.")

            # --- Log Network Info ---
            try:
                hostname = socket.gethostname()
                # Try getting IP associated with hostname first
                try:
                    local_ip = socket.gethostbyname(hostname)
                except socket.gaierror:
                    # Fallback: try connecting to an external IP to find the preferred outbound IP
                    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    s.settimeout(1.0) # Add timeout
                    s.connect(("8.8.8.8", 80)) # Google DNS as target
                    local_ip = s.getsockname()[0]
                    s.close()

                logging.info(f"--- System Running --- Host: {hostname}")
                logging.info(f"--- Access web interface at: http://{local_ip}:{config.WEB_PORT} or http://{hostname}.local:{config.WEB_PORT} (if mDNS works)")
            except (socket.gaierror, socket.timeout, OSError) as ip_e:
                 logging.warning(f"Could not determine local IP address: {ip_e}")
                 logging.info(f"--- System Running --- Access web interface at: http://<YOUR_PI_IP>:{config.WEB_PORT}")


            # --- Start the Main Application Loop ---
            main_loop(hw_manager, cam_manager)

            # If main_loop exits normally (which it shouldn't unless shutdown is set), check event
            if shutdown_event.is_set():
                 logging.info("Shutdown event received during main loop, exiting initialization loop.")
                 break
            else:
                 # This case indicates an unexpected exit from the main loop
                 logging.warning("Main loop exited without shutdown signal. Incrementing error count.")
                 raise RuntimeError("Main loop exited unexpectedly.")


        except Exception as e:
            logging.exception(f"!!! Unhandled exception in main setup or loop: {e}")
            main_loop_error_count += 1
            logging.error(f"Main loop error count: {main_loop_error_count}/{MAX_MAIN_LOOP_ERRORS}")
            if main_loop_error_count >= MAX_MAIN_LOOP_ERRORS:
                logging.critical("Maximum main loop error count reached. Forcing shutdown.")
                shutdown_event.set() # Ensure shutdown is signaled
            elif not shutdown_event.is_set(): # Don't wait if already shutting down
                logging.warning("Attempting restart after 10 seconds...")
                # Perform partial cleanup before retry
                if cam_manager:
                    logging.info("Attempting camera manager shutdown before restart...")
                    cam_manager.shutdown()
                    cam_manager = None # Reset manager instance
                if hw_manager:
                    logging.info("Attempting hardware manager cleanup before restart...")
                    hw_manager.cleanup()
                    hw_manager = None # Reset manager instance

                # Flask thread is daemon, should exit if main exits, but check anyway
                if flask_thread and flask_thread.is_alive():
                     logging.warning("Flask thread still alive during restart sequence.")
                     # We might not be able to cleanly stop it here without more complex signaling
                time.sleep(10.0) # Wait before retrying initialization

    # --- Final Cleanup ---
    logging.info("--- Initiating Final Cleanup Sequence ---")

    # Ensure shutdown event is set before cleanup
    if not shutdown_event.is_set():
        logging.warning("Shutdown event was not set before final cleanup. Setting now.")
        shutdown_event.set()

    # Stop managers if they exist
    if cam_manager:
        logging.info("Shutting down Camera Manager...")
        cam_manager.shutdown()
    else:
        logging.warning("Camera Manager instance not available during final cleanup.")

    if hw_manager:
        logging.info("Cleaning up Hardware Manager...")
        hw_manager.cleanup()
    else:
        logging.warning("Hardware Manager instance not available during final cleanup.")

    # Wait for Flask thread (with timeout)
    if flask_thread and flask_thread.is_alive():
        logging.info("Waiting up to 3 seconds for Flask thread to exit...")
        flask_thread.join(timeout=3.0)
        if flask_thread.is_alive():
            logging.warning("Flask thread did not exit cleanly after join timeout.")
            # At this point, the main process will exit, and the daemon thread will be terminated abruptly.
        else:
            logging.info("Flask thread exited.")
    elif flask_thread:
         logging.info("Flask thread already exited.")
    else:
         logging.warning("Flask thread instance not available during final cleanup.")


    # --- Handle Reboot Request ---
    reboot_flag = False
    # Check _app_state safely (it might not exist if setup failed early)
    try:
        with _ui_lock:
            reboot_flag = _app_state.get("reboot_requested", False)
    except NameError: # _ui_lock might not be defined if setup failed very early
        logging.warning("Could not check reboot flag: UI state potentially not initialized.")


    if reboot_flag:
        logging.info("Reboot flag is set, attempting to execute disable script.")
        run_disable_script() # This attempts to reboot the Pi
    else:
        logging.info("Reboot flag not set, performing normal exit.")

    logging.info("========================================================")
    logging.info("----- Pi Camera Stream & Record Service Stopped ------")
    logging.info("========================================================")
    logging.shutdown() # Explicitly shutdown logging


if __name__ == "__main__":
    main()
