"""
main.py

Main application script for the Pi Camera Stream & Record service.
Initializes hardware and camera managers (now handling dual cameras),
starts the web UI, and runs the main control loop.

*** Uses logging.basicConfig() for root logger configuration ***
"""

import os
import time
import logging # <<< Keep standard logging import
import threading
import signal
import subprocess
import sys
import socket # <<< Keep socket import

# Import local modules
import config
from hardware_manager import HardwareManager
from camera_manager import CameraManager
from web_ui import setup_web_app, run_web_server, _app_state, _ui_lock # Import shared state vars too

# --- Global Variables ---
shutdown_event = threading.Event()
flask_thread = None
main_loop_error_count = 0
MAX_MAIN_LOOP_ERRORS = 5 # Restart main loop up to 5 times on unexpected errors

# --- No specific app_logger needed when using basicConfig ---

def signal_handler(sig, frame):
    """Handles SIGINT and SIGTERM signals for graceful shutdown."""
    if shutdown_event.is_set():
        logging.warning("Shutdown already in progress, ignoring additional signal.") # <<< Use logging.warning
        return
    logging.warning(f"Received signal {sig}. Initiating graceful shutdown...") # <<< Use logging.warning
    shutdown_event.set()

def setup_logging():
    """Configures the root logging system using basicConfig."""
    print("--- setup_logging() called (basicConfig) ---")
    try:
        # 1. Get the log level from config (or default)
        log_level_name = config.LOG_LEVEL.upper()
        log_level = getattr(logging, log_level_name, logging.INFO)

        # 2. Configure the root logger using basicConfig
        logging.basicConfig(level=log_level,
                            format=config.LOG_FORMAT,
                            datefmt=config.LOG_DATE_FORMAT,
                            stream=sys.stderr) # Explicitly set stream

        # Optionally configure levels for other libraries if needed
        # These will apply *after* basicConfig has set the root level
        logging.getLogger("werkzeug").setLevel(logging.WARNING)
        logging.getLogger("PIL.PngImagePlugin").setLevel(logging.WARNING)

        logging.info(f"--- Logging configured via basicConfig (Root Level={log_level_name}) ---") # <<< Use logging.info

    except Exception as e:
        print(f"--- CRITICAL ERROR during logging setup: {e} ---")
        # Fallback to basic print logging if setup fails
        logging.basicConfig(level=logging.DEBUG) # Basic fallback
        logging.error(f"Fallback logging activated due to error: {e}") # <<< Use logging.error

def run_disable_script():
    """Executes the toggle.sh disable script for reboot."""
    logging.warning("Reboot requested. Executing disable script...") # <<< Use logging.warning
    toggle_script_path = config.TOGGLE_SCRIPT_PATH

    if not toggle_script_path or not os.path.exists(toggle_script_path):
        logging.error(f"Disable script path not found or invalid: '{toggle_script_path}'. Cannot execute.") # <<< Use logging.error
        return False
    if not os.access(toggle_script_path, os.X_OK):
        logging.error(f"Disable script '{toggle_script_path}' not executable! Run 'chmod +x'."); return False # <<< Use logging.error
    if os.geteuid() != 0:
         logging.error("Disable script requires root privileges (sudo). Cannot execute."); return False # <<< Use logging.error

    disable_command = ["sudo", toggle_script_path, "disable"]
    logging.info(f"Executing: {' '.join(disable_command)}") # <<< Use logging.info
    try:
        subprocess.Popen(disable_command)
        logging.warning("Disable script launched (includes reboot). Service will now exit.") # <<< Use logging.warning
        time.sleep(5); return True
    except Exception as e:
        logging.critical(f"!!! FAILED TO EXECUTE DISABLE SCRIPT: {e}", exc_info=True) # <<< Use logging.critical
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
    logging.info("--- Main loop started ---") # <<< Use logging.info
    consecutive_capture_errors = 0
    last_battery_check_time = time.monotonic()
    # Get initial primary camera FPS for loop timing
    try:
        # Use the actual FPS stored in cam_manager if available
        current_cam0_fps = cam_manager.actual_cam0_fps
        if current_cam0_fps is None:
            # Fallback to getting from state if actual isn't set yet
             cam_state = cam_manager.get_camera_state()
             current_cam0_fps = cam_state.get('resolution_fps', 30.0) # Default 30 if state also fails

        if not isinstance(current_cam0_fps, (int, float)) or current_cam0_fps <= 0:
             logging.warning(f"Invalid initial Cam0 FPS ({current_cam0_fps}), defaulting loop timing to 30fps.") # <<< Use logging.warning
             current_cam0_fps = 30.0
        logging.debug(f"Main loop starting with target FPS: {current_cam0_fps:.2f}") # <<< Use logging.debug
    except Exception:
         logging.exception("Error getting initial Cam0 FPS, defaulting loop timing.") # <<< Use logging.exception
         current_cam0_fps = 30.0

    while not shutdown_event.is_set():
        loop_start_time = time.monotonic()

        # --- Check for Cam0 Reconfiguration Request from Web UI ---
        requested_resolution_index = None
        with _ui_lock:
            if _app_state.get("reconfigure_resolution_index") is not None:
                requested_resolution_index = _app_state["reconfigure_resolution_index"]
                _app_state["reconfigure_resolution_index"] = None # Consume the request

        if requested_resolution_index is not None:
            logging.info(f"--- Processing Cam0 resolution change request to index {requested_resolution_index} ---") # <<< Use logging.info
            was_recording = cam_manager.is_recording # Check before stopping

            if was_recording:
                logging.info("Stopping recording for reconfiguration...") # <<< Use logging.info
                cam_manager.stop_recording() # Handles video and audio stop/mux
                time.sleep(0.5) # Allow time for files to close/mux

            logging.info(f"Attempting to re-initialize cameras (Cam0 index {requested_resolution_index})...") # <<< Use logging.info
            # Re-initialize BOTH cameras, passing the new index for Cam0
            if cam_manager.initialize_cameras(requested_resolution_index):
                logging.info("--- Reconfiguration successful ---") # <<< Use logging.info
                # Update loop timing FPS using the newly stored actual FPS
                try:
                    current_cam0_fps = cam_manager.actual_cam0_fps # Get updated actual FPS
                    if current_cam0_fps is None or current_cam0_fps <= 0:
                        # Fallback if actual FPS wasn't set correctly during re-init
                        logging.warning("Actual FPS not available after reconfig, using state.") # <<< Use logging.warning
                        cam_state = cam_manager.get_camera_state()
                        current_cam0_fps = cam_state.get('resolution_fps', 30.0) # Default 30
                    if not isinstance(current_cam0_fps, (int, float)) or current_cam0_fps <= 0:
                        logging.warning(f"Invalid FPS after reconfig ({current_cam0_fps}), defaulting.") # <<< Use logging.warning
                        current_cam0_fps = 30.0
                except Exception:
                    logging.exception("Error getting FPS after reconfig, defaulting.") # <<< Use logging.exception
                    current_cam0_fps = 30.0
                logging.info(f"Updated loop timing target FPS: {current_cam0_fps:.2f}") # <<< Use logging.info

                if was_recording:
                    logging.info("Restarting recording after successful reconfiguration...") # <<< Use logging.info
                    time.sleep(1.0) # Give cameras time to settle
                    if not cam_manager.start_recording():
                        logging.error(f"Failed to restart recording after reconfiguration! Error: {cam_manager.last_error} / Audio: {cam_manager.audio_last_error}") # <<< Use logging.error
            else:
                logging.error(f"!!! Failed to reconfigure cameras (Cam0 index {requested_resolution_index}). Error: {cam_manager.last_error}. Attempting restore... !!!") # <<< Use logging.error
                # Try to re-initialize with the index *before* the failed attempt
                # Note: initialize_cameras doesn't store the *previous* index, so we just call it without index
                if cam_manager.initialize_cameras(): # Restore previous state (best effort)
                     logging.info("Successfully restored previous camera configuration.") # <<< Use logging.info
                     # Update loop timing FPS back to previous state's actual FPS
                     try:
                         current_cam0_fps = cam_manager.actual_cam0_fps # Get restored actual FPS
                         if current_cam0_fps is None or current_cam0_fps <= 0:
                              logging.warning("Actual FPS not available after restore, using state.") # <<< Use logging.warning
                              cam_state = cam_manager.get_camera_state()
                              current_cam0_fps = cam_state.get('resolution_fps', 30.0) # Default 30
                         if not isinstance(current_cam0_fps, (int, float)) or current_cam0_fps <= 0:
                              logging.warning(f"Invalid FPS after restore ({current_cam0_fps}), defaulting.") # <<< Use logging.warning
                              current_cam0_fps = 30.0
                     except Exception:
                         logging.exception("Error getting FPS after restore, defaulting.") # <<< Use logging.exception
                         current_cam0_fps = 30.0
                     logging.info(f"Restored loop timing target FPS: {current_cam0_fps:.2f}") # <<< Use logging.info

                     if was_recording:
                         logging.warning("Attempting recording restart with restored configuration...") # <<< Use logging.warning
                         time.sleep(1.0)
                         if not cam_manager.start_recording():
                             logging.error(f"Failed to restart recording after failed reconfig and restore. Error: {cam_manager.last_error} / Audio: {cam_manager.audio_last_error}") # <<< Use logging.error
                else:
                    logging.critical(f"!!! Failed to restore previous camera configuration after failed reconfig. Error: {cam_manager.last_error}. Signaling shutdown. !!!") # <<< Use logging.critical
                    shutdown_event.set()
                    break # Exit main loop

            logging.info("--- Finished handling reconfiguration request ---") # <<< Use logging.info
            time.sleep(0.5) # Brief pause after reconfig action
            continue # Skip rest of the loop iteration


        # --- Capture and Combine Frames ---
        # This now gets frames from both cameras and combines them
        # It also writes Cam0 frame to disk if recording is active
        combined_frame = cam_manager.capture_and_combine_frames()

        if combined_frame is None:
            consecutive_capture_errors += 1
            # Log less frequently if errors persist
            if consecutive_capture_errors == 1 or consecutive_capture_errors % 10 == 0:
                logging.warning(f"Combined frame capture failed ({consecutive_capture_errors}/{config.MAX_CONSECUTIVE_CAPTURE_ERRORS}). Error: {cam_manager.last_error}") # <<< Use logging.warning
            if consecutive_capture_errors >= config.MAX_CONSECUTIVE_CAPTURE_ERRORS:
                logging.error("Too many consecutive frame capture errors. Signaling shutdown.") # <<< Use logging.error
                shutdown_event.set()
                break
            time.sleep(0.5) # Wait longer on capture error
            continue
        else:
            if consecutive_capture_errors > 0:
                logging.info(f"Frame capture recovered after {consecutive_capture_errors} errors.") # <<< Use logging.info
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
            logging.info(f"Recording trigger active ({log_msg}) - initiating start.") # <<< Use logging.info
            if not cam_manager.start_recording():
                logging.error(f"Attempt to start recording failed. Error: {cam_manager.last_error} / Audio: {cam_manager.audio_last_error}") # <<< Use logging.error
                # Optionally disable digital trigger on failure?
                # with _ui_lock: _app_state["digital_recording_active"] = False

        elif not should_be_recording and is_currently_recording:
            logging.info("Recording trigger(s) OFF - initiating stop.") # <<< Use logging.info
            cam_manager.stop_recording() # Handles video/audio stop and muxing


        # --- Periodic Hardware Checks ---
        current_time = time.monotonic()
        # Battery Check
        if hw_manager.ina219_sensor and (current_time - last_battery_check_time > config.BATTERY_READ_INTERVAL):
             hw_manager.read_battery_level() # Reads and updates internal state
             last_battery_check_time = current_time
             # Add other periodic checks here if needed (e.g., disk space?)


        # --- Loop Delay ---
        # Base delay on primary camera's current actual FPS
        loop_duration = time.monotonic() - loop_start_time
        if current_cam0_fps > 0:
             # Try running loop exactly at target FPS first
             target_loop_time = 1.0 / current_cam0_fps
             # Original approach: run slightly faster
             # target_loop_time = 1.0 / (current_cam0_fps + 5)
        else:
             target_loop_time = 1.0 / 30.0 # Fallback if FPS is invalid

        sleep_time = max(0.001, target_loop_time - loop_duration) # Ensure minimal sleep, prevent 0
        # logging.debug(f"Loop duration: {loop_duration:.4f}s, Sleep time: {sleep_time:.4f}s (Target FPS: {current_cam0_fps:.1f})") # <<< Use logging.debug
        time.sleep(sleep_time)


    logging.info("--- Main loop finished ---") # <<< Use logging.info


def main():
    """Main function to initialize and run the application."""
    global flask_thread, main_loop_error_count
    print("--- main() function started ---") # Keep basic print

    # *** Logging setup is called first ***
    setup_logging()
    # If setup_logging fails catastrophically, the script might exit here

    logging.info("========================================================") # <<< Use logging.info
    logging.info("=== Starting Pi Multi-Camera Stream & Record Service ===") # <<< Use logging.info
    logging.info("========================================================") # <<< Use logging.info
    logging.info(f"Using Python: {sys.version}") # <<< Use logging.info
    logging.info(f"Script Directory: {config.SCRIPT_DIR}") # <<< Use logging.info
    logging.info(f"Toggle Script Path: {config.TOGGLE_SCRIPT_PATH}") # <<< Use logging.info
    if os.geteuid() == 0: logging.warning("Script is running with root privileges (sudo).") # <<< Use logging.warning
    else: logging.warning("Script is NOT running with root privileges. Servo/Toggle script/Hardware access may fail.") # <<< Use logging.warning

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    hw_manager = None
    cam_manager = None

    while main_loop_error_count < MAX_MAIN_LOOP_ERRORS and not shutdown_event.is_set():
        try:
            logging.info(f"--- Initializing Managers (Attempt {main_loop_error_count + 1}) ---") # <<< Use logging.info
            hw_manager = HardwareManager()
            cam_manager = CameraManager()
            # Initialize BOTH cameras now
            if not cam_manager.initialize_cameras():
                # Error message should be set in cam_manager.last_error
                logging.critical(f"Initial camera setup failed: {cam_manager.last_error}") # <<< Use logging.critical
                raise RuntimeError(f"Initial camera setup failed: {cam_manager.last_error}")

            logging.info("Setting up web application...") # <<< Use logging.info
            setup_web_app(cam_manager, hw_manager, shutdown_event)

            logging.info("Starting Flask web server thread...") # <<< Use logging.info
            flask_thread = threading.Thread(target=run_web_server, name="FlaskThread", daemon=True)
            flask_thread.start()
            time.sleep(2.0) # Give thread time to start
            if not flask_thread.is_alive():
                logging.critical("Flask web server thread failed to start.") # <<< Use logging.critical
                raise RuntimeError("Flask web server thread failed to start.")

            # Try to get local IP for user convenience
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(("8.8.8.8", 80)) # Connect to external address (doesn't send data)
                local_ip = s.getsockname()[0]
                s.close()
                logging.info(f"--- System Running --- Access web interface at: http://{local_ip}:{config.WEB_PORT}") # <<< Use logging.info
            except Exception as ip_e:
                 logging.warning(f"Could not determine local IP address: {ip_e}") # <<< Use logging.warning (Fixed NameError)
                 logging.info(f"--- System Running --- Access web interface at: http://<YOUR_PI_IP>:{config.WEB_PORT}") # <<< Use logging.info

            # Start the main application loop
            main_loop(hw_manager, cam_manager)

            # If main_loop exits without shutdown_event being set, it's likely an error handled inside
            if shutdown_event.is_set():
                 logging.info("Shutdown event received during main execution, exiting initialization loop.") # <<< Use logging.info
                 break
            else:
                 # This path might be reached if main_loop exits unexpectedly without setting the event
                 logging.warning("Main loop exited without shutdown event. Checking error count.") # <<< Use logging.warning


        except Exception as e:
            # Catch exceptions during the setup/initialization phase
            # Use standard logging now
            logging.exception(f"!!! Unhandled exception in main setup or loop: {e}") # Log full traceback
            main_loop_error_count += 1
            logging.error(f"Main loop error count: {main_loop_error_count}/{MAX_MAIN_LOOP_ERRORS}") # <<< Use logging.error
            if main_loop_error_count >= MAX_MAIN_LOOP_ERRORS:
                logging.critical("Maximum main loop error count reached. Forcing shutdown.") # <<< Use logging.critical
                shutdown_event.set() # Ensure shutdown is triggered
            else:
                logging.warning("Attempting restart after 10 seconds...") # <<< Use logging.warning
                # Perform partial cleanup before retry
                if cam_manager:
                    try: cam_manager.shutdown()
                    except Exception as cs_e: logging.error(f"Error during cam_manager shutdown on retry: {cs_e}") # <<< Use logging.error
                if hw_manager:
                    try: hw_manager.cleanup()
                    except Exception as hs_e: logging.error(f"Error during hw_manager cleanup on retry: {hs_e}") # <<< Use logging.error

                # Flask thread is daemon, should exit when main exits, but check anyway
                if flask_thread and flask_thread.is_alive():
                     logging.warning("Flask thread still alive during restart sequence.") # <<< Use logging.warning
                     # Consider more forceful termination if needed, but generally avoid

                # Reset managers to force re-initialization
                hw_manager = None
                cam_manager = None
                flask_thread = None

                time.sleep(10.0) # Wait before retrying

    # --- Final Cleanup ---
    # This block runs after the loop exits (due to shutdown_event or max errors)
    logging.info("--- Initiating Final Cleanup ---") # <<< Use logging.info

    # Ensure managers exist before trying to clean them up
    if cam_manager:
        logging.info("Shutting down Camera Manager...") # <<< Use logging.info
        try: cam_manager.shutdown()
        except Exception as e: logging.error(f"Error during final Camera Manager shutdown: {e}") # <<< Use logging.error
    else:
        logging.warning("Camera Manager was not initialized during cleanup.") # <<< Use logging.warning

    if hw_manager:
        logging.info("Cleaning up Hardware Manager...") # <<< Use logging.info
        try: hw_manager.cleanup()
        except Exception as e: logging.error(f"Error during final Hardware Manager cleanup: {e}") # <<< Use logging.error
    else:
        logging.warning("Hardware Manager was not initialized during cleanup.") # <<< Use logging.warning

    # Wait briefly for Flask thread if it was started
    if flask_thread and flask_thread.is_alive():
        logging.info("Waiting briefly for Flask thread to exit...") # <<< Use logging.info
        flask_thread.join(timeout=2.0)
        if flask_thread.is_alive():
            logging.warning("Flask thread did not exit cleanly.") # <<< Use logging.warning

    # --- Handle Reboot Request ---
    reboot_flag = False
    # Check _app_state safely
    try:
        with _ui_lock:
            reboot_flag = _app_state.get("reboot_requested", False)
    except Exception as lock_e:
         logging.error(f"Error accessing UI lock during reboot check: {lock_e}") # <<< Use logging.error


    if reboot_flag:
        logging.info("Reboot flag is set, attempting to execute disable script.") # <<< Use logging.info
        run_disable_script() # Ensure this handles permissions correctly
    else:
        logging.info("Reboot flag not set, performing normal exit.") # <<< Use logging.info

    logging.info("========================================================") # <<< Use logging.info
    logging.info("--- Pi Multi-Camera Stream & Record Service Stopped ---") # <<< Use logging.info
    logging.info("========================================================") # <<< Use logging.info
    print("--- main() function finished ---") # Add print statement


if __name__ == "__main__":
    # This check prevents code from running automatically if the script is imported
    main()