# -*- coding: utf-8 -*-
"""
main.py

Main application script for the Pi Camera Stream & Record service.
Initializes hardware and camera managers, starts the web UI,
and runs the main control loop.
"""

import os
import time
import logging
import threading
import signal
import subprocess
import sys # Import sys for interpreter check

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

def signal_handler(sig, frame):
    """Handles SIGINT and SIGTERM signals for graceful shutdown."""
    if shutdown_event.is_set():
        logging.warning("Shutdown already in progress, ignoring additional signal.")
        return
    logging.warning(f"Received signal {sig}. Initiating graceful shutdown...")
    # Ensure reboot is NOT triggered by external signal unless specifically requested via UI
    # (reboot_requested flag is handled separately)
    shutdown_event.set()

def setup_logging():
    """Configures the logging system."""
    log_level = getattr(logging, config.LOG_LEVEL.upper(), logging.INFO)
    logging.basicConfig(level=log_level,
                        format=config.LOG_FORMAT,
                        datefmt=config.LOG_DATE_FORMAT)
    # Quieten overly verbose libraries if needed
    logging.getLogger("werkzeug").setLevel(logging.WARNING)
    logging.getLogger("PIL.PngImagePlugin").setLevel(logging.WARNING)


def run_disable_script():
    """Executes the toggle.sh disable script for reboot."""
    logging.warning("Reboot requested. Executing disable script...")
    toggle_script_path = config.TOGGLE_SCRIPT_PATH

    if not toggle_script_path or not os.path.exists(toggle_script_path):
        logging.error(f"Disable script path not found or invalid: '{toggle_script_path}'. Cannot execute proper disable sequence.")
        # Update last_error in managers? Maybe not necessary if shutting down anyway.
        return False

    # Check for execute permissions
    if not os.access(toggle_script_path, os.X_OK):
        logging.error(f"Disable script '{toggle_script_path}' exists but is not executable! Run 'chmod +x {toggle_script_path}'.")
        return False

    # Check if running with sudo (required for the script)
    if os.geteuid() != 0:
         logging.error("Disable script requires root privileges (sudo) but script is not running as root. Cannot execute.")
         return False

    disable_command = ["sudo", toggle_script_path, "disable"] # toggle.sh should handle the reboot
    logging.info(f"Executing: {' '.join(disable_command)}")
    try:
        # Launch the disable script. It handles stopping the service and rebooting.
        # Use Popen and don't wait for it to finish, as it will kill this process.
        subprocess.Popen(disable_command)
        logging.warning("Disable script launched (includes reboot). Service will now exit.")
        # Give the script a moment to start before Python fully exits
        time.sleep(5)
        return True # Technically won't be reached if reboot is immediate
    except FileNotFoundError:
        logging.critical(f"!!! FAILED TO EXECUTE '{' '.join(disable_command)}'. 'sudo' command not found or script path incorrect?", exc_info=True)
    except PermissionError:
        logging.critical(f"!!! FAILED TO EXECUTE '{' '.join(disable_command)}'. Permission denied. Check sudoers configuration for the script.", exc_info=True)
    except Exception as e:
        logging.critical(f"!!! FAILED TO EXECUTE DISABLE SCRIPT: {e}", exc_info=True)

    return False # Failed to execute


def main_loop(hw_manager, cam_manager):
    """
    The main operational loop of the application.

    Args:
        hw_manager (HardwareManager): Instance of the hardware manager.
        cam_manager (CameraManager): Instance of the camera manager.
    """
    global _app_state, _ui_lock # Access shared state with UI
    logging.info("--- Main loop started ---")
    consecutive_capture_errors = 0
    last_battery_check_time = time.monotonic()

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
                cam_manager.stop_recording()
                time.sleep(0.5) # Allow time for files to close

            logging.info(f"Attempting to re-initialize camera to index {requested_resolution_index}...")
            if cam_manager.initialize_camera(requested_resolution_index):
                logging.info("--- Reconfiguration successful ---")
                if was_recording:
                    logging.info("Restarting recording after successful reconfiguration...")
                    time.sleep(1.0) # Give camera time to settle before recording
                    if not cam_manager.start_recording():
                        logging.error("Failed to restart recording after reconfiguration!")
                        # Error should be set in cam_manager.last_error
            else:
                logging.error(f"!!! Failed to reconfigure camera to index {requested_resolution_index}. Attempting to restore previous... !!!")
                # Error should be set in cam_manager.last_error
                # Try to re-initialize with the index *before* the failed attempt
                # Note: cam_manager.current_resolution_index should still hold the old index if init failed
                if cam_manager.initialize_camera(): # Try initializing with its current internal index
                     logging.info("Successfully restored previous camera resolution.")
                     if was_recording:
                         logging.warning("Attempting recording restart with restored resolution...")
                         time.sleep(1.0)
                         if not cam_manager.start_recording():
                             logging.error("Failed to restart recording after failed reconfig and restore.")
                else:
                    logging.critical("!!! Failed to restore previous camera resolution after failed reconfig. Signaling shutdown. !!!")
                    # This is likely a fatal error
                    shutdown_event.set()
                    break # Exit main loop

            logging.info("--- Finished handling reconfiguration request ---")
            # Continue to next loop iteration to allow changes to settle
            time.sleep(0.5)
            continue


        # --- Capture Frame ---
        frame = cam_manager.capture_frame()
        if frame is None:
            consecutive_capture_errors += 1
            logging.warning(f"Frame capture failed ({consecutive_capture_errors}/{config.MAX_CONSECUTIVE_CAPTURE_ERRORS}). Error: {cam_manager.last_error}")
            if consecutive_capture_errors >= config.MAX_CONSECUTIVE_CAPTURE_ERRORS:
                logging.error("Too many consecutive frame capture errors. Signaling shutdown.")
                shutdown_event.set()
                break # Exit main loop
            time.sleep(0.5) # Wait a bit longer after a capture error
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
                logging.error(f"Attempt to start recording failed. Error: {cam_manager.last_error}")
                # Potentially disable digital trigger on failure?
                # with _ui_lock:
                #     _app_state["digital_recording_active"] = False

        elif not should_be_recording and is_currently_recording:
            logging.info("Recording trigger(s) OFF - initiating stop.")
            cam_manager.stop_recording()


        # --- Periodic Hardware Checks ---
        current_time = time.monotonic()
        # Battery Check
        if hw_manager.ina219_sensor and (current_time - last_battery_check_time > config.BATTERY_READ_INTERVAL):
             # logging.debug("Performing periodic battery check...") # Can be verbose
             hw_manager.read_battery_level() # Reads and updates internal state
             last_battery_check_time = current_time


        # --- Loop Delay ---
        # Control loop speed - aim for slightly faster than frame rate?
        # This prevents busy-waiting if capture is fast.
        loop_duration = time.monotonic() - loop_start_time
        sleep_time = max(0.01, (1.0 / (config.DEFAULT_FRAME_RATE + 10)) - loop_duration) # Target slightly > FPS
        # logging.debug(f"Loop duration: {loop_duration:.4f}s, Sleep time: {sleep_time:.4f}s")
        time.sleep(sleep_time)


    logging.info("--- Main loop finished ---")


def main():
    """Main function to initialize and run the application."""
    global flask_thread, main_loop_error_count

    setup_logging()
    logging.info("========================================================")
    logging.info("=== Starting Pi Camera Stream & Record Service ===")
    logging.info("========================================================")
    logging.info(f"Using Python: {sys.version}")
    logging.info(f"Script Directory: {config.SCRIPT_DIR}")
    logging.info(f"Toggle Script Path: {config.TOGGLE_SCRIPT_PATH}")
    if os.geteuid() == 0:
         logging.warning("Script is running with root privileges (sudo).")
    else:
         logging.warning("Script is NOT running with root privileges. Servo control and toggle script execution will likely fail.")


    # Setup signal handling
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    hw_manager = None
    cam_manager = None

    while main_loop_error_count < MAX_MAIN_LOOP_ERRORS and not shutdown_event.is_set():
        try:
            logging.info(f"--- Initializing Managers (Attempt {main_loop_error_count + 1}) ---")
            # Initialize Hardware Manager
            hw_manager = HardwareManager()
            # Initialize Camera Manager
            cam_manager = CameraManager()
            if not cam_manager.initialize_camera():
                raise RuntimeError(f"Initial camera setup failed: {cam_manager.last_error}")

            # Setup Web UI (pass manager instances and shutdown event)
            setup_web_app(cam_manager, hw_manager, shutdown_event)

            # Start Flask Web Server in a separate thread
            flask_thread = threading.Thread(target=run_web_server, name="FlaskThread", daemon=True)
            flask_thread.start()
            time.sleep(2.0) # Give Flask time to start
            if not flask_thread.is_alive():
                raise RuntimeError("Flask web server thread failed to start.")

            # Log IP address for user convenience
            try:
                import socket; s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.connect(("8.8.8.8", 80)); local_ip = s.getsockname()[0]; s.close()
                logging.info(f"--- System Running --- Access web interface at: http://{local_ip}:{config.WEB_PORT}")
            except Exception as ip_e:
                 logging.warning(f"Could not determine local IP address: {ip_e}")
                 logging.info(f"--- System Running --- Access web interface at: http://<YOUR_PI_IP>:{config.WEB_PORT}")


            # Start the main application loop
            main_loop(hw_manager, cam_manager)

            # If main_loop exits gracefully (due to shutdown_event), break the while loop
            if shutdown_event.is_set():
                 logging.info("Shutdown event received, exiting initialization loop.")
                 break

        except Exception as e:
            logging.exception(f"!!! Unhandled exception in main setup or loop: {e}")
            main_loop_error_count += 1
            logging.error(f"Main loop error count: {main_loop_error_count}/{MAX_MAIN_LOOP_ERRORS}")
            if main_loop_error_count >= MAX_MAIN_LOOP_ERRORS:
                logging.critical("Maximum main loop error count reached. Forcing shutdown.")
                shutdown_event.set() # Ensure shutdown is triggered
            else:
                logging.warning("Attempting restart after 10 seconds...")
                # Perform partial cleanup before retry
                if cam_manager: cam_manager.shutdown()
                if hw_manager: hw_manager.cleanup()
                # Flask thread is daemon, should exit if main thread restarts,
                # but might need explicit handling if it hangs.
                time.sleep(10.0)
                # Clear shutdown event if it wasn't set by signal handler during error handling
                # shutdown_event.clear() # Let the loop condition handle this

    # --- Final Cleanup ---
    logging.info("--- Initiating Final Cleanup ---")

    # Ensure managers are cleaned up (even if loop failed)
    if cam_manager:
        logging.info("Shutting down Camera Manager...")
        cam_manager.shutdown()
    else:
        logging.warning("Camera Manager not initialized during cleanup.")

    if hw_manager:
        logging.info("Cleaning up Hardware Manager...")
        hw_manager.cleanup()
    else:
        logging.warning("Hardware Manager not initialized during cleanup.")

    # Wait briefly for Flask thread (optional, as it's daemon)
    if flask_thread and flask_thread.is_alive():
        logging.info("Waiting briefly for Flask thread to exit...")
        flask_thread.join(timeout=2.0)
        if flask_thread.is_alive():
            logging.warning("Flask thread did not exit cleanly.")

    # --- Handle Reboot Request ---
    reboot_flag = False
    with _ui_lock:
        reboot_flag = _app_state.get("reboot_requested", False)

    if reboot_flag:
        logging.info("Reboot flag is set, attempting to execute disable script.")
        run_disable_script()
        # If run_disable_script fails, we just continue to normal exit.
    else:
        logging.info("Reboot flag not set, performing normal exit.")

    logging.info("========================================================")
    logging.info("--- Pi Camera Stream & Record Service Stopped ---")
    logging.info("========================================================")


if __name__ == "__main__":
    main()
