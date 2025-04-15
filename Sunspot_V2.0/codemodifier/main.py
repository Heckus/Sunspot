# -*- coding: utf-8 -*-
"""
main.py

Main application script for the Pi Camera Stream & Record service.
Initializes hardware and camera managers (now handling dual cameras),
starts the web UI, and runs the main control loop.
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
    shutdown_event.set()

def setup_logging():
    """Configures the logging system."""
    log_level = getattr(logging, config.LOG_LEVEL.upper(), logging.INFO)
    logging.basicConfig(level=log_level,
                        format=config.LOG_FORMAT,
                        datefmt=config.LOG_DATE_FORMAT)
    logging.getLogger("werkzeug").setLevel(logging.WARNING)
    logging.getLogger("PIL.PngImagePlugin").setLevel(logging.WARNING)
    # Quieten sounddevice/soundfile if needed during audio dev
    # logging.getLogger("sounddevice").setLevel(logging.WARNING)
    # logging.getLogger("soundfile").setLevel(logging.WARNING)


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
        subprocess.Popen(disable_command)
        logging.warning("Disable script launched (includes reboot). Service will now exit.")
        time.sleep(5); return True
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
    # Get initial primary camera FPS for loop timing
    try:
        current_cam0_fps = cam_manager.get_camera_state().get('resolution_fps', 30.0)
        if not isinstance(current_cam0_fps, (int, float)) or current_cam0_fps <= 0:
             logging.warning(f"Invalid initial Cam0 FPS ({current_cam0_fps}), defaulting loop timing to 30fps.")
             current_cam0_fps = 30.0
    except Exception:
         logging.exception("Error getting initial Cam0 FPS, defaulting loop timing.")
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
            logging.info(f"--- Processing Cam0 resolution change request to index {requested_resolution_index} ---")
            was_recording = cam_manager.is_recording # Check before stopping

            if was_recording:
                logging.info("Stopping recording for reconfiguration...")
                cam_manager.stop_recording() # Handles video and audio stop/mux
                time.sleep(0.5) # Allow time for files to close/mux

            logging.info(f"Attempting to re-initialize cameras (Cam0 index {requested_resolution_index})...")
            # Re-initialize BOTH cameras, passing the new index for Cam0
            if cam_manager.initialize_cameras(requested_resolution_index):
                logging.info("--- Reconfiguration successful ---")
                # Update loop timing FPS
                try:
                    current_cam0_fps = cam_manager.get_camera_state().get('resolution_fps', current_cam0_fps) # Keep old on error
                    if not isinstance(current_cam0_fps, (int, float)) or current_cam0_fps <= 0: current_cam0_fps = 30.0
                except Exception: pass # Keep old FPS on error
                logging.info(f"Updated loop timing target FPS: {current_cam0_fps:.1f}")

                if was_recording:
                    logging.info("Restarting recording after successful reconfiguration...")
                    time.sleep(1.0) # Give cameras time to settle
                    if not cam_manager.start_recording():
                        logging.error(f"Failed to restart recording after reconfiguration! Error: {cam_manager.last_error} / Audio: {cam_manager.audio_last_error}")
            else:
                logging.error(f"!!! Failed to reconfigure cameras (Cam0 index {requested_resolution_index}). Error: {cam_manager.last_error}. Attempting restore... !!!")
                # Try to re-initialize with the index *before* the failed attempt
                if cam_manager.initialize_cameras(): # Restore previous state
                     logging.info("Successfully restored previous camera configuration.")
                     # Update loop timing FPS back to previous state
                     try:
                         current_cam0_fps = cam_manager.get_camera_state().get('resolution_fps', 30.0)
                         if not isinstance(current_cam0_fps, (int, float)) or current_cam0_fps <= 0: current_cam0_fps = 30.0
                     except Exception: pass
                     logging.info(f"Restored loop timing target FPS: {current_cam0_fps:.1f}")
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
            time.sleep(0.5)
            continue


        # --- Capture and Combine Frames ---
        # This now gets frames from both cameras and combines them
        combined_frame = cam_manager.capture_and_combine_frames()

        if combined_frame is None:
            consecutive_capture_errors += 1
            logging.warning(f"Combined frame capture failed ({consecutive_capture_errors}/{config.MAX_CONSECUTIVE_CAPTURE_ERRORS}). Error: {cam_manager.last_error}")
            if consecutive_capture_errors >= config.MAX_CONSECUTIVE_CAPTURE_ERRORS:
                logging.error("Too many consecutive frame capture errors. Signaling shutdown.")
                shutdown_event.set()
                break
            time.sleep(0.5)
            continue
        else:
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
             hw_manager.read_battery_level() # Reads and updates internal state
             last_battery_check_time = current_time
             # Add other periodic checks here if needed (e.g., disk space?)


        # --- Loop Delay ---
        # Base delay on primary camera's current FPS
        loop_duration = time.monotonic() - loop_start_time
        target_loop_time = 1.0 / (current_cam0_fps + 5) # Aim slightly faster than FPS
        sleep_time = max(0.005, target_loop_time - loop_duration) # Ensure minimal sleep
        # logging.debug(f"Loop duration: {loop_duration:.4f}s, Sleep time: {sleep_time:.4f}s (Target FPS: {current_cam0_fps:.1f})")
        time.sleep(sleep_time)


    logging.info("--- Main loop finished ---")


def main():
    """Main function to initialize and run the application."""
    global flask_thread, main_loop_error_count

    setup_logging()
    logging.info("========================================================")
    logging.info("=== Starting Pi Multi-Camera Stream & Record Service ===")
    logging.info("========================================================")
    logging.info(f"Using Python: {sys.version}")
    logging.info(f"Script Directory: {config.SCRIPT_DIR}")
    logging.info(f"Toggle Script Path: {config.TOGGLE_SCRIPT_PATH}")
    if os.geteuid() == 0: logging.warning("Script is running with root privileges (sudo).")
    else: logging.warning("Script is NOT running with root privileges. Servo/Toggle script may fail.")

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    hw_manager = None
    cam_manager = None

    while main_loop_error_count < MAX_MAIN_LOOP_ERRORS and not shutdown_event.is_set():
        try:
            logging.info(f"--- Initializing Managers (Attempt {main_loop_error_count + 1}) ---")
            hw_manager = HardwareManager()
            cam_manager = CameraManager()
            # Initialize BOTH cameras now
            if not cam_manager.initialize_cameras():
                # Error message should be set in cam_manager.last_error
                raise RuntimeError(f"Initial camera setup failed: {cam_manager.last_error}")

            setup_web_app(cam_manager, hw_manager, shutdown_event)

            flask_thread = threading.Thread(target=run_web_server, name="FlaskThread", daemon=True)
            flask_thread.start()
            time.sleep(2.0)
            if not flask_thread.is_alive():
                raise RuntimeError("Flask web server thread failed to start.")

            try:
                import socket; s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.connect(("8.8.8.8", 80)); local_ip = s.getsockname()[0]; s.close()
                logging.info(f"--- System Running --- Access web interface at: http://{local_ip}:{config.WEB_PORT}")
            except Exception as ip_e:
                 logging.warning(f"Could not determine local IP address: {ip_e}")
                 logging.info(f"--- System Running --- Access web interface at: http://<YOUR_PI_IP>:{config.WEB_PORT}")

            # Start the main application loop
            main_loop(hw_manager, cam_manager)

            if shutdown_event.is_set():
                 logging.info("Shutdown event received, exiting initialization loop.")
                 break

        except Exception as e:
            logging.exception(f"!!! Unhandled exception in main setup or loop: {e}")
            main_loop_error_count += 1
            logging.error(f"Main loop error count: {main_loop_error_count}/{MAX_MAIN_LOOP_ERRORS}")
            if main_loop_error_count >= MAX_MAIN_LOOP_ERRORS:
                logging.critical("Maximum main loop error count reached. Forcing shutdown.")
                shutdown_event.set()
            else:
                logging.warning("Attempting restart after 10 seconds...")
                # Perform partial cleanup before retry
                if cam_manager: cam_manager.shutdown()
                if hw_manager: hw_manager.cleanup()
                # Flask thread is daemon, should exit, but check anyway
                if flask_thread and flask_thread.is_alive():
                     logging.warning("Flask thread still alive during restart sequence.")
                     # Consider more forceful termination if needed, but risky
                time.sleep(10.0)

    # --- Final Cleanup ---
    logging.info("--- Initiating Final Cleanup ---")

    if cam_manager: logging.info("Shutting down Camera Manager..."); cam_manager.shutdown()
    else: logging.warning("Camera Manager not initialized during cleanup.")

    if hw_manager: logging.info("Cleaning up Hardware Manager..."); hw_manager.cleanup()
    else: logging.warning("Hardware Manager not initialized during cleanup.")

    if flask_thread and flask_thread.is_alive():
        logging.info("Waiting briefly for Flask thread to exit..."); flask_thread.join(timeout=2.0)
        if flask_thread.is_alive(): logging.warning("Flask thread did not exit cleanly.")

    # --- Handle Reboot Request ---
    reboot_flag = False
    with _ui_lock: reboot_flag = _app_state.get("reboot_requested", False)

    if reboot_flag:
        logging.info("Reboot flag is set, attempting to execute disable script.")
        run_disable_script()
    else:
        logging.info("Reboot flag not set, performing normal exit.")

    logging.info("========================================================")
    logging.info("--- Pi Multi-Camera Stream & Record Service Stopped ---")
    logging.info("========================================================")


if __name__ == "__main__":
    main()
