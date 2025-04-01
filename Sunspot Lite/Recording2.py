import cv2
import time
import datetime
import os
import threading
import signal
from flask import Flask, Response, render_template_string, jsonify, redirect, url_for
from gpiozero import Button, Device
from gpiozero.pins.native import NativeFactory
import logging
from picamera2 import Picamera2 # Import picamera2
from libcamera import controls # For potential controls like FPS

# --- Configuration ---
# CAMERA_DEVICE_INDEX is less relevant with Picamera2 unless you have multiple cams
SUPPORTED_RESOLUTIONS = [
    (640, 480),
    (1280, 720), # 1640x1232 is native, but let's use common ones
    (1920, 1080),
    (1640, 1232), # Native mode might be useful
]
DEFAULT_RESOLUTION_INDEX = 0 # Start with a smaller one for stability first
FRAME_RATE = 30
SWITCH_GPIO_PIN = 17
SWITCH_BOUNCE_TIME = 0.1
USB_BASE_PATH = "/media/hecke/"
RECORDING_FORMAT = "mp4v"
RECORDING_EXTENSION = ".mp4"
WEB_PORT = 8000

# --- Global Variables ---
app = Flask(__name__)
picam2 = None # Use picamera2 object instead of video_capture
output_frame = None
frame_lock = threading.Lock()
config_lock = threading.Lock()
capture_thread = None
flask_thread = None
shutdown_event = threading.Event()
switch = None
current_resolution_index = DEFAULT_RESOLUTION_INDEX
is_recording = False
video_writers = []
recording_paths = []
reconfigure_resolution_index = None
last_error = None

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# --- Setup GPIO (Your existing functions setup_gpio, cleanup_gpio) ---
# ... (Keep your setup_gpio and cleanup_gpio functions using gpiozero) ...
def setup_gpio():
    """Sets up the GPIO pin using gpiozero."""
    global switch, last_error
    logging.info(f"Setting up GPIO pin {SWITCH_GPIO_PIN} using gpiozero Button")
    try:
        # Set pin factory BEFORE initializing devices if needed
        try:
            Device.pin_factory = NativeFactory()
            logging.info("Using gpiozero NativeFactory (recommended for Pi 5/modern systems)")
        except Exception as e:
            logging.warning(f"Could not set NativeFactory for gpiozero, using default pin factory: {e}")

        switch = Button(SWITCH_GPIO_PIN, pull_up=True, bounce_time=SWITCH_BOUNCE_TIME)
        logging.info(f"gpiozero Button on pin {SWITCH_GPIO_PIN} setup complete.")
        return True
    except Exception as e:
        logging.error(f"!!! Failed to setup gpiozero Button: {e}")
        last_error = f"GPIO Setup Error (gpiozero): {e}"
        switch = None
        return False

def cleanup_gpio():
    """Cleans up GPIO resources using gpiozero."""
    global switch
    logging.info("Cleaning up GPIO (gpiozero).")
    try:
        if switch:
            switch.close()
            switch = None
            logging.info("gpiozero Button closed.")
    except Exception as e:
        logging.warning(f"Error during gpiozero cleanup: {e}")


def get_current_resolution():
    with config_lock:
        if 0 <= current_resolution_index < len(SUPPORTED_RESOLUTIONS):
            return SUPPORTED_RESOLUTIONS[current_resolution_index]
        else:
            logging.warning(f"Invalid resolution index {current_resolution_index}, falling back to default.")
            return SUPPORTED_RESOLUTIONS[DEFAULT_RESOLUTION_INDEX]

# --- Modified initialize_camera for Picamera2 ---
def initialize_camera(target_width, target_height):
    global picam2, last_error
    logging.info(f"Attempting to initialize camera with Picamera2 at {target_width}x{target_height}...")

    if picam2 is not None:
        try:
            if picam2.started:
                picam2.stop_preview()
                picam2.stop()
            picam2.close() # Close properly
            logging.info("Previous Picamera2 instance stopped and closed.")
        except Exception as e:
            logging.warning(f"Error stopping/closing previous Picamera2 instance: {e}")
        picam2 = None
        time.sleep(0.5) # Give time for resources to release

    try:
        picam2 = Picamera2()
        # Configure for video recording and preview (which we'll grab frames from)
        # 'main' is for high-res stills/video, 'lores' for low-res (if needed), 'raw'
        config = picam2.create_video_configuration(
            main={"size": (target_width, target_height), "format": "RGB888"}, # OpenCV uses BGR, but RGB888 is common input
            controls={"FrameRate": float(FRAME_RATE)} # Set framerate
        )
        picam2.configure(config)

        # Optional: Check sensor modes if needed
        # sensor_modes = picam2.sensor_modes
        # logging.info(f"Available sensor modes: {sensor_modes}")

        picam2.start()
        time.sleep(1.0) # Allow camera to initialize and AGC/AWB to settle

        actual_format = picam2.camera_configuration()['main']
        actual_w = actual_format['size'][0]
        actual_h = actual_format['size'][1]
        # Note: Getting exact FPS after start might be tricky, rely on configuration

        logging.info(f"Picamera2 initialized. Actual config: {actual_w}x{actual_h}, Format: {actual_format['format']}")

        last_error = None
        return True

    except Exception as e:
        logging.error(f"!!! Failed to initialize Picamera2 at {target_width}x{target_height}: {e}", exc_info=True) # Log traceback
        last_error = f"Picamera2 Init Error ({target_width}x{target_height}): {e}"
        if picam2 is not None:
            try:
                if picam2.started: picam2.stop()
                picam2.close()
            except Exception as close_e:
                logging.error(f"Error closing picam2 after init failure: {close_e}")
        picam2 = None
        return False

# --- get_usb_mounts, start_recording, stop_recording (adapt slightly) ---

def get_usb_mounts():
   # Your existing function is likely fine
   # ...
    mounts = []
    logging.debug(f"Checking for USB mounts under: {USB_BASE_PATH}")
    if not os.path.isdir(USB_BASE_PATH):
        logging.warning(f"USB base path '{USB_BASE_PATH}' does not exist or is not a directory.")
        return mounts
    try:
        for item in os.listdir(USB_BASE_PATH):
            path = os.path.join(USB_BASE_PATH, item)
            if os.path.isdir(path) and os.access(path, os.W_OK):
                mounts.append(path)
        if not mounts:
            logging.debug("No writable USB mounts found.")
        else:
            logging.debug(f"Found writable USB mounts: {mounts}")
    except Exception as e:
        logging.error(f"Error finding USB mounts in {USB_BASE_PATH}: {e}")
    return mounts


def start_recording():
    global is_recording, video_writers, recording_paths, last_error, picam2
    if is_recording:
        logging.warning("Request to start recording, but already recording.")
        return True

    logging.info("Attempting to start recording...")
    usb_drives = get_usb_mounts()
    if not usb_drives:
        logging.warning(f"Switch is ON, but no writable USB drives found in {USB_BASE_PATH}. Recording cannot start.")
        return False

    video_writers.clear()
    recording_paths.clear()
    success_count = 0
    start_error = None

    if picam2 is None or not picam2.started: # Check picam2 status
        logging.error("Cannot start recording, camera (Picamera2) is not available.")
        last_error = "Camera (Picamera2) not available for recording."
        return False

    try:
        # Get dimensions from the running configuration
        cam_config = picam2.camera_configuration()['main']
        width = cam_config['size'][0]
        height = cam_config['size'][1]
        # Use the configured frame rate
        fps = float(FRAME_RATE) # Use the rate we set

        if width <= 0 or height <= 0:
            logging.error(f"Picamera2 reported invalid dimensions ({width}x{height}). Cannot start recording.")
            last_error = "Invalid camera dimensions reported by Picamera2."
            return False

        fourcc = cv2.VideoWriter_fourcc(*RECORDING_FORMAT)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

        for drive_path in usb_drives:
            try:
                filename = f"recording_{timestamp}_{width}x{height}{RECORDING_EXTENSION}"
                full_path = os.path.join(drive_path, filename)
                # Make sure fps is a float or int suitable for VideoWriter
                writer = cv2.VideoWriter(full_path, fourcc, float(fps), (width, height))

                if not writer.isOpened():
                    logging.error(f"!!! Failed to open VideoWriter for: {full_path}. Check codec ('{RECORDING_FORMAT}') and permissions.")
                    start_error = f"Failed to open writer for {full_path} (Codec '{RECORDING_FORMAT}' issue?)"
                    continue

                video_writers.append(writer)
                recording_paths.append(full_path)
                logging.info(f"Successfully started recording to: {full_path}")
                success_count += 1

            except Exception as e:
                logging.error(f"!!! Failed to create VideoWriter for {drive_path}: {e}")
                start_error = f"Error creating writer for {drive_path}: {e}"

        if success_count > 0:
            is_recording = True
            logging.info(f"Recording started on {success_count} drive(s).")
            if start_error:
                logging.warning(f"Note: Encountered issues starting recording on all drives: {start_error}")
            last_error = None
            return True
        else:
            is_recording = False
            logging.error("Failed to start recording on any USB drive.")
            if start_error: last_error = f"Recording Start Failed: {start_error}"
            else: last_error = "Recording Start Failed: No writers could be opened."
            return False

    except Exception as e:
        logging.error(f"!!! Critical error during recording setup: {e}", exc_info=True)
        last_error = f"Recording Setup Error: {e}"
        stop_recording() # Ensure cleanup
        return False

def stop_recording():
    global is_recording, video_writers, recording_paths
    if not is_recording:
        return

    logging.info("Stopping recording...")
    is_recording = False # Set flag immediately
    released_count = 0

    # Make copies to avoid modification issues if called again quickly
    writers_to_release = list(video_writers)
    paths_recorded = list(recording_paths)
    video_writers.clear()
    recording_paths.clear()

    for i, writer in enumerate(writers_to_release):
        try:
            writer.release()
            logging.info(f"Stopped recording and saved: {paths_recorded[i]}")
            released_count += 1
        except Exception as e:
            logging.error(f"Error releasing VideoWriter for {paths_recorded[i]}: {e}")

    logging.info(f"Recording stopped. Released {released_count} writer(s).")


# --- Modified Capture Loop for Picamera2 ---
def capture_and_process_loop():
    global output_frame, is_recording, last_error, picam2, switch
    global current_resolution_index, reconfigure_resolution_index

    logging.info("Starting frame capture loop (using Picamera2)...")
    consecutive_error_count = 0
    max_consecutive_errors = 10 # Reduced errors needed as picam2 capture is different

    width, height = get_current_resolution()
    if not initialize_camera(width, height):
        logging.error("Initial camera setup failed (Picamera2). Capture thread cannot start.")
        return # Exit thread if camera init fails

    while not shutdown_event.is_set():
        try:
            # --- Check for Reconfiguration Request ---
            if reconfigure_resolution_index is not None:
                # (Your existing reconfiguration logic - needs testing with Picamera2)
                # It should call stop_recording, initialize_camera(new_res), start_recording
                with config_lock:
                   target_index = reconfigure_resolution_index
                   reconfigure_resolution_index = None # Clear the request flag

                logging.info(f"--- Received request to reconfigure resolution to index {target_index} ---")
                was_recording = is_recording
                if was_recording:
                    logging.info("Pausing recording for reconfiguration...")
                    stop_recording()

                new_width, new_height = SUPPORTED_RESOLUTIONS[target_index]
                if initialize_camera(new_width, new_height): # Re-init with new size
                    current_resolution_index = target_index
                    logging.info(f"--- Reconfiguration successful to {new_width}x{new_height} ---")
                    if was_recording:
                        logging.info("Resuming recording after reconfiguration...")
                        time.sleep(0.5) # Allow cam to settle
                        if not start_recording():
                            logging.error("Failed to restart recording after reconfiguration!")
                else:
                    logging.error(f"!!! Failed to reconfigure camera to index {target_index}. Attempting to restore previous resolution... !!!")
                    # Attempt to restore previous
                    prev_width, prev_height = SUPPORTED_RESOLUTIONS[current_resolution_index]
                    if not initialize_camera(prev_width, prev_height):
                        logging.critical("!!! Failed to restore previous camera resolution. Capture loop stopping. !!!")
                        last_error = "Camera failed fatally during reconfiguration."
                        shutdown_event.set() # Signal shutdown
                        break # Exit loop
                    # If restore succeeded, try restarting recording if it was on
                    if was_recording:
                        logging.info("Attempting to restart recording with restored resolution...")
                        time.sleep(0.5)
                        if not start_recording():
                            logging.error("Failed to restart recording after failed reconfiguration attempt.")


            # --- Check Camera Status ---
            if picam2 is None or not picam2.started:
                if not last_error: last_error = "Picamera2 became unavailable unexpectedly."
                logging.error(f"Camera unavailable: {last_error}. Stopping capture loop.")
                shutdown_event.set()
                break

            # --- Read Frame using Picamera2 ---
            # This blocks until a frame is available
            frame_array = picam2.capture_array("main") # Capture from the 'main' stream configured earlier

            # Picamera2 usually gives RGB, OpenCV expects BGR for many functions (like saving/displaying)
            # For writing with VideoWriter and encoding for stream, it might handle RGB ok,
            # but conversion is safer if you experience color issues.
            frame_bgr = cv2.cvtColor(frame_array, cv2.COLOR_RGB2BGR) # Convert if needed

            if frame_bgr is None: # Should not happen if capture_array succeeded
                logging.warning("Failed to get valid frame array from Picamera2. Retrying...")
                consecutive_error_count += 1
                if consecutive_error_count > max_consecutive_errors:
                   last_error = f"Failed to get frame array {max_consecutive_errors} consecutive times."
                   logging.error(last_error)
                   shutdown_event.set()
                   break
                time.sleep(0.1) # Short pause before retry
                continue
            consecutive_error_count = 0 # Reset error count on success

            # --- Handle Switch and Recording (Your existing logic) ---
            try:
                if switch is None: raise RuntimeError("GPIO Switch object not initialized")
                if switch.is_pressed:
                    if not is_recording:
                        logging.info("Switch pressed (ON) - attempting to start recording.")
                        start_recording()
                else:
                    if is_recording:
                        logging.info("Switch released (OFF) - stopping recording.")
                        stop_recording()
            except Exception as e:
                logging.error(f"Error reading gpiozero switch state: {e}")


            # --- Write Frame if Recording ---
            if is_recording:
                if not video_writers:
                    logging.warning("Recording flag is ON, but no active video writers. Attempting restart...")
                    stop_recording()
                    start_recording() # Try starting again immediately

                write_errors = 0
                # Make sure to write the BGR frame if you converted
                frame_to_write = frame_bgr
                for i, writer in enumerate(list(video_writers)): # Use list copy in case it changes
                    try:
                        writer.write(frame_to_write)
                    except Exception as e:
                        logging.error(f"!!! Failed to write frame to {recording_paths[i]}: {e}")
                        write_errors += 1
                        # Optional: consider removing the failed writer
                        # try:
                        #     writer.release()
                        #     video_writers.pop(i)
                        #     recording_paths.pop(i)
                        # except: pass


                if write_errors > 0 and not video_writers: # Check if all writers failed/were removed
                    logging.error("All video writers failed or were removed. Stopping recording.")
                    stop_recording()


            # --- Update Shared Frame for Streaming ---
            with frame_lock:
                 # Use the BGR frame for consistency if you converted
                output_frame = frame_bgr.copy()

        except Exception as e:
            logging.exception(f"!!! Unexpected Error in capture loop: {e}") # Log full traceback
            last_error = f"Capture Loop Error: {e}"
            consecutive_error_count += 1
            if consecutive_error_count > max_consecutive_errors / 2:
                logging.error(f"Too many consecutive errors ({consecutive_error_count}) in capture loop. Signaling shutdown.")
                shutdown_event.set()
            time.sleep(1) # Pause after unexpected error

    # --- Cleanup after loop exit ---
    logging.info("Exiting frame capture thread.")
    if is_recording: stop_recording()
    if picam2:
        try:
            if picam2.started: picam2.stop()
            picam2.close()
            logging.info("Picamera2 resource released by capture thread.")
        except Exception as e:
            logging.error(f"Error stopping/closing Picamera2 in capture thread cleanup: {e}")
    picam2 = None

# --- MJPEG Streaming (Your existing generate_stream_frames function should be okay) ---
# Ensure it uses the 'output_frame' which is now updated by the picamera2 loop
# def generate_stream_frames():
# ... (Keep your existing function) ...


# --- Flask Web Routes (Your existing routes should mostly work) ---
# Make sure the index() route reads the initial resolution correctly
# The /status route should be fine
# The /set_resolution route triggers reconfiguration which now uses picamera2's initialize_camera
# @app.route("/")
# ... (Keep your existing routes) ...


# --- Main Execution (Adapt slightly for Picamera2) ---
def signal_handler(sig, frame):
    logging.warning(f"Received signal {sig}. Initiating graceful shutdown...")
    shutdown_event.set()

def main():
    global last_error, capture_thread, flask_thread, picam2 # Change video_capture to picam2

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    logging.info(" --- Starting Camera Stream & Record Service (Picamera2 & gpiozero) --- ")

    while not shutdown_event.is_set():
        last_error = None
        capture_thread = None
        flask_thread = None
        # Ensure picam2 is None at the start of the loop
        if picam2:
            try:
                if picam2.started: picam2.stop()
                picam2.close()
            except Exception: pass # Ignore errors here, just ensuring cleanup
            picam2 = None

        try:
            logging.info("Initializing Hardware...")
            if not setup_gpio(): # Uses gpiozero now
                # Decide if this is fatal. For now, log and continue, switch won't work.
                 logging.error(f"GPIO setup failed: {last_error}. Switch control will be unavailable.")
                 # If switch is critical, you might want to raise RuntimeError here

            logging.info("Starting frame capture thread (Picamera2)...")
            capture_thread = threading.Thread(target=capture_and_process_loop, name="CaptureThread", daemon=True)
            capture_thread.start()

            time.sleep(3) # Give thread time to initialize camera
            if not capture_thread.is_alive():
                 # Check last_error set by the thread if it failed init
                 raise RuntimeError(f"Capture thread failed to start or initialize camera (Picamera2): {last_error if last_error else 'Unknown reason'}")
            if last_error and ("Picamera2 Init Error" in last_error or "Camera failed fatally" in last_error):
                 raise RuntimeError(f"Camera initialization failed within thread: {last_error}")


            logging.info(f"Starting Flask web server on port {WEB_PORT}...")
            # Ensure Flask runs without debug/reloader in threaded mode
            flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=WEB_PORT, debug=False, use_reloader=False), name="FlaskThread", daemon=True)
            flask_thread.start()

            logging.info("--- System Running ---")
            logging.info(f"Access stream at: http://<YOUR_PI_IP>:{WEB_PORT}")

            while not shutdown_event.is_set():
                if not capture_thread.is_alive():
                    if not last_error: last_error = "Capture thread terminated unexpectedly."
                    raise RuntimeError(last_error)
                if not flask_thread.is_alive():
                    if not last_error: last_error = "Flask web server thread terminated unexpectedly."
                    raise RuntimeError(last_error)
                if last_error and "fatal" in last_error.lower(): # Check for fatal errors flagged by thread
                    raise RuntimeError(f"Restarting due to critical error: {last_error}")

                shutdown_event.wait(timeout=2.0) # Wait for event or timeout
            break # Exit outer loop if shutdown_event is set

        except RuntimeError as e:
            logging.error(f"!!! Runtime Error Encountered in Main Loop: {e}")
            logging.error("Attempting recovery/restart after pause...")
            # Ensure resources are released before retrying
            shutdown_event.set() # Signal threads to stop if they haven't already
            if capture_thread and capture_thread.is_alive():
                 capture_thread.join(timeout=3.0) # Wait briefly for thread cleanup
            if is_recording: stop_recording()
            if picam2:
                try:
                   if picam2.started: picam2.stop()
                   picam2.close()
                except Exception: pass
                picam2 = None
            # GPIO cleanup happens in the final block or next setup attempt
            shutdown_event.clear() # Clear event for next restart attempt
            last_error = None # Reset last error for retry
            time.sleep(5.0) # Pause before retry
        except Exception as e:
            logging.exception(f"!!! Unhandled Exception in Main Loop: {e}")
            logging.error("Attempting restart after 5 seconds...")
            shutdown_event.set() # Signal threads
            # Add similar cleanup as RuntimeError block
            if capture_thread and capture_thread.is_alive(): capture_thread.join(timeout=3.0)
            if is_recording: stop_recording()
            if picam2:
                try:
                   if picam2.started: picam2.stop()
                   picam2.close()
                except Exception: pass
                picam2 = None
            shutdown_event.clear()
            last_error = None
            time.sleep(5.0)


    # --- Final Cleanup ---
    logging.info("--- Shutdown initiated ---")
    # Shutdown event should already be set, but ensure it is
    shutdown_event.set()

    if capture_thread and capture_thread.is_alive():
        logging.info("Waiting for capture thread to exit...")
        capture_thread.join(timeout=5.0)

    # Ensure recording is stopped (might be called again, but safe)
    stop_recording()

    # Ensure Picamera2 is closed (might be called again, but safe)
    if picam2:
        try:
            if picam2.started: picam2.stop()
            picam2.close()
            logging.info("Picamera2 released during final shutdown.")
        except Exception as e:
            logging.error(f"Error closing Picamera2 during final shutdown: {e}")

    cleanup_gpio() # Use gpiozero cleanup
    logging.info("--- Program Exit ---")


if __name__ == '__main__':
    main()