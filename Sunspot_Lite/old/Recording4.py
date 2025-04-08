# -*- coding: utf-8 -*-
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
# Import controls for AWB setting:
from libcamera import controls

# --- Configuration ---
# ### CHANGE 2 START ###
# Reordered resolutions and added max native resolution for IMX219
SUPPORTED_RESOLUTIONS = [
    (640, 480),      # VGA
    (1280, 720),     # 720p HD
    (1640, 1232),    # Custom Pi Cam V2 mode
    (1920, 1080),    # 1080p FHD
    (3280, 2464)     # Max Native Resolution IMX219
]
# Default to 720p maybe? Index 1
DEFAULT_RESOLUTION_INDEX = 1
# ### CHANGE 2 END ###

FRAME_RATE = 30 # Note: Max resolution might not achieve this FPS
SWITCH_GPIO_PIN = 17
SWITCH_BOUNCE_TIME = 0.1
USB_BASE_PATH = "/media/hecke/"
RECORDING_FORMAT = "mp4v" # Use 'mp4v' for .mp4, check codec availability
RECORDING_EXTENSION = ".mp4"
WEB_PORT = 8000

# --- Global Variables ---
app = Flask(__name__)
picam2 = None
output_frame = None
frame_lock = threading.Lock()
config_lock = threading.Lock() # Lock for shared config like resolution index AND digital record state
capture_thread = None
flask_thread = None
shutdown_event = threading.Event()
switch = None
current_resolution_index = DEFAULT_RESOLUTION_INDEX
is_recording = False # Actual recording state
video_writers = []
recording_paths = []
reconfigure_resolution_index = None
last_error = None

# ### CHANGE 3 START ###
# State for the digital recording button
digital_recording_active = False
# ### CHANGE 3 END ###

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')

# --- Setup GPIO ---
def setup_gpio():
    """Sets up the GPIO pin using gpiozero."""
    global switch, last_error
    logging.info(f"Setting up GPIO pin {SWITCH_GPIO_PIN} using gpiozero Button")
    try:
        try:
            Device.pin_factory = NativeFactory()
            logging.info("Using gpiozero NativeFactory")
        except Exception as e:
            logging.warning(f"Could not set NativeFactory for gpiozero, using default: {e}")
        switch = Button(SWITCH_GPIO_PIN, pull_up=True, bounce_time=SWITCH_BOUNCE_TIME)
        logging.info(f"gpiozero Button on pin {SWITCH_GPIO_PIN} setup complete.")
        return True
    except Exception as e:
        logging.error(f"!!! Failed to setup gpiozero Button: {e}", exc_info=True)
        last_error = f"GPIO Setup Error: {e}"
        switch = None
        return False

def cleanup_gpio():
    # (Keep your existing cleanup_gpio function)
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
    # (Keep your existing get_current_resolution function)
    """Gets the resolution tuple based on the current index."""
    with config_lock: # Protect access to current_resolution_index
        if 0 <= current_resolution_index < len(SUPPORTED_RESOLUTIONS):
            return SUPPORTED_RESOLUTIONS[current_resolution_index]
        else:
            logging.warning(f"Invalid resolution index {current_resolution_index}, falling back to default.")
            safe_default_index = max(0, min(len(SUPPORTED_RESOLUTIONS) - 1, DEFAULT_RESOLUTION_INDEX))
            return SUPPORTED_RESOLUTIONS[safe_default_index]

# --- Modified initialize_camera for Picamera2 ---
def initialize_camera(target_width, target_height):
    # (Code is mostly the same, just adding set_controls for AWB)
    """Initializes or re-initializes the camera capture object with specific resolution."""
    global picam2, last_error
    logging.info(f"Attempting to initialize camera with Picamera2 at {target_width}x{target_height}...")

    # --- Stop/Close existing instance ---
    if picam2 is not None:
        try:
            if picam2.started: picam2.stop()
            picam2.close()
            logging.info("Previous Picamera2 instance stopped and closed.")
        except Exception as e:
            logging.warning(f"Error stopping/closing previous Picamera2 instance: {e}")
        picam2 = None
        time.sleep(0.5)

    try:
        picam2 = Picamera2()
        config = picam2.create_video_configuration(
            main={"size": (target_width, target_height), "format": "RGB888"},
            # Try limiting buffer count if memory issues arise, default is often 4
            # buffer_count=3,
            controls={
                "FrameRate": float(FRAME_RATE),
                # ### CHANGE 1 START ###
                # Explicitly set Auto White Balance mode to try and fix blue tinge
                "AwbEnable": True,
                "AwbMode": controls.AwbModeEnum.Auto # Options: Auto, Tungsten, Fluorescent, Indoor, Daylight, Cloudy
                # If Auto doesn't work well, try Daylight:
                # "AwbMode": controls.AwbModeEnum.Daylight
                # ### CHANGE 1 END ###
                }
        )
        logging.info(f"Configuring Picamera2 with: {config}")
        picam2.configure(config)
        logging.info("Configuration successful!")

        # Set controls *after* configure might be more reliable for some settings
        # picam2.set_controls({"AwbEnable": True, "AwbMode": controls.AwbModeEnum.Auto})
        # logging.info("AWB Controls applied.")

        picam2.start()
        logging.info("Camera started")
        time.sleep(1.5) # Allow more time for AWB to settle

        actual_config = picam2.camera_configuration()
        if not actual_config:
             raise RuntimeError("Failed to get camera configuration after start.")
        actual_format = actual_config['main']
        actual_w = actual_format['size'][0]
        actual_h = actual_format['size'][1]
        logging.info(f"Picamera2 initialized. Actual config: {actual_w}x{actual_h}, Format: {actual_format['format']}")
        last_error = None
        return True

    except Exception as e:
        logging.error(f"!!! Failed to initialize Picamera2 at {target_width}x{target_height}: {e}", exc_info=True)
        last_error = f"Picamera2 Init Error ({target_width}x{target_height}): {e}"
        if picam2 is not None:
            try:
                if picam2.started: picam2.stop()
                picam2.close()
            except Exception as close_e:
                logging.error(f"Error closing picam2 after init failure: {close_e}")
        picam2 = None
        return False

# --- USB Mounts and Recording ---
def get_usb_mounts():
    # (Keep your existing get_usb_mounts function)
    """Finds mounted, writable directories under the USB base path."""
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
    # (Keep your existing start_recording function - it correctly uses current config)
    # It already reads the current width/height after potential reconfiguration
    """Starts recording to all detected writable USB drives."""
    global is_recording, video_writers, recording_paths, last_error, picam2
    if is_recording:
        logging.warning("Request to start recording, but already recording.")
        return True

    logging.info("Attempting to start recording...")
    usb_drives = get_usb_mounts()
    if not usb_drives:
        logging.warning(f"Cannot start recording: No writable USB drives found in {USB_BASE_PATH}.")
        last_error = f"Cannot start recording: No writable USB drives found in {USB_BASE_PATH}"
        return False

    video_writers.clear()
    recording_paths.clear()
    success_count = 0
    start_error = None

    if picam2 is None or not picam2.started:
        logging.error("Cannot start recording, camera (Picamera2) is not available.")
        last_error = "Camera (Picamera2) not available for recording."
        return False

    try:
        cam_config = picam2.camera_configuration()['main']
        width = cam_config['size'][0]
        height = cam_config['size'][1]
        fps = float(FRAME_RATE)
        # Cap FPS for recording if it exceeds a threshold? Max resolution might struggle at 30fps.
        # max_rec_fps = 25 if width > 1920 else FRAME_RATE
        # fps = min(float(max_rec_fps), float(FRAME_RATE))


        if width <= 0 or height <= 0:
            logging.error(f"Picamera2 reported invalid dimensions ({width}x{height}). Cannot start recording.")
            last_error = "Invalid camera dimensions reported by Picamera2."
            return False

        logging.info(f"Starting recording with dimensions: {width}x{height} @ {fps}fps") # Log resolution used

        fourcc = cv2.VideoWriter_fourcc(*RECORDING_FORMAT)
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        for drive_path in usb_drives:
            try:
                filename = f"recording_{timestamp}_{width}x{height}{RECORDING_EXTENSION}"
                full_path = os.path.join(drive_path, filename)
                writer = cv2.VideoWriter(full_path, fourcc, fps, (width, height)) # Use determined fps

                if not writer.isOpened():
                    logging.error(f"!!! Failed to open VideoWriter for: {full_path}. Check codec ('{RECORDING_FORMAT}'), permissions, disk space.")
                    start_error = f"Failed writer: {full_path} (Codec: {RECORDING_FORMAT})"
                    continue

                video_writers.append(writer)
                recording_paths.append(full_path)
                logging.info(f"Successfully started recording to: {full_path}")
                success_count += 1

            except Exception as e:
                logging.error(f"!!! Failed to create VideoWriter for {drive_path}: {e}", exc_info=True)
                start_error = f"Error writer: {drive_path}: {e}"

        if success_count > 0:
            is_recording = True
            logging.info(f"Recording started on {success_count} drive(s).")
            if start_error:
                logging.warning(f"Note: Issues starting recording on all drives: {start_error}")
            last_error = None
            return True
        else:
            is_recording = False
            logging.error("Failed to start recording on any USB drive.")
            if start_error: last_error = f"Recording Start Failed: {start_error}"
            else: last_error = "Recording Start Failed: No writers could be opened."
            # Clean up lists if no writer succeeded
            video_writers.clear()
            recording_paths.clear()
            return False

    except Exception as e:
        logging.error(f"!!! Critical error during recording setup: {e}", exc_info=True)
        last_error = f"Recording Setup Error: {e}"
        stop_recording()
        return False


def stop_recording():
    # (Keep your existing stop_recording function)
    """Stops recording and releases video writer objects."""
    global is_recording, video_writers, recording_paths
    if not is_recording:
        if video_writers or recording_paths:
             logging.warning("stop_recording called while not 'is_recording', but writers/paths exist. Clearing.")
             video_writers.clear()
             recording_paths.clear()
        return

    logging.info("Stopping recording...")
    is_recording = False
    released_count = 0
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
    """Main loop executed in a background thread for capturing frames."""
    global output_frame, is_recording, last_error, picam2, switch, digital_recording_active # Added digital_recording_active
    global current_resolution_index, reconfigure_resolution_index

    logging.info("Starting frame capture loop (using Picamera2)...")
    consecutive_error_count = 0
    max_consecutive_errors = 15

    width, height = get_current_resolution()
    if not initialize_camera(width, height):
        logging.error("Initial camera setup failed (Picamera2). Capture thread cannot start.")
        last_error = last_error or "Initial camera setup failed"
        return

    while not shutdown_event.is_set():
        try:
            # --- Check for Reconfiguration Request ---
            if reconfigure_resolution_index is not None:
                target_index = -1
                with config_lock:
                    if reconfigure_resolution_index is not None:
                       target_index = reconfigure_resolution_index
                       reconfigure_resolution_index = None

                if target_index != -1:
                    logging.info(f"--- Reconfiguring resolution to index {target_index} ---")
                    was_recording = is_recording
                    # ### CHANGE 3 START ###
                    # Also check digital toggle state before stopping/starting
                    was_digital_active = digital_recording_active
                    # Decide if recording *should* resume based on combined state AFTER reconfig
                    should_be_recording_after = (switch is not None and switch.is_pressed) or was_digital_active
                    # ### CHANGE 3 END ###

                    if was_recording: # Stop if it was actually recording
                        logging.info("Pausing recording for reconfiguration...")
                        stop_recording()

                    new_width, new_height = SUPPORTED_RESOLUTIONS[target_index]
                    if initialize_camera(new_width, new_height):
                        current_resolution_index = target_index
                        logging.info(f"--- Reconfiguration successful to {new_width}x{new_height} ---")
                        # ### CHANGE 3 START ###
                        # Restart recording only if the combined state requires it
                        if should_be_recording_after:
                            logging.info("Resuming recording after reconfiguration...")
                            time.sleep(1.0)
                            if not start_recording():
                                logging.error("Failed to restart recording after reconfiguration!")
                        # ### CHANGE 3 END ###
                    else:
                        logging.error(f"!!! Failed reconfigure to index {target_index}. Restoring previous... !!!")
                        prev_width, prev_height = SUPPORTED_RESOLUTIONS[current_resolution_index]
                        if not initialize_camera(prev_width, prev_height):
                            logging.critical("!!! Failed to restore previous camera resolution. Stopping. !!!")
                            last_error = "Camera failed fatally during reconfiguration."
                            shutdown_event.set()
                            break
                        else:
                             logging.info("Successfully restored previous camera resolution.")
                             # ### CHANGE 3 START ###
                             # Restart recording only if the combined state requires it
                             if should_be_recording_after:
                                logging.info("Attempting recording restart with restored resolution...")
                                time.sleep(1.0)
                                if not start_recording():
                                    logging.error("Failed to restart recording after failed reconfig.")
                             # ### CHANGE 3 END ###


            # --- Check Camera Status ---
            if picam2 is None or not picam2.started:
                # ... (keep existing camera status check) ...
                if not last_error: last_error = "Picamera2 became unavailable unexpectedly."
                logging.error(f"Camera unavailable: {last_error}. Stopping capture loop.")
                shutdown_event.set()
                break


            # --- Read Frame using Picamera2 ---
            frame_array = picam2.capture_array("main")
            frame_bgr = cv2.cvtColor(frame_array, cv2.COLOR_RGB2BGR)

            if frame_bgr is None:
                # ... (keep existing frame read error handling) ...
                logging.warning("Failed capture/convert. Retrying...")
                consecutive_error_count += 1
                if consecutive_error_count > max_consecutive_errors:
                   last_error = f"Failed capture/convert {max_consecutive_errors} consecutive times."
                   logging.error(last_error)
                   shutdown_event.set()
                   break
                time.sleep(0.1)
                continue
            if consecutive_error_count > 0:
                logging.info(f"Recovered frame grab after {consecutive_error_count} errors.")
            consecutive_error_count = 0


            # --- Handle Recording State (Combined Switch + Digital) ---
            # ### CHANGE 3 START ###
            should_be_recording = False # Default to false
            physical_switch_on = False
            if switch is not None: # Check physical switch if available
                try:
                    physical_switch_on = switch.is_pressed
                except Exception as e:
                    logging.error(f"Error reading gpiozero switch state: {e}")
                    last_error = f"Switch Read Error: {e}"
                    # Decide: treat error as ON or OFF? Safer to treat as OFF?
                    physical_switch_on = False # Assume off on error

            # Check digital state (needs lock for read?) Use config_lock for simplicity
            with config_lock:
                digital_switch_on = digital_recording_active

            # OR logic: if either physical OR digital switch is on, we should record
            should_be_recording = physical_switch_on or digital_switch_on

            if should_be_recording:
                if not is_recording: # If we should be recording, but aren't...
                    log_msg = "Physical switch ON" if physical_switch_on else ""
                    if digital_switch_on: log_msg += " / Digital toggle ON"
                    logging.info(f"Recording trigger active ({log_msg.strip(' / ')}) - attempting start.")
                    if not start_recording() and not last_error:
                         last_error = "Attempted recording start (trigger ON) failed."
            else: # Should NOT be recording
                if is_recording: # If we are recording, but shouldn't be...
                    logging.info("Recording trigger(s) OFF - stopping recording.")
                    stop_recording()
            # ### CHANGE 3 END ###


            # --- Write Frame if Recording ---
            if is_recording:
                # (Keep existing frame writing logic)
                if not video_writers:
                    logging.warning("is_recording=True, but no video writers. Stopping recording state.")
                    is_recording = False
                    last_error = "Recording stopped: writers missing unexpectedly."
                else:
                    write_errors = 0
                    frame_to_write = frame_bgr
                    current_writers = list(video_writers)
                    current_paths = list(recording_paths)
                    for i, writer in enumerate(current_writers):
                        try:
                            writer.write(frame_to_write)
                        except Exception as e:
                            logging.error(f"!!! Failed write frame to {current_paths[i]}: {e}")
                            write_errors += 1
                    if write_errors > 0 and write_errors == len(current_writers):
                        logging.error("All writers failed. Stopping recording.")
                        last_error = "Recording stopped: All writers failed."
                        stop_recording()


            # --- Update Shared Frame for Streaming ---
            with frame_lock:
                output_frame = frame_bgr.copy()


        except Exception as e:
            # (Keep existing general exception handling)
            logging.exception(f"!!! Unexpected Error in capture loop: {e}")
            last_error = f"Capture Loop Error: {e}"
            consecutive_error_count += 1
            if consecutive_error_count > max_consecutive_errors / 2:
                logging.error(f"Too many errors ({consecutive_error_count}). Signaling shutdown.")
                shutdown_event.set()
            time.sleep(1)

    # --- Cleanup after loop exit ---
    # (Keep existing capture loop cleanup)
    logging.info("Exiting frame capture thread.")
    if is_recording:
        logging.warning("Capture loop exiting while recording. Forcing stop.")
        stop_recording()
    if picam2:
        try:
            if picam2.started: picam2.stop()
            picam2.close()
            logging.info("Picamera2 resource released by capture thread.")
        except Exception as e:
            logging.error(f"Error stopping/closing Picamera2 in thread cleanup: {e}")
    picam2 = None


# ===========================================================
# === FLASK ROUTES START HERE ===
# ===========================================================

# --- MJPEG Streaming Frame Generator ---
def generate_stream_frames():
    # (Keep existing generate_stream_frames function)
    """Generator function for the MJPEG stream."""
    global output_frame
    frame_counter = 0
    last_frame_time = time.monotonic()
    logging.info("MJPEG stream generator started.")
    while not shutdown_event.is_set():
        frame_to_encode = None
        with frame_lock:
            if output_frame is not None:
                frame_to_encode = output_frame.copy()
        if frame_to_encode is None:
            time.sleep(0.05)
            continue
        try:
            (flag, encodedImage) = cv2.imencode(".jpg", frame_to_encode, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if not flag:
                logging.warning("Stream generator: Could not encode frame.")
                time.sleep(0.1)
                continue
            yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
                  bytearray(encodedImage) + b'\r\n')
            frame_counter += 1
            current_time = time.monotonic()
            elapsed = current_time - last_frame_time
            wait_time = (1.0 / FRAME_RATE) - elapsed # Try to match target FPS
            if wait_time > 0:
                 time.sleep(wait_time)
            last_frame_time = time.monotonic()
        except GeneratorExit:
            logging.info(f"Streaming client disconnected after {frame_counter} frames.")
            break
        except Exception as e:
            logging.exception(f"!!! Error in streaming generator: {e}")
            time.sleep(0.5)
    logging.info("Stream generator thread exiting.")


# --- Flask Web Routes ---
@app.route("/")
def index():
    """Serves the main HTML page with controls."""
    global last_error, digital_recording_active # Access state
    current_w, current_h = get_current_resolution()
    resolution_text = f"{current_w}x{current_h}"
    err_msg = last_error if last_error else ""
    # Pass initial digital recording state to template
    digital_rec_state_initial = "ON" if digital_recording_active else "OFF"

    # ### CHANGE 3 START ###
    # Added Record Toggle button and modified script
    return render_template_string("""
    <!DOCTYPE html>
    <html>
      <head>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Pi Camera Stream & Record</title>
        <style>
          body { font-family: sans-serif; line-height: 1.4; margin: 1em;}
          #status, #rec-status { font-weight: bold; }
          .controls button { padding: 10px 15px; margin: 5px; font-size: 1em; cursor: pointer; border-radius: 5px; border: 1px solid #ccc;}
          #error { color: red; margin-top: 10px; white-space: pre-wrap; font-weight: bold; min-height: 1.2em;} /* Allow error wrap */
          img#stream { display: block; margin: 15px 0; border: 1px solid black; max-width: 100%; height: auto; background-color: #eee; }
          button#btn-record.recording-active { background-color: #ff4d4d; color: white; border-color: #ff1a1a; }
          button#btn-record.recording-inactive { background-color: #4CAF50; color: white; border-color: #367c39;}
          button:disabled { background-color: #cccccc; cursor: not-allowed; border-color: #999; color: #666;}
        </style>
      </head>
      <body>
        <h1>Pi Camera Stream & Record</h1>
        <p>Status: <span id="status">Initializing...</span></p>
        <p>Recording: <span id="rec-status">OFF</span></p> <p>Current Resolution: <span id="resolution">{{ resolution_text }}</span></p>

        <div class="controls">
          <button onclick="changeResolution('down')" id="btn-down">&laquo; Lower Res</button>
          <button onclick="changeResolution('up')" id="btn-up">Higher Res &raquo;</button>
          <button onclick="toggleRecording()" id="btn-record" class="recording-inactive">Start Recording</button> </div>

        <div id="error">{{ err_msg }}</div>
        <img id="stream" src="{{ url_for('video_feed') }}" width="{{ current_w }}" height="{{ current_h }}" alt="Loading stream...">

        <script>
          const statusElement = document.getElementById('status');
          const resolutionElement = document.getElementById('resolution');
          const errorElement = document.getElementById('error');
          const streamImage = document.getElementById('stream');
          const btnUp = document.getElementById('btn-up');
          const btnDown = document.getElementById('btn-down');
          const btnRecord = document.getElementById('btn-record');
          const recStatusElement = document.getElementById('rec-status'); // Get recording status span

          let isChangingResolution = false;
          let isTogglingRecording = false;
          let currentDigitalRecordState = {{ 'true' if digital_recording_active else 'false' }}; // Get initial state from template

          // Function to update button based on digital state
          function updateRecordButtonState() {
             if (currentDigitalRecordState) {
                 btnRecord.textContent = "Stop Recording (Digital)";
                 btnRecord.classList.remove('recording-inactive');
                 btnRecord.classList.add('recording-active');
             } else {
                 btnRecord.textContent = "Start Recording (Digital)";
                 btnRecord.classList.add('recording-inactive');
                 btnRecord.classList.remove('recording-active');
             }
          }

          // Update status display, including recording status and button state
          function updateStatus() {
              if (isChangingResolution || isTogglingRecording) return;

              fetch('/status')
                  .then(response => response.ok ? response.json() : Promise.reject(`HTTP error! status: ${response.status}`))
                  .then(data => {
                      statusElement.textContent = data.status_text;
                      recStatusElement.textContent = data.is_recording ? "ACTIVE" : "OFF"; // Update recording status display
                      if (data.resolution) {
                          resolutionElement.textContent = data.resolution;
                          const [w, h] = data.resolution.split('x');
                           if (streamImage.width != w || streamImage.height != h) {
                            streamImage.width = w;
                            streamImage.height = h;
                           }
                      }
                      if (errorElement.textContent !== data.error) {
                         errorElement.textContent = data.error || '';
                      }
                      // Update digital button state based on server status
                      if (typeof data.digital_recording_active === 'boolean') {
                           currentDigitalRecordState = data.digital_recording_active;
                           updateRecordButtonState();
                      }
                  })
                  .catch(err => {
                      console.error("Error fetching status:", err);
                      statusElement.textContent = "Error fetching status";
                      errorElement.textContent = 'Error fetching status from server.';
                      recStatusElement.textContent = "Unknown"; // Indicate unknown recording state on error
                  });
          }

          // Function for resolution change
          function changeResolution(direction) {
              if (isChangingResolution || isTogglingRecording) return;
              isChangingResolution = true;
              btnUp.disabled = true; btnDown.disabled = true; btnRecord.disabled = true; // Disable all buttons
              statusElement.textContent = 'Changing resolution... Please wait.';
              errorElement.textContent = '';

              fetch(`/set_resolution/${direction}`, { method: 'POST' })
                   .then(response => response.json().then(data => ({ status: response.status, body: data })))
                   .then(({ status, body }) => {
                        if (status === 200 && body.success) {
                            statusElement.textContent = 'Resolution change initiated.';
                            resolutionElement.textContent = body.new_resolution;
                            const [w, h] = body.new_resolution.split('x');
                            streamImage.width = w; streamImage.height = h;
                            setTimeout(() => { // Re-enable after delay
                                isChangingResolution = false;
                                btnUp.disabled = false; btnDown.disabled = false; btnRecord.disabled = false;
                                updateStatus();
                            }, 4000);
                        } else {
                            errorElement.textContent = `Error: ${body.message || 'Failed change resolution.'}`;
                            console.error("Resolution change failed:", body);
                            isChangingResolution = false;
                            btnUp.disabled = false; btnDown.disabled = false; btnRecord.disabled = false;
                            updateStatus();
                        }
                   })
                   .catch(err => {
                        console.error("Error sending resolution change:", err);
                        errorElement.textContent = 'Network error changing resolution.';
                        isChangingResolution = false;
                        btnUp.disabled = false; btnDown.disabled = false; btnRecord.disabled = false;
                        updateStatus();
                   });
          }

          // Function for toggling recording via web UI
          function toggleRecording() {
              if (isChangingResolution || isTogglingRecording) return; // Prevent overlaps
              isTogglingRecording = true;
              btnRecord.disabled = true; // Disable button during request
              statusElement.textContent = 'Sending record command...';

              fetch('/toggle_recording', { method: 'POST' })
                   .then(response => response.ok ? response.json() : Promise.reject(`HTTP error! Status: ${response.status}`))
                   .then(data => {
                       if (data.success) {
                           currentDigitalRecordState = data.digital_recording_active; // Update local state immediately
                           updateRecordButtonState(); // Update button appearance
                           statusElement.textContent = `Digital recording ${currentDigitalRecordState ? 'enabled' : 'disabled'}.`;
                           // Fetch full status shortly after to confirm actual recording state
                           setTimeout(updateStatus, 1500);
                       } else {
                           errorElement.textContent = `Error: ${data.message || 'Failed to toggle recording.'}`;
                           statusElement.textContent = 'Command failed.';
                           // Fetch status to see if state changed anyway or error occurred
                            setTimeout(updateStatus, 1000);
                       }
                   })
                   .catch(err => {
                       console.error("Error toggling recording:", err);
                       errorElement.textContent = 'Network error toggling recording.';
                       statusElement.textContent = 'Command failed (Network).';
                       // Fetch status after error
                       setTimeout(updateStatus, 1000);
                   })
                   .finally(() => {
                        // Re-enable button after request finishes (success or fail)
                        isTogglingRecording = false;
                        btnRecord.disabled = false;
                   });
          }


          // --- Initial setup and intervals ---
          document.addEventListener('DOMContentLoaded', () => {
              updateRecordButtonState(); // Set initial button state
              updateStatus(); // Initial status fetch
          });
          setInterval(() => { // Periodic status update
                if (!isChangingResolution && !isTogglingRecording) { updateStatus(); }
            }, 5000);

          // --- Stream error handling ---
          let errorReloadTimeout = null;
          streamImage.onerror = function() {
              console.warn("Stream image error detected (onerror).");
              if (errorReloadTimeout) return; // Already scheduled
              statusElement.textContent = 'Stream interrupted. Reloading...';
              errorReloadTimeout = setTimeout(() => {
                   console.log("Attempting stream reload...");
                   streamImage.src = "{{ url_for('video_feed') }}?" + new Date().getTime(); // Cache bust
                   errorReloadTimeout = null;
                   setTimeout(updateStatus, 1000); // Update status after trying reload
              }, 3000);
          };
          streamImage.onload = function() { // Clear error timeout on success
              if (errorReloadTimeout) { clearTimeout(errorReloadTimeout); errorReloadTimeout = null; }
          };

        </script>
      </body>
    </html>
    """, resolution_text=resolution_text, current_w=current_w, current_h=current_h, err_msg=err_msg, digital_recording_active=digital_recording_active)
    # ### CHANGE 3 END ###


@app.route("/video_feed")
def video_feed():
    # (Keep existing video_feed function)
    """Returns the MJPEG stream."""
    logging.info("Client connected to video feed.")
    return Response(generate_stream_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/status")
def status():
    """Returns the current status as JSON."""
    # ### CHANGE 3 START ###
    # Include digital recording state in status
    global last_error, digital_recording_active, is_recording # Access globals
    status_text = "Streaming"
    rec_stat = ""
    current_w, current_h = get_current_resolution()

    # Check actual recording state (is_recording reflects combined logic outcome)
    if is_recording:
        if not video_writers:
            rec_stat = " (ERROR: Rec stopped - writers missing)"
            if not last_error: last_error = "Recording stopped: writers missing"
        else:
            rec_stat = f" (Recording to {len(recording_paths)} USBs)"
    status_text += rec_stat

    # Get the digital button's desired state
    with config_lock:
        current_digital_state = digital_recording_active

    err_msg = last_error if last_error else ""

    # Simple logic to clear non-fatal errors
    if switch is not None and last_error and "GPIO Setup Error" in last_error:
        last_error = None; err_msg = ""
    if output_frame is not None and last_error and ("Init Error" in last_error or "unavailable" in last_error):
         last_error = None; err_msg = ""

    return jsonify({
        'is_recording': is_recording, # Actual recording state
        'digital_recording_active': current_digital_state, # Digital button's state
        'resolution': f"{current_w}x{current_h}",
        'status_text': status_text,
        'error': err_msg,
        'active_recordings': recording_paths
    })
    # ### CHANGE 3 END ###


@app.route("/set_resolution/<direction>", methods=['POST'])
def set_resolution(direction):
    # (Keep existing set_resolution function)
    """Endpoint to request resolution change."""
    global current_resolution_index, reconfigure_resolution_index, last_error
    with config_lock:
        if reconfigure_resolution_index is not None:
            logging.warning("Reconfiguration already in progress.")
            return jsonify({'success': False, 'message': 'Reconfiguration already in progress.'}), 429
        if not (0 <= current_resolution_index < len(SUPPORTED_RESOLUTIONS)):
             logging.error(f"Invalid current index {current_resolution_index}!")
             return jsonify({'success': False, 'message': 'Internal error: Invalid index.'}), 500
        original_index = current_resolution_index
        new_index = current_resolution_index
        if direction == 'up': new_index += 1
        elif direction == 'down': new_index -= 1
        else:
             return jsonify({'success': False, 'message': 'Invalid direction.'}), 400
        new_index = max(0, min(len(SUPPORTED_RESOLUTIONS) - 1, new_index))
        if new_index == original_index:
            msg = 'Already at highest.' if direction == 'up' else 'Already at lowest.'
            return jsonify({'success': False, 'message': msg}), 400
        new_w, new_h = SUPPORTED_RESOLUTIONS[new_index]
        logging.info(f"Web request: change resolution index {original_index} -> {new_index} ({new_w}x{new_h})")
        reconfigure_resolution_index = new_index
        last_error = None
        return jsonify({'success': True, 'message': 'Resolution change requested.', 'new_resolution': f"{new_w}x{new_h}"})


# ### CHANGE 3 START ###
# New route to handle the digital record button toggle
@app.route('/toggle_recording', methods=['POST'])
def toggle_recording():
    """Toggles the digital recording state."""
    global digital_recording_active, last_error
    new_state = False
    with config_lock: # Ensure thread-safe modification
        digital_recording_active = not digital_recording_active
        new_state = digital_recording_active
        logging.info(f"Digital recording toggled via web UI to: {'ON' if new_state else 'OFF'}")
        # Clear any previous recording start/stop errors when user interacts
        if last_error and ("Recording" in last_error or "writers" in last_error):
             last_error = None

    # The capture loop will pick up this change and start/stop recording accordingly
    return jsonify({'success': True, 'digital_recording_active': new_state})
# ### CHANGE 3 END ###


# ===========================================================
# === FLASK ROUTES END HERE ===
# ===========================================================


# --- Main Execution ---
def signal_handler(sig, frame):
    """Handles termination signals for graceful shutdown."""
    logging.warning(f"Received signal {sig}. Initiating graceful shutdown...")
    shutdown_event.set()

def main():
    # (Keep existing main function structure)
    """Main function to initialize and manage threads."""
    global last_error, capture_thread, flask_thread, picam2

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    logging.info(" --- Starting Camera Stream & Record Service (Picamera2 & gpiozero) --- ")

    while not shutdown_event.is_set():
        last_error = None
        capture_thread = None
        flask_thread = None
        if picam2: # Pre-loop cleanup
            try:
                if picam2.started: picam2.stop()
                picam2.close()
            except Exception as cleanup_e: pass
            picam2 = None

        try:
            logging.info("Initializing Hardware...")
            if not setup_gpio():
                 logging.error(f"GPIO setup failed: {last_error}. Switch control unavailable.")

            logging.info("Starting frame capture thread (Picamera2)...")
            capture_thread = threading.Thread(target=capture_and_process_loop, name="CaptureThread", daemon=True)
            capture_thread.start()

            time.sleep(4) # Wait for camera init
            if not capture_thread.is_alive():
                 raise RuntimeError(f"Capture thread failed start/init: {last_error or 'Thread died'}")
            if last_error and ("Error" in last_error or "failed" in last_error):
                 raise RuntimeError(f"Camera init failed in thread: {last_error}")
            logging.info("Capture thread running.")

            logging.info(f"Starting Flask web server on port {WEB_PORT}...")
            flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=WEB_PORT, debug=False, use_reloader=False, threaded=True), name="FlaskThread", daemon=True)
            flask_thread.start()

            time.sleep(1)
            if not flask_thread.is_alive():
                raise RuntimeError("Flask thread failed to start.")

            logging.info("--- System Running ---")
            logging.info(f"Access stream at: http://<YOUR_PI_IP>:{WEB_PORT}")

            while not shutdown_event.is_set():
                if not capture_thread.is_alive():
                    raise RuntimeError(last_error or "Capture thread terminated.")
                if not flask_thread.is_alive():
                    raise RuntimeError(last_error or "Flask thread terminated.")
                shutdown_event.wait(timeout=5.0)
            break # Exit outer loop if shutdown_event is set

        except RuntimeError as e:
            logging.error(f"!!! Runtime Error for Restart: {e}")
            logging.error("Attempting restart after 10s pause...")
            shutdown_event.set() # Signal threads
            if capture_thread and capture_thread.is_alive(): capture_thread.join(timeout=3.0)
            if flask_thread and flask_thread.is_alive(): flask_thread.join(timeout=1.0)
            shutdown_event.clear() # Reset for next loop
            time.sleep(10.0)
        except Exception as e:
            logging.exception(f"!!! Unhandled Exception in Main: {e}")
            logging.error("Attempting restart after 10s pause...")
            shutdown_event.set();
            if capture_thread and capture_thread.is_alive(): capture_thread.join(timeout=3.0)
            if flask_thread and flask_thread.is_alive(): flask_thread.join(timeout=1.0)
            shutdown_event.clear()
            time.sleep(10.0)

    # --- Final Cleanup ---
    logging.info("--- Shutdown initiated ---")
    shutdown_event.set() # Ensure flag is set
    if capture_thread and capture_thread.is_alive():
        logging.info("Waiting for capture thread exit...")
        capture_thread.join(timeout=5.0)
        if capture_thread.is_alive(): logging.warning("Capture thread did not exit cleanly.")
    if flask_thread and flask_thread.is_alive():
        logging.info("Flask thread cleanup (daemon should exit).")
    stop_recording()
    if picam2:
        try:
            if picam2.started: picam2.stop()
            picam2.close()
            logging.info("Picamera2 released final.")
        except Exception as e: pass
    cleanup_gpio()
    logging.info("--- Program Exit ---")


if __name__ == '__main__':
    main()