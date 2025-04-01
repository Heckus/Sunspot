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
RECORDING_FORMAT = "mp4v" # Use 'mp4v' for .mp4, check codec availability
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

# Correct timestamp format might be needed depending on locale
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')

# --- Setup GPIO ---
def setup_gpio():
    """Sets up the GPIO pin using gpiozero."""
    global switch, last_error
    logging.info(f"Setting up GPIO pin {SWITCH_GPIO_PIN} using gpiozero Button")
    try:
        # Set pin factory BEFORE initializing devices if needed
        try:
            # This might fail if user 'hecke' is not in 'gpio' group AND logged out/in
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
    """Gets the resolution tuple based on the current index."""
    with config_lock: # Protect access to current_resolution_index
        if 0 <= current_resolution_index < len(SUPPORTED_RESOLUTIONS):
            return SUPPORTED_RESOLUTIONS[current_resolution_index]
        else:
            logging.warning(f"Invalid resolution index {current_resolution_index}, falling back to default.")
            # Ensure default index is valid too
            safe_default_index = max(0, min(len(SUPPORTED_RESOLUTIONS) - 1, DEFAULT_RESOLUTION_INDEX))
            return SUPPORTED_RESOLUTIONS[safe_default_index]

# --- Modified initialize_camera for Picamera2 ---
def initialize_camera(target_width, target_height):
    """Initializes or re-initializes the camera capture object with specific resolution."""
    global picam2, last_error
    logging.info(f"Attempting to initialize camera with Picamera2 at {target_width}x{target_height}...")

    if picam2 is not None:
        try:
            if picam2.started:
                # picam2.stop_preview() # Only if preview window was used
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
        # Use RGB888 format which OpenCV can handle (via conversion)
        config = picam2.create_video_configuration(
            main={"size": (target_width, target_height), "format": "RGB888"},
            controls={"FrameRate": float(FRAME_RATE)} # Ensure FrameRate is float
        )
        logging.info(f"Configuring Picamera2 with: {config}")
        picam2.configure(config)
        logging.info("Configuration successful!")


        # Optional: Check sensor modes if needed
        # sensor_modes = picam2.sensor_modes
        # logging.info(f"Available sensor modes: {sensor_modes}")

        picam2.start()
        logging.info("Camera started")
        time.sleep(1.0) # Allow camera to initialize and AGC/AWB to settle

        # Verify configuration after start
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

# --- USB Mounts and Recording ---

def get_usb_mounts():
    """Finds mounted, writable directories under the USB base path."""
    mounts = []
    logging.debug(f"Checking for USB mounts under: {USB_BASE_PATH}")
    if not os.path.isdir(USB_BASE_PATH):
        logging.warning(f"USB base path '{USB_BASE_PATH}' does not exist or is not a directory.")
        return mounts
    try:
        for item in os.listdir(USB_BASE_PATH):
            path = os.path.join(USB_BASE_PATH, item)
            # Check if it's a directory and writable
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
    """Starts recording to all detected writable USB drives."""
    global is_recording, video_writers, recording_paths, last_error, picam2
    if is_recording:
        logging.warning("Request to start recording, but already recording.")
        return True

    logging.info("Attempting to start recording...")
    usb_drives = get_usb_mounts()
    if not usb_drives:
        logging.warning(f"Switch is ON, but no writable USB drives found in {USB_BASE_PATH}. Recording cannot start.")
        last_error = f"Cannot start recording: No writable USB drives found in {USB_BASE_PATH}"
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

        # Ensure the codec is supported by the OpenCV build
        fourcc = cv2.VideoWriter_fourcc(*RECORDING_FORMAT)
        # Use a more robust timestamp format
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        for drive_path in usb_drives:
            try:
                filename = f"recording_{timestamp}_{width}x{height}{RECORDING_EXTENSION}"
                full_path = os.path.join(drive_path, filename)
                # Make sure fps is a float or int suitable for VideoWriter
                writer = cv2.VideoWriter(full_path, fourcc, float(fps), (width, height))

                if not writer.isOpened():
                    logging.error(f"!!! Failed to open VideoWriter for: {full_path}. Check codec ('{RECORDING_FORMAT}'), permissions, and disk space.")
                    # Try alternative codec/extension as fallback?
                    # Example: If mp4v fails, try 'XVID' with '.avi'
                    # if RECORDING_FORMAT == "mp4v": ... try XVID ...
                    start_error = f"Failed to open writer for {full_path} (Codec '{RECORDING_FORMAT}' issue?)"
                    continue # Try next drive

                video_writers.append(writer)
                recording_paths.append(full_path)
                logging.info(f"Successfully started recording to: {full_path}")
                success_count += 1

            except Exception as e:
                logging.error(f"!!! Failed to create VideoWriter for {drive_path}: {e}", exc_info=True)
                start_error = f"Error creating writer for {drive_path}: {e}"

        if success_count > 0:
            is_recording = True
            logging.info(f"Recording started on {success_count} drive(s).")
            if start_error: # Log if some drives failed but at least one succeeded
                logging.warning(f"Note: Encountered issues starting recording on all drives: {start_error}")
            last_error = None # Clear previous errors if recording started successfully
            return True
        else:
            # Failed to start on any drive
            is_recording = False
            logging.error("Failed to start recording on any USB drive.")
            if start_error: last_error = f"Recording Start Failed: {start_error}"
            else: last_error = "Recording Start Failed: No writers could be opened."
            return False

    except Exception as e:
        logging.error(f"!!! Critical error during recording setup: {e}", exc_info=True)
        last_error = f"Recording Setup Error: {e}"
        # Attempt cleanup even on setup error
        stop_recording() # Ensure any partially opened writers are closed
        return False

def stop_recording():
    """Stops recording and releases video writer objects."""
    global is_recording, video_writers, recording_paths
    if not is_recording:
        # If called when not recording, just ensure lists are clear
        if video_writers or recording_paths:
             logging.warning("stop_recording called while not 'is_recording', but writers/paths exist. Clearing.")
             video_writers.clear()
             recording_paths.clear()
        return

    logging.info("Stopping recording...")
    is_recording = False # Set flag immediately
    released_count = 0

    # Make copies to avoid modification issues if called again quickly or from another thread
    writers_to_release = list(video_writers)
    paths_recorded = list(recording_paths)
    video_writers.clear()
    recording_paths.clear()

    for i, writer in enumerate(writers_to_release):
        try:
            writer.release()
            # Check if file exists and has size > 0? Optional.
            logging.info(f"Stopped recording and saved: {paths_recorded[i]}")
            released_count += 1
        except Exception as e:
            logging.error(f"Error releasing VideoWriter for {paths_recorded[i]}: {e}")

    logging.info(f"Recording stopped. Released {released_count} writer(s).")


# --- Modified Capture Loop for Picamera2 ---
def capture_and_process_loop():
    """Main loop executed in a background thread for capturing frames."""
    global output_frame, is_recording, last_error, picam2, switch
    global current_resolution_index, reconfigure_resolution_index

    logging.info("Starting frame capture loop (using Picamera2)...")
    consecutive_error_count = 0
    max_consecutive_errors = 15 # Allow slightly more errors before giving up

    # Perform initial camera setup
    width, height = get_current_resolution()
    if not initialize_camera(width, height):
        logging.error("Initial camera setup failed (Picamera2). Capture thread cannot start.")
        last_error = last_error or "Initial camera setup failed" # Ensure error is set
        # Signal main thread? Or just exit? For now, just exit thread.
        return # Exit thread if camera init fails

    while not shutdown_event.is_set():
        try:
            # --- Check for Reconfiguration Request ---
            if reconfigure_resolution_index is not None:
                target_index = -1 # Init value
                with config_lock: # Lock before accessing shared variable
                    if reconfigure_resolution_index is not None: # Check again inside lock
                       target_index = reconfigure_resolution_index
                       reconfigure_resolution_index = None # Clear the request flag

                # Check if target_index was actually set
                if target_index != -1:
                    logging.info(f"--- Received request to reconfigure resolution to index {target_index} ---")
                    was_recording = is_recording
                    if was_recording:
                        logging.info("Pausing recording for reconfiguration...")
                        stop_recording()

                    new_width, new_height = SUPPORTED_RESOLUTIONS[target_index]
                    if initialize_camera(new_width, new_height): # Re-init with new size
                        current_resolution_index = target_index # Update state only on success
                        logging.info(f"--- Reconfiguration successful to {new_width}x{new_height} ---")
                        if was_recording:
                            logging.info("Resuming recording after reconfiguration...")
                            time.sleep(1.0) # Allow cam to settle more
                            if not start_recording():
                                logging.error("Failed to restart recording after reconfiguration!")
                    else:
                        logging.error(f"!!! Failed to reconfigure camera to index {target_index}. Attempting to restore previous resolution... !!!")
                        # Attempt to restore previous resolution
                        prev_width, prev_height = SUPPORTED_RESOLUTIONS[current_resolution_index] # Use the index that was last known working
                        if not initialize_camera(prev_width, prev_height):
                            logging.critical("!!! Failed to restore previous camera resolution. Capture loop stopping. !!!")
                            last_error = "Camera failed fatally during reconfiguration."
                            shutdown_event.set() # Signal shutdown
                            break # Exit loop
                        else:
                             logging.info("Successfully restored previous camera resolution.")
                             # If restore succeeded, try restarting recording if it was on
                             if was_recording:
                                logging.info("Attempting to restart recording with restored resolution...")
                                time.sleep(1.0)
                                if not start_recording():
                                    logging.error("Failed to restart recording after failed reconfiguration attempt.")
                    # End of reconfiguration block


            # --- Check Camera Status ---
            if picam2 is None or not picam2.started:
                if not last_error: last_error = "Picamera2 became unavailable unexpectedly."
                logging.error(f"Camera unavailable: {last_error}. Stopping capture loop.")
                shutdown_event.set() # Signal main thread to handle shutdown/restart
                break # Exit capture loop

            # --- Read Frame using Picamera2 ---
            # capture_array() blocks until a frame is ready
            frame_array = picam2.capture_array("main") # Capture from the 'main' stream

            # Picamera2 gives RGB, OpenCV mostly uses BGR
            frame_bgr = cv2.cvtColor(frame_array, cv2.COLOR_RGB2BGR)

            if frame_bgr is None: # Should be rare if capture_array succeeded
                logging.warning("Failed to get valid frame array (cvtColor failed?). Retrying...")
                consecutive_error_count += 1
                if consecutive_error_count > max_consecutive_errors:
                   last_error = f"Failed capture/convert {max_consecutive_errors} consecutive times."
                   logging.error(last_error)
                   shutdown_event.set()
                   break
                time.sleep(0.1) # Short pause before retry
                continue

            # Reset error count on successful frame grab and conversion
            if consecutive_error_count > 0:
                logging.info(f"Successfully grabbed frame after {consecutive_error_count} errors.")
            consecutive_error_count = 0


            # --- Handle Switch and Recording State ---
            if switch is not None: # Only check if switch was set up successfully
                try:
                    if switch.is_pressed: # Switch is ON (connected to GND)
                        if not is_recording:
                            logging.info("Switch pressed (ON) - attempting to start recording.")
                            if not start_recording() and not last_error: # Update last_error if start failed
                                 last_error = "Attempted recording start (switch ON) failed."
                    else: # Switch is OFF (open)
                        if is_recording:
                            logging.info("Switch released (OFF) - stopping recording.")
                            stop_recording()
                except Exception as e:
                    logging.error(f"Error reading gpiozero switch state: {e}")
                    last_error = f"Switch Read Error: {e}" # Record error
                    # Maybe disable switch checks after repeated errors?
            # else: # Log if switch wasn't available (optional)
            #    logging.debug("Switch not available for checking.")


            # --- Write Frame if Recording ---
            if is_recording:
                if not video_writers: # If recording flag is on but no writers, something went wrong
                    logging.warning("is_recording=True, but no active video writers found. Stopping recording state.")
                    is_recording = False # Correct state
                    last_error = "Recording stopped: video writers disappeared unexpectedly."
                else:
                    write_errors = 0
                    # Use the BGR frame
                    frame_to_write = frame_bgr
                    # Iterate over a copy in case the list is modified (e.g., by error handling)
                    current_writers = list(video_writers)
                    current_paths = list(recording_paths)
                    for i, writer in enumerate(current_writers):
                        try:
                            writer.write(frame_to_write)
                        except Exception as e:
                            logging.error(f"!!! Failed to write frame to {current_paths[i]}: {e}")
                            write_errors += 1
                            # Optional: Decide how to handle write errors. Remove writer? Stop all?
                            # For now, just log and count.

                    # If all writers failed, stop recording
                    if write_errors > 0 and write_errors == len(current_writers):
                        logging.error("All active video writers failed to write frame. Stopping recording.")
                        last_error = "Recording stopped: All writers failed."
                        stop_recording() # stop_recording clears the lists


            # --- Update Shared Frame for Streaming ---
            # This should be the last step using the frame in this loop
            with frame_lock:
                 # Use the BGR frame for consistency
                output_frame = frame_bgr.copy()


        except Exception as e:
            # Catch unexpected errors in the loop
            logging.exception(f"!!! Unexpected Error in capture loop: {e}") # Log full traceback
            last_error = f"Capture Loop Error: {e}"
            consecutive_error_count += 1
            if consecutive_error_count > max_consecutive_errors / 2: # More sensitive to general errors
                logging.error(f"Too many consecutive errors ({consecutive_error_count}) in capture loop. Signaling shutdown.")
                shutdown_event.set() # Signal main thread
            time.sleep(1) # Pause after unexpected error before continuing loop

    # --- Cleanup after loop exit ---
    logging.info("Exiting frame capture thread.")
    # Ensure recording is stopped if loop is exited while recording
    if is_recording:
        logging.warning("Capture loop exiting while recording. Forcing stop.")
        stop_recording()
    # Release camera resources
    if picam2:
        try:
            if picam2.started: picam2.stop()
            picam2.close()
            logging.info("Picamera2 resource released by capture thread.")
        except Exception as e:
            logging.error(f"Error stopping/closing Picamera2 in capture thread cleanup: {e}")
    picam2 = None # Ensure it's None after cleanup

# ===========================================================
# === ADDED FLASK ROUTES START HERE ===
# ===========================================================

# --- MJPEG Streaming Frame Generator ---
def generate_stream_frames():
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
            #logging.debug("Stream generator: No frame available, sleeping.") # Optional debug log
            time.sleep(0.05) # Wait briefly if no frame
            continue

        try:
            (flag, encodedImage) = cv2.imencode(".jpg", frame_to_encode, [cv2.IMWRITE_JPEG_QUALITY, 90]) # Adjust quality?
            if not flag:
                logging.warning("Stream generator: Could not encode frame to JPEG.")
                time.sleep(0.1)
                continue

            # Yield the frame in MJPEG format
            yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
                  bytearray(encodedImage) + b'\r\n')
            frame_counter += 1

            # Simple frame rate limiting for the stream loop itself if needed
            current_time = time.monotonic()
            elapsed = current_time - last_frame_time
            wait_time = (1.0 / FRAME_RATE) - elapsed
            if wait_time > 0:
                 time.sleep(wait_time)
            last_frame_time = time.monotonic()


        except GeneratorExit:
            # This happens naturally when a client disconnects
            logging.info(f"Streaming client disconnected after {frame_counter} frames.")
            break # Exit loop if client disconnects
        except Exception as e:
            logging.error(f"!!! Error in streaming generator: {e}")
            # Log traceback for more details on stream errors
            logging.exception("Streaming generator exception details:")
            time.sleep(0.5) # Pause after error before continuing

    logging.info("Stream generator thread exiting.")


# --- Flask Web Routes ---
@app.route("/")
def index():
    """Serves the main HTML page with controls."""
    global last_error # Access global error status
    current_w, current_h = get_current_resolution()
    resolution_text = f"{current_w}x{current_h}"
    err_msg = last_error if last_error else "" # Get last error message

    # --- HTML and Javascript for the web page ---
    return render_template_string("""
    <!DOCTYPE html>
    <html>
      <head>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Pi Camera Stream</title>
        <style>
          body { font-family: sans-serif; line-height: 1.4; margin: 1em;}
          #status { font-weight: bold; }
          .controls button { padding: 10px 15px; margin: 5px; font-size: 1em; cursor: pointer; }
          #error { color: red; margin-top: 10px; white-space: pre-wrap; font-weight: bold; } /* Allow error wrap */
          img#stream { display: block; margin: 15px 0; border: 1px solid black; max-width: 100%; height: auto; background-color: #eee; } /* Style image */
        </style>
      </head>
      <body>
        <h1>Pi Camera Stream</h1>
        <p>Status: <span id="status">Initializing...</span></p>
        <p>Current Resolution: <span id="resolution">{{ resolution_text }}</span></p>
        <div class="controls">
          <button onclick="changeResolution('down')" id="btn-down">&laquo; Lower Res</button>
          <button onclick="changeResolution('up')" id="btn-up">Higher Res &raquo;</button>
        </div>
        <div id="error">{{ err_msg }}</div>  <img id="stream" src="{{ url_for('video_feed') }}" width="{{ current_w }}" height="{{ current_h }}" alt="Loading stream...">

        <script>
          const statusElement = document.getElementById('status');
          const resolutionElement = document.getElementById('resolution');
          const errorElement = document.getElementById('error');
          const streamImage = document.getElementById('stream');
          const btnUp = document.getElementById('btn-up');
          const btnDown = document.getElementById('btn-down');
          let isChangingResolution = false; // Flag to prevent multiple clicks

          // Function to update status display
          function updateStatus() {
              // Don't fetch status if we are currently changing resolution
              if (isChangingResolution) return;

              fetch('/status')
                  .then(response => {
                      if (!response.ok) {
                          throw new Error(`HTTP error! status: ${response.status}`);
                      }
                      return response.json();
                   })
                  .then(data => {
                      statusElement.textContent = data.status_text;
                      if (data.resolution) {
                          resolutionElement.textContent = data.resolution;
                          const [w, h] = data.resolution.split('x');
                           // Only resize if different to avoid flicker/reload
                           if (streamImage.width != w || streamImage.height != h) {
                            streamImage.width = w;
                            streamImage.height = h;
                           }
                      }
                       // Update error message only if it changed or is new
                       // Avoid clearing user-triggered errors immediately
                       if (data.error || errorElement.textContent) {
                           // Only update if the error from server is different or new
                           if (errorElement.textContent !== data.error) {
                                errorElement.textContent = data.error || '';
                           }
                       } else {
                           errorElement.textContent = ''; // Clear error if server reports none
                       }
                  })
                  .catch(err => {
                      console.error("Error fetching status:", err);
                      statusElement.textContent = "Error fetching status";
                      errorElement.textContent = 'Error fetching status from server.';
                  });
          }

          // Function to handle resolution change request
          function changeResolution(direction) {
              if (isChangingResolution) {
                  console.log("Resolution change already in progress, ignoring click.");
                  return; // Prevent multiple simultaneous requests
              }
              isChangingResolution = true; // Set flag
              btnUp.disabled = true; // Disable buttons during change
              btnDown.disabled = true;
              statusElement.textContent = 'Changing resolution... Please wait.';
              errorElement.textContent = ''; // Clear previous errors

              fetch(`/set_resolution/${direction}`, { method: 'POST' })
                  .then(response => response.json().then(data => ({ status: response.status, body: data }))) // Parse JSON regardless of status
                  .then(({ status, body }) => {
                      if (status === 200 && body.success) {
                          statusElement.textContent = 'Resolution change initiated. Stream should update.';
                          resolutionElement.textContent = body.new_resolution;
                          const [w, h] = body.new_resolution.split('x');
                          streamImage.width = w;
                          streamImage.height = h;
                          // Give ample time for reconfiguration before next status update
                          setTimeout(() => {
                              isChangingResolution = false; // Reset flag
                              btnUp.disabled = false; // Re-enable buttons
                              btnDown.disabled = false;
                              updateStatus(); // Update status after delay
                          }, 4000); // Increased delay
                      } else {
                          // Handle errors reported by the server (like 400, 429, or success=false)
                          errorElement.textContent = `Error: ${body.message || 'Failed to change resolution.'}`;
                          console.error("Resolution change failed:", body);
                          // Re-enable buttons and update status immediately on error
                          isChangingResolution = false;
                          btnUp.disabled = false;
                          btnDown.disabled = false;
                          updateStatus();
                      }
                  })
                  .catch(err => {
                      // Handle network errors or fetch failures
                      console.error("Error sending resolution change request:", err);
                      errorElement.textContent = 'Network error: Failed to send resolution change request.';
                      // Re-enable buttons and update status
                      isChangingResolution = false;
                      btnUp.disabled = false;
                      btnDown.disabled = false;
                      updateStatus();
                  });
          }

          // Initial status update on page load
          document.addEventListener('DOMContentLoaded', updateStatus);
          // Periodic status update only if not changing resolution
          setInterval(() => {
                if (!isChangingResolution) {
                    updateStatus();
                }
            }, 5000); // Update status every 5 seconds


          // Reload image on error, prevents broken image icon staying forever
          let errorReloadTimeout = null; // Store timeout to prevent multiple concurrent reloads
          streamImage.onerror = function() {
              console.warn("Stream image error detected (onerror event).");
              if (errorReloadTimeout) {
                  // If already scheduled, don't schedule again immediately
                   console.log("Reload already scheduled, ignoring duplicate error event for now.");
                   return;
              }
              statusElement.textContent = 'Stream interrupted. Attempting reload...';
              errorReloadTimeout = setTimeout(() => {
                   console.log("Attempting stream reload now...");
                   // Add cache-busting query parameter to force reload
                   streamImage.src = "{{ url_for('video_feed') }}?" + new Date().getTime();
                   errorReloadTimeout = null; // Clear timeout ID after execution
                   // Update status again after attempting reload
                   setTimeout(updateStatus, 1000);
              }, 3000); // Wait 3 seconds before retry
          };

          // If the image loads successfully, ensure any pending error reload is cancelled
          streamImage.onload = function() {
              if (errorReloadTimeout) {
                  console.log("Stream image loaded successfully, cancelling pending error reload.");
                  clearTimeout(errorReloadTimeout);
                  errorReloadTimeout = null;
              }
              // Update status immediately on successful load if needed
              // updateStatus();
          };

        </script>
      </body>
    </html>
    """, resolution_text=resolution_text, current_w=current_w, current_h=current_h, err_msg=err_msg)

@app.route("/video_feed")
def video_feed():
    """Returns the MJPEG stream."""
    logging.info("Client connected to video feed.") # Log client connection
    # Create a new generator instance for each client connection
    return Response(generate_stream_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/status")
def status():
    """Returns the current status as JSON."""
    global last_error # Allow modification if we clear errors
    status_text = "Streaming"
    rec_stat = ""
    current_w, current_h = get_current_resolution() # Get current res

    if is_recording:
        # Check if writers still exist, update status if inconsistent
        if not video_writers:
            rec_stat = " (ERROR: Recording stopped unexpectedly - writers missing)"
            if not last_error: last_error = "Recording stopped: writers missing"
        else:
            rec_stat = f" (Recording to {len(recording_paths)} USBs)"
    status_text += rec_stat

    err_msg = last_error if last_error else ""

    # Simple logic to clear non-fatal errors once things look okay
    # Clear GPIO setup error if switch object exists later
    if switch is not None and last_error and "GPIO Setup Error" in last_error:
        logging.info("Clearing previous GPIO setup error as switch object exists.")
        last_error = None
        err_msg = ""
    # Clear camera init error if output frame exists
    if output_frame is not None and last_error and "Init Error" in last_error:
         logging.info("Clearing previous camera init error as frames are being processed.")
         last_error = None
         err_msg = ""

    return jsonify({
        'is_recording': is_recording,
        'resolution': f"{current_w}x{current_h}",
        'status_text': status_text,
        'error': err_msg,
        'active_recordings': recording_paths # Send list of recording paths
    })

@app.route("/set_resolution/<direction>", methods=['POST'])
def set_resolution(direction):
    """Endpoint to request resolution change."""
    global current_resolution_index, reconfigure_resolution_index, last_error

    with config_lock: # Use config lock to safely check/set reconfiguration flag
        if reconfigure_resolution_index is not None:
            logging.warning("Resolution change request ignored: Reconfiguration already in progress.")
            # Return 429 Too Many Requests
            return jsonify({'success': False, 'message': 'Reconfiguration already in progress.'}), 429

        # Check current index is valid before proceeding
        if not (0 <= current_resolution_index < len(SUPPORTED_RESOLUTIONS)):
             logging.error(f"Cannot change resolution: Current index {current_resolution_index} is invalid!")
             return jsonify({'success': False, 'message': 'Internal error: Invalid current resolution index.'}), 500

        original_index = current_resolution_index
        new_index = current_resolution_index

        if direction == 'up': new_index += 1
        elif direction == 'down': new_index -= 1
        else:
             logging.warning(f"Invalid direction received in set_resolution: {direction}")
             return jsonify({'success': False, 'message': 'Invalid direction specified.'}), 400 # Bad Request

        # Clamp index within bounds [0, len-1]
        new_index = max(0, min(len(SUPPORTED_RESOLUTIONS) - 1, new_index))

        if new_index == original_index:
            msg = 'Already at highest resolution.' if direction == 'up' else 'Already at lowest resolution.'
            logging.info(f"Resolution change request ignored: {msg}")
            return jsonify({'success': False, 'message': msg}), 400 # Bad Request

        new_w, new_h = SUPPORTED_RESOLUTIONS[new_index]
        logging.info(f"Web request received to change resolution index from {original_index} to {new_index} ({new_w}x{new_h})")

        # Set the flag for the capture thread to handle reconfiguration
        reconfigure_resolution_index = new_index
        last_error = None # Clear previous errors on user action initiating change

        return jsonify({'success': True, 'message': 'Resolution change requested.', 'new_resolution': f"{new_w}x{new_h}"})

# ===========================================================
# === ADDED FLASK ROUTES END HERE ===
# ===========================================================


# --- Main Execution (Adapt slightly for Picamera2) ---
def signal_handler(sig, frame):
    """Handles termination signals for graceful shutdown."""
    logging.warning(f"Received signal {sig}. Initiating graceful shutdown...")
    shutdown_event.set() # Signal all threads to stop

def main():
    """Main function to initialize and manage threads."""
    global last_error, capture_thread, flask_thread, picam2 # Reference globals

    # Setup signal handlers for SIGINT (Ctrl+C) and SIGTERM
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    logging.info(" --- Starting Camera Stream & Record Service (Picamera2 & gpiozero) --- ")

    # Main loop allows for recovery/restart on certain errors
    while not shutdown_event.is_set():
        last_error = None # Reset error at start of attempt
        capture_thread = None
        flask_thread = None
        # Ensure picam2 object is cleaned up from previous run if looping
        if picam2:
            try:
                if picam2.started: picam2.stop()
                picam2.close()
            except Exception as cleanup_e:
                 logging.warning(f"Exception during pre-loop cleanup of picam2: {cleanup_e}")
            picam2 = None

        try:
            logging.info("Initializing Hardware...")
            if not setup_gpio(): # Setup GPIO using gpiozero
                # Log the error but continue, switch functionality will be disabled
                 logging.error(f"GPIO setup failed: {last_error}. Switch control will be unavailable.")
                 # If switch is critical, could raise RuntimeError here to stop startup

            logging.info("Starting frame capture thread (Picamera2)...")
            capture_thread = threading.Thread(target=capture_and_process_loop, name="CaptureThread", daemon=True)
            capture_thread.start()

            # Wait a bit for camera thread to initialize. Crucial!
            time.sleep(4) # Increased wait time for camera init
            # Check if capture thread started and camera initialized successfully
            if not capture_thread.is_alive():
                 # If thread died immediately, likely camera init failed. last_error should be set.
                 raise RuntimeError(f"Capture thread failed to start or initialize camera (Picamera2): {last_error or 'Unknown reason - thread died'}")
            # Explicitly check for critical camera errors set during init
            if last_error and ("Picamera2 Init Error" in last_error or "Camera failed fatally" in last_error):
                 raise RuntimeError(f"Camera initialization failed within thread: {last_error}")
            logging.info("Capture thread appears to be running.")


            logging.info(f"Starting Flask web server on port {WEB_PORT}...")
            # Run Flask in a separate thread. Disable debug and reloader for stability.
            flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=WEB_PORT, debug=False, use_reloader=False, threaded=True), name="FlaskThread", daemon=True)
            flask_thread.start()

            time.sleep(1) # Short pause for Flask to start
            if not flask_thread.is_alive():
                raise RuntimeError("Flask web server thread failed to start.")


            logging.info("--- System Running ---")
            # Dynamically get IP? For now, rely on Flask's output or manual check.
            logging.info(f"Access stream at: http://<YOUR_PI_IP>:{WEB_PORT}")

            # Keep main thread alive, monitoring worker threads
            while not shutdown_event.is_set():
                if not capture_thread.is_alive():
                    if not last_error: last_error = "Capture thread terminated unexpectedly."
                    logging.error(last_error)
                    raise RuntimeError(last_error) # Trigger restart logic
                if not flask_thread.is_alive():
                    if not last_error: last_error = "Flask web server thread terminated unexpectedly."
                    logging.error(last_error)
                    raise RuntimeError(last_error) # Trigger restart logic

                # Check for fatal errors flagged by threads (optional, relies on last_error content)
                # if last_error and "fatal" in last_error.lower():
                #    logging.error(f"Detected fatal error: {last_error}. Triggering restart.")
                #    raise RuntimeError(f"Restarting due to fatal error: {last_error}")

                # Wait with a timeout, checking periodically
                shutdown_event.wait(timeout=5.0)
            # If shutdown_event is set, break the outer loop cleanly
            break

        except RuntimeError as e:
            # Catch errors specifically raised for restart purposes
            logging.error(f"!!! Runtime Error for Restart: {e}")
            logging.error("Attempting recovery/restart after 10 second pause...")
            # Signal threads to stop cleanly if they somehow are still running
            shutdown_event.set()
            # Wait for threads to exit (with timeout)
            if capture_thread and capture_thread.is_alive():
                 capture_thread.join(timeout=3.0)
            if flask_thread and flask_thread.is_alive():
                 # Flask thread might need a different shutdown mechanism if join hangs
                 logging.warning("Flask thread might not exit cleanly on join.")
                 flask_thread.join(timeout=1.0)

            # Reset shutdown event for the next loop iteration
            shutdown_event.clear()
            last_error = None # Clear error before retry
            time.sleep(10.0) # Pause before restarting the loop
        except Exception as e:
            # Catch any other unexpected exceptions in the main setup/monitoring part
            logging.exception(f"!!! Unhandled Exception in Main Loop: {e}")
            logging.error("Attempting restart after 10 seconds...")
            shutdown_event.set() # Signal threads
            if capture_thread and capture_thread.is_alive(): capture_thread.join(timeout=3.0)
            if flask_thread and flask_thread.is_alive(): flask_thread.join(timeout=1.0)
            shutdown_event.clear()
            last_error = None
            time.sleep(10.0)


    # --- Final Cleanup (when shutdown_event is set) ---
    logging.info("--- Shutdown initiated ---")

    # Threads should have been signaled by shutdown_event.set()
    # Wait for them to finish, but with timeouts
    if capture_thread and capture_thread.is_alive():
        logging.info("Waiting for capture thread to exit...")
        capture_thread.join(timeout=5.0)
        if capture_thread.is_alive():
             logging.warning("Capture thread did not exit cleanly.")

    # Flask thread shutdown can be tricky, often requires external signal/request
    # The daemon=True setting means it should exit when main thread exits, but cleanup is good
    if flask_thread and flask_thread.is_alive():
        logging.info("Attempting Flask thread cleanup (may not exit immediately).")
        # Sending a request to a shutdown route is cleaner if implemented
        # flask_thread.join(timeout=2.0) # Join might hang

    # Final stop recording call (should be redundant if threads exited cleanly)
    stop_recording()

    # Final camera release (should be redundant)
    if picam2:
        try:
            if picam2.started: picam2.stop()
            picam2.close()
            logging.info("Picamera2 released during final shutdown.")
        except Exception as e:
            logging.error(f"Error closing Picamera2 during final shutdown: {e}")

    # Cleanup GPIO
    cleanup_gpio()
    logging.info("--- Program Exit ---")


if __name__ == '__main__':
    main()