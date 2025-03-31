import cv2
import time
import datetime
import os
import threading
import signal # For graceful shutdown
from flask import Flask, Response, render_template_string, jsonify, redirect, url_for
# --- Use gpiozero instead of RPi.GPIO ---
from gpiozero import Button, Device
from gpiozero.pins.native import NativeFactory # Recommended for Pi 5 backend selection
# --- End gpiozero import ---
import logging

# --- Configuration (!! PLEASE REVIEW AND EDIT !! ) ---

# !! CRITICAL: You MUST determine the correct camera index by testing !!
# !! Try 0, 1, 2... up to 35, until your PCIe camera feed appears. !!
CAMERA_DEVICE_INDEX = 0

# Supported resolutions (Width, Height) - MUST be supported by your camera
SUPPORTED_RESOLUTIONS = [
    (640, 480),
    (1280, 720),
    (1920, 1080),
    # Add more or remove resolutions your camera supports
]
DEFAULT_RESOLUTION_INDEX = len(SUPPORTED_RESOLUTIONS) - 1 # Start with the highest by default (1920x1080)
FRAME_RATE = 30          # Desired frame rate (camera might override)

# --- Switch Configuration (gpiozero) ---
# Connect switch between GPIO 17 and a GND pin.
SWITCH_GPIO_PIN = 17      # GPIO pin (BCM numbering)
# Button defaults to pull_up=True, use bounce_time for debouncing
SWITCH_BOUNCE_TIME = 0.1 # Debounce time in seconds (e.g., 100ms)
# --- End Switch Configuration ---

# !! CRITICAL: Verify this path matches where your USB drives auto-mount !!
USB_BASE_PATH = "/media/hecke/"

# Recording Format - Uses mp4v codec for MP4.
# !! May fail if codec not installed/supported by OpenCV backend !!
# !! If recording fails, try 'XVID' with '.avi' or install codecs (libavcodec-dev etc.) !!
RECORDING_FORMAT = "mp4v" # Use 'mp4v' for .mp4
RECORDING_EXTENSION = ".mp4" # File extension

WEB_PORT = 8000             # Port for the web stream
# --- End Configuration ---

# --- Set gpiozero Pin Factory (Optional but Recommended for Pi 5) ---
try:
     # Using NativeFactory attempts to select the best low-level library (like lgpio) automatically
     Device.pin_factory = NativeFactory()
     logging.info("Using gpiozero NativeFactory (recommended for Pi 5/modern systems)")
except Exception as e:
     logging.warning(f"Could not set NativeFactory for gpiozero, using default pin factory: {e}")
     # gpiozero will fall back to other available factories (like lgpio, RPi.GPIO)
# --- End Pin Factory Setting ---


# --- Global Variables ---
app = Flask(__name__)
video_capture = None
output_frame = None
frame_lock = threading.Lock() # Lock for safely accessing output_frame
config_lock = threading.Lock() # Lock for reconfiguration process
capture_thread = None
flask_thread = None
shutdown_event = threading.Event() # Used to signal threads to stop

# --- Add gpiozero switch object ---
switch = None
# --- End gpiozero switch object ---

# State variables
current_resolution_index = DEFAULT_RESOLUTION_INDEX
is_recording = False
video_writers = []
recording_paths = []
reconfigure_resolution_index = None # Target index for reconfiguration
last_error = None # Store the last critical error message

# --- Logging Setup ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# --- Utility Functions ---
def get_current_resolution():
    """Gets the resolution tuple based on the current index."""
    with config_lock: # Protect access to current_resolution_index
      if 0 <= current_resolution_index < len(SUPPORTED_RESOLUTIONS):
          return SUPPORTED_RESOLUTIONS[current_resolution_index]
      else:
          logging.warning(f"Invalid resolution index {current_resolution_index}, falling back to default.")
          return SUPPORTED_RESOLUTIONS[DEFAULT_RESOLUTION_INDEX]

# --- Modified setup_gpio for gpiozero ---
def setup_gpio():
    """Sets up the GPIO pin using gpiozero."""
    global switch, last_error
    logging.info(f"Setting up GPIO pin {SWITCH_GPIO_PIN} using gpiozero Button")
    try:
        # pull_up=True: Button assumes input is pulled high, reads LOW when pressed (connected to GND).
        # This matches the recommended wiring (Switch between GPIO and GND).
        # bounce_time handles debouncing automatically.
        switch = Button(SWITCH_GPIO_PIN, pull_up=True, bounce_time=SWITCH_BOUNCE_TIME)
        logging.info(f"gpiozero Button on pin {SWITCH_GPIO_PIN} setup complete (pull_up=True, bounce={SWITCH_BOUNCE_TIME}s).")
        return True
    except Exception as e:
        logging.error(f"!!! Failed to setup gpiozero Button: {e}")
        last_error = f"GPIO Setup Error (gpiozero): {e}"
        switch = None # Ensure switch is None on failure
        return False
# --- End setup_gpio modification ---

# --- Modified cleanup_gpio for gpiozero ---
def cleanup_gpio():
    """Cleans up GPIO resources using gpiozero."""
    global switch
    logging.info("Cleaning up GPIO (gpiozero).")
    try:
        if switch:
            switch.close() # Release GPIO resources held by the Button object
            switch = None
            logging.info("gpiozero Button closed.")
    except Exception as e:
        logging.warning(f"Error during gpiozero cleanup: {e}")
# --- End cleanup_gpio modification ---

def initialize_camera(target_width, target_height):
    """Initializes or re-initializes the camera capture object with specific resolution."""
    global video_capture, last_error
    logging.info(f"Attempting to initialize camera {CAMERA_DEVICE_INDEX} at {target_width}x{target_height}...")

    if video_capture is not None:
        try:
            video_capture.release()
            logging.info("Previous camera instance released.")
        except Exception as e:
             logging.warning(f"Error releasing previous camera instance: {e}")
        video_capture = None
        time.sleep(0.5)

    try:
        cap = cv2.VideoCapture(CAMERA_DEVICE_INDEX)

        if not cap.isOpened():
            raise IOError(f"Cannot open camera device index {CAMERA_DEVICE_INDEX}")

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, target_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target_height)
        cap.set(cv2.CAP_PROP_FPS, FRAME_RATE)

        time.sleep(0.5)
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)

        if actual_width != target_width or actual_height != target_height:
             logging.warning(f"Camera resolution mismatch. Requested {target_width}x{target_height}, Got {actual_width}x{actual_height}")

        logging.info(f"Camera initialized. Actual resolution: {actual_width}x{actual_height}, FPS: {actual_fps if actual_fps > 0 else 'N/A'}")

        video_capture = cap
        last_error = None
        return True

    except Exception as e:
        logging.error(f"!!! Failed to initialize camera at {target_width}x{target_height}: {e}")
        last_error = f"Camera Init Error ({target_width}x{target_height}): {e}"
        if 'cap' in locals() and cap is not None:
            cap.release()
        video_capture = None
        return False

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
    global is_recording, video_writers, recording_paths, last_error
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

    if video_capture is None or not video_capture.isOpened():
        logging.error("Cannot start recording, camera is not available.")
        last_error = "Camera not available for recording."
        return False

    try:
        width = int(video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = video_capture.get(cv2.CAP_PROP_FPS)
        if fps <= 0 or fps > 200:
            logging.warning(f"Camera reported invalid FPS ({fps}), using configured {FRAME_RATE}fps for recording.")
            fps = FRAME_RATE
        if width <= 0 or height <= 0:
             logging.error(f"Camera reported invalid dimensions ({width}x{height}). Cannot start recording.")
             last_error = "Invalid camera dimensions reported."
             return False

        fourcc = cv2.VideoWriter_fourcc(*RECORDING_FORMAT)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

        for drive_path in usb_drives:
            try:
                filename = f"recording_{timestamp}_{width}x{height}{RECORDING_EXTENSION}"
                full_path = os.path.join(drive_path, filename)
                writer = cv2.VideoWriter(full_path, fourcc, fps, (width, height))

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
            if start_error:
                last_error = f"Recording Start Failed: {start_error}"
            else:
                last_error = "Recording Start Failed: No writers could be opened."
            return False

    except Exception as e:
        logging.error(f"!!! Critical error during recording setup: {e}")
        last_error = f"Recording Setup Error: {e}"
        stop_recording()
        return False


def stop_recording():
    """Stops recording and releases video writer objects."""
    global is_recording, video_writers, recording_paths
    if not is_recording:
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


# --- Frame Capture and Processing Thread ---
def capture_and_process_loop():
    """Main loop executed in a background thread."""
    global output_frame, is_recording, last_error, video_capture, switch # Add switch here
    global current_resolution_index, reconfigure_resolution_index

    logging.info("Starting frame capture loop...")
    consecutive_error_count = 0
    max_consecutive_errors = 10

    width, height = get_current_resolution()
    if not initialize_camera(width, height):
        logging.error("Initial camera setup failed. Capture thread cannot start.")
        return

    while not shutdown_event.is_set():
        try:
            # --- Check for Reconfiguration Request ---
            if reconfigure_resolution_index is not None:
                with config_lock:
                    target_index = reconfigure_resolution_index
                    reconfigure_resolution_index = None

                logging.info(f"--- Received request to reconfigure resolution to index {target_index} ---")
                was_recording = is_recording
                if was_recording:
                    logging.info("Pausing recording for reconfiguration...")
                    stop_recording()

                new_width, new_height = SUPPORTED_RESOLUTIONS[target_index]
                if initialize_camera(new_width, new_height):
                    current_resolution_index = target_index
                    logging.info(f"--- Reconfiguration successful to {new_width}x{new_height} ---")
                    if was_recording:
                        logging.info("Resuming recording after reconfiguration...")
                        time.sleep(0.5)
                        if not start_recording():
                             logging.error("Failed to restart recording after reconfiguration!")
                else:
                    logging.error(f"!!! Failed to reconfigure camera to index {target_index}. Attempting to restore previous resolution... !!!")
                    prev_width, prev_height = SUPPORTED_RESOLUTIONS[current_resolution_index]
                    if not initialize_camera(prev_width, prev_height):
                         logging.critical("!!! Failed to restore previous camera resolution. Capture loop stopping. !!!")
                         last_error = "Camera failed fatally during reconfiguration."
                         shutdown_event.set()
                         break
                    if was_recording:
                         logging.info("Attempting to restart recording with restored resolution...")
                         time.sleep(0.5)
                         if not start_recording():
                             logging.error("Failed to restart recording after failed reconfiguration attempt.")


            # --- Check Camera Status ---
            if video_capture is None or not video_capture.isOpened():
                if not last_error: last_error = "Camera became unavailable unexpectedly."
                logging.error(f"Camera unavailable: {last_error}. Stopping capture loop.")
                shutdown_event.set()
                break

            # --- Read Frame ---
            ret, frame = video_capture.read()
            if not ret or frame is None:
                logging.warning("Failed to grab frame from camera. Retrying...")
                consecutive_error_count += 1
                if consecutive_error_count > max_consecutive_errors:
                    last_error = f"Failed to grab frame {max_consecutive_errors} consecutive times."
                    logging.error(last_error)
                    shutdown_event.set()
                    break
                time.sleep(0.5)
                continue
            consecutive_error_count = 0

            # --- Handle Switch and Recording (Using gpiozero) ---
            try:
                if switch is None:
                     # This might happen if setup failed but the thread started anyway
                     raise RuntimeError("GPIO Switch object (gpiozero) not initialized")

                # switch.is_pressed is True when Button pin is connected to GND (LOW)
                if switch.is_pressed: # Switch is ON (closed)
                    if not is_recording:
                        logging.info("Switch pressed (ON) - attempting to start recording.")
                        start_recording() # Attempt to start
                else: # Switch is OFF (open)
                    if is_recording:
                        logging.info("Switch released (OFF) - stopping recording.")
                        stop_recording() # Stop if recording

                # Debouncing is handled by gpiozero's bounce_time setting

            except Exception as e:
                 logging.error(f"Error reading gpiozero switch state: {e}")
                 # Continue running for now, but log the error

            # --- Write Frame if Recording ---
            if is_recording:
                if not video_writers:
                     logging.warning("Recording flag is ON, but no active video writers found. Attempting restart...")
                     stop_recording()
                     start_recording()

                write_errors = 0
                for i, writer in enumerate(video_writers):
                    try:
                        writer.write(frame)
                    except Exception as e:
                        logging.error(f"!!! Failed to write frame to {recording_paths[i]}: {e}")
                        write_errors += 1

                if write_errors > 0 and write_errors == len(video_writers):
                     logging.error("All video writers failed to write frame. Stopping recording.")
                     stop_recording()


            # --- Update Shared Frame for Streaming ---
            with frame_lock:
                output_frame = frame.copy()


        except Exception as e:
            logging.exception(f"!!! Unexpected Error in capture loop: {e}")
            last_error = f"Capture Loop Error: {e}"
            consecutive_error_count += 1
            if consecutive_error_count > max_consecutive_errors / 2:
                logging.error(f"Too many consecutive errors in capture loop. Signaling shutdown.")
                shutdown_event.set()
            time.sleep(1)

    # --- Cleanup after loop exit ---
    logging.info("Exiting frame capture thread.")
    if is_recording: stop_recording()
    if video_capture: video_capture.release()
    logging.info("Camera resource released by capture thread.")


# --- MJPEG Streaming Frame Generator ---
def generate_stream_frames():
    """Generator function for the MJPEG stream."""
    global output_frame
    frame_counter = 0
    while not shutdown_event.is_set():
        frame_to_encode = None
        with frame_lock:
            if output_frame is not None:
                frame_to_encode = output_frame.copy()

        if frame_to_encode is None:
            time.sleep(0.1)
            continue

        try:
            (flag, encodedImage) = cv2.imencode(".jpg", frame_to_encode)
            if not flag:
                logging.warning("Stream generator: Could not encode frame to JPEG.")
                time.sleep(0.1)
                continue

            yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
                  bytearray(encodedImage) + b'\r\n')
            frame_counter += 1

        except GeneratorExit:
            logging.info("Streaming client disconnected.")
            break
        except Exception as e:
            logging.error(f"!!! Error in streaming generator: {e}")
            time.sleep(0.5)

    logging.info("Stream generator thread exiting.")


# --- Flask Web Routes ---
@app.route("/")
def index():
    """Serves the main HTML page with controls."""
    current_w, current_h = get_current_resolution()
    resolution_text = f"{current_w}x{current_h}"
    return render_template_string("""
    <!DOCTYPE html>
    <html>
      <head>
        <title>Pi Camera Stream</title>
        <style>
            body { font-family: sans-serif; }
            #status { font-weight: bold; }
            .controls button { padding: 10px; margin: 5px; }
            #error { color: red; margin-top: 10px; white-space: pre-wrap; } /* Allow error wrap */
        </style>
      </head>
      <body>
        <h1>Pi Camera Stream</h1>
        <p>Status: <span id="status">Initializing...</span></p>
        <p>Current Resolution: <span id="resolution">{{ resolution_text }}</span></p>
        <div class="controls">
          <button onclick="changeResolution('down')">&laquo; Lower Res</button>
          <button onclick="changeResolution('up')">Higher Res &raquo;</button>
        </div>
        <div id="error"></div>
        <img id="stream" src="{{ url_for('video_feed') }}" width="{{ current_w }}" height="{{ current_h }}" alt="Loading stream...">

        <script>
            const statusElement = document.getElementById('status');
            const resolutionElement = document.getElementById('resolution');
            const errorElement = document.getElementById('error');
            const streamImage = document.getElementById('stream');

            function updateStatus() {
                fetch('/status')
                    .then(response => response.json())
                    .then(data => {
                        statusElement.textContent = data.status_text;
                        if (data.resolution) {
                             resolutionElement.textContent = data.resolution;
                             const [w, h] = data.resolution.split('x');
                             streamImage.width = w;
                             streamImage.height = h;
                        }
                         errorElement.textContent = data.error || '';
                    })
                    .catch(err => {
                        console.error("Error fetching status:", err);
                        statusElement.textContent = "Error fetching status";
                    });
            }

            function changeResolution(direction) {
                statusElement.textContent = 'Changing resolution...';
                errorElement.textContent = '';
                fetch(`/set_resolution/${direction}`, { method: 'POST' })
                    .then(response => response.json())
                    .then(data => {
                        if (data.success) {
                            statusElement.textContent = 'Resolution change initiated. Stream will reset.';
                             resolutionElement.textContent = data.new_resolution;
                              const [w, h] = data.new_resolution.split('x');
                              streamImage.width = w;
                              streamImage.height = h;
                            setTimeout(updateStatus, 2000);
                        } else {
                            errorElement.textContent = `Error: ${data.message}`;
                            updateStatus();
                        }
                    })
                    .catch(err => {
                        console.error("Error changing resolution:", err);
                        errorElement.textContent = 'Failed to send resolution change request.';
                        updateStatus();
                    });
            }

            setInterval(updateStatus, 5000);
            document.addEventListener('DOMContentLoaded', updateStatus);

             streamImage.onerror = function() {
                console.warn("Stream image error detected, attempting reload...");
                setTimeout(() => {
                    streamImage.src = "{{ url_for('video_feed') }}?" + new Date().getTime();
                }, 2000);
            };
        </script>
      </body>
    </html>
    """, resolution_text=resolution_text, current_w=current_w, current_h=current_h)

@app.route("/video_feed")
def video_feed():
    """Returns the MJPEG stream."""
    return Response(generate_stream_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/status")
def status():
    """Returns the current status as JSON."""
    global last_error
    status_text = "Streaming"
    rec_stat = ""
    if is_recording:
        rec_stat = f" (Recording to {len(recording_paths)} USBs)"
    status_text += rec_stat

    current_w, current_h = get_current_resolution()
    err_msg = last_error if last_error else ""
    # Optional: Clear non-critical errors after reporting them once
    # Example: Clear recording errors once recording stops/starts successfully
    # if not is_recording and last_error and "Recording" in last_error:
    #    last_error = None # Clear recording error once stopped

    return jsonify({
        'is_recording': is_recording,
        'resolution': f"{current_w}x{current_h}",
        'status_text': status_text,
        'error': err_msg,
        'active_recordings': recording_paths
    })

@app.route("/set_resolution/<direction>", methods=['POST'])
def set_resolution(direction):
    """Endpoint to request resolution change."""
    global current_resolution_index, reconfigure_resolution_index, last_error

    with config_lock:
        if reconfigure_resolution_index is not None:
             return jsonify({'success': False, 'message': 'Reconfiguration already in progress.'}), 429

        current_w, current_h = SUPPORTED_RESOLUTIONS[current_resolution_index]
        original_index = current_resolution_index
        new_index = current_resolution_index

        if direction == 'up': new_index += 1
        elif direction == 'down': new_index -= 1
        else: return jsonify({'success': False, 'message': 'Invalid direction.'}), 400

        new_index = max(0, min(len(SUPPORTED_RESOLUTIONS) - 1, new_index))

        if new_index == original_index:
            return jsonify({'success': False, 'message': 'Already at max/min resolution.'}), 400

        new_w, new_h = SUPPORTED_RESOLUTIONS[new_index]
        logging.info(f"Web request received to change resolution index from {original_index} to {new_index} ({new_w}x{new_h})")

        reconfigure_resolution_index = new_index
        last_error = None # Clear previous errors on user action

        return jsonify({'success': True, 'message': 'Resolution change requested.', 'new_resolution': f"{new_w}x{new_h}"})


# --- Main Execution and Graceful Shutdown ---
def signal_handler(sig, frame):
    """Handles termination signals for graceful shutdown."""
    logging.warning(f"Received signal {sig}. Initiating graceful shutdown...")
    shutdown_event.set()

def main():
    global last_error, capture_thread, flask_thread, video_capture

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    logging.info(" --- Starting Camera Stream & Record Service (gpiozero) --- ")

    while not shutdown_event.is_set():
        last_error = None
        capture_thread = None
        flask_thread = None

        try:
            logging.info("Initializing Hardware...")
            if not setup_gpio(): # Uses gpiozero now
                 raise RuntimeError(f"GPIO setup failed: {last_error}")

            logging.info("Starting frame capture thread...")
            capture_thread = threading.Thread(target=capture_and_process_loop, name="CaptureThread", daemon=True)
            capture_thread.start()

            time.sleep(3)
            if not capture_thread.is_alive() or (last_error and "Camera" in last_error): # Check specifically for camera errors too
                 raise RuntimeError(f"Capture thread failed to start or initialize camera: {last_error}")

            logging.info(f"Starting Flask web server on port {WEB_PORT}...")
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
                if last_error and "fatal" in last_error.lower():
                     raise RuntimeError(f"Restarting due to critical error: {last_error}")

                shutdown_event.wait(timeout=2.0)
            break # Exit outer loop if shutdown_event is set

        except RuntimeError as e:
            logging.error(f"!!! Runtime Error Encountered: {e}")
            logging.error("Attempting recovery/restart after pause...")
            if is_recording: stop_recording()
            if video_capture and video_capture.isOpened(): video_capture.release()
            # No need to cleanup GPIO here, it will be done in finally or on next setup attempt
            shutdown_event.wait(timeout=5.0) # Pause before retry


    # --- Final Cleanup ---
    logging.info("--- Shutdown initiated ---")
    if capture_thread and capture_thread.is_alive():
        logging.info("Waiting for capture thread to exit...")
        capture_thread.join(timeout=5.0)

    if is_recording:
        logging.warning("Forcing stop recording during final shutdown.")
        stop_recording()
    if video_capture and video_capture.isOpened():
        video_capture.release()
        logging.info("Camera released during final shutdown.")
    cleanup_gpio() # Use gpiozero cleanup
    logging.info("--- Program Exit ---")


if __name__ == '__main__':
    main()