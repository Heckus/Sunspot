import cv2
import time
import datetime
import os
import threading
from flask import Flask, Response, render_template_string
import RPi.GPIO as GPIO
import logging

# --- Configuration (!! PLEASE EDIT THESE BASED ON YOUR SETUP !!) ---
CAMERA_DEVICE_INDEX = 0  # Try 0, 1, etc. Corresponds to /dev/videoX
FRAME_WIDTH = 1280       # Desired frame width
FRAME_HEIGHT = 720       # Desired frame height
FRAME_RATE = 15          # Desired frame rate

SWITCH_GPIO_PIN = 17      # GPIO pin (BCM numbering) for the switch
SWITCH_ON_STATE = GPIO.LOW # GPIO.LOW if connected to GND, GPIO.HIGH if connected to 3.3V
                           # Use GPIO.PUD_UP if connected to GND, GPIO.PUD_DOWN if connected to 3.3V
SWITCH_PULL_UP_DOWN = GPIO.PUD_UP

USB_BASE_PATH = "/media/pi/" # Base path where USB drives are mounted
RECORDING_FORMAT = "XVID"   # FourCC code for recording codec (e.g., 'XVID', 'MJPG', 'mp4v')
RECORDING_EXTENSION = ".avi" # File extension (e.g., '.avi', '.mp4')

WEB_PORT = 8000             # Port for the web stream
# --- End Configuration ---

# --- Global Variables ---
app = Flask(__name__)
video_capture = None
output_frame = None
lock = threading.Lock() # To safely share frames between threads
is_recording = False
video_writers = [] # List to hold VideoWriter objects for each USB
recording_paths = [] # List to hold paths for active recordings
last_error = None # Store the last critical error

# --- Logging Setup ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# --- Utility Functions ---
def setup_gpio():
    """Sets up the GPIO pin for the switch."""
    logging.info(f"Setting up GPIO pin {SWITCH_GPIO_PIN}")
    try:
        GPIO.setmode(GPIO.BCM)
        # Setup the pin with pull-up or pull-down resistor
        GPIO.setup(SWITCH_GPIO_PIN, GPIO.IN, pull_up_down=SWITCH_PULL_UP_DOWN)
        logging.info(f"GPIO pin {SWITCH_GPIO_PIN} setup complete.")
        return True
    except Exception as e:
        logging.error(f"!!! Failed to setup GPIO: {e}")
        global last_error
        last_error = f"GPIO Setup Error: {e}"
        return False

def cleanup_gpio():
    """Cleans up GPIO configuration."""
    logging.info("Cleaning up GPIO.")
    try:
        GPIO.cleanup()
    except Exception as e:
        logging.warning(f"Error during GPIO cleanup: {e}") # Non-critical

def initialize_camera():
    """Initializes the camera capture object."""
    global video_capture
    logging.info(f"Initializing camera device index {CAMERA_DEVICE_INDEX}...")
    try:
        cap = cv2.VideoCapture(CAMERA_DEVICE_INDEX) # Might need API preference like cv2.CAP_V4L2

        if not cap.isOpened():
            raise IOError(f"Cannot open camera device index {CAMERA_DEVICE_INDEX}")

        # Attempt to set camera properties
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, FRAME_RATE) # Note: Setting FPS might not always work

        # Verify settings
        actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        logging.info(f"Camera initialized. Requested: {FRAME_WIDTH}x{FRAME_HEIGHT} @ {FRAME_RATE}fps. Actual: {actual_width}x{actual_height} @ {actual_fps}fps")

        video_capture = cap
        return True

    except Exception as e:
        logging.error(f"!!! Failed to initialize camera: {e}")
        global last_error
        last_error = f"Camera Initialization Error: {e}"
        if video_capture:
            video_capture.release()
            video_capture = None
        return False

def get_usb_mounts():
    """Finds mounted directories under the USB base path."""
    mounts = []
    if not os.path.isdir(USB_BASE_PATH):
        logging.warning(f"USB base path '{USB_BASE_PATH}' does not exist.")
        return mounts
    try:
        for item in os.listdir(USB_BASE_PATH):
            path = os.path.join(USB_BASE_PATH, item)
            if os.path.isdir(path) and os.access(path, os.W_OK): # Check if it's a directory and writable
                mounts.append(path)
        if not mounts:
            logging.warning("No writable USB mounts found.")
        else:
            logging.info(f"Found writable USB mounts: {mounts}")
    except Exception as e:
        logging.error(f"Error finding USB mounts: {e}")
    return mounts

def start_recording():
    """Starts recording to all detected writable USB drives."""
    global is_recording, video_writers, recording_paths, last_error
    if is_recording:
        logging.warning("Recording already in progress.")
        return

    usb_drives = get_usb_mounts()
    if not usb_drives:
        logging.warning("Switch is ON, but no writable USB drives found to record to.")
        # Optional: Add user feedback here if desired, though it might spam
        return

    video_writers.clear()
    recording_paths.clear()
    success = False

    # Use the frame size reported by the camera
    if video_capture is None:
        logging.error("Cannot start recording, camera not available.")
        last_error = "Camera not available for recording."
        return

    try:
        width = int(video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = video_capture.get(cv2.CAP_PROP_FPS)
        if fps <= 0: # If camera doesn't report FPS, use configured one
            fps = FRAME_RATE
            logging.warning(f"Camera did not report FPS, using configured {fps}fps for recording.")

        fourcc = cv2.VideoWriter_fourcc(*RECORDING_FORMAT)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

        for drive_path in usb_drives:
            try:
                filename = f"recording_{timestamp}{RECORDING_EXTENSION}"
                full_path = os.path.join(drive_path, filename)
                writer = cv2.VideoWriter(full_path, fourcc, fps, (width, height))
                if not writer.isOpened():
                    logging.error(f"!!! Failed to open VideoWriter for: {full_path}")
                    continue # Try next drive

                video_writers.append(writer)
                recording_paths.append(full_path)
                logging.info(f"Started recording to: {full_path}")
                success = True
            except Exception as e:
                logging.error(f"!!! Failed to create VideoWriter for {drive_path}: {e}")
                last_error = f"Recording Start Error: {e}" # Store last significant error

        if success:
            is_recording = True
        elif not last_error: # No specific error, but failed to start any writer
             last_error = "Could not start recording on any USB drive."


    except Exception as e:
        logging.error(f"!!! Critical error during recording setup: {e}")
        last_error = f"Recording Setup Error: {e}"
        # Clean up any writers that might have been partially opened
        stop_recording()


def stop_recording():
    """Stops recording and releases video writer objects."""
    global is_recording, video_writers, recording_paths
    if not is_recording:
        return

    logging.info("Stopping recording...")
    is_recording = False # Set flag immediately
    released_count = 0
    for i, writer in enumerate(video_writers):
        try:
            writer.release()
            logging.info(f"Stopped recording to: {recording_paths[i]}")
            released_count += 1
        except Exception as e:
            logging.error(f"Error releasing VideoWriter for {recording_paths[i]}: {e}")

    video_writers.clear()
    recording_paths.clear()
    logging.info(f"Recording stopped. Released {released_count} writer(s).")

# --- Frame Generation and Processing ---
def capture_and_process_frames():
    """Main loop to capture frames, handle recording, and update shared frame."""
    global output_frame, is_recording, last_error

    logging.info("Starting frame capture loop...")

    last_switch_state = None

    while True:
        try:
            if video_capture is None or not video_capture.isOpened():
                raise RuntimeError("Camera is not initialized or has been closed.")

            # --- Read Frame ---
            ret, frame = video_capture.read()
            if not ret or frame is None:
                logging.warning("Failed to grab frame from camera. Retrying...")
                time.sleep(0.5) # Wait a bit before retrying
                continue

            # --- Handle Switch and Recording ---
            current_switch_state = GPIO.input(SWITCH_GPIO_PIN)

            # Debounce or state change detection
            if current_switch_state != last_switch_state:
                if current_switch_state == SWITCH_ON_STATE:
                    if not is_recording:
                        logging.info("Switch turned ON - attempting to start recording.")
                        start_recording()
                        # Check if start_recording set an error
                        if last_error and "Recording" in last_error:
                           raise RuntimeError(f"Failed to start recording: {last_error}")
                else: # Switch is OFF
                    if is_recording:
                        logging.info("Switch turned OFF - stopping recording.")
                        stop_recording()
                last_switch_state = current_switch_state
                time.sleep(0.1) # Simple debounce delay

            # --- Write Frame if Recording ---
            if is_recording:
                if not video_writers:
                     logging.warning("Recording flag is ON, but no active video writers. Trying to restart recording.")
                     # Attempt to restart recording if writers somehow disappeared
                     stop_recording() # Clear state
                     start_recording() # Try again
                     # If it failed again, it will be logged by start_recording

                for i, writer in enumerate(video_writers):
                    try:
                        writer.write(frame)
                    except Exception as e:
                        logging.error(f"!!! Failed to write frame to {recording_paths[i]}: {e}")
                        # Decide how to handle this: stop this writer, stop all, etc.
                        # For simplicity, log and continue trying for now.
                        # Could implement logic to stop/remove just the failing writer.
                        last_error = f"Frame Write Error: {e}" # Update last error

            # --- Update Shared Frame for Streaming ---
            with lock:
                output_frame = frame.copy()

        except KeyboardInterrupt:
            logging.info("Keyboard interrupt received. Exiting frame capture loop.")
            break
        except Exception as e:
            logging.error(f"!!! Error in frame capture loop: {e}")
            global last_error
            last_error = f"Capture Loop Error: {e}"
            # Critical error, stop the loop and let the main thread handle it
            break # Exit the loop on error

    # --- Cleanup after loop exit ---
    logging.info("Exiting frame capture thread.")
    if is_recording:
        stop_recording()
    if video_capture:
        video_capture.release()
        logging.info("Camera resource released.")

def generate_stream_frames():
    """Generator function for the MJPEG stream."""
    global output_frame, last_error
    while True:
        try:
            with lock:
                if output_frame is None:
                    # Wait briefly if the first frame isn't ready
                    time.sleep(0.1)
                    continue

                # Encode frame as JPEG
                (flag, encodedImage) = cv2.imencode(".jpg", output_frame)

                if not flag:
                    logging.warning("Could not encode frame to JPEG.")
                    time.sleep(0.1)
                    continue

            # Yield the frame in the correct format for MJPEG stream
            yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
                  bytearray(encodedImage) + b'\r\n')

        except GeneratorExit:
            logging.info("Streaming client disconnected.")
            break # Stop if client disconnects
        except Exception as e:
            logging.error(f"!!! Error in streaming generator: {e}")
            last_error = f"Streaming Error: {e}"
            # Don't break the generator usually, but log the error
            time.sleep(1) # Avoid spamming logs if error persists

# --- Flask Web Routes ---
@app.route("/")
def index():
    """Serves the main HTML page with the video stream."""
    # Simple HTML page
    return render_template_string("""
    <html>
      <head>
        <title>Pi Camera Stream</title>
      </head>
      <body>
        <h1>Pi Camera Stream</h1>
        <p>Current Status: <span id="status">Streaming</span></p>
        <img src="{{ url_for('video_feed') }}" width="{{ width }}" height="{{ height }}">
        <script>
            // Optional: Add JS to show recording status if needed
            // Could poll a status endpoint or use WebSockets
            // For now, just shows the stream.
        </script>
      </body>
    </html>
    """, width=FRAME_WIDTH, height=FRAME_HEIGHT) # Pass dimensions for img tag

@app.route("/video_feed")
def video_feed():
    """Returns the MJPEG stream."""
    # Uses the generator function
    return Response(generate_stream_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# --- Main Execution ---
def main():
    global last_error, video_capture

    while True: # Outer loop for retrying after critical errors
        last_error = None # Reset error status for this run
        capture_thread = None
        flask_thread = None

        try:
            # --- Initialize Hardware ---
            logging.info(" --- Initializing System ---")
            if not setup_gpio():
                 raise RuntimeError("GPIO setup failed.") # Error logged in function

            if not initialize_camera():
                raise RuntimeError("Camera initialization failed.") # Error logged in function

            # --- Start Background Threads ---
            logging.info("Starting frame capture thread...")
            capture_thread = threading.Thread(target=capture_and_process_frames, daemon=True)
            capture_thread.start()

            # Give the capture thread a moment to get the first frame
            time.sleep(2)
            with lock:
                if output_frame is None and last_error is None:
                    logging.warning("Capture thread started but no frame received yet.")
                    # Could add more robust check here or wait longer

            logging.info(f"Starting Flask web server on port {WEB_PORT}...")
            # Running Flask in a thread. For production, use a proper WSGI server like Gunicorn or Waitress.
            flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=WEB_PORT, debug=False, threaded=True), daemon=True)
            flask_thread.start()

            logging.info("--- System Running ---")
            logging.info(f"Access stream at: http://<YOUR_PI_IP>:{WEB_PORT}")

            # --- Monitor Threads and Errors ---
            while True:
                if not capture_thread.is_alive():
                    # If capture thread died, last_error should be set
                    if not last_error:
                        last_error = "Capture thread terminated unexpectedly."
                    raise RuntimeError(last_error)

                if not flask_thread.is_alive():
                    if not last_error:
                       last_error = "Flask web server thread terminated unexpectedly."
                    raise RuntimeError(last_error)

                # Check for errors set by threads (might be non-fatal to thread itself)
                if last_error and "Error" in last_error: # Check if a significant error occured
                     raise RuntimeError(f"Error detected: {last_error}")


                time.sleep(1) # Check periodically

        except RuntimeError as e:
            logging.error(f"!!! Runtime Error Encountered: {e}")
            # Stop recording if it was running
            if is_recording:
                stop_recording()

            # Wait for user input on how to proceed
            while True:
                action = input("A critical error occurred. Enter 'R' to restart the script, or 'Q' to quit: ").upper()
                if action == 'R':
                    logging.info("Attempting to restart script...")
                    # Release camera if it's still held
                    if video_capture and video_capture.isOpened():
                        video_capture.release()
                    # Cleanup GPIO before restarting setup
                    cleanup_gpio()
                    time.sleep(1) # Brief pause before retry
                    break # Break inner loop to retry outer loop
                elif action == 'Q':
                    logging.info("Exiting program upon user request.")
                    # Perform final cleanup
                    cleanup_gpio()
                    if video_capture and video_capture.isOpened():
                        video_capture.release()
                    return # Exit the main function
                else:
                    print("Invalid input. Please enter 'R' or 'Q'.")

        except KeyboardInterrupt:
            logging.info("Keyboard interrupt received. Shutting down.")
            # Let finally block handle cleanup
            break # Exit the outer loop

        finally:
            # --- Cleanup Actions ---
            logging.info("--- Initiating Cleanup ---")
            # Stop recording if necessary
            if is_recording:
                stop_recording()

            # Signal threads to stop if possible (though daemons might exit abruptly)
            # For cleaner shutdown, would need event flags passed to threads.

            # Release camera resource
            if video_capture and video_capture.isOpened():
                video_capture.release()
                logging.info("Camera resource released in finally block.")
            # Cleanup GPIO
            cleanup_gpio()
            logging.info("--- Cleanup Complete ---")


if __name__ == '__main__':
    main()