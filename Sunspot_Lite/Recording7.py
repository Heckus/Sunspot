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
from picamera2 import Picamera2
from libcamera import controls
import smbus
import math
import subprocess # For running external commands

# --- Configuration ---
SUPPORTED_RESOLUTIONS = [
    (640, 480),      # VGA
    (1280, 720),     # 720p HD
    (1640, 1232),    # Custom Pi Cam V2 mode
    (1920, 1080),    # 1080p FHD
    (3280, 2464)     # Max Native Resolution IMX219
]
DEFAULT_RESOLUTION_INDEX = 1
FRAME_RATE = 30
SWITCH_GPIO_PIN = 17            # Set to None to disable
SWITCH_BOUNCE_TIME = 0.1
USB_BASE_PATH = "/media/hecke/" # CHANGE this if your mount point base is different
RECORDING_FORMAT = "mp4v"
RECORDING_EXTENSION = ".mp4"
WEB_PORT = 8000
USE_NOIR_TUNING = True
NOIR_TUNING_FILE_PATH = "/usr/share/libcamera/ipa/rpi/pisp/imx219_noir.json"
INA219_I2C_ADDRESS = 0x41       # Set to None to disable
BATTERY_READ_INTERVAL = 30.0
BATTERY_MAX_VOLTAGE = 12.6      # Voltage when fully charged (e.g., 3S LiPo)
BATTERY_MIN_VOLTAGE = 9.0       # Voltage when empty (e.g., 3S LiPo cut-off)
# SHUTDOWN_WAIT_SECONDS = 15    # No longer needed here, handled by main thread wait
AP_PROFILE_NAME = "PiCamAP"     # NetworkManager profile name for the AP (used in main shutdown)

# --- Global Variables ---
app = Flask(__name__)
picam2 = None
output_frame = None
frame_lock = threading.Lock()
config_lock = threading.Lock()
capture_thread = None
flask_thread = None
shutdown_event = threading.Event()
reboot_requested = False # <-- NEW FLAG
switch = None
current_resolution_index = DEFAULT_RESOLUTION_INDEX
is_recording = False
video_writers = []
recording_paths = []
reconfigure_resolution_index = None
last_error = None
digital_recording_active = False
ina219_sensor = None
battery_percentage = None
last_battery_read_time = 0.0

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')

# ===========================================================
# === BATTERY MONITOR / GPIO / CAMERA / RECORDING CODE ===
# ===========================================================
# ... (Keep all these sections exactly the same as the previous version) ...
# INA219 Class...
# setup_gpio()...
# cleanup_gpio()...
# setup_battery_monitor()...
# read_battery_level()...
# get_current_resolution()...
# initialize_camera()...
# get_usb_mounts()...
# start_recording()...
# stop_recording()...
# capture_and_process_loop()...

# ===========================================================
# === FLASK ROUTES START HERE ===
# ===========================================================
def generate_stream_frames():
    # ... (Keep generate_stream_frames function exactly the same) ...
    global output_frame, frame_lock, shutdown_event
    frame_counter = 0
    last_frame_time = time.monotonic()
    logging.info("MJPEG stream client connected. Starting frame generation.")

    while not shutdown_event.is_set():
        frame_to_encode = None
        with frame_lock:
            if output_frame is not None:
                frame_to_encode = output_frame.copy()

        if frame_to_encode is None:
            time.sleep(0.05)
            continue

        try:
            (flag, encodedImage) = cv2.imencode(".jpg", frame_to_encode, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not flag:
                logging.warning("Stream generator: Could not encode frame to JPEG.")
                time.sleep(0.1)
                continue

            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
                   bytearray(encodedImage) + b'\r\n')
            frame_counter += 1
            current_time = time.monotonic()
            elapsed = current_time - last_frame_time
            target_delay = 1.0 / (FRAME_RATE + 5)
            sleep_time = max(0.01, target_delay - elapsed)
            time.sleep(sleep_time)
            last_frame_time = time.monotonic()

        except GeneratorExit:
            logging.info(f"Streaming client disconnected after {frame_counter} frames.")
            break
        except Exception as e:
            # Log other errors (e.g., network issues, encoding problems)
            logging.exception(f"!!! Error in MJPEG streaming generator: {e}")
            time.sleep(0.5) # Pause after an error before retrying

    logging.info("Stream generator thread exiting.")


# --- index() function with Jinja raw block fix ---
@app.route("/")
def index():
    # ... (Keep index function and HTML template exactly the same) ...
    global last_error, digital_recording_active, battery_percentage, config_lock
    current_w, current_h = get_current_resolution()
    resolution_text = f"{current_w}x{current_h}"
    err_msg = last_error if last_error else ""
    with config_lock:
        digital_rec_state_initial = digital_recording_active
        batt_perc_initial = battery_percentage
    batt_text_initial = f"{batt_perc_initial:.1f}" if batt_perc_initial is not None else "--"

    # Using triple quotes for the large HTML string
    # Added {% raw %} and {% endraw %} around the <style> block
    return render_template_string("""
    <!DOCTYPE html>
    <html>
      <head>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Pi Camera Stream & Record</title>
        {% raw %}
        <style>
          body { font-family: sans-serif; line-height: 1.4; margin: 1em; background-color: #f0f0f0;}
          .container { max-width: 960px; margin: auto; background: #fff; padding: 15px; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
          h1 { text-align: center; color: #333; margin-bottom: 20px; }
          .status-grid { display: grid; grid-template-columns: auto 1fr; gap: 5px 15px; margin-bottom: 15px; align-items: center; background-color: #eef; padding: 10px; border-radius: 5px;}
          .status-grid span:first-child { font-weight: bold; color: #555; text-align: right;}
          #status, #rec-status, #resolution, #battery-level { color: #0056b3; font-weight: normal;}
          #rec-status.active { color: #D83B01; font-weight: bold;}
          .controls { text-align: center; margin-bottom: 15px; display: flex; justify-content: center; align-items: center; flex-wrap: wrap; gap: 10px;} /* Flex layout for buttons */
          .controls button { padding: 10px 20px; margin: 5px; font-size: 1em; cursor: pointer; border-radius: 5px; border: 1px solid #ccc; background-color: #e9e9e9; transition: background-color 0.2s, border-color 0.2s;}
          .controls button:hover:not(:disabled) { background-color: #dcdcdc; border-color: #bbb;}
          #error { color: red; margin-top: 10px; white-space: pre-wrap; font-weight: bold; min-height: 1.2em; text-align: center; background-color: #ffebeb; border: 1px solid red; padding: 8px; border-radius: 4px; display: none; /* Initially hidden */}
          img#stream { display: block; margin: 15px auto; border: 1px solid black; max-width: 100%; height: auto; background-color: #ddd; } /* Placeholder color */
          button#btn-record.recording-active { background-color: #ff4d4d; color: white; border-color: #ff1a1a; }
          button#btn-record.recording-active:hover:not(:disabled) { background-color: #e60000; }
          button#btn-record.recording-inactive { background-color: #4CAF50; color: white; border-color: #367c39;}
          button#btn-record.recording-inactive:hover:not(:disabled) { background-color: #45a049; }
          button#btn-powerdown { background-color: #f44336; color: white; border-color: #d32f2f;} /* Style for Power Down button */
          button#btn-powerdown:hover:not(:disabled) { background-color: #c62828; }
          button:disabled { background-color: #cccccc !important; cursor: not-allowed !important; border-color: #999 !important; color: #666 !important;}
        </style>
        {% endraw %}
      </head>
      <body>
        <div class="container">
          <h1>Pi Camera Stream & Record</h1>
          <div class="status-grid">
            <span>Status:</span> <span id="status">Initializing...</span>
            <span>Recording:</span> <span id="rec-status">OFF</span>
            <span>Resolution:</span> <span id="resolution">{{ resolution_text }}</span>
            <span>Battery:</span> <span id="battery-level">{{ batt_text_initial }}%</span>
          </div>
          <div class="controls">
            <button onclick="changeResolution('down')" id="btn-down" title="Decrease resolution">&laquo; Lower Res</button>
            <button onclick="toggleRecording()" id="btn-record" class="recording-inactive" title="Toggle recording via web interface">Start Rec (Web)</button>
            <button onclick="changeResolution('up')" id="btn-up" title="Increase resolution">Higher Res &raquo;</button>
            <button onclick="powerDown()" id="btn-powerdown" title="Gracefully stop service and reboot Pi">Power Down</button> </div>
          <div id="error" {% if err_msg %}style="display: block;"{% endif %}>{{ err_msg }}</div> <img id="stream" src="{{ url_for('video_feed') }}" width="{{ current_w }}" height="{{ current_h }}" alt="Loading stream..."
               onerror="handleStreamError()" onload="handleStreamLoad()"> </div>

        <script>
          // ... (Keep all JavaScript functions exactly the same as previous version) ...
          // Including: updateRecordButtonState, updateStatus, disableControls, enableControls,
          // changeResolution, toggleRecording, powerDown, handleStreamError, handleStreamLoad,
          // Initialization, beforeunload listener.
          // The powerDown function will still make the fetch request, but the backend
          // will now return immediately after setting the flags.
          const statusElement = document.getElementById('status');
          const resolutionElement = document.getElementById('resolution');
          const errorElement = document.getElementById('error');
          const streamImage = document.getElementById('stream');
          const btnUp = document.getElementById('btn-up');
          const btnDown = document.getElementById('btn-down');
          const btnRecord = document.getElementById('btn-record');
          const btnPowerdown = document.getElementById('btn-powerdown');
          const recStatusElement = document.getElementById('rec-status');
          const batteryLevelElement = document.getElementById('battery-level');

          let isChangingResolution = false;
          let isTogglingRecording = false;
          let isPoweringDown = false;
          let currentDigitalRecordState = {{ 'true' if digital_rec_state_initial else 'false' }};
          let statusUpdateInterval;
          let streamErrorTimeout = null;

          function updateRecordButtonState() { /* ... same ... */
              if (currentDigitalRecordState) {
                  btnRecord.textContent = "Stop Rec (Web)";
                  btnRecord.classList.remove('recording-inactive');
                  btnRecord.classList.add('recording-active');
              } else {
                  btnRecord.textContent = "Start Rec (Web)";
                  btnRecord.classList.add('recording-inactive');
                  btnRecord.classList.remove('recording-active');
              }
          }
          function updateStatus() { /* ... same ... */
               if (isChangingResolution || isTogglingRecording || isPoweringDown) return;
               fetch('/status')
                  .then(response => { if (!response.ok) { throw new Error(`HTTP error! Status: ${response.status}`); } return response.json(); })
                  .then(data => {
                      statusElement.textContent = data.status_text || 'Unknown';
                      recStatusElement.textContent = data.is_recording ? "ACTIVE" : "OFF";
                      recStatusElement.classList.toggle('active', data.is_recording);
                      if (data.resolution && resolutionElement.textContent !== data.resolution) {
                          resolutionElement.textContent = data.resolution;
                          const [w, h] = data.resolution.split('x');
                           if (streamImage.getAttribute('width') != w || streamImage.getAttribute('height') != h) { streamImage.setAttribute('width', w); streamImage.setAttribute('height', h); }
                      }
                      if (data.error) { errorElement.textContent = data.error; errorElement.style.display = 'block'; }
                      else { if (errorElement.style.display !== 'none') { errorElement.textContent = ''; errorElement.style.display = 'none'; } }
                      if (typeof data.digital_recording_active === 'boolean' && currentDigitalRecordState !== data.digital_recording_active) { currentDigitalRecordState = data.digital_recording_active; updateRecordButtonState(); }
                      if (data.battery_percent !== null && data.battery_percent !== undefined) { batteryLevelElement.textContent = data.battery_percent.toFixed(1); }
                      else { batteryLevelElement.textContent = "--"; }
                  })
                  .catch(err => { console.error("Error fetching status:", err); statusElement.textContent = "Error fetching status"; errorElement.textContent = `Failed to fetch status: ${err.message}. Check server connection.`; errorElement.style.display = 'block'; recStatusElement.textContent = "Unknown"; batteryLevelElement.textContent = "Err"; });
          }
          function disableControls(poweringDown = false) { /* ... same ... */
              btnUp.disabled = true; btnDown.disabled = true; btnRecord.disabled = true; btnPowerdown.disabled = true;
              if(poweringDown) { document.body.style.opacity = '0.7'; }
          }
          function enableControls() { /* ... same ... */
              if (!isPoweringDown) { btnUp.disabled = false; btnDown.disabled = false; btnRecord.disabled = false; btnPowerdown.disabled = false; document.body.style.opacity = '1'; }
          }
          function changeResolution(direction) { /* ... same ... */
              if (isChangingResolution || isTogglingRecording || isPoweringDown) return;
              isChangingResolution = true; disableControls(); statusElement.textContent = 'Changing resolution... Please wait.'; errorElement.textContent = ''; errorElement.style.display = 'none';
              fetch(`/set_resolution/${direction}`, { method: 'POST' })
                  .then(response => response.json().then(data => ({ status: response.status, body: data })))
                  .then(({ status, body }) => {
                      if (status === 200 && body.success) { statusElement.textContent = 'Resolution change initiated. Stream will update.'; resolutionElement.textContent = body.new_resolution; const [w, h] = body.new_resolution.split('x'); streamImage.setAttribute('width', w); streamImage.setAttribute('height', h); console.log("Resolution change request sent..."); }
                      else { errorElement.textContent = `Error changing resolution: ${body.message || 'Unknown error.'}`; errorElement.style.display = 'block'; statusElement.textContent = 'Resolution change failed.'; isChangingResolution = false; enableControls(); updateStatus(); }
                  })
                  .catch(err => { console.error("Network error sending resolution change:", err); errorElement.textContent = `Network error changing resolution: ${err.message}`; errorElement.style.display = 'block'; statusElement.textContent = 'Resolution change failed (Network).'; isChangingResolution = false; enableControls(); updateStatus(); })
                  .finally(() => { if (isChangingResolution) { setTimeout(() => { if (isChangingResolution) { isChangingResolution = false; enableControls(); updateStatus(); } }, 7000); } });
          }
          function toggleRecording() { /* ... same ... */
              if (isChangingResolution || isTogglingRecording || isPoweringDown) return;
              isTogglingRecording = true; disableControls(); statusElement.textContent = 'Sending record command...'; errorElement.textContent = ''; errorElement.style.display = 'none';
              fetch('/toggle_recording', { method: 'POST' })
                  .then(response => { if (!response.ok) { throw new Error(`HTTP error! Status: ${response.status}`); } return response.json(); })
                  .then(data => { if (data.success) { currentDigitalRecordState = data.digital_recording_active; updateRecordButtonState(); statusElement.textContent = `Digital recording ${currentDigitalRecordState ? 'enabled' : 'disabled'}. State updating...`; setTimeout(updateStatus, 1500); } else { errorElement.textContent = `Error toggling recording: ${data.message || 'Unknown error.'}`; errorElement.style.display = 'block'; statusElement.textContent = 'Record command failed.'; setTimeout(updateStatus, 1000); } })
                  .catch(err => { console.error("Error toggling recording:", err); errorElement.textContent = `Network error toggling recording: ${err.message}`; errorElement.style.display = 'block'; statusElement.textContent = 'Command failed (Network).'; setTimeout(updateStatus, 1000); })
                  .finally(() => { isTogglingRecording = false; enableControls(); });
          }
          function powerDown() { /* ... same ... */
              if (isChangingResolution || isTogglingRecording || isPoweringDown) return;
              if (!confirm("Are you sure you want to power down the Raspberry Pi? This will stop the camera service and reboot.")) { return; }
              isPoweringDown = true; disableControls(true); statusElement.textContent = 'Powering down... Signalling service to stop.'; errorElement.textContent = ''; errorElement.style.display = 'none';
              if (statusUpdateInterval) clearInterval(statusUpdateInterval);
              fetch('/power_down', { method: 'POST' })
                  .then(response => { if (!response.ok) { return response.json().then(data => { throw new Error(data.message || `HTTP error! Status: ${response.status}`); }).catch(() => { throw new Error(`HTTP error! Status: ${response.status}`); }); } return response.json(); })
                  .then(data => { if (data.success) { statusElement.textContent = 'Shutdown initiated. Reboot will occur shortly.'; /* Backend handles reboot */ } else { errorElement.textContent = `Shutdown request failed: ${data.message || 'Unknown error.'}`; errorElement.style.display = 'block'; statusElement.textContent = 'Shutdown failed.'; isPoweringDown = false; enableControls(); } })
                  .catch(err => { console.error("Error sending power down command:", err); errorElement.textContent = `Error initiating shutdown: ${err.message}.`; errorElement.style.display = 'block'; statusElement.textContent = 'Shutdown error.'; isPoweringDown = false; enableControls(); });
          }
          function handleStreamError() { /* ... same ... */
              console.warn("Stream image 'onerror' event triggered."); if (streamErrorTimeout || isPoweringDown) return; statusElement.textContent = 'Stream interrupted. Attempting reload...'; streamErrorTimeout = setTimeout(() => { streamImage.src = "{{ url_for('video_feed') }}?" + new Date().getTime(); streamErrorTimeout = null; setTimeout(updateStatus, 1000); }, 3000);
          }
          function handleStreamLoad() { /* ... same ... */
              if (streamErrorTimeout) { clearTimeout(streamErrorTimeout); streamErrorTimeout = null; if (!isPoweringDown) statusElement.textContent = 'Stream active.'; }
          }
          document.addEventListener('DOMContentLoaded', () => { /* ... same ... */
              updateRecordButtonState(); updateStatus(); statusUpdateInterval = setInterval(() => { if (!isChangingResolution && !isTogglingRecording && !isPoweringDown) { updateStatus(); } }, 5000);
          });
          window.addEventListener('beforeunload', () => { /* ... same ... */
              if (statusUpdateInterval) clearInterval(statusUpdateInterval);
          });
        </script>
      </body>
    </html>
    """, resolution_text=resolution_text, current_w=current_w, current_h=current_h,
         err_msg=err_msg, digital_rec_state_initial=digital_rec_state_initial, batt_text_initial=batt_text_initial)


@app.route("/video_feed")
def video_feed():
    logging.info("Client connected to video feed.")
    return Response(generate_stream_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route("/status")
def status():
    # ... (Keep status function exactly the same) ...
    global last_error, digital_recording_active, is_recording, battery_percentage, config_lock
    global video_writers, recording_paths
    status_text = "Streaming"
    rec_stat_detail = ""
    current_w, current_h = get_current_resolution()
    batt_perc = None
    current_digital_state = False
    with config_lock:
        current_digital_state = digital_recording_active
        batt_perc = battery_percentage
        current_is_recording = is_recording
        current_recording_paths = list(recording_paths)
    if current_is_recording:
        if current_recording_paths: rec_stat_detail = f" (Recording to {len(current_recording_paths)} USB(s))"
        else: rec_stat_detail = " (ERROR: Recording active but no paths!)"; logging.warning("Status check found is_recording=True but recording_paths is empty."); last_error = last_error or "Inconsistent State: Recording active but no paths."
        status_text += rec_stat_detail
    err_msg = last_error if last_error else ""
    if output_frame is not None and err_msg and ("Init Error" in err_msg or "unavailable" in err_msg or "capture" in err_msg): logging.info("Auto-clearing previous camera/capture error..."); last_error = None; err_msg = ""
    if batt_perc is not None and err_msg and ("Battery Monitor" in err_msg or "INA219" in err_msg or "I2C Error" in err_msg): logging.info("Auto-clearing previous battery monitor error..."); last_error = None; err_msg = ""
    return jsonify({'is_recording': current_is_recording, 'digital_recording_active': current_digital_state, 'resolution': f"{current_w}x{current_h}", 'status_text': status_text, 'error': err_msg, 'active_recordings': current_recording_paths, 'battery_percent': batt_perc })


@app.route("/set_resolution/<direction>", methods=['POST'])
def set_resolution(direction):
    # ... (Keep set_resolution function exactly the same) ...
    global current_resolution_index, reconfigure_resolution_index, last_error, config_lock
    with config_lock:
        if reconfigure_resolution_index is not None: return jsonify({'success': False, 'message': 'Reconfiguration already in progress.'}), 429
        if not (0 <= current_resolution_index < len(SUPPORTED_RESOLUTIONS)): logging.error(f"Internal Error: Invalid current res index {current_resolution_index}!"); return jsonify({'success': False, 'message': 'Internal state error.'}), 500
        original_index = current_resolution_index; new_index = current_resolution_index
        if direction == 'up': new_index += 1
        elif direction == 'down': new_index -= 1
        else: return jsonify({'success': False, 'message': 'Invalid direction specified.'}), 400
        new_index = max(0, min(len(SUPPORTED_RESOLUTIONS) - 1, new_index))
        if new_index == original_index: msg = 'Already at highest resolution.' if direction == 'up' else 'Already at lowest resolution.'; return jsonify({'success': False, 'message': msg}), 400
        new_w, new_h = SUPPORTED_RESOLUTIONS[new_index]; logging.info(f"Web request: change res index {original_index} -> {new_index} ({new_w}x{new_h})"); reconfigure_resolution_index = new_index; last_error = None
        return jsonify({'success': True, 'message': 'Resolution change requested.', 'new_resolution': f"{new_w}x{new_h}"})

@app.route('/toggle_recording', methods=['POST'])
def toggle_recording():
    # ... (Keep toggle_recording function exactly the same) ...
    global digital_recording_active, last_error, config_lock
    new_state = False
    with config_lock:
        digital_recording_active = not digital_recording_active; new_state = digital_recording_active
        logging.info(f"Digital recording trigger toggled via web UI to: {'ON' if new_state else 'OFF'}")
        if last_error and ("Recording" in last_error or "writers" in last_error or "USB" in last_error or "sync" in last_error): logging.info(f"Clearing previous recording error: '{last_error}'"); last_error = None
    return jsonify({'success': True, 'digital_recording_active': new_state})


# --- MODIFIED Power Down Route (Option 1 - Signal Only) ---
@app.route('/power_down', methods=['POST'])
def power_down():
    """
    Handles the power down request. Sets flags to trigger shutdown
    and reboot from the main thread after cleanup.
    """
    global shutdown_event, reboot_requested, last_error

    logging.warning("Received request for power down via web UI. Setting flags.")
    last_error = "Shutdown initiated via web UI..." # Update status

    # Set flags for the main thread
    reboot_requested = True
    shutdown_event.set()

    # Return success immediately to the client
    return jsonify({'success': True, 'message': 'Shutdown initiated. System will reboot after cleanup.'})


# ===========================================================
# === FLASK ROUTES END HERE ===
# ===========================================================

# --- Signal Handling ---
def signal_handler(sig, frame):
    global shutdown_event
    if shutdown_event.is_set():
        logging.warning("Shutdown already in progress, ignoring additional signal.")
        return
    logging.warning(f"Received signal {sig}. Initiating graceful shutdown...")
    shutdown_event.set() # No reboot requested for external signals

# --- Main Execution (with Debug Logs and Final Commands) ---
def main():
    global last_error, capture_thread, flask_thread, picam2, shutdown_event, reboot_requested
    global AP_PROFILE_NAME # Access AP profile name for final commands

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    logging.info(" --- Starting Camera Stream & Record Service --- ")
    # ...(keep other initial logs)...
    logging.info(f"--- Power Down Method: Web UI sets flag, Main thread reboots ---") # Updated log
    logging.info(f"--- Power Down AP Profile Deactivation: '{AP_PROFILE_NAME}' (if set) ---")


    logging.info(f"MAIN: Initial shutdown_event state: {shutdown_event.is_set()}")
    logging.info(f"MAIN: Initial reboot_requested state: {reboot_requested}")

    run_counter = 0 # Counter for loop iterations

    # Main application loop (runs once unless restart logic is triggered by error)
    while not shutdown_event.is_set():
        run_counter += 1
        logging.info(f"MAIN: Entering main while loop (Iteration {run_counter})...")

        # Reset state variables for this run attempt
        last_error = None
        capture_thread = None
        flask_thread = None
        # DO NOT reset reboot_requested here, it persists until handled or script exits

        # Pre-loop cleanup for picam2
        logging.info(f"MAIN: Checking picam2 pre-loop cleanup block (picam2 is {'set' if picam2 else 'None'})...")
        if picam2:
            # ...(cleanup code as before)...
            logging.warning("MAIN: Found existing picam2 object - attempting cleanup.")
            try:
                if picam2.started: picam2.stop()
                picam2.close()
            except Exception as e: logging.error(f"MAIN: Error during picam2 pre-loop cleanup: {e}", exc_info=True)
            finally: picam2 = None; logging.info("MAIN: picam2 object cleared.")
        logging.info("MAIN: Finished picam2 pre-loop cleanup block.")

        # Main setup and execution block
        try:
            logging.info("MAIN: Entering main setup try block...")
            # Initialize Hardware
            logging.info("Initializing Hardware Components...")
            if SWITCH_GPIO_PIN is not None:
                if not setup_gpio(): logging.error(f"GPIO setup failed: {last_error}. Physical switch will be unavailable.")
            else: logging.info("Physical switch disabled.")
            if INA219_I2C_ADDRESS is not None:
                if not setup_battery_monitor(): logging.warning(f"Battery monitor setup failed: {last_error}. Battery level unavailable.")
            else: logging.info("Battery monitor disabled.")

            # Start Capture Thread
            logging.info("Starting frame capture thread...")
            capture_thread = threading.Thread(target=capture_and_process_loop, name="CaptureThread", daemon=True)
            capture_thread.start()
            time.sleep(4.0)
            if not capture_thread.is_alive(): raise RuntimeError(f"Capture thread failed startup: {last_error or 'Unknown'}")
            if last_error and ("Init Error" in last_error or "failed fatally" in last_error): raise RuntimeError(f"Camera init failed: {last_error}")
            logging.info("Capture thread appears running.")

            # Start Flask Thread
            logging.info(f"Starting Flask web server on 0.0.0.0:{WEB_PORT}...")
            flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=WEB_PORT, debug=False, use_reloader=False, threaded=True), name="FlaskThread", daemon=True)
            flask_thread.start()
            time.sleep(1.5)
            if not flask_thread.is_alive(): raise RuntimeError("Flask thread failed startup.")

            # System Running
            logging.info("--- System Running ---")
            try:
                # ...(IP address logging)...
                import socket; s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.connect(("8.8.8.8", 80)); local_ip = s.getsockname()[0]; s.close()
                logging.info(f"Access web interface at: http://{local_ip}:{WEB_PORT} (or Pi's IP/hostname)")
            except Exception as ip_e: logging.warning(f"Could not determine IP: {ip_e}"); logging.info(f"Access web interface at: http://<YOUR_PI_IP>:{WEB_PORT}")

            # Inner monitoring loop
            logging.info("MAIN: Entering inner monitoring loop...")
            while not shutdown_event.is_set():
                if not capture_thread.is_alive(): raise RuntimeError(last_error or "Capture thread terminated.")
                if not flask_thread.is_alive(): raise RuntimeError(last_error or "Flask thread terminated.")
                shutdown_event.wait(timeout=5.0)

            logging.info("MAIN: Exited inner monitoring loop (shutdown_event received).")
            # If shutdown_event was set, the outer loop condition will be false, exiting normally.
            # No 'break' needed here, loop condition handles it.

        # Exception handling for setup/runtime errors
        except RuntimeError as e:
            logging.error(f"!!! Runtime Error in Main Loop: {e}")
            logging.error("Attempting restart after 10 seconds...")
            shutdown_event.set() # Signal threads if possible
            if capture_thread and capture_thread.is_alive(): capture_thread.join(timeout=3.0)
            shutdown_event.clear(); reboot_requested = False # Reset flags for restart
            time.sleep(10.0)
            # Continue to next iteration of outer while loop for restart
        except Exception as e:
            logging.exception(f"!!! Unhandled Exception in Main Loop: {e}")
            logging.error("Attempting restart after 10 seconds...")
            shutdown_event.set();
            if capture_thread and capture_thread.is_alive(): capture_thread.join(timeout=3.0)
            shutdown_event.clear(); reboot_requested = False # Reset flags for restart
            time.sleep(10.0)
            # Continue to next iteration of outer while loop for restart


    # --- Final Cleanup (Only reached when shutdown_event is set) ---
    logging.info("--- Shutdown initiated ---")
    # shutdown_event is already set

    # Wait for capture thread (handles its own cleanup)
    if capture_thread and capture_thread.is_alive():
        logging.info("Waiting for capture thread to exit...")
        capture_thread.join(timeout=10.0) # Give it a reasonable time
        if capture_thread.is_alive(): logging.warning("Capture thread did not exit cleanly.")
        else: logging.info("Capture thread finished.")

    # Flask thread is daemon, will exit when main thread exits.

    # Ensure recording stopped / camera closed (belt-and-braces)
    if is_recording: logging.warning("Force stopping recording during final shutdown."); stop_recording()
    if picam2:
        try:
            logging.info("Ensuring Picamera2 closed...");
            if picam2.started: picam2.stop()
            picam2.close()
            logging.info("Picamera2 closed final.")
        except Exception as e: logging.warning(f"Error during final Picamera2 close: {e}")

    # Cleanup GPIO
    if SWITCH_GPIO_PIN is not None: cleanup_gpio()

    # --- Execute final commands if reboot was requested from web UI ---
    logging.info(f"MAIN: Checking if reboot was requested: {reboot_requested}")
    if reboot_requested:
        logging.warning("Reboot requested via web UI. Executing final commands...")

        # Step 1 (Optional): Deactivate AP
        if AP_PROFILE_NAME:
            # Requires sudoers: your_username ALL=(ALL) NOPASSWD: /usr/bin/nmcli connection down YourAPProfileName
            ap_deactivate_command = ["sudo", "/usr/bin/nmcli", "connection", "down", AP_PROFILE_NAME]
            logging.info(f"MAIN: Attempting final AP deactivation: {' '.join(ap_deactivate_command)}")
            try:
                result = subprocess.run(ap_deactivate_command, check=False, capture_output=True, text=True, timeout=5)
                if result.returncode == 0: logging.info("MAIN: AP profile deactivated.")
                else: logging.warning(f"MAIN: nmcli connection down failed (Code {result.returncode}): {result.stderr.strip()}")
            except Exception as e: logging.warning(f"MAIN: Error during final AP deactivation: {e}", exc_info=True)
        else:
            logging.info("MAIN: AP_PROFILE_NAME not set, skipping final AP deactivation.")

        # Step 2: Reboot
        # Requires sudoers: your_username ALL=(ALL) NOPASSWD: /sbin/reboot
        reboot_command = ["sudo", "/sbin/reboot"]
        logging.info(f"MAIN: Executing reboot command: {' '.join(reboot_command)}")
        try:
            subprocess.Popen(reboot_command) # Fire and forget
            logging.warning("MAIN: Reboot command issued. Exiting script.")
            # Give reboot command a moment to process before script fully exits
            time.sleep(2)
        except Exception as e:
            logging.critical(f"!!! MAIN: FAILED TO EXECUTE REBOOT COMMAND: {e}", exc_info=True)
            last_error = f"Final Reboot Failed: {e}" # Set error state just before exit

    logging.info("--- Program Exit ---")

if __name__ == '__main__':
    main()