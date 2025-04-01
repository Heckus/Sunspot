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
import subprocess

# --- Configuration ---
# ... (Keep all configurations EXCEPT TOGGLE_SCRIPT_PATH) ...
SUPPORTED_RESOLUTIONS = [
    (640, 480),      # VGA
    (1280, 720),     # 720p HD
    (1640, 1232),    # Custom Pi Cam V2 mode
    (1920, 1080),    # 1080p FHD
    (3280, 2464)     # Max Native Resolution IMX219
]
DEFAULT_RESOLUTION_INDEX = 1
FRAME_RATE = 30
SWITCH_GPIO_PIN = 17
SWITCH_BOUNCE_TIME = 0.1
USB_BASE_PATH = "/media/hecke/"
RECORDING_FORMAT = "mp4v"
RECORDING_EXTENSION = ".mp4"
WEB_PORT = 8000
USE_NOIR_TUNING = True
NOIR_TUNING_FILE_PATH = "/usr/share/libcamera/ipa/rpi/pisp/imx219_noir.json"
INA219_I2C_ADDRESS = 0x41
BATTERY_READ_INTERVAL = 30.0
BATTERY_MAX_VOLTAGE = 12.6
BATTERY_MIN_VOLTAGE = 9.0
SHUTDOWN_WAIT_SECONDS = 15 # Time (seconds) to wait after signaling stop before rebooting

# --- Global Variables ---
# ... (Keep globals the same) ...
app = Flask(__name__)
# ... rest of globals ...
picam2 = None
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
# ... (Keep generate_stream_frames function exactly the same) ...
def generate_stream_frames():
    # ... (same code) ...
    pass # Placeholder

# ... (Keep index function and HTML template exactly the same) ...
@app.route("/")
def index():
    # ... (same code) ...
    pass # Placeholder

# ... (Keep video_feed function exactly the same) ...
@app.route("/video_feed")
def video_feed():
    # ... (same code) ...
    pass # Placeholder

# ... (Keep status function exactly the same) ...
@app.route("/status")
def status():
    # ... (same code) ...
    pass # Placeholder

# ... (Keep set_resolution function exactly the same) ...
@app.route("/set_resolution/<direction>", methods=['POST'])
def set_resolution(direction):
     # ... (same code) ...
    pass # Placeholder

# ... (Keep toggle_recording function exactly the same) ...
@app.route('/toggle_recording', methods=['POST'])
def toggle_recording():
     # ... (same code) ...
    pass # Placeholder


# --- MODIFIED Power Down Route ---
@app.route('/power_down', methods=['POST'])
def power_down():
    """
    Handles the power down request. Dynamically finds toggle.sh.
    Assumes 'sudo ./toggle.sh disable' sends SIGINT/SIGTERM to this service.
    Waits for cleanup, then initiates reboot.
    Requires passwordless sudo configuration for the specific toggle.sh script and reboot.
    """
    global shutdown_event, last_error

    logging.warning("Received request for power down via web UI.")
    last_error = "Power down initiated..."

    # --- Step 0: Determine the path to toggle.sh dynamically ---
    try:
        # Get the directory where this Python script is located
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # Construct the full path to toggle.sh within that directory
        toggle_script_full_path = os.path.join(script_dir, "toggle.sh")
        logging.info(f"Dynamically determined toggle script path: {toggle_script_full_path}")

        # Check if the script actually exists before trying to run it
        if not os.path.isfile(toggle_script_full_path):
            logging.error(f"!!! POWER DOWN FAILED: Toggle script not found at determined path: {toggle_script_full_path}")
            last_error = "Power Down Failed: toggle.sh not found in script directory."
            return jsonify({'success': False, 'message': f"toggle.sh not found at {toggle_script_full_path}"}), 404 # Not Found

    except Exception as e:
        logging.error(f"!!! POWER DOWN FAILED: Could not determine toggle script path: {e}", exc_info=True)
        last_error = "Power Down Failed: Error finding toggle.sh path."
        return jsonify({'success': False, 'message': f"Error determining toggle script path: {e}"}), 500

    # --- Step 1: Signal the service to stop using toggle.sh ---
    # Use the dynamically determined absolute path
    # !! User MUST configure sudoers for this *specific absolute path* !!
    toggle_command = ["sudo", toggle_script_full_path, "disable"]
    logging.info(f"Executing command: {' '.join(toggle_command)}")
    try:
        result = subprocess.run(toggle_command, check=True, capture_output=True, text=True, timeout=10)
        logging.info(f"Toggle script ({toggle_script_full_path} disable) executed successfully.")
        logging.debug(f"Toggle script stdout:\n{result.stdout}")
        logging.debug(f"Toggle script stderr:\n{result.stderr}")

        # --- Step 2: Wait for Graceful Shutdown ---
        logging.warning(f"Waiting {SHUTDOWN_WAIT_SECONDS} seconds for service cleanup before reboot...")
        time.sleep(SHUTDOWN_WAIT_SECONDS)

        # --- Step 3: Initiate Reboot ---
        reboot_command = ["sudo", "/sbin/reboot"]
        logging.info(f"Executing command: {' '.join(reboot_command)}")
        try:
            subprocess.Popen(reboot_command)
            logging.warning("Reboot command issued.")
            return jsonify({'success': True, 'message': 'Reboot initiated.'})

        except FileNotFoundError:
            logging.critical("!!! FAILED TO EXECUTE REBOOT: '/sbin/reboot' command not found.")
            last_error = "Power Down Failed: Reboot command not found."
            return jsonify({'success': False, 'message': "Reboot command not found."}), 500
        except Exception as e:
            logging.critical(f"!!! FAILED TO EXECUTE REBOOT: {e}", exc_info=True)
            last_error = f"Power Down Failed: Error executing reboot: {e}"
            return jsonify({'success': False, 'message': f"Error executing reboot command: {e}"}), 500

    # Error handling specifically for the toggle script execution
    except FileNotFoundError:
        # This case should be caught by the earlier check, but keep for robustness
        logging.error(f"!!! FAILED TO EXECUTE TOGGLE SCRIPT: '{toggle_script_full_path}' command not found (should have been checked).")
        last_error = f"Power Down Failed: Sudo/toggle script '{os.path.basename(toggle_script_full_path)}' not found."
        return jsonify({'success': False, 'message': f"Toggle script or sudo not found at {toggle_script_full_path}."}), 500
    except subprocess.CalledProcessError as e:
        logging.error(f"!!! TOGGLE SCRIPT FAILED (Exit Code {e.returncode}): {toggle_script_full_path} disable")
        logging.error(f"Toggle script stdout:\n{e.stdout}")
        logging.error(f"Toggle script stderr:\n{e.stderr}")
        last_error = f"Power Down Failed: Toggle script returned error {e.returncode}."
        return jsonify({'success': False, 'message': f"Toggle script failed with code {e.returncode}. Check logs."}), 500
    except subprocess.TimeoutExpired:
        logging.error(f"!!! TIMEOUT executing toggle script: {toggle_script_full_path} disable")
        last_error = "Power Down Failed: Timeout executing toggle script."
        return jsonify({'success': False, 'message': "Timeout executing toggle script."}), 500
    except Exception as e:
        # Catch other potential errors like permission denied if sudoers isn't right
        logging.error(f"!!! FAILED TO EXECUTE TOGGLE SCRIPT: {e}", exc_info=True)
        last_error = f"Power Down Failed: Error executing toggle script: {e}"
        return jsonify({'success': False, 'message': f"Error executing toggle script: {e}. Check permissions/sudoers."}), 500


# ===========================================================
# === FLASK ROUTES END HERE ===
# ===========================================================

# --- Signal Handling ---
# ... (Keep signal_handler function exactly the same) ...
def signal_handler(sig, frame):
    # ... (same code) ...
    pass # Placeholder


# --- Main Execution ---
def main():
    global last_error, capture_thread, flask_thread, picam2, shutdown_event

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    logging.info(" --- Starting Camera Stream & Record Service --- ")
    # --- Remove or comment out the specific TOGGLE_SCRIPT_PATH log message ---
    # logging.info(f"--- Power Down uses toggle script: {TOGGLE_SCRIPT_PATH} ---")
    logging.info(f"--- Power Down will look for toggle.sh in script directory ---") # New log message
    logging.info(f"--- Power Down wait time: {SHUTDOWN_WAIT_SECONDS} seconds ---")

    # ... (Rest of the main function remains the same, handling startup and restart logic) ...

    while not shutdown_event.is_set():
        # ... (same setup/restart logic) ...
        pass # Placeholder for brevity

    # --- Final Cleanup ---
    # ... (Keep final cleanup logic the same) ...
    logging.info("--- Shutdown initiated ---")
    # ... (same cleanup code) ...
    logging.info("--- Program Exit ---")


if __name__ == '__main__':
    main()