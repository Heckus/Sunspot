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
# --- New Config: Path to the toggle script ---
TOGGLE_SCRIPT_PATH = "/home/hecke/Sunspot_Lite/toggle.sh" # <<< IMPORTANT: Set the correct full path to your toggle.sh

# --- Global Variables ---
app = Flask(__name__)
picam2 = None
output_frame = None
frame_lock = threading.Lock()
config_lock = threading.Lock()
capture_thread = None
flask_thread = None
shutdown_event = threading.Event()
# reboot_requested flag is NO LONGER USED for reboot logic
# reboot_requested = False
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
# === BATTERY MONITOR (INA219) CODE START ===
# ===========================================================
# Config Register (R/W)
_REG_CONFIG                     = 0x00
_REG_SHUNTVOLTAGE               = 0x01
_REG_BUSVOLTAGE                 = 0x02
_REG_POWER                      = 0x03
_REG_CURRENT                    = 0x04
_REG_CALIBRATION                = 0x05

class BusVoltageRange:
    RANGE_16V                   = 0x00
    RANGE_32V                   = 0x01

class Gain:
    DIV_1_40MV                  = 0x00
    DIV_2_80MV                  = 0x01
    DIV_4_160MV                 = 0x02
    DIV_8_320MV                 = 0x03

class ADCResolution:
    ADCRES_9BIT_1S              = 0x00
    ADCRES_10BIT_1S             = 0x01
    ADCRES_11BIT_1S             = 0x02
    ADCRES_12BIT_1S             = 0x03
    ADCRES_12BIT_2S             = 0x09
    ADCRES_12BIT_4S             = 0x0A
    ADCRES_12BIT_8S             = 0x0B
    ADCRES_12BIT_16S            = 0x0C
    ADCRES_12BIT_32S            = 0x0D
    ADCRES_12BIT_64S            = 0x0E
    ADCRES_12BIT_128S           = 0x0F

class Mode:
    POWERDOW                    = 0x00
    SVOLT_TRIGGERED             = 0x01
    BVOLT_TRIGGERED             = 0x02
    SANDBVOLT_TRIGGERED         = 0x03
    ADCOFF                      = 0x04
    SVOLT_CONTINUOUS            = 0x05
    BVOLT_CONTINUOUS            = 0x06
    SANDBVOLT_CONTINUOUS        = 0x07

class INA219:
    def __init__(self, i2c_bus=1, addr=0x40):
        try:
            self.bus = smbus.SMBus(i2c_bus)
            self.addr = addr
            self._cal_value = 0
            self._current_lsb = 0
            self._power_lsb = 0
            self.set_calibration_16V_5A()
            logging.info(f"INA219 sensor initialized at address 0x{addr:X} on bus {i2c_bus}")
        except FileNotFoundError:
            logging.error(f"!!! I2C bus {i2c_bus} not found. Check raspi-config.")
            raise
        except Exception as e:
            logging.error(f"!!! Failed to initialize INA219 sensor at 0x{addr:X}: {e}", exc_info=False)
            raise

    def read(self,address):
        data = self.bus.read_i2c_block_data(self.addr, address, 2)
        return ((data[0] * 256 ) + data[1])

    def write(self,address,data):
        temp = [0,0]
        temp[1] = data & 0xFF
        temp[0] =(data & 0xFF00) >> 8
        self.bus.write_i2c_block_data(self.addr,address,temp)

    def set_calibration_16V_5A(self):
        self._current_lsb = 0.0001524
        self._cal_value = 26868
        self._power_lsb = 0.003048
        self.write(_REG_CALIBRATION, self._cal_value)
        self.bus_voltage_range = BusVoltageRange.RANGE_16V
        self.gain = Gain.DIV_2_80MV
        self.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.mode = Mode.SANDBVOLT_CONTINUOUS
        config = self.bus_voltage_range << 13 | self.gain << 11 | self.bus_adc_resolution << 7 | self.shunt_adc_resolution << 3 | self.mode
        self.write(_REG_CONFIG, config)

    def set_calibration_32V_2A(self):
        # Keep this function for reference if needed
        self._current_lsb = 0.0001
        self._cal_value = 4096
        self._power_lsb = 0.002
        self.write(_REG_CALIBRATION, self._cal_value)
        self.bus_voltage_range = BusVoltageRange.RANGE_32V
        self.gain = Gain.DIV_8_320MV
        self.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.mode = Mode.SANDBVOLT_CONTINUOUS
        config = self.bus_voltage_range << 13 | self.gain << 11 | self.bus_adc_resolution << 7 | self.shunt_adc_resolution << 3 | self.mode
        self.write(_REG_CONFIG, config)
        logging.info("INA219 calibrated for 32V 2A range.")

    def getShuntVoltage_mV(self):
        value = self.read(_REG_SHUNTVOLTAGE)
        if value > 32767: value -= 65536
        return value * 0.01

    def getBusVoltage_V(self):
        self.read(_REG_BUSVOLTAGE) # Still include dummy read? Test without it.
        raw_val = self.read(_REG_BUSVOLTAGE)
        shifted_val = raw_val >> 3
        return shifted_val * 0.004

    def getCurrent_mA(self):
        value = self.read(_REG_CURRENT)
        if value > 32767: value -= 65536
        return value * self._current_lsb * 1000

    def getPower_W(self):
        value = self.read(_REG_POWER)
        if value > 32767: value -= 65536
        return value * self._power_lsb
# ===========================================================
# === END BATTERY MONITOR CODE ===
# ===========================================================

# --- Setup GPIO ---
def setup_gpio():
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

# --- Cleanup GPIO ---
def cleanup_gpio():
    global switch
    logging.info("Cleaning up GPIO (gpiozero).")
    try:
        if switch:
            switch.close()
            switch = None
            logging.info("gpiozero Button closed.")
    except Exception as e:
        logging.warning(f"Error during gpiozero cleanup: {e}")


# --- Setup Battery Monitor ---
def setup_battery_monitor():
    global ina219_sensor, last_error
    logging.info("Setting up Battery Monitor (INA219)...")
    try:
        ina219_sensor = INA219(addr=INA219_I2C_ADDRESS)
        voltage = ina219_sensor.getBusVoltage_V()
        logging.info(f"INA219 Sensor setup complete. Initial voltage reading: {voltage:.2f}V")
        read_battery_level()
        return True
    except Exception as e:
        logging.error(f"!!! Failed to setup INA219 Battery Monitor: {e}", exc_info=False)
        last_error = f"Battery Monitor Setup Error: {e}"
        ina219_sensor = None
        return False

# --- Read Battery Level ---
def read_battery_level():
    global battery_percentage, ina219_sensor, last_error, config_lock
    if ina219_sensor is None:
        return

    try:
        bus_voltage = ina219_sensor.getBusVoltage_V()
        voltage_range = BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE
        if voltage_range <= 0:
             percent = None
        else:
            percent = ((bus_voltage - BATTERY_MIN_VOLTAGE) / voltage_range) * 100.0
            percent = max(0.0, min(100.0, percent)) # Clamp 0-100

        with config_lock:
            battery_percentage = percent

    except OSError as e:
        logging.error(f"!!! I2C Error reading INA219 sensor: {e}")
        with config_lock:
            battery_percentage = None
    except Exception as e:
        logging.error(f"!!! Error reading battery level: {e}", exc_info=True)
        with config_lock:
            battery_percentage = None


# --- Get Current Resolution ---
def get_current_resolution():
    global current_resolution_index, config_lock
    with config_lock:
        if 0 <= current_resolution_index < len(SUPPORTED_RESOLUTIONS):
            return SUPPORTED_RESOLUTIONS[current_resolution_index]
        else:
            logging.warning(f"Invalid resolution index {current_resolution_index} detected, falling back to default.")
            safe_default_index = max(0, min(len(SUPPORTED_RESOLUTIONS) - 1, DEFAULT_RESOLUTION_INDEX))
            return SUPPORTED_RESOLUTIONS[safe_default_index]


# --- Initialize Camera ---
def initialize_camera(target_width, target_height):
    global picam2, last_error
    logging.info(f"Attempting to initialize camera with Picamera2 at {target_width}x{target_height}...")

    if picam2 is not None:
        try:
            if picam2.started:
                logging.info("Stopping existing Picamera2 instance...")
                picam2.stop()
            logging.info("Closing existing Picamera2 instance...")
            picam2.close()
            logging.info("Previous Picamera2 instance stopped and closed.")
        except Exception as e:
            logging.warning(f"Error stopping/closing previous Picamera2 instance: {e}")
        finally:
            picam2 = None
        time.sleep(0.5)

    try:
        tuning = None
        if USE_NOIR_TUNING:
            if os.path.exists(NOIR_TUNING_FILE_PATH):
                try:
                    tuning = Picamera2.load_tuning_file(NOIR_TUNING_FILE_PATH)
                    logging.info(f"Loading NoIR tuning from: {NOIR_TUNING_FILE_PATH}")
                except Exception as e:
                    logging.error(f"!!! Failed to load tuning file '{NOIR_TUNING_FILE_PATH}': {e}. Using default tuning.", exc_info=True)
                    tuning = None
            else:
                logging.warning(f"NoIR tuning file not found at {NOIR_TUNING_FILE_PATH}. Using default tuning.")
        else:
             logging.info("Using default tuning (NoIR tuning disabled).")

        picam2 = Picamera2(tuning=tuning)
        logging.info("Picamera2 object created.")

        config = picam2.create_video_configuration(
            main={"size": (target_width, target_height), "format": "RGB888"}, # Use RGB888 for capture_array
            controls={
                "FrameRate": float(FRAME_RATE), "AwbEnable": True, "AwbMode": controls.AwbModeEnum.Auto,
                "Brightness": 0.0, "Contrast": 1.0, "Saturation": 1.0,
            }
        )
        logging.info(f"Configuring Picamera2 with: {config}")
        picam2.configure(config)
        logging.info("Configuration successful!")

        picam2.start()
        logging.info("Camera started")
        time.sleep(2.0) # Allow AWB/AE to settle

        actual_config = picam2.camera_configuration()
        if not actual_config: raise RuntimeError("Failed to get camera config after start.")
        actual_format = actual_config.get('main', {})
        actual_w, actual_h = actual_format.get('size', (0,0))
        actual_fmt_str = actual_format.get('format', 'Unknown')
        logging.info(f"Picamera2 initialized. Actual main stream: {actual_w}x{actual_h}, Format: {actual_fmt_str}")
        last_error = None
        return True

    except Exception as e:
        logging.error(f"!!! Failed to initialize Picamera2: {e}", exc_info=True)
        last_error = f"Picamera2 Init Error ({target_width}x{target_height}): {e}"
        if picam2:
            try: picam2.close() # Try close even on failure
            except Exception: pass
        picam2 = None
        return False


# --- USB Mounts and Recording ---
def get_usb_mounts():
    mounts = []
    logging.debug(f"Checking for USB mounts under: {USB_BASE_PATH}")
    if not os.path.isdir(USB_BASE_PATH):
        logging.warning(f"USB base path '{USB_BASE_PATH}' does not exist or is not a directory.")
        return mounts

    try:
        for item in os.listdir(USB_BASE_PATH):
            path = os.path.join(USB_BASE_PATH, item)
            if os.path.isdir(path) and os.access(path, os.W_OK):
                test_file = os.path.join(path, f".write_test_{os.getpid()}")
                try:
                    with open(test_file, 'w') as f: f.write('test')
                    os.remove(test_file)
                    mounts.append(path)
                    logging.debug(f"Found writable mount: {path}")
                except Exception as write_err:
                     logging.warning(f"Directory {path} appears mounted but test write failed: {write_err}")
            elif os.path.isdir(path):
                 logging.debug(f"Directory {path} found but is not writable.")
        if not mounts: logging.debug("No writable USB mounts found.")
    except Exception as e:
        logging.error(f"Error finding USB mounts in {USB_BASE_PATH}: {e}")
    return mounts

def start_recording():
    global is_recording, video_writers, recording_paths, last_error, picam2
    if is_recording:
        logging.warning("Start recording called, but already recording.")
        return True
    logging.info("Attempting to start recording...")
    usb_drives = get_usb_mounts()
    if not usb_drives:
        logging.warning(f"Cannot start recording: No writable USB drives found in {USB_BASE_PATH}.")
        last_error = f"Cannot start recording: No writable USB drives found"
        return False
    video_writers.clear(); recording_paths.clear()
    success_count = 0; start_error = None
    if picam2 is None or not picam2.started:
        logging.error("Cannot start recording, camera is not available."); last_error = "Camera not available for recording."
        return False
    try:
        cam_config = picam2.camera_configuration().get('main', {})
        width, height = cam_config.get('size', (0,0))
        fps = float(FRAME_RATE)
        if width >= 3280 and fps > 15:
            logging.warning(f"High resolution ({width}x{height}). Capping recording FPS to 15.")
            fps = 15.0
        if width <= 0 or height <= 0: raise ValueError(f"Invalid dimensions {width}x{height}")
        logging.info(f"Starting recording: {width}x{height} @ {fps:.1f}fps")
        fourcc = cv2.VideoWriter_fourcc(*RECORDING_FORMAT)
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        for drive_path in usb_drives:
            try:
                filename = f"recording_{timestamp}_{width}x{height}{RECORDING_EXTENSION}"
                full_path = os.path.join(drive_path, filename)
                writer = cv2.VideoWriter(full_path, fourcc, fps, (width, height))
                if not writer.isOpened(): raise IOError(f"Failed open writer: {full_path}")
                video_writers.append(writer); recording_paths.append(full_path)
                logging.info(f"Successfully recording to: {full_path}")
                success_count += 1
            except Exception as e:
                logging.error(f"!!! Failed create writer for {drive_path}: {e}", exc_info=True)
                if start_error is None: start_error = f"Failed writer {os.path.basename(drive_path)}: {e}"
        if success_count > 0:
            is_recording = True; logging.info(f"Recording started on {success_count} drive(s).")
            last_error = f"Partial Rec Start Fail: {start_error}" if start_error and success_count < len(usb_drives) else None
            return True
        else:
            is_recording = False; logging.error("Failed start recording on ANY drive.")
            last_error = f"Rec Start Failed: {start_error or 'No writers opened'}"
            video_writers.clear(); recording_paths.clear()
            return False
    except Exception as e:
        logging.error(f"!!! Critical error during recording setup: {e}", exc_info=True)
        last_error = f"Recording Setup Error: {e}"; stop_recording()
        return False

def stop_recording():
    global is_recording, video_writers, recording_paths, last_error
    if not is_recording:
        if video_writers or recording_paths:
            logging.warning("stop_recording called but not recording; clearing potentially stale writer lists.")
            video_writers.clear(); recording_paths.clear()
        return
    logging.info("Stopping recording...")
    is_recording = False; released_count = 0
    writers_to_release = list(video_writers); paths_recorded = list(recording_paths)
    video_writers.clear(); recording_paths.clear()
    logging.info(f"Releasing {len(writers_to_release)} video writer(s)...")
    for i, writer in enumerate(writers_to_release):
        path = paths_recorded[i] if i < len(paths_recorded) else f"Unknown Path (Writer {i})"
        try:
            writer.release(); logging.info(f"Successfully released writer for: {path}"); released_count += 1
        except Exception as e: logging.error(f"Error releasing writer for {path}: {e}", exc_info=True)
    if released_count > 0:
        logging.info("Syncing filesystem to ensure data is written to USB drives...")
        try:
            sync_start_time = time.monotonic(); os.system('sync'); sync_duration = time.monotonic() - sync_start_time
            logging.info(f"Filesystem sync completed in {sync_duration:.2f} seconds.")
        except Exception as e:
            logging.error(f"!!! Failed to execute 'sync': {e}", exc_info=True); last_error = "Filesystem sync failed."
    else: logging.warning("No writers released, skipping sync.")
    logging.info(f"Recording stopped. Released {released_count}/{len(writers_to_release)} writers.")


# --- Capture Loop ---
def capture_and_process_loop():
    global output_frame, is_recording, last_error, picam2, switch, digital_recording_active
    global current_resolution_index, reconfigure_resolution_index, last_battery_read_time
    global video_writers, recording_paths

    logging.info("Starting frame capture loop...")
    consecutive_error_count = 0; max_consecutive_errors = 15

    width, height = get_current_resolution()
    if not initialize_camera(width, height):
        logging.error("Initial camera setup failed. Capture thread cannot start."); return

    while not shutdown_event.is_set():
        loop_start_time = time.monotonic()
        try:
            target_index = -1
            if reconfigure_resolution_index is not None:
                with config_lock:
                    if reconfigure_resolution_index is not None:
                        target_index = reconfigure_resolution_index; reconfigure_resolution_index = None
            if target_index != -1:
                logging.info(f"--- Reconfiguring resolution to index {target_index} ---")
                physical_switch_on_before = (switch is not None and switch.is_pressed)
                digital_switch_on_before = digital_recording_active
                should_be_recording_after = physical_switch_on_before or digital_switch_on_before
                was_actually_recording = is_recording
                if was_actually_recording: stop_recording()
                new_width, new_height = SUPPORTED_RESOLUTIONS[target_index]
                if initialize_camera(new_width, new_height):
                    with config_lock: current_resolution_index = target_index
                    logging.info(f"--- Reconfig successful to {new_width}x{new_height} ---")
                    if should_be_recording_after:
                        logging.info("Resuming recording..."); time.sleep(1.0)
                        if not start_recording(): logging.error("Failed restart recording after reconfig!")
                else:
                    logging.error(f"!!! Failed reconfigure index {target_index}. Restoring previous... !!!")
                    prev_width, prev_height = get_current_resolution()
                    if not initialize_camera(prev_width, prev_height):
                        logging.critical("!!! Failed restore previous camera. Stopping service. !!!")
                        last_error = "Camera failed fatally during reconfig restore."; shutdown_event.set(); break
                    else:
                        logging.info("Successfully restored previous camera resolution.")
                        if should_be_recording_after:
                            logging.info("Attempting recording restart with restored resolution...")
                            time.sleep(1.0)
                            if not start_recording(): logging.error("Failed restart recording after failed reconfig.")
                continue

            if picam2 is None or not picam2.started:
                if not last_error: last_error = "Picamera2 became unavailable."
                logging.error(f"Camera unavailable: {last_error}. Attempting reinit...")
                width, height = get_current_resolution()
                if initialize_camera(width, height):
                     logging.info("Camera re-initialized successfully."); last_error = None; consecutive_error_count = 0
                else:
                    logging.error(f"Camera re-init failed: {last_error}. Stopping loop."); shutdown_event.set(); break
                continue

            # Capture frame - expecting RGB888 from config
            frame_rgb = picam2.capture_array("main")
            if frame_rgb is None:
                logging.warning("Failed capture frame. Retrying..."); consecutive_error_count += 1
                if consecutive_error_count > max_consecutive_errors:
                    last_error = f"Failed capture {max_consecutive_errors} times."; logging.error(last_error); shutdown_event.set(); break
                time.sleep(0.1); continue
            if consecutive_error_count > 0: logging.info(f"Recovered frame grab after {consecutive_error_count} errors.")
            consecutive_error_count = 0

            # Convert RGB to BGR for OpenCV writer and MJPEG stream
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

            physical_switch_on = False
            if switch is not None:
                try: physical_switch_on = switch.is_pressed
                except Exception as e: logging.error(f"Error reading switch: {e}"); last_error = f"Switch Read Error: {e}"
            with config_lock: digital_switch_on = digital_recording_active
            should_be_recording = physical_switch_on or digital_switch_on

            if should_be_recording and not is_recording:
                log_msg = "Physical ON" if physical_switch_on else ""; log_msg += (" / " if log_msg and digital_switch_on else "") + ("Digital ON" if digital_switch_on else "")
                logging.info(f"Recording trigger active ({log_msg}) - starting.")
                if not start_recording(): logging.error("Attempt start recording failed.")
            elif not should_be_recording and is_recording:
                logging.info("Recording trigger(s) OFF - stopping."); stop_recording()

            if is_recording:
                if not video_writers:
                    logging.warning("Inconsistent: is_recording=True, no writers. Forcing stop."); is_recording = False; last_error = "Rec stopped: writer list empty."
                else:
                    write_errors = 0; current_writers = list(video_writers); current_paths = list(recording_paths)
                    for i, writer in enumerate(current_writers):
                        try: writer.write(frame_bgr) # Write BGR frame
                        except Exception as e:
                             path_str = current_paths[i] if i < len(current_paths) else f"Writer {i}"
                             logging.error(f"!!! Failed write frame {path_str}: {e}"); write_errors += 1; last_error = f"Frame write error: {os.path.basename(path_str)}"
                    if write_errors > 0 and write_errors == len(current_writers):
                        logging.error("All writers failed write. Stopping recording."); last_error = "Rec stopped: All writers failed write."; stop_recording()

            with frame_lock:
                output_frame = frame_bgr.copy() # Share BGR frame

            if ina219_sensor and (loop_start_time - last_battery_read_time > BATTERY_READ_INTERVAL):
                 read_battery_level(); last_battery_read_time = loop_start_time
        except Exception as e:
            logging.exception(f"!!! Unexpected Error in capture loop: {e}")
            last_error = f"Capture Loop Error: {e}"; consecutive_error_count += 1
            if consecutive_error_count > max_consecutive_errors / 2:
                logging.error(f"Too many errors ({consecutive_error_count}). Signaling shutdown."); shutdown_event.set()
            time.sleep(1)
    logging.info("Exiting frame capture thread.");
    if is_recording: logging.info("Capture loop exiting: Stopping active recording..."); stop_recording()
    if picam2:
        try: logging.info("Capture loop exiting: Closing Picamera2..."); picam2.close()
        except Exception as e: logging.error(f"Error closing Picamera2 in thread cleanup: {e}")
    picam2 = None


# ===========================================================
# === FLASK ROUTES START HERE ===
# ===========================================================
# --- generate_stream_frames ---
def generate_stream_frames():
    global output_frame, frame_lock, shutdown_event
    frame_counter = 0; last_frame_time = time.monotonic()
    logging.info("MJPEG stream client connected. Starting frame generation.")
    while not shutdown_event.is_set():
        frame_to_encode = None
        with frame_lock:
            if output_frame is not None: frame_to_encode = output_frame.copy()
        if frame_to_encode is None: time.sleep(0.05); continue
        try:
            (flag, encodedImage) = cv2.imencode(".jpg", frame_to_encode, [cv2.IMWRITE_JPEG_QUALITY, 85]) # Encode BGR frame
            if not flag: logging.warning("Stream: Could not encode frame."); time.sleep(0.1); continue
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage) + b'\r\n')
            frame_counter += 1; current_time = time.monotonic(); elapsed = current_time - last_frame_time
            target_delay = 1.0 / (FRAME_RATE + 5); sleep_time = max(0.01, target_delay - elapsed); time.sleep(sleep_time)
            last_frame_time = time.monotonic()
        except GeneratorExit: logging.info(f"Streaming client disconnected after {frame_counter} frames."); break
        except Exception as e: logging.exception(f"!!! Error in MJPEG stream: {e}"); time.sleep(0.5)
    logging.info("Stream generator thread exiting.")

# --- index() HTML Route ---
@app.route("/")
def index():
    global last_error, digital_recording_active, battery_percentage, config_lock
    current_w, current_h = get_current_resolution()
    resolution_text = f"{current_w}x{current_h}"
    err_msg = last_error if last_error else ""
    with config_lock:
        digital_rec_state_initial = digital_recording_active
        batt_perc_initial = battery_percentage
    batt_text_initial = f"{batt_perc_initial:.1f}" if batt_perc_initial is not None else "--"
    # Using triple quotes and raw block for HTML/CSS
    return render_template_string("""
    <!DOCTYPE html>
    <html>
      <head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1.0"><title>Pi Camera Stream & Record</title>{% raw %}<style>body{font-family:sans-serif;line-height:1.4;margin:1em;background-color:#f0f0f0}.container{max-width:960px;margin:auto;background:#fff;padding:15px;border-radius:8px;box-shadow:0 2px 5px rgba(0,0,0,.1)}h1{text-align:center;color:#333;margin-bottom:20px}.status-grid{display:grid;grid-template-columns:auto 1fr;gap:5px 15px;margin-bottom:15px;align-items:center;background-color:#eef;padding:10px;border-radius:5px}.status-grid span:first-child{font-weight:700;color:#555;text-align:right}#status,#rec-status,#resolution,#battery-level{color:#0056b3;font-weight:400}#rec-status.active{color:#d83b01;font-weight:700}.controls{text-align:center;margin-bottom:15px;display:flex;justify-content:center;align-items:center;flex-wrap:wrap;gap:10px}.controls button{padding:10px 20px;margin:5px;font-size:1em;cursor:pointer;border-radius:5px;border:1px solid #ccc;background-color:#e9e9e9;transition:background-color .2s,border-color .2s}.controls button:hover:not(:disabled){background-color:#dcdcdc;border-color:#bbb}#error{color:red;margin-top:10px;white-space:pre-wrap;font-weight:700;min-height:1.2em;text-align:center;background-color:#ffebeb;border:1px solid red;padding:8px;border-radius:4px;display:none}img#stream{display:block;margin:15px auto;border:1px solid #000;max-width:100%;height:auto;background-color:#ddd}button#btn-record.recording-active{background-color:#ff4d4d;color:#fff;border-color:#ff1a1a}button#btn-record.recording-active:hover:not(:disabled){background-color:#e60000}button#btn-record.recording-inactive{background-color:#4caf50;color:#fff;border-color:#367c39}button#btn-record.recording-inactive:hover:not(:disabled){background-color:#45a049}button#btn-powerdown{background-color:#f44336;color:#fff;border-color:#d32f2f}button#btn-powerdown:hover:not(:disabled){background-color:#c62828}button:disabled{background-color:#ccc!important;cursor:not-allowed!important;border-color:#999!important;color:#666!important}</style>{% endraw %}</head>
      <body><div class="container"><h1>Pi Camera Stream & Record</h1><div class="status-grid"><span>Status:</span> <span id="status">Initializing...</span> <span>Recording:</span> <span id="rec-status">OFF</span> <span>Resolution:</span> <span id="resolution">{{ resolution_text }}</span> <span>Battery:</span> <span id="battery-level">{{ batt_text_initial }}%</span></div><div class="controls"><button onclick="changeResolution('down')" id="btn-down" title="Decrease resolution">&laquo; Lower Res</button> <button onclick="toggleRecording()" id="btn-record" class="recording-inactive" title="Toggle recording via web interface">Start Rec (Web)</button> <button onclick="changeResolution('up')" id="btn-up" title="Increase resolution">Higher Res &raquo;</button> <button onclick="powerDown()" id="btn-powerdown" title="Gracefully stop service and reboot Pi">Power Down</button></div><div id="error" {% if err_msg %}style="display: block;"{% endif %}>{{ err_msg }}</div><img id="stream" src="{{ url_for('video_feed') }}" width="{{ current_w }}" height="{{ current_h }}" alt="Loading stream..." onerror="handleStreamError()" onload="handleStreamLoad()"></div>
        <script>const statusElement=document.getElementById("status"),resolutionElement=document.getElementById("resolution"),errorElement=document.getElementById("error"),streamImage=document.getElementById("stream"),btnUp=document.getElementById("btn-up"),btnDown=document.getElementById("btn-down"),btnRecord=document.getElementById("btn-record"),btnPowerdown=document.getElementById("btn-powerdown"),recStatusElement=document.getElementById("rec-status"),batteryLevelElement=document.getElementById("battery-level");let isChangingResolution=!1,isTogglingRecording=!1,isPoweringDown=!1,currentDigitalRecordState={{'true' if digital_rec_state_initial else 'false'}},statusUpdateInterval,streamErrorTimeout=null;function updateRecordButtonState(){currentDigitalRecordState?(btnRecord.textContent="Stop Rec (Web)",btnRecord.classList.remove("recording-inactive"),btnRecord.classList.add("recording-active")):(btnRecord.textContent="Start Rec (Web)",btnRecord.classList.add("recording-inactive"),btnRecord.classList.remove("recording-active"))}function updateStatus(){if(isChangingResolution||isTogglingRecording||isPoweringDown)return;fetch("/status").then(e=>{if(!e.ok)throw new Error(`HTTP error! Status: ${e.status}`);return e.json()}).then(e=>{var t,o;statusElement.textContent=e.status_text||"Unknown",recStatusElement.textContent=e.is_recording?"ACTIVE":"OFF",recStatusElement.classList.toggle("active",e.is_recording),e.resolution&&resolutionElement.textContent!==e.resolution&&(resolutionElement.textContent=e.resolution,[t,o]=e.resolution.split("x"),(streamImage.getAttribute("width")!=t||streamImage.getAttribute("height")!=o)&&(streamImage.setAttribute("width",t),streamImage.setAttribute("height",o))),e.error?(errorElement.textContent=e.error,errorElement.style.display="block"):"none"!==errorElement.style.display&&(errorElement.textContent="",errorElement.style.display="none"),"boolean"==typeof e.digital_recording_active&&currentDigitalRecordState!==e.digital_recording_active&&(currentDigitalRecordState=e.digital_recording_active,updateRecordButtonState()),null!==e.battery_percent&&void 0!==e.battery_percent?batteryLevelElement.textContent=e.battery_percent.toFixed(1):batteryLevelElement.textContent="--"}).catch(e=>{console.error("Error fetching status:",e),statusElement.textContent="Error fetching status",errorElement.textContent=`Failed to fetch status: ${e.message}. Check server connection.`,errorElement.style.display="block",recStatusElement.textContent="Unknown",batteryLevelElement.textContent="Err"})}function disableControls(e=!1){btnUp.disabled=!0,btnDown.disabled=!0,btnRecord.disabled=!0,btnPowerdown.disabled=!0,e&&(document.body.style.opacity="0.7")}function enableControls(){isPoweringDown||(btnUp.disabled=!1,btnDown.disabled=!1,btnRecord.disabled=!1,btnPowerdown.disabled=!1,document.body.style.opacity="1")}function changeResolution(t){if(isChangingResolution||isTogglingRecording||isPoweringDown)return;isChangingResolution=!0,disableControls(),statusElement.textContent="Changing resolution... Please wait.",errorElement.textContent="",errorElement.style.display="none",fetch(`/set_resolution/${t}`,{method:"POST"}).then(e=>e.json().then(t=>({status:e.status,body:t}))).then(({status:e,body:t})=>{200===e&&t.success?(statusElement.textContent="Resolution change initiated. Stream will update.",resolutionElement.textContent=t.new_resolution,[o,n]=t.new_resolution.split("x"),streamImage.setAttribute("width",o),streamImage.setAttribute("height",n),console.log("Resolution change request sent...")):(errorElement.textContent=`Error changing resolution: ${t.message||"Unknown error."}`,errorElement.style.display="block",statusElement.textContent="Resolution change failed.",isChangingResolution=!1,enableControls(),updateStatus());var o,n}).catch(e=>{console.error("Network error sending resolution change:",e),errorElement.textContent=`Network error changing resolution: ${e.message}`,errorElement.style.display="block",statusElement.textContent="Resolution change failed (Network).",isChangingResolution=!1,enableControls(),updateStatus()}).finally(()=>{isChangingResolution&&setTimeout(()=>{isChangingResolution&&(isChangingResolution=!1,enableControls(),updateStatus())},7e3)})}function toggleRecording(){if(isChangingResolution||isTogglingRecording||isPoweringDown)return;isTogglingRecording=!0,disableControls(),statusElement.textContent="Sending record command...",errorElement.textContent="",errorElement.style.display="none",fetch("/toggle_recording",{method:"POST"}).then(e=>{if(!e.ok)throw new Error(`HTTP error! Status: ${e.status}`);return e.json()}).then(e=>{e.success?(currentDigitalRecordState=e.digital_recording_active,updateRecordButtonState(),statusElement.textContent=`Digital recording ${currentDigitalRecordState?"enabled":"disabled"}. State updating...`,setTimeout(updateStatus,1500)):(errorElement.textContent=`Error toggling recording: ${e.message||"Unknown error."}`,errorElement.style.display="block",statusElement.textContent="Record command failed.",setTimeout(updateStatus,1e3))}).catch(e=>{console.error("Error toggling recording:",e),errorElement.textContent=`Network error toggling recording: ${e.message}`,errorElement.style.display="block",statusElement.textContent="Command failed (Network).",setTimeout(updateStatus,1e3)}).finally(()=>{isTogglingRecording=!1,enableControls()})}function powerDown(){if(isChangingResolution||isTogglingRecording||isPoweringDown||!confirm("Are you sure you want to power down the Raspberry Pi? This will stop the camera service and reboot."))return;isPoweringDown=!0,disableControls(!0),statusElement.textContent="Powering down... Signalling service to stop.",errorElement.textContent="",errorElement.style.display="none",statusUpdateInterval&&clearInterval(statusUpdateInterval),fetch("/power_down",{method:"POST"}).then(e=>{if(!e.ok)return e.json().then(e=>{throw new Error(e.message||`HTTP error! Status: ${e.status}`)}).catch(()=>{throw new Error(`HTTP error! Status: ${e.status}`)});return e.json()}).then(e=>{e.success?statusElement.textContent="Shutdown initiated. Reboot will occur shortly.": (errorElement.textContent=`Shutdown request failed: ${e.message||"Unknown error."}`,errorElement.style.display="block",statusElement.textContent="Shutdown failed.",isPoweringDown=!1,enableControls())}).catch(e=>{console.error("Error sending power down command:",e),errorElement.textContent=`Error initiating shutdown: ${e.message}.`,errorElement.style.display="block",statusElement.textContent="Shutdown error.",isPoweringDown=!1,enableControls()})}function handleStreamError(){console.warn("Stream image 'onerror' event triggered."),streamErrorTimeout||isPoweringDown||(statusElement.textContent="Stream interrupted. Attempting reload...",streamErrorTimeout=setTimeout(()=>{streamImage.src="{{ url_for('video_feed') }}?"+Date.now(),streamErrorTimeout=null,setTimeout(updateStatus,1e3)},3e3))}function handleStreamLoad(){streamErrorTimeout&&(clearTimeout(streamErrorTimeout),streamErrorTimeout=null,isPoweringDown||(statusElement.textContent="Stream active."))}document.addEventListener("DOMContentLoaded",()=>{updateRecordButtonState(),updateStatus(),statusUpdateInterval=setInterval(()=>{isChangingResolution||isTogglingRecording||isPoweringDown||updateStatus()},5e3)}),window.addEventListener("beforeunload",()=>{statusUpdateInterval&&clearInterval(statusUpdateInterval)});</script>
      </body>
    </html>
    """, resolution_text=resolution_text, current_w=current_w, current_h=current_h,
         err_msg=err_msg, digital_rec_state_initial=digital_rec_state_initial, batt_text_initial=batt_text_initial)

# --- video_feed() Route ---
@app.route("/video_feed")
def video_feed():
    logging.info("Client connected to video feed.")
    return Response(generate_stream_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# --- status() Route ---
@app.route("/status")
def status():
    global last_error, digital_recording_active, is_recording, battery_percentage, config_lock
    global video_writers, recording_paths

    status_text = "Streaming"; rec_stat_detail = ""; current_w, current_h = get_current_resolution(); batt_perc = None; current_digital_state = False
    with config_lock: current_digital_state = digital_recording_active; batt_perc = battery_percentage; current_is_recording = is_recording; current_recording_paths = list(recording_paths)
    if current_is_recording:
        rec_stat_detail = f" (Recording to {len(current_recording_paths)} USB(s))" if current_recording_paths else " (ERROR: Recording active but no paths!)"
        if not current_recording_paths and not last_error: last_error = "Inconsistent State: Recording active but no paths."
        status_text += rec_stat_detail
    err_msg = last_error if last_error else ""
    if output_frame is not None and err_msg and ("Init Error" in err_msg or "unavailable" in err_msg or "capture" in err_msg): logging.info("Auto-clearing camera/capture error."); last_error = None; err_msg = ""
    if batt_perc is not None and err_msg and ("Battery Monitor" in err_msg or "INA219" in err_msg or "I2C Error" in err_msg): logging.info("Auto-clearing battery monitor error."); last_error = None; err_msg = ""
    return jsonify({'is_recording': current_is_recording,'digital_recording_active': current_digital_state,'resolution': f"{current_w}x{current_h}",'status_text': status_text,'error': err_msg,'active_recordings': current_recording_paths,'battery_percent': batt_perc})

# --- set_resolution() Route ---
@app.route("/set_resolution/<direction>", methods=['POST'])
def set_resolution(direction):
    global current_resolution_index, reconfigure_resolution_index, last_error, config_lock
    with config_lock:
        if reconfigure_resolution_index is not None: return jsonify({'success': False, 'message': 'Reconfiguration already in progress.'}), 429
        if not (0 <= current_resolution_index < len(SUPPORTED_RESOLUTIONS)): logging.error(f"Internal Error: Invalid index {current_resolution_index}!"); return jsonify({'success': False, 'message': 'Internal state error.'}), 500
        original_index = current_resolution_index; new_index = current_resolution_index
        if direction == 'up': new_index += 1
        elif direction == 'down': new_index -= 1
        else: return jsonify({'success': False, 'message': 'Invalid direction.'}), 400
        new_index = max(0, min(len(SUPPORTED_RESOLUTIONS) - 1, new_index))
        if new_index == original_index: msg = 'Already at highest.' if direction == 'up' else 'Already at lowest.'; return jsonify({'success': False, 'message': msg}), 400
        new_w, new_h = SUPPORTED_RESOLUTIONS[new_index]
        logging.info(f"Web request: change resolution {original_index} -> {new_index} ({new_w}x{new_h})")
        reconfigure_resolution_index = new_index; last_error = None
        return jsonify({'success': True, 'message': 'Resolution change requested.', 'new_resolution': f"{new_w}x{new_h}"})

# --- toggle_recording() Route ---
@app.route('/toggle_recording', methods=['POST'])
def toggle_recording():
    global digital_recording_active, last_error, config_lock
    new_state = False
    with config_lock:
        digital_recording_active = not digital_recording_active; new_state = digital_recording_active
        logging.info(f"Digital recording trigger toggled via web UI to: {'ON' if new_state else 'OFF'}")
        if last_error and ("Recording" in last_error or "writers" in last_error or "USB" in last_error or "sync" in last_error): logging.info(f"Clearing previous recording error: '{last_error}'"); last_error = None
    return jsonify({'success': True, 'digital_recording_active': new_state})


# --- MODIFIED Power Down Route (Calls toggle.sh) ---
@app.route('/power_down', methods=['POST'])
def power_down():
    """
    Handles the power down request by executing the toggle.sh script
    with the 'disable' argument. The script handles service stop and reboot.
    Requires passwordless sudo for the user running this script to execute toggle.sh.
    """
    global shutdown_event, last_error, TOGGLE_SCRIPT_PATH

    logging.warning("Received request for power down via web UI. Executing toggle script.")
    last_error = "Shutdown initiated via web UI..." # Update status displayed

    # Check if script path is set and exists
    if not TOGGLE_SCRIPT_PATH or not os.path.exists(TOGGLE_SCRIPT_PATH):
         error_msg = "Toggle script path not configured or file not found."
         logging.error(f"!!! Power Down Failed: {error_msg}")
         last_error = f"Power Down Failed: {error_msg}"
         return jsonify({'success': False, 'message': error_msg}), 500

    # Command to run the toggle script with disable argument using sudo
    command = ["sudo", TOGGLE_SCRIPT_PATH, "disable"]

    try:
        logging.info(f"Executing command: {' '.join(command)}")
        # Use Popen for fire-and-forget. The toggle script handles the rest.
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        logging.info(f"Toggle script executed (PID: {process.pid}). It will handle service stop and reboot.")

        # Signal local threads to shut down AFTER triggering the script.
        logging.info("Setting local shutdown event as well...")
        shutdown_event.set()

        return jsonify({'success': True, 'message': 'Shutdown initiated via toggle script.'})

    except FileNotFoundError:
        error_msg = f"Failed to execute shutdown script: 'sudo' command not found?"
        logging.critical(f"!!! {error_msg}")
        last_error = f"Power Down Failed: {error_msg}"
        return jsonify({'success': False, 'message': error_msg}), 500
    except Exception as e:
        error_msg = f"Failed to execute shutdown script: {e}"
        logging.critical(f"!!! {error_msg}", exc_info=True)
        last_error = f"Power Down Failed: {error_msg}"
        return jsonify({'success': False, 'message': error_msg}), 500

# ===========================================================
# === FLASK ROUTES END HERE ===
# ===========================================================

# --- Signal Handling ---
def signal_handler(sig, frame):
    global shutdown_event
    if shutdown_event.is_set(): logging.warning("Shutdown already in progress."); return
    logging.warning(f"Received signal {sig}. Initiating graceful shutdown...")
    shutdown_event.set()


# --- Main Execution (Removed Reboot Logic) ---
def main():
    global last_error, capture_thread, flask_thread, picam2, shutdown_event
    # No need for reboot_requested or AP_PROFILE_NAME in main anymore

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # --- Startup Logging ---
    logging.info(" --- Starting Camera Stream & Record Service --- ")
    logging.info(f"--- Using Picamera2 and gpiozero ---")
    logging.info(f"--- Configured for {FRAME_RATE} FPS, {len(SUPPORTED_RESOLUTIONS)} resolutions ---")
    logging.info(f"--- Recording to {USB_BASE_PATH} using {RECORDING_FORMAT}{RECORDING_EXTENSION} ---")
    logging.info(f"--- Web UI on port {WEB_PORT} ---")
    if USE_NOIR_TUNING: logging.info(f"--- Attempting to use NoIR Tuning: {NOIR_TUNING_FILE_PATH} ---")
    if INA219_I2C_ADDRESS is not None: logging.info(f"--- Battery Monitor Enabled (INA219 @ 0x{INA219_I2C_ADDRESS:X}) ---")
    else: logging.info("--- Battery Monitor Disabled ---")
    if SWITCH_GPIO_PIN is not None: logging.info(f"--- Physical Record Switch Enabled (GPIO {SWITCH_GPIO_PIN}) ---")
    else: logging.info("--- Physical Record Switch Disabled ---")
    logging.info(f"--- Power Down Method: Web UI calls '{TOGGLE_SCRIPT_PATH} disable' ---")

    logging.info(f"MAIN: Initial shutdown_event state: {shutdown_event.is_set()}")
    run_counter = 0

    # --- Main Loop (Runs once unless restart logic is added) ---
    if not shutdown_event.is_set():
        run_counter += 1
        logging.info(f"MAIN: Entering main execution block (Run {run_counter})...")
        last_error = None; capture_thread = None; flask_thread = None

        try:
            # --- Hardware Init ---
            logging.info("Initializing Hardware Components...")
            if SWITCH_GPIO_PIN is not None:
                if not setup_gpio(): logging.error(f"GPIO setup failed: {last_error}. Switch unavailable.")
            if INA219_I2C_ADDRESS is not None:
                if not setup_battery_monitor(): logging.warning(f"Battery monitor setup failed: {last_error}. Level unavailable.")

            # --- Start Threads ---
            logging.info("Starting frame capture thread...")
            capture_thread = threading.Thread(target=capture_and_process_loop, name="CaptureThread", daemon=True)
            capture_thread.start(); time.sleep(4.0) # Allow time for camera init
            if not capture_thread.is_alive(): raise RuntimeError(f"Capture thread failed startup: {last_error or 'Unknown'}")
            if last_error and ("Init Error" in last_error or "failed fatally" in last_error): raise RuntimeError(f"Camera init failed: {last_error}")
            logging.info("Capture thread running.")

            logging.info(f"Starting Flask web server on 0.0.0.0:{WEB_PORT}...")
            flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=WEB_PORT, debug=False, use_reloader=False, threaded=True), name="FlaskThread", daemon=True)
            flask_thread.start(); time.sleep(1.5) # Allow time for Flask
            if not flask_thread.is_alive(): raise RuntimeError("Flask thread failed startup.")

            # --- System Running ---
            logging.info("--- System Running ---")
            try: # Get IP
                import socket; s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.connect(("8.8.8.8", 80)); local_ip = s.getsockname()[0]; s.close()
                logging.info(f"Access web UI at: http://{local_ip}:{WEB_PORT}")
            except Exception as ip_e: logging.warning(f"Could not get IP: {ip_e}"); logging.info(f"Access web UI at: http://<PI_IP>:{WEB_PORT}")

            # --- Wait for Shutdown Signal ---
            logging.info("MAIN: Waiting for shutdown signal...")
            shutdown_event.wait() # Wait indefinitely until event is set
            logging.info("MAIN: Shutdown signal received.")

        except RuntimeError as e:
            logging.error(f"!!! Runtime Error in Main: {e}"); last_error = f"Runtime Error: {e}"
            shutdown_event.set() # Ensure shutdown on runtime error
        except Exception as e:
            logging.exception(f"!!! Unhandled Exception in Main: {e}"); last_error = f"Unhandled Exception: {e}"
            shutdown_event.set() # Ensure shutdown on other errors


    # --- Final Cleanup (Executed after shutdown_event is set) ---
    logging.info("--- Shutdown Sequence Initiated ---")

    # Wait for capture thread (handles its own cleanup)
    if capture_thread and capture_thread.is_alive():
        logging.info("Waiting for capture thread to exit (timeout 15s)...")
        capture_thread.join(timeout=15.0)
        if capture_thread.is_alive(): logging.warning("Capture thread did not exit cleanly.")
        else: logging.info("Capture thread finished.")

    # Flask thread is daemon

    # Final cleanup (belt-and-braces)
    if is_recording: logging.warning("Force stopping recording during final shutdown."); stop_recording()
    if picam2:
        try: logging.info("Ensuring Picamera2 closed..."); picam2.close()
        except Exception as e: logging.warning(f"Error during final Picamera2 close: {e}")
    if SWITCH_GPIO_PIN is not None: cleanup_gpio()

    # --- Exit ---
    logging.info("--- Program Exit (Cleanup Complete) ---")
    # Reboot is handled externally by toggle.sh if initiated from web UI

if __name__ == '__main__':
    main()