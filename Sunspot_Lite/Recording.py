# -*- coding: utf-8 -*-
import cv2
import time
import datetime
import os # <<< Ensure os is imported
import sys # <<< Import sys for path determination
import threading
import signal
from flask import Flask, Response, render_template_string, jsonify, redirect, url_for, request
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
TOGGLE_SCRIPT_NAME = "toggle.sh" # Name of the bash script in the same directory

# --- Global Variables ---
app = Flask(__name__)
picam2 = None
output_frame = None
frame_lock = threading.Lock()
config_lock = threading.Lock()
capture_thread = None
flask_thread = None
shutdown_event = threading.Event()
reboot_requested = False # Flag to signal reboot after cleanup
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

AVAILABLE_AWB_MODES = [m.name for m in list(controls.AwbModeEnum)] # Use list()
DEFAULT_AWB_MODE_NAME = "Auto"
current_awb_mode = controls.AwbModeEnum.Auto

AVAILABLE_AE_MODES = [m.name for m in list(controls.AeExposureModeEnum)] # Use list()
DEFAULT_AE_MODE_NAME = "Normal"
current_ae_mode = controls.AeExposureModeEnum.Normal

AVAILABLE_METERING_MODES = [m.name for m in list(controls.AeMeteringModeEnum)] # Use list()
DEFAULT_METERING_MODE_NAME = "CentreWeighted"
current_metering_mode = controls.AeMeteringModeEnum.CentreWeighted

AVAILABLE_NOISE_REDUCTION_MODES = [m.name for m in list(controls.NoiseReductionModeEnum)] # Use list()
DEFAULT_NOISE_REDUCTION_MODE_NAME = "Fast"
current_noise_reduction_mode = controls.NoiseReductionModeEnum.Fast

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
        # logging.info("INA219 calibrated for 16V 5A range.")

    def set_calibration_32V_2A(self):
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
        self.read(_REG_BUSVOLTAGE)
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
# === BATTERY MONITOR (INA219) CODE END ===
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
            # Log warning if NativeFactory fails, script will continue with default
            logging.warning(f"Could not set NativeFactory for gpiozero, using default: {e}")
        switch = Button(SWITCH_GPIO_PIN, pull_up=True, bounce_time=SWITCH_BOUNCE_TIME)
        logging.info(f"gpiozero Button on pin {SWITCH_GPIO_PIN} setup complete.")
        return True
    except Exception as e:
        # Log error if Button setup fails completely
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
             # logging.warning("Battery min/max voltages invalid (max <= min). Cannot calculate percentage.") # Reduce log spam
             percent = None
        else:
            percent = ((bus_voltage - BATTERY_MIN_VOLTAGE) / voltage_range) * 100.0
            percent = max(0.0, min(100.0, percent)) # Clamp 0-100

        with config_lock:
            battery_percentage = percent

    except OSError as e:
        # Log less verbosely on I2C errors after first occurrence?
        # For now, log every time.
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

# --- Helper Function for FPS ---
def get_target_fps(width, height):
    """Determines the target FPS based on resolution."""
    if width >= 3280: # Max resolution
        return 15.0
    elif width == 1920 and height == 1080: # 1080p
        return 30.0
    # Includes (1640, 1232), (1280, 720), (640, 480) and any others below 1080p
    elif width <= 1640:
         return 40.0
    else: # Default fallback (should ideally cover all SUPPORTED_RESOLUTIONS)
        return 30.0 # Default to 30 if resolution doesn't match specific cases

# --- Initialize Camera ---
def initialize_camera(target_width, target_height):
    global picam2, last_error, current_awb_mode # <<< Added current_awb_mode
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
        logging.info("Picamera2 object created.") # Log after successful Picamera2() call

        # --- Determine Target FPS ---
        target_fps = get_target_fps(target_width, target_height)
        logging.info(f"Targeting FPS: {target_fps:.1f} for resolution {target_width}x{target_height}")

        # --- Get current AWB mode safely ---
        with config_lock:
            awb_mode_to_set = current_awb_mode # Use the global state

        config = picam2.create_video_configuration(
            main={"size": (target_width, target_height), "format": "RGB888"},
            controls={
                "FrameRate": target_fps, # <<< Use dynamic FPS
                "AwbEnable": True,
                "AwbMode": awb_mode_to_set, # <<< Use current AWB mode
                "Brightness": 0.0,
                "Contrast": 1.0,
                "Saturation": 1.0,
                 # Add other controls as needed, e.g., Sharpness, AeMeteringMode later
            }
        )
        logging.info(f"Configuring Picamera2 with: main={config['main']}, controls={config['controls']}") # Improved log
        original_config_str = str(config) # Keep for comparison if needed

        picam2.configure(config)
        # Give driver time to settle
        time.sleep(0.5)
        new_config = picam2.camera_configuration() # Use actual applied config

        # Log potential differences
        if not new_config:
             logging.warning("Could not get camera configuration after applying.")
        else:
            applied_controls = new_config.get('controls', {})
            applied_main = new_config.get('main', {})
            logging.info(f"Applied Config: main={applied_main}, controls={applied_controls}")
            if applied_controls.get('FrameRate') != target_fps:
                 logging.warning(f"Camera driver adjusted FrameRate from {target_fps} to {applied_controls.get('FrameRate')}")
            if applied_controls.get('AwbMode') != awb_mode_to_set:
                 logging.warning(f"Camera driver adjusted AwbMode from {awb_mode_to_set} to {applied_controls.get('AwbMode')}")


        logging.info("Configuration successful!")

        picam2.start()
        logging.info("Camera started")
        # Allow more time for sensor settings (like AWB, AE) to stabilize
        time.sleep(2.0)

        actual_config = picam2.camera_configuration()
        if not actual_config:
                raise RuntimeError("Failed to get camera configuration after start.")

        actual_format = actual_config.get('main', {})
        actual_w = actual_format.get('size', (0,0))[0]
        actual_h = actual_format.get('size', (0,0))[1]
        actual_fmt_str = actual_format.get('format', 'Unknown')
        actual_fps = actual_config.get('controls', {}).get('FrameRate', 'N/A') # <<< Get actual FPS
        logging.info(f"Picamera2 initialized. Actual main stream config: {actual_w}x{actual_h}, Format: {actual_fmt_str}, Actual FPS: {actual_fps}")

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
                    with open(test_file, 'w') as f:
                        f.write('test')
                    os.remove(test_file)
                    mounts.append(path)
                    logging.debug(f"Found writable mount: {path}")
                except Exception as write_err:
                     logging.warning(f"Directory {path} appears mounted but test write failed: {write_err}")
            elif os.path.isdir(path):
                 logging.debug(f"Directory {path} found but is not writable.")

        if not mounts:
            logging.debug("No writable USB mounts found.")
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

    video_writers.clear()
    recording_paths.clear()
    success_count = 0
    start_error = None

    if picam2 is None or not picam2.started:
        logging.error("Cannot start recording, camera is not available.")
        last_error = "Camera not available for recording."
        return False

    try:
        # --- Get ACTUAL configuration ---
        cam_config = picam2.camera_configuration()
        if not cam_config:
             raise RuntimeError("Failed to get camera configuration before recording start.")

        main_stream_config = cam_config.get('main', {})
        controls_config = cam_config.get('controls', {})

        width = main_stream_config.get('size', (0,0))[0]
        height = main_stream_config.get('size', (0,0))[1]

        # --- Use ACTUAL FPS from camera controls ---
        actual_fps = controls_config.get('FrameRate')
        if actual_fps is None:
            logging.error("!!! Could not determine actual FrameRate from camera config. Falling back.")
            # Fallback logic: Use the function based on current resolution
            # This might be slightly off if the driver adjusted it, but better than nothing
            current_w_fb, current_h_fb = get_current_resolution()
            actual_fps = get_target_fps(current_w_fb, current_h_fb)
            last_error = "Rec FPS Fallback: Couldn't read actual FPS."

        # Ensure FPS is a float, handle potential errors
        try:
            fps_for_writer = float(actual_fps)
            if fps_for_writer <= 0:
                raise ValueError("Invalid FPS value <= 0")
        except (TypeError, ValueError) as fps_err:
             logging.error(f"!!! Invalid FPS value obtained ({actual_fps}). Defaulting to 15 FPS. Error: {fps_err}")
             fps_for_writer = 15.0
             last_error = f"Rec FPS Error: Invalid value {actual_fps}"


        # --- REMOVED: Hardcoded FPS cap based on width (now handled by get_target_fps) ---
        # if width >= 3280 and fps > 15:
        #     logging.warning(f"High resolution ({width}x{height}) detected. Capping recording FPS to 15 to reduce load.")
        #     fps = 15.0

        if width <= 0 or height <= 0:
            raise ValueError(f"Invalid camera dimensions obtained: {width}x{height}")

        # --- Log the FPS being used for the writer ---
        logging.info(f"Starting recording with dimensions: {width}x{height} @ {fps_for_writer:.2f} fps (Actual FPS reported by camera)")

        fourcc = cv2.VideoWriter_fourcc(*RECORDING_FORMAT)
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        for drive_path in usb_drives:
            try:
                filename = f"recording_{timestamp}_{width}x{height}{RECORDING_EXTENSION}"
                full_path = os.path.join(drive_path, filename)
                # --- Use fps_for_writer for VideoWriter ---
                writer = cv2.VideoWriter(full_path, fourcc, fps_for_writer, (width, height))
                if not writer.isOpened():
                    raise IOError(f"Failed to open VideoWriter for path: {full_path}")
                video_writers.append(writer)
                recording_paths.append(full_path)
                logging.info(f"Successfully started recording to: {full_path}")
                success_count += 1
            except Exception as e:
                logging.error(f"!!! Failed to create VideoWriter for drive {drive_path}: {e}", exc_info=True)
                if start_error is None:
                    start_error = f"Failed writer {os.path.basename(drive_path)}: {e}"

        if success_count > 0:
            is_recording = True
            logging.info(f"Recording successfully started on {success_count} drive(s).")
            if start_error and success_count < len(usb_drives):
                    last_error = f"Partial Rec Start Fail: {start_error}"
            # --- Clear previous recording errors on successful start ---
            elif not start_error:
                 if last_error and ("Recording" in last_error or "writers" in last_error or "USB" in last_error or "sync" in last_error or "Rec FPS" in last_error):
                      logging.info(f"Clearing previous recording error: '{last_error}'")
                      last_error = None
            # --- Only set last_error if there was a failure but some succeeded ---
            # else: # This case means no errors at all
            #    last_error = None # Explicitly clear if needed
            return True
        else:
            is_recording = False
            logging.error("Failed to start recording on ANY USB drive.")
            last_error = f"Recording Start Failed: {start_error or 'No writers opened'}"
            # Clean up any potentially opened (but failed) writers - belt and braces
            for writer in video_writers:
                 try: writer.release()
                 except: pass
            video_writers.clear()
            recording_paths.clear()
            return False

    except Exception as e:
        logging.error(f"!!! Critical error during recording setup: {e}", exc_info=True)
        last_error = f"Recording Setup Error: {e}"
        # Ensure cleanup even if setup fails midway
        stop_recording() # stop_recording now handles empty lists safely
        return False


def stop_recording():
    global is_recording, video_writers, recording_paths, last_error
    if not is_recording:
        if video_writers or recording_paths:
            logging.warning("stop_recording called but not recording; clearing potentially stale writer lists.")
            video_writers.clear(); recording_paths.clear()
        return

    logging.info("Stopping recording...")
    is_recording = False
    released_count = 0
    writers_to_release = list(video_writers)
    paths_recorded = list(recording_paths)
    video_writers.clear(); recording_paths.clear()

    logging.info(f"Releasing {len(writers_to_release)} video writer(s)...")
    for i, writer in enumerate(writers_to_release):
        path = paths_recorded[i] if i < len(paths_recorded) else f"Unknown Path (Writer Index {i})"
        try:
            writer.release()
            logging.info(f"Successfully released writer for: {path}")
            released_count += 1
        except Exception as e:
            logging.error(f"Error releasing VideoWriter for {path}: {e}", exc_info=True)

    if released_count > 0:
        logging.info("Syncing filesystem to ensure data is written to USB drives...")
        try:
            sync_start_time = time.monotonic()
            os.system('sync') # <<< Force OS write cache to disk
            sync_duration = time.monotonic() - sync_start_time
            logging.info(f"Filesystem sync completed in {sync_duration:.2f} seconds.")
            # time.sleep(0.5) # Optional extra safety sleep
        except Exception as e:
            logging.error(f"!!! Failed to execute 'sync' command: {e}", exc_info=True)
            last_error = "Filesystem sync failed after recording."
    else:
        logging.warning("No video writers were successfully released, skipping filesystem sync.")

    logging.info(f"Recording fully stopped. Released {released_count} writer(s) out of {len(writers_to_release)} attempted.")

# --- Capture Loop ---
def capture_and_process_loop():
    global output_frame, is_recording, last_error, picam2, switch, digital_recording_active
    global current_resolution_index, reconfigure_resolution_index, last_battery_read_time
    global video_writers, recording_paths

    logging.info("Starting frame capture loop...")
    consecutive_error_count = 0
    max_consecutive_errors = 15

    width, height = get_current_resolution()
    if not initialize_camera(width, height):
        logging.error("Initial camera setup failed. Capture thread cannot start.")
        return # Exit thread

    while not shutdown_event.is_set():
        loop_start_time = time.monotonic()

        try:
            target_index = -1
            if reconfigure_resolution_index is not None:
                with config_lock:
                    if reconfigure_resolution_index is not None:
                        target_index = reconfigure_resolution_index
                        reconfigure_resolution_index = None

            if target_index != -1:
                logging.info(f"--- Reconfiguring resolution to index {target_index} ---")
                physical_switch_on_before = (switch is not None and switch.is_pressed)
                digital_switch_on_before = digital_recording_active
                should_be_recording_after = physical_switch_on_before or digital_switch_on_before
                was_actually_recording = is_recording

                if was_actually_recording:
                    logging.info("Stopping recording for reconfiguration...")
                    stop_recording() # Includes sync

                new_width, new_height = SUPPORTED_RESOLUTIONS[target_index]
                if initialize_camera(new_width, new_height):
                    with config_lock:
                        current_resolution_index = target_index
                    logging.info(f"--- Reconfiguration successful to {new_width}x{new_height} ---")
                    if should_be_recording_after:
                        logging.info("Resuming recording after successful reconfiguration...")
                        time.sleep(1.0)
                        if not start_recording():
                            logging.error("Failed to restart recording after reconfiguration!")
                else:
                    logging.error(f"!!! Failed reconfigure to index {target_index}. Attempting to restore previous resolution... !!!")
                    prev_width, prev_height = get_current_resolution()
                    if not initialize_camera(prev_width, prev_height):
                        logging.critical("!!! Failed to restore previous camera resolution after failed reconfig. Stopping service. !!!")
                        last_error = "Camera failed fatally during reconfig restore."
                        shutdown_event.set(); break
                    else:
                        logging.info("Successfully restored previous camera resolution.")
                        if should_be_recording_after:
                            logging.info("Attempting recording restart with restored resolution...")
                            time.sleep(1.0)
                            if not start_recording():
                                logging.error("Failed to restart recording after failed reconfig and restore.")
                continue # Skip rest of loop iteration

            if picam2 is None or not picam2.started:
                if not last_error: last_error = "Picamera2 became unavailable."
                logging.error(f"Camera unavailable: {last_error}. Attempting reinitialization...")
                width, height = get_current_resolution()
                if initialize_camera(width, height):
                     logging.info("Camera re-initialized successfully after unexpected stop.")
                     last_error = None; consecutive_error_count = 0
                else:
                    logging.error(f"Camera re-initialization failed: {last_error}. Stopping capture loop.")
                    shutdown_event.set(); break
                continue # Skip rest of loop

            frame_bgr = picam2.capture_array("main")
            if frame_bgr is None:
                logging.warning("Failed to capture frame from Picamera2. Retrying...")
                consecutive_error_count += 1
                if consecutive_error_count > max_consecutive_errors:
                    last_error = f"Failed capture {max_consecutive_errors} consecutive times. Assuming camera failure."
                    logging.error(last_error); shutdown_event.set(); break
                time.sleep(0.1); continue
            if consecutive_error_count > 0:
                logging.info(f"Recovered frame grab after {consecutive_error_count} errors.")
            consecutive_error_count = 0

            physical_switch_on = False
            if switch is not None:
                try:
                    physical_switch_on = switch.is_pressed
                except Exception as e:
                    logging.error(f"Error reading switch state: {e}")
                    last_error = f"Switch Read Error: {e}"
                    physical_switch_on = False

            with config_lock:
                digital_switch_on = digital_recording_active

            should_be_recording = physical_switch_on or digital_switch_on

            if should_be_recording and not is_recording:
                log_msg = "Physical switch ON" if physical_switch_on else ""
                if digital_switch_on: log_msg += (" / " if log_msg else "") + "Digital trigger ON"
                logging.info(f"Recording trigger active ({log_msg}) - initiating start.")
                if not start_recording():
                    logging.error("Attempt to start recording failed.")
            elif not should_be_recording and is_recording:
                logging.info("Recording trigger(s) OFF - initiating stop.")
                stop_recording()

            if is_recording:
                if not video_writers:
                    logging.warning("Inconsistent state: is_recording=True, but no video writers. Forcing stop.")
                    is_recording = False
                    last_error = "Rec stopped: writer list was empty."
                else:
                    write_errors = 0
                    current_writers = list(video_writers)
                    current_paths = list(recording_paths)
                    for i, writer in enumerate(current_writers):
                        try:
                            writer.write(frame_bgr)
                        except Exception as e:
                             path_str = current_paths[i] if i < len(current_paths) else f"Writer {i}"
                             logging.error(f"!!! Failed to write frame to {path_str}: {e}")
                             write_errors += 1
                             last_error = f"Frame write error: {os.path.basename(path_str)}"
                    if write_errors > 0 and write_errors == len(current_writers):
                        logging.error("All active video writers failed to write frame. Stopping recording.")
                        last_error = "Rec stopped: All writers failed write."
                        stop_recording()

            with frame_lock:
                output_frame = frame_bgr.copy()

            if ina219_sensor and (loop_start_time - last_battery_read_time > BATTERY_READ_INTERVAL):
                 read_battery_level()
                 last_battery_read_time = loop_start_time

        except Exception as e:
            logging.exception(f"!!! Unexpected Error in capture loop: {e}")
            last_error = f"Capture Loop Error: {e}"
            consecutive_error_count += 1
            if consecutive_error_count > max_consecutive_errors / 2:
                logging.error(f"Too many consecutive errors ({consecutive_error_count}). Signaling shutdown.")
                shutdown_event.set()
            time.sleep(1)

    # --- Cleanup after loop exit ---
    logging.info("Exiting frame capture thread.")
    if is_recording:
        logging.info("Capture loop exiting: Stopping active recording...")
        stop_recording()
    if picam2:
        try:
            logging.info("Capture loop exiting: Stopping and closing Picamera2...")
            if picam2.started: picam2.stop()
            picam2.close()
            logging.info("Picamera2 released by capture thread.")
        except Exception as e:
            logging.error(f"Error stopping/closing Picamera2 in thread cleanup: {e}")
    picam2 = None


# ===========================================================
# === FLASK ROUTES START HERE ===
# ===========================================================
def generate_stream_frames():
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
            (flag, encodedImage) = cv2.imencode(".jpg", frame_to_encode, [cv2.IMWRITE_JPEG_QUALITY, 75])
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
            logging.exception(f"!!! Error in MJPEG streaming generator: {e}")
            time.sleep(0.5)

    logging.info("Stream generator thread exiting.")


# --- index() function with Jinja raw block fix ---
@app.route("/")
def index():
    global last_error, digital_recording_active, battery_percentage, config_lock
    global current_awb_mode, AVAILABLE_AWB_MODES # <<< Add globals needed for AWB

    current_w, current_h = get_current_resolution()
    resolution_text = f"{current_w}x{current_h}"
    err_msg = last_error if last_error else ""

    with config_lock:
        digital_rec_state_initial = digital_recording_active
        batt_perc_initial = battery_percentage
        # <<< Get current AWB mode name for initial select value >>>
        try:
            current_awb_mode_name_initial = current_awb_mode.name
        except AttributeError:
             current_awb_mode_name_initial = DEFAULT_AWB_MODE_NAME # Fallback if not set

    batt_text_initial = f"{batt_perc_initial:.1f}" if batt_perc_initial is not None else "--"

    # Build options for AWB dropdown
    awb_options_html = ""
    for mode_name in AVAILABLE_AWB_MODES:
        selected_attr = ' selected' if mode_name == current_awb_mode_name_initial else ''
        awb_options_html += f'<option value="{mode_name}"{selected_attr}>{mode_name}</option>'

    # Using triple quotes for the large HTML string
    return render_template_string(f"""
    <!DOCTYPE html>
    <html>
    <head>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Pi Camera Stream & Record</title>
        {{% raw %}}
        <style>
            body {{ font-family: sans-serif; line-height: 1.4; margin: 1em; background-color: #f0f0f0;}}
            .container {{ max-width: 960px; margin: auto; background: #fff; padding: 15px; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }}
            h1 {{ text-align: center; color: #333; margin-bottom: 20px; }}
            .status-grid {{ display: grid; grid-template-columns: auto 1fr; gap: 5px 15px; margin-bottom: 15px; align-items: center; background-color: #eef; padding: 10px; border-radius: 5px;}}
            .status-grid span:first-child {{ font-weight: bold; color: #555; text-align: right;}}
            .status-grid select {{ padding: 3px; }} /* Style for select */
            #status, #rec-status, #resolution, #battery-level, #awb-mode-status {{ color: #0056b3; font-weight: normal;}} /* Added awb status id */
            #rec-status.active {{ color: #D83B01; font-weight: bold;}}
            .controls {{ text-align: center; margin-bottom: 15px; display: flex; justify-content: center; align-items: center; flex-wrap: wrap; gap: 10px;}} /* Flex layout for buttons */
            .controls button, .controls select {{ padding: 10px 15px; margin: 5px; font-size: 1em; cursor: pointer; border-radius: 5px; border: 1px solid #ccc; background-color: #e9e9e9; transition: background-color 0.2s, border-color 0.2s; }} /* Apply similar style to select */
            .controls button:hover:not(:disabled), .controls select:hover:not(:disabled) {{ background-color: #dcdcdc; border-color: #bbb;}}
            #error {{ color: red; margin-top: 10px; white-space: pre-wrap; font-weight: bold; min-height: 1.2em; text-align: center; background-color: #ffebeb; border: 1px solid red; padding: 8px; border-radius: 4px; display: none; /* Initially hidden */ }}
            img#stream {{ display: block; margin: 15px auto; border: 1px solid black; max-width: 100%; height: auto; background-color: #ddd; }} /* Placeholder color */
            button#btn-record.recording-active {{ background-color: #ff4d4d; color: white; border-color: #ff1a1a; }}
            button#btn-record.recording-active:hover:not(:disabled) {{ background-color: #e60000; }}
            button#btn-record.recording-inactive {{ background-color: #4CAF50; color: white; border-color: #367c39;}}
            button#btn-record.recording-inactive:hover:not(:disabled) {{ background-color: #45a049; }}
            button#btn-powerdown {{ background-color: #f44336; color: white; border-color: #d32f2f;}} /* Style for Power Down button */
            button#btn-powerdown:hover:not(:disabled) {{ background-color: #c62828; }}
            button:disabled, select:disabled {{ background-color: #cccccc !important; cursor: not-allowed !important; border-color: #999 !important; color: #666 !important;}}
        </style>
        {{% endraw %}}
    </head>
    <body>
        <div class="container">
            <h1>Pi Camera Stream & Record</h1>
            <div class="status-grid">
                <span>Status:</span> <span id="status">Initializing...</span>
                <span>Recording:</span> <span id="rec-status">OFF</span>
                <span>Resolution:</span> <span id="resolution">{resolution_text}</span>
                <span>Battery:</span> <span id="battery-level">{batt_text_initial}%</span>
                <span>AWB Mode:</span> <span id="awb-mode-status">{current_awb_mode_name_initial}</span>
             </div>
             <div class="controls">
                 <button onclick="changeResolution('down')" id="btn-down" title="Decrease resolution">&laquo; Lower Res</button>
                 <button onclick="toggleRecording()" id="btn-record" class="recording-inactive" title="Toggle recording via web interface">Start Rec (Web)</button>
                 <button onclick="changeResolution('up')" id="btn-up" title="Increase resolution">Higher Res &raquo;</button>
                 <select id="awb-select" onchange="changeAwbMode()" title="Select Auto White Balance Mode">
                    {awb_options_html}
                </select>
                <button onclick="powerDown()" id="btn-powerdown" title="Gracefully stop service and reboot Pi">Power Down</button>
            </div>
             <div id="error" {'style="display: block;"' if err_msg else ''}>{err_msg}</div>
             <img id="stream" src="{{{{ url_for('video_feed') }}}}" width="{current_w}" height="{current_h}" alt="Loading stream..."
                    onerror="handleStreamError()" onload="handleStreamLoad()">
        </div>

        <script>
            // --- JavaScript ---
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
            const awbStatusElement = document.getElementById('awb-mode-status'); // <<< Get AWB status span
            const awbSelectElement = document.getElementById('awb-select'); // <<< Get AWB select dropdown

            let isChangingResolution = false;
            let isTogglingRecording = false;
            let isChangingAwb = false; // <<< Add flag for AWB change
            let isPoweringDown = false;
            let currentDigitalRecordState = {'true' if digital_rec_state_initial else 'false'};
            let statusUpdateInterval;
            let streamErrorTimeout = null;

            function updateRecordButtonState() {{
                if (currentDigitalRecordState) {{
                    btnRecord.textContent = "Stop Rec (Web)";
                    btnRecord.classList.remove('recording-inactive');
                    btnRecord.classList.add('recording-active');
                }} else {{
                    btnRecord.textContent = "Start Rec (Web)";
                    btnRecord.classList.add('recording-inactive');
                    btnRecord.classList.remove('recording-active');
                }}
            }}

            function updateStatus() {{
                if (isChangingResolution || isTogglingRecording || isChangingAwb || isPoweringDown) return; // <<< Check AWB flag
                fetch('/status')
                    .then(response => {{ if (!response.ok) {{ throw new Error(`HTTP error! Status: ${{response.status}}`); }} return response.json(); }})
                    .then(data => {{
                        statusElement.textContent = data.status_text || 'Unknown';
                        recStatusElement.textContent = data.is_recording ? "ACTIVE" : "OFF";
                        recStatusElement.classList.toggle('active', data.is_recording);
                        if (data.resolution && resolutionElement.textContent !== data.resolution) {{
                            resolutionElement.textContent = data.resolution;
                            const [w, h] = data.resolution.split('x');
                            if (streamImage.getAttribute('width') != w || streamImage.getAttribute('height') != h) {{
                                streamImage.setAttribute('width', w);
                                streamImage.setAttribute('height', h);
                            }}
                        }}
                        if (data.error) {{ errorElement.textContent = data.error; errorElement.style.display = 'block'; }}
                        else {{ if (errorElement.style.display !== 'none') {{ errorElement.textContent = ''; errorElement.style.display = 'none'; }} }}
                        if (typeof data.digital_recording_active === 'boolean' && currentDigitalRecordState !== data.digital_recording_active) {{
                            currentDigitalRecordState = data.digital_recording_active;
                            updateRecordButtonState();
                        }}
                         // <<< Update AWB status and select dropdown if changed elsewhere >>>
                        if (data.awb_mode && awbStatusElement.textContent !== data.awb_mode) {{
                            awbStatusElement.textContent = data.awb_mode;
                            if (awbSelectElement.value !== data.awb_mode) {{
                                awbSelectElement.value = data.awb_mode;
                                console.log("AWB mode updated by status check: " + data.awb_mode);
                            }}
                        }}
                         // <<< END AWB update >>>
                        if (data.battery_percent !== null && data.battery_percent !== undefined) {{
                            batteryLevelElement.textContent = data.battery_percent.toFixed(1);
                        }} else {{
                            batteryLevelElement.textContent = "--";
                        }}
                    }})
                    .catch(err => {{ console.error("Error fetching status:", err); statusElement.textContent = "Error fetching status"; errorElement.textContent = `Failed to fetch status: ${{err.message}}. Check server connection.`; errorElement.style.display = 'block'; recStatusElement.textContent = "Unknown"; batteryLevelElement.textContent = "Err"; awbStatusElement.textContent = "Err"; }});
            }}

            function disableControls(poweringDown = false) {{
                btnUp.disabled = true; btnDown.disabled = true; btnRecord.disabled = true; btnPowerdown.disabled = true;
                awbSelectElement.disabled = true; // <<< Disable AWB select
                if(poweringDown) {{ document.body.style.opacity = '0.7'; }}
            }}

            function enableControls() {{
                if (!isPoweringDown) {{
                     btnUp.disabled = false; btnDown.disabled = false; btnRecord.disabled = false; btnPowerdown.disabled = false;
                     awbSelectElement.disabled = false; // <<< Enable AWB select
                     document.body.style.opacity = '1';
                 }}
            }}

            function changeResolution(direction) {{
                if (isChangingResolution || isTogglingRecording || isChangingAwb || isPoweringDown) return; // <<< Check AWB flag
                isChangingResolution = true; disableControls(); statusElement.textContent = 'Changing resolution... Please wait.'; errorElement.textContent = ''; errorElement.style.display = 'none';
                fetch(`/set_resolution/${{direction}}`, {{ method: 'POST' }})
                    .then(response => response.json().then(data => ({{ status: response.status, body: data }})))
                    .then(({{ status, body }}) => {{
                        if (status === 200 && body.success) {{
                            statusElement.textContent = 'Resolution change initiated. Stream will update.'; resolutionElement.textContent = body.new_resolution; const [w, h] = body.new_resolution.split('x'); streamImage.setAttribute('width', w); streamImage.setAttribute('height', h); console.log("Resolution change request sent...");
                        }} else {{
                            errorElement.textContent = `Error changing resolution: ${{body.message || 'Unknown error.'}}`; errorElement.style.display = 'block'; statusElement.textContent = 'Resolution change failed.'; isChangingResolution = false; enableControls(); updateStatus(); // Re-enable controls on failure
                        }}
                    }})
                    .catch(err => {{ console.error("Network error sending resolution change:", err); errorElement.textContent = `Network error changing resolution: ${{err.message}}`; errorElement.style.display = 'block'; statusElement.textContent = 'Resolution change failed (Network).'; isChangingResolution = false; enableControls(); updateStatus(); }}) // Re-enable controls on failure
                    .finally(() => {{
                        // Use a timeout to re-enable controls if the process seems stuck (might happen if camera re-init takes long)
                        if (isChangingResolution) {{ setTimeout(() => {{ if (isChangingResolution) {{ isChangingResolution = false; enableControls(); updateStatus(); }} }}, 7000); }} // Increased timeout
                    }});
            }}

            function toggleRecording() {{
                if (isChangingResolution || isTogglingRecording || isChangingAwb || isPoweringDown) return; // <<< Check AWB flag
                isTogglingRecording = true; disableControls(); statusElement.textContent = 'Sending record command...'; errorElement.textContent = ''; errorElement.style.display = 'none';
                fetch('/toggle_recording', {{ method: 'POST' }})
                    .then(response => {{ if (!response.ok) {{ throw new Error(`HTTP error! Status: ${{response.status}}`); }} return response.json(); }})
                    .then(data => {{
                        if (data.success) {{
                            currentDigitalRecordState = data.digital_recording_active; updateRecordButtonState(); statusElement.textContent = `Digital recording ${{currentDigitalRecordState ? 'enabled' : 'disabled'}}. State updating...`; setTimeout(updateStatus, 1500); // Update status soon after
                        }} else {{
                            errorElement.textContent = `Error toggling recording: ${{data.message || 'Unknown error.'}}`; errorElement.style.display = 'block'; statusElement.textContent = 'Record command failed.'; setTimeout(updateStatus, 1000); // Update status anyway
                        }}
                    }})
                    .catch(err => {{ console.error("Error toggling recording:", err); errorElement.textContent = `Network error toggling recording: ${{err.message}}`; errorElement.style.display = 'block'; statusElement.textContent = 'Command failed (Network).'; setTimeout(updateStatus, 1000); }})
                    .finally(() => {{ isTogglingRecording = false; enableControls(); }}); // Enable controls when done
            }}

            // <<< NEW: Function to change AWB mode >>>
            function changeAwbMode() {{
                 if (isChangingResolution || isTogglingRecording || isChangingAwb || isPoweringDown) return;
                 const selectedMode = awbSelectElement.value;
                 console.log(`Requesting AWB change to: ${{selectedMode}}`);
                 isChangingAwb = true; disableControls(); statusElement.textContent = `Setting AWB to ${{selectedMode}}...`; errorElement.textContent = ''; errorElement.style.display = 'none';

                 fetch('/set_awb_mode', {{
                    method: 'POST',
                    headers: {{ 'Content-Type': 'application/json' }},
                    body: JSON.stringify({{ awb_mode: selectedMode }})
                 }})
                 .then(response => response.json().then(data => ({{ status: response.status, body: data }})))
                 .then(({{ status, body }}) => {{
                     if (status === 200 && body.success) {{
                         statusElement.textContent = `AWB mode set to ${{body.current_awb_mode}}.`;
                         awbStatusElement.textContent = body.current_awb_mode; // Update status display
                         console.log("AWB change successful.");
                     }} else {{
                         errorElement.textContent = `Error setting AWB: ${{body.message || 'Unknown error.'}}`; errorElement.style.display = 'block'; statusElement.textContent = 'AWB change failed.';
                         // Revert dropdown if failed? Optional, but might be less confusing
                         // updateStatus(); // Fetch current status to maybe reset dropdown
                     }}
                 }})
                 .catch(err => {{
                     console.error("Network error setting AWB:", err); errorElement.textContent = `Network error setting AWB: ${{err.message}}`; errorElement.style.display = 'block'; statusElement.textContent = 'AWB change failed (Network).';
                     // updateStatus(); // Fetch current status to maybe reset dropdown
                 }})
                 .finally(() => {{
                     isChangingAwb = false;
                     enableControls();
                     // Update status after a short delay to confirm
                     setTimeout(updateStatus, 1000);
                 }});
            }}
            // <<< END NEW AWB function >>>

            function powerDown() {{
                if (isChangingResolution || isTogglingRecording || isChangingAwb || isPoweringDown) return; // <<< Check AWB flag
                if (!confirm("Are you sure you want to power down the Raspberry Pi? This will stop the camera service and reboot.")) {{ return; }}
                isPoweringDown = true; disableControls(true); statusElement.textContent = 'Powering down... Signalling service to stop.'; errorElement.textContent = ''; errorElement.style.display = 'none';
                if (statusUpdateInterval) clearInterval(statusUpdateInterval);
                fetch('/power_down', {{ method: 'POST' }}) // Request backend to set flags
                    .then(response => {{ if (!response.ok) {{ return response.json().then(data => {{ throw new Error(data.message || `HTTP error! Status: ${{response.status}}`); }}).catch(() => {{ throw new Error(`HTTP error! Status: ${{response.status}}`); }}); }} return response.json(); }})
                    .then(data => {{ if (data.success) {{ statusElement.textContent = 'Shutdown initiated. Reboot will occur shortly.'; /* Backend handles reboot */ }} else {{ errorElement.textContent = `Shutdown request failed: ${{data.message || 'Unknown error.'}}`; errorElement.style.display = 'block'; statusElement.textContent = 'Shutdown failed.'; isPoweringDown = false; enableControls(); }} }}) // Re-enable if failed
                    .catch(err => {{ console.error("Error sending power down command:", err); errorElement.textContent = `Error initiating shutdown: ${{err.message}}.`; errorElement.style.display = 'block'; statusElement.textContent = 'Shutdown error.'; isPoweringDown = false; enableControls(); }}); // Re-enable on error
            }}

            function handleStreamError() {{
                console.warn("Stream image 'onerror' event triggered."); if (streamErrorTimeout || isPoweringDown) return; statusElement.textContent = 'Stream interrupted. Attempting reload...'; streamErrorTimeout = setTimeout(() => {{ streamImage.src = "{{{{ url_for('video_feed') }}}}?" + Date.now(); streamErrorTimeout = null; setTimeout(updateStatus, 1000); }}, 3000);
            }}

            function handleStreamLoad() {{
                if (streamErrorTimeout) {{ clearTimeout(streamErrorTimeout); streamErrorTimeout = null; if (!isPoweringDown) statusElement.textContent = 'Stream active.'; }}
                // If resolution change was in progress, re-enable controls after stream reloads
                if (isChangingResolution) {{
                    console.log("Stream reloaded after potential resolution change.");
                    isChangingResolution = false;
                    enableControls();
                    updateStatus(); // Ensure status is up-to-date
                }}
            }}

            document.addEventListener('DOMContentLoaded', () => {{
                updateRecordButtonState(); updateStatus(); statusUpdateInterval = setInterval(() => {{ if (!isChangingResolution && !isTogglingRecording && !isChangingAwb && !isPoweringDown) {{ updateStatus(); }} }}, 5000); // <<< Check AWB flag
            }});

            window.addEventListener('beforeunload', () => {{
                if (statusUpdateInterval) clearInterval(statusUpdateInterval);
            }});
        </script>
    </body>
    </html>
    """, resolution_text=resolution_text, current_w=current_w, current_h=current_h,
       err_msg=err_msg, digital_rec_state_initial=digital_rec_state_initial,
       batt_text_initial=batt_text_initial,
       # <<< Pass AWB related variables to template >>>
       current_awb_mode_name_initial=current_awb_mode_name_initial,
       awb_options_html=awb_options_html
    )

@app.route('/set_awb_mode', methods=['POST'])
def set_awb_mode():
    global picam2, last_error, current_awb_mode, config_lock

    if not picam2 or not picam2.started:
        return jsonify({'success': False, 'message': 'Camera not available.'}), 503

    try:
        data = request.get_json()
        if not data or 'awb_mode' not in data:
             logging.warning("/set_awb_mode called without 'awb_mode' data.")
             return jsonify({'success': False, 'message': 'Missing awb_mode in request.'}), 400

        requested_mode_name = data['awb_mode']
        logging.info(f"Web request: Set AWB mode to '{requested_mode_name}'")

        # Find the corresponding enum value
        new_awb_enum = None
        for mode_enum in controls.AwbModeEnum:
            if mode_enum.name == requested_mode_name:
                new_awb_enum = mode_enum
                break

        if new_awb_enum is None:
             logging.error(f"Invalid AWB mode requested: {requested_mode_name}")
             return jsonify({'success': False, 'message': f'Invalid AWB mode name: {requested_mode_name}.'}), 400

        # Apply the control change immediately
        with config_lock:
             picam2.set_controls({"AwbMode": new_awb_enum, "AwbEnable": True})
             current_awb_mode = new_awb_enum # Update global state
             last_error = None # Clear potential previous errors
             logging.info(f"Successfully set AWB mode to: {current_awb_mode.name}")

        # Give AWB some time to adjust
        time.sleep(0.5)

        return jsonify({'success': True, 'message': f'AWB mode set to {current_awb_mode.name}.', 'current_awb_mode': current_awb_mode.name})

    except Exception as e:
        logging.error(f"Error setting AWB mode: {e}", exc_info=True)
        last_error = f"AWB Set Error: {e}"
        return jsonify({'success': False, 'message': f'Failed to set AWB mode: {e}'}), 500
    
@app.route("/video_feed")
def video_feed():
    logging.info("Client connected to video feed.")
    return Response(generate_stream_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route("/status")
def status():
    global last_error, digital_recording_active, is_recording, battery_percentage, config_lock
    global video_writers, recording_paths, current_awb_mode # <<< Added current_awb_mode here

    status_text = "Streaming"
    rec_stat_detail = ""
    current_w, current_h = get_current_resolution()
    batt_perc = None
    current_digital_state = False
    awb_mode_name = "Unknown" # Default

    with config_lock:
        current_digital_state = digital_recording_active
        batt_perc = battery_percentage
        current_is_recording = is_recording
        current_recording_paths = list(recording_paths)
        # <<< NEW: Safely get AWB mode name >>>
        try:
             awb_mode_name = current_awb_mode.name
        except AttributeError:
             logging.warning("current_awb_mode might not be initialized correctly yet.")
             awb_mode_name = "Initializing"
        # <<< END NEW >>>

    if current_is_recording:
        if current_recording_paths:
            rec_stat_detail = f" (Recording to {len(current_recording_paths)} USB(s))"
        else:
            rec_stat_detail = " (ERROR: Recording active but no paths!)"
            logging.warning("Status check found is_recording=True but recording_paths is empty.")
            if not last_error: last_error = "Inconsistent State: Recording active but no paths."
        status_text += rec_stat_detail

    err_msg = last_error if last_error else ""

    # Auto-clear errors logic (remains the same)
    if output_frame is not None and err_msg and ("Init Error" in err_msg or "unavailable" in err_msg or "capture" in err_msg):
            logging.info("Auto-clearing previous camera/capture error as frames are being received.")
            last_error = None; err_msg = ""
    if batt_perc is not None and err_msg and ("Battery Monitor" in err_msg or "INA219" in err_msg or "I2C Error" in err_msg):
            logging.info("Auto-clearing previous battery monitor error as a reading was successful.")
            last_error = None; err_msg = ""

    return jsonify({
        'is_recording': current_is_recording,
        'digital_recording_active': current_digital_state,
        'resolution': f"{current_w}x{current_h}",
        'status_text': status_text,
        'error': err_msg,
        'active_recordings': current_recording_paths,
        'battery_percent': batt_perc,
        'awb_mode': awb_mode_name # <<< NEW: Added AWB mode to status >>>
    })


@app.route("/set_resolution/<direction>", methods=['POST'])
def set_resolution(direction):
    global current_resolution_index, reconfigure_resolution_index, last_error, config_lock
    with config_lock:
        if reconfigure_resolution_index is not None:
            return jsonify({'success': False, 'message': 'Reconfiguration already in progress.'}), 429 # Too Many Requests
        if not (0 <= current_resolution_index < len(SUPPORTED_RESOLUTIONS)):
             logging.error(f"Internal Error: Invalid current resolution index {current_resolution_index} before change!")
             return jsonify({'success': False, 'message': 'Internal state error: Invalid current resolution.'}), 500
        original_index = current_resolution_index
        new_index = current_resolution_index
        if direction == 'up': new_index += 1
        elif direction == 'down': new_index -= 1
        else:
            return jsonify({'success': False, 'message': 'Invalid direction specified.'}), 400
        new_index = max(0, min(len(SUPPORTED_RESOLUTIONS) - 1, new_index))
        if new_index == original_index:
            msg = 'Already at highest resolution.' if direction == 'up' else 'Already at lowest resolution.'
            return jsonify({'success': False, 'message': msg}), 400
        new_w, new_h = SUPPORTED_RESOLUTIONS[new_index]
        logging.info(f"Web request: change resolution index {original_index} -> {new_index} ({new_w}x{new_h})")
        reconfigure_resolution_index = new_index
        last_error = None
        return jsonify({'success': True, 'message': 'Resolution change requested.', 'new_resolution': f"{new_w}x{new_h}"})


@app.route('/toggle_recording', methods=['POST'])
def toggle_recording():
    global digital_recording_active, last_error, config_lock
    new_state = False
    with config_lock:
        digital_recording_active = not digital_recording_active
        new_state = digital_recording_active
        logging.info(f"Digital recording trigger toggled via web UI to: {'ON' if new_state else 'OFF'}")
        if last_error and ("Recording" in last_error or "writers" in last_error or "USB" in last_error or "sync" in last_error):
             logging.info(f"Clearing previous recording error: '{last_error}'")
             last_error = None
    return jsonify({'success': True, 'digital_recording_active': new_state})


# --- MODIFIED Power Down Route (Sets Flags Only) ---
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
    with config_lock: # Ensure thread safety setting flag
        reboot_requested = True
    shutdown_event.set() # Signal all threads to stop

    # Return success immediately to the client
    # The actual reboot happens later in the main thread's cleanup phase
    return jsonify({'success': True, 'message': 'Shutdown initiated. System will reboot after cleanup.'})


# ===========================================================
# === FLASK ROUTES END HERE ===
# ===========================================================

# --- Signal Handling ---
def signal_handler(sig, frame):
    global shutdown_event, reboot_requested # Access reboot flag
    if shutdown_event.is_set():
        logging.warning("Shutdown already in progress, ignoring additional signal.")
        return
    logging.warning(f"Received signal {sig}. Initiating graceful shutdown...")
    # reboot_requested = False # Ensure external signal doesn't trigger reboot
    shutdown_event.set() # Set event


# --- Main Execution (Relying on toggle.sh for disable/reboot) ---
def main():
    global last_error, capture_thread, flask_thread, picam2, shutdown_event, reboot_requested
    global TOGGLE_SCRIPT_NAME # Access script name

    # --- Determine Paths ---
    try:
        # Get the directory where the current Python script is located
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # Construct the full path to the toggle script
        toggle_script_path = os.path.join(script_dir, TOGGLE_SCRIPT_NAME)
        logging.info(f"MAIN: Expecting toggle script at: {toggle_script_path}")
    except NameError:
        # __file__ might not be defined if running interactively, fallback or error
        logging.error("MAIN: Could not determine script directory automatically. Ensure toggle.sh is accessible.")
        toggle_script_path = None # Or provide a default fallback path if needed

    # --- Sudoers Configuration Requirement ---
    # IMPORTANT: sudoers configuration needed for power down via toggle.sh!
    # The user running this script (e.g., 'hecke') needs this line via `sudo visudo`
    # (Replace '/path/to/your/script/dir' with the actual directory):
    # hecke ALL=(ALL) NOPASSWD: /path/to/your/script/dir/toggle.sh disable
    #
    # Ensure toggle.sh has execute permissions: chmod +x /path/to/your/script/dir/toggle.sh

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    logging.info(" --- Starting Camera Stream & Record Service --- ")
    logging.info(f"--- Using Picamera2 and gpiozero ---")
    # ... (other initial log messages remain the same) ...
    logging.info(f"--- Power Down Method: Web UI triggers '{TOGGLE_SCRIPT_NAME} disable' ---")


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
        # reboot_requested persists across restarts until handled or script exits

        # Pre-loop cleanup for picam2
        logging.info(f"MAIN: Checking picam2 pre-loop cleanup block (picam2 is {'set' if picam2 else 'None'})...")
        if picam2:
            logging.warning("MAIN: Found existing picam2 object - attempting cleanup.")
            try:
                if picam2.started: picam2.stop()
                picam2.close()
                logging.info("MAIN: Existing picam2 cleaned up.")
            except Exception as e:
                 logging.error(f"MAIN: Error during picam2 pre-loop cleanup: {e}", exc_info=True)
            finally:
                 picam2 = None
        logging.info("MAIN: Finished picam2 pre-loop cleanup block.")

        # Main setup and execution block
        try:
            logging.info("MAIN: Entering main setup try block...")
            # Initialize Hardware
            logging.info("Initializing Hardware Components...")
            if SWITCH_GPIO_PIN is not None:
                if not setup_gpio(): logging.error(f"GPIO setup failed: {last_error}. Physical switch will be unavailable.")
            else: logging.info("Physical switch disabled (SWITCH_GPIO_PIN not set).")
            if INA219_I2C_ADDRESS is not None:
                if not setup_battery_monitor(): logging.warning(f"Battery monitor setup failed: {last_error}. Battery level unavailable.")
            else: logging.info("Battery monitor disabled (INA219_I2C_ADDRESS not set).")

            # Start Capture Thread
            logging.info("Starting frame capture thread...")
            capture_thread = threading.Thread(target=capture_and_process_loop, name="CaptureThread", daemon=True)
            capture_thread.start()
            time.sleep(4.0) # Give capture thread time to init camera
            if not capture_thread.is_alive(): raise RuntimeError(f"Capture thread failed startup: {last_error or 'Unknown'}")
            if last_error and ("Init Error" in last_error or "failed fatally" in last_error): raise RuntimeError(f"Camera init failed: {last_error}")
            logging.info("Capture thread appears running.")

            # Start Flask Thread
            logging.info(f"Starting Flask web server on 0.0.0.0:{WEB_PORT}...")
            flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=WEB_PORT, debug=False, use_reloader=False, threaded=True), name="FlaskThread", daemon=True)
            flask_thread.start()
            time.sleep(1.5) # Give flask time to start
            if not flask_thread.is_alive(): raise RuntimeError("Flask thread failed startup.")

            # System Running
            logging.info("--- System Running ---")
            try:
                import socket; s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.connect(("8.8.8.8", 80)); local_ip = s.getsockname()[0]; s.close()
                logging.info(f"Access web interface at: http://{local_ip}:{WEB_PORT} (or Pi's IP/hostname)")
            except Exception as ip_e: logging.warning(f"Could not determine IP: {ip_e}"); logging.info(f"Access web interface at: http://<YOUR_PI_IP>:{WEB_PORT}")

            # Inner monitoring loop
            logging.info("MAIN: Entering inner monitoring loop...")
            while not shutdown_event.is_set():
                if not capture_thread.is_alive(): raise RuntimeError(last_error or "Capture thread terminated.")
                if not flask_thread.is_alive(): raise RuntimeError(last_error or "Flask thread terminated.")
                shutdown_event.wait(timeout=5.0) # Wait for shutdown or check threads every 5s

            logging.info("MAIN: Exited inner monitoring loop (shutdown_event received).")

        # Exception handling for setup/runtime errors
        except RuntimeError as e:
            logging.error(f"!!! Runtime Error in Main Loop: {e}")
            logging.error("Attempting restart after 10 seconds...")
            shutdown_event.set() # Signal threads if possible
            if capture_thread and capture_thread.is_alive(): capture_thread.join(timeout=3.0)
            shutdown_event.clear(); # Reset flags for restart
            # Keep reboot_requested flag if set before error
            time.sleep(10.0)
        except Exception as e:
            logging.exception(f"!!! Unhandled Exception in Main Loop: {e}")
            logging.error("Attempting restart after 10 seconds...")
            shutdown_event.set();
            if capture_thread and capture_thread.is_alive(): capture_thread.join(timeout=3.0)
            shutdown_event.clear(); # Reset flags for restart
            # Keep reboot_requested flag if set before error
            time.sleep(10.0)


    # --- Final Cleanup (Only reached when shutdown_event is set and outer loop finishes) ---
    logging.info("--- Shutdown initiated ---")

    # Wait for capture thread (handles its own cleanup including sync)
    if capture_thread and capture_thread.is_alive():
        logging.info("Waiting for capture thread to exit...")
        capture_thread.join(timeout=15.0)
        if capture_thread.is_alive(): logging.warning("Capture thread did not exit cleanly within timeout.")
        else: logging.info("Capture thread finished.")

    # Flask thread is daemon, exits automatically.

    # Ensure recording stopped / camera closed (belt-and-braces)
    if is_recording: logging.warning("Force stopping recording during final shutdown."); stop_recording()
    if picam2:
        try:
            logging.info("Ensuring Picamera2 closed...");
            if picam2.started: picam2.stop()
            picam2.close()
        except Exception as e: logging.warning(f"Error during final Picamera2 close: {e}")

    # Cleanup GPIO
    if SWITCH_GPIO_PIN is not None: cleanup_gpio()

    # --- Execute toggle.sh disable IF reboot was requested from web UI ---
    with config_lock:
        reboot_flag = reboot_requested

    logging.info(f"MAIN: Checking if reboot was requested via web UI: {reboot_flag}")
    if reboot_flag:
        logging.warning("Reboot requested via web UI. Executing disable script...")

        if toggle_script_path and os.path.exists(toggle_script_path):
            # Optional: Check for execute permissions
            if not os.access(toggle_script_path, os.X_OK):
                 logging.error(f"MAIN: Toggle script '{toggle_script_path}' exists but is not executable! Run 'chmod +x {toggle_script_path}'.")
                 last_error = "Disable script not executable."
            else:
                disable_command = ["sudo", toggle_script_path, "disable"]
                logging.info(f"MAIN: Executing: {' '.join(disable_command)}")
                try:
                    # Launch the disable script. It handles everything including the reboot.
                    subprocess.Popen(disable_command)
                    logging.warning("MAIN: Disable script launched (which includes reboot). Service will now exit.")
                    # Give the script a moment to start before Python fully exits
                    time.sleep(5)
                except FileNotFoundError:
                     logging.critical(f"!!! MAIN: FAILED TO EXECUTE '{' '.join(disable_command)}'. 'sudo' command not found or script path incorrect despite existence check?", exc_info=True)
                     last_error = f"Disable Script Failed: Command not found."
                except PermissionError:
                     logging.critical(f"!!! MAIN: FAILED TO EXECUTE '{' '.join(disable_command)}'. Permission denied. Check sudoers configuration.", exc_info=True)
                     last_error = f"Disable Script Failed: Permission denied (sudoers)."
                except Exception as e:
                    logging.critical(f"!!! MAIN: FAILED TO EXECUTE DISABLE SCRIPT: {e}", exc_info=True)
                    last_error = f"Disable Script Failed: {e}"
        else:
            logging.error(f"MAIN: Disable script path not determined or script not found at '{toggle_script_path}'. Cannot execute proper disable sequence.")
            last_error = "Disable script not found."

    # Only log Program Exit if reboot wasn't issued (or failed)
    if not reboot_flag:
        logging.info("--- Program Exit (Shutdown Complete) ---")
    else:
         # This might not always be logged if reboot is fast
         logging.info("--- Program Exit (Rebooting via Disable Script) ---")

if __name__ == "__main__":
    # --- Main Execution ---
    main()