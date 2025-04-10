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
import numpy as np # <<< Add numpy import

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

AVAILABLE_AWB_MODES = list(controls.AwbModeEnum.__members__.keys()) # Use __members__.keys()
DEFAULT_AWB_MODE_NAME = "Auto"
# Ensure the default value exists in the retrieved keys, otherwise fallback needed
if DEFAULT_AWB_MODE_NAME not in AVAILABLE_AWB_MODES:
    DEFAULT_AWB_MODE_NAME = AVAILABLE_AWB_MODES[0] if AVAILABLE_AWB_MODES else "Auto" # Fallback safely
current_awb_mode = getattr(controls.AwbModeEnum, DEFAULT_AWB_MODE_NAME)

AVAILABLE_AE_MODES = list(controls.AeExposureModeEnum.__members__.keys()) # Use __members__.keys()
DEFAULT_AE_MODE_NAME = "Normal"
if DEFAULT_AE_MODE_NAME not in AVAILABLE_AE_MODES:
     DEFAULT_AE_MODE_NAME = AVAILABLE_AE_MODES[0] if AVAILABLE_AE_MODES else "Normal"
current_ae_mode = getattr(controls.AeExposureModeEnum, DEFAULT_AE_MODE_NAME)

AVAILABLE_METERING_MODES = list(controls.AeMeteringModeEnum.__members__.keys()) # Use __members__.keys()
DEFAULT_METERING_MODE_NAME = "CentreWeighted"
if DEFAULT_METERING_MODE_NAME not in AVAILABLE_METERING_MODES:
     DEFAULT_METERING_MODE_NAME = AVAILABLE_METERING_MODES[0] if AVAILABLE_METERING_MODES else "CentreWeighted"
current_metering_mode = getattr(controls.AeMeteringModeEnum, DEFAULT_METERING_MODE_NAME)

# picam2.set_controls({"NoiseReductionMode": controls.draft.NoiseReductionModeEnum.Fast})
#this causes issues like it doesnt exist. may need to look into dependances before implementing
AVAILABLE_NOISE_REDUCTION_MODES = list(controls.draft.NoiseReductionModeEnum.__members__.keys()) # Use __members__.keys()
DEFAULT_NOISE_REDUCTION_MODE_NAME = "Fast"
if DEFAULT_NOISE_REDUCTION_MODE_NAME not in AVAILABLE_NOISE_REDUCTION_MODES:
     DEFAULT_NOISE_REDUCTION_MODE_NAME = AVAILABLE_NOISE_REDUCTION_MODES[0] if AVAILABLE_NOISE_REDUCTION_MODES else "Fast"
current_noise_reduction_mode = getattr(controls.draft.NoiseReductionModeEnum, DEFAULT_NOISE_REDUCTION_MODE_NAME)

DEFAULT_BRIGHTNESS = 0.0
current_brightness = DEFAULT_BRIGHTNESS
MIN_BRIGHTNESS = -1.0
MAX_BRIGHTNESS = 1.0
STEP_BRIGHTNESS = 0.1

DEFAULT_CONTRAST = 1.0
current_contrast = DEFAULT_CONTRAST
MIN_CONTRAST = 0.0
MAX_CONTRAST = 2.0 # Allow a bit more range
STEP_CONTRAST = 0.1

DEFAULT_SATURATION = 1.0
current_saturation = DEFAULT_SATURATION
MIN_SATURATION = 0.0
MAX_SATURATION = 2.0 # Allow a bit more range
STEP_SATURATION = 0.1

DEFAULT_SHARPNESS = 1.0
current_sharpness = DEFAULT_SHARPNESS
MIN_SHARPNESS = 0.0
MAX_SHARPNESS = 2.0 # Allow a bit more range
STEP_SHARPNESS = 0.1

undistort_active = False
# --- CAMERA CALIBRATION DATA (PLACEHOLDERS - REPLACE WITH YOUR VALUES!) ---
# Example K (3x3 Camera Matrix - Needs YOUR values)
CAMERA_MATRIX_K = np.array([[1000.,    0.,  820.], # fx, 0, cx
                           [   0., 1000.,  616.], # 0, fy, cy
                           [   0.,    0.,    1.]]) # 0, 0, 1
# Example D (Distortion Coefficients [k1, k2, p1, p2, k3] - Needs YOUR values for 160deg lens)
DISTORTION_COEFF_D = np.array([[-0.1, 0.01, 0., 0., 0.]]) 
# --- Undistortion Map Variables (Calculated later) ---
undistort_map1 = None
undistort_map2 = None
undistort_optimal_matrix = None

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

def prepare_undistort_maps(width, height):
    """Calculates the undistortion maps for cv2.remap based on global K and D"""
    global undistort_map1, undistort_map2, undistort_optimal_matrix, CAMERA_MATRIX_K, DISTORTION_COEFF_D
    
    if CAMERA_MATRIX_K is None or DISTORTION_COEFF_D is None:
         logging.error("Cannot prepare undistort maps: CAMERA_MATRIX_K or DISTORTION_COEFF_D is not set.")
         undistort_map1 = None
         undistort_map2 = None
         return False

    logging.info(f"Preparing undistort maps for {width}x{height}...")
    try:
        # Get the optimal new camera matrix (alpha=0 crops the black borders)
        # If you want to keep all pixels, try alpha=1, but you'll get black areas/curves
        alpha = 0 
        undistort_optimal_matrix, roi = cv2.getOptimalNewCameraMatrix(CAMERA_MATRIX_K, DISTORTION_COEFF_D, (width, height), alpha, (width, height))
        
        if undistort_optimal_matrix is None:
             raise ValueError("getOptimalNewCameraMatrix failed to return a matrix.")

        # Calculate the undistortion mapping
        # CV_32FC1 means the maps use 32-bit floats and have 1 channel
        undistort_map1, undistort_map2 = cv2.initUndistortRectifyMap(
            CAMERA_MATRIX_K, 
            DISTORTION_COEFF_D, 
            None, # R (Rectification transform) - None for monocular
            undistort_optimal_matrix, 
            (width, height), 
            cv2.CV_32FC1 # Map type
        )
        logging.info("Undistort maps prepared successfully.")
        return True
    except Exception as e:
        logging.error(f"!!! Failed to prepare undistort maps: {e}", exc_info=True)
        undistort_map1 = None
        undistort_map2 = None
        undistort_optimal_matrix = None
        return False
    
# --- Initialize Camera ---
def initialize_camera(target_width, target_height):
    # <<< Update global list to include all control variables >>>
    global picam2, last_error, config_lock
    global current_awb_mode, current_ae_mode, current_metering_mode, current_noise_reduction_mode
    global current_brightness, current_contrast, current_saturation, current_sharpness

    logging.info(f"Attempting to initialize camera with Picamera2 at {target_width}x{target_height}...")

    # --- Stop/Close existing instance ---
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
        time.sleep(0.5) # Give time for resources to release

    # --- Create and Configure New Instance ---
    try:
        # --- Tuning File ---
        tuning = None
        if USE_NOIR_TUNING:
            if os.path.exists(NOIR_TUNING_FILE_PATH):
                try:
                    tuning = Picamera2.load_tuning_file(NOIR_TUNING_FILE_PATH)
                    logging.info(f"Loading NoIR tuning from: {NOIR_TUNING_FILE_PATH}")
                except Exception as e:
                    logging.error(f"!!! Failed to load tuning file '{NOIR_TUNING_FILE_PATH}': {e}. Using default tuning.", exc_info=False)
                    tuning = None # Explicitly set back to None on failure
            else:
                logging.warning(f"NoIR tuning file not found at {NOIR_TUNING_FILE_PATH}. Using default tuning.")
        else:
                logging.info("Using default tuning (NoIR tuning disabled).")

        picam2 = Picamera2(tuning=tuning)
        logging.info("Picamera2 object created.")

        # --- Determine Target FPS ---
        target_fps = get_target_fps(target_width, target_height)
        logging.info(f"Targeting FPS: {target_fps:.1f} for resolution {target_width}x{target_height}")

        # --- Get current control values safely ---
        # (Using global variables set earlier)
        with config_lock:
            awb_mode_to_set = current_awb_mode
            ae_mode_to_set = current_ae_mode
            metering_mode_to_set = current_metering_mode
            noise_reduction_mode_to_set = current_noise_reduction_mode
            brightness_to_set = current_brightness
            contrast_to_set = current_contrast
            saturation_to_set = current_saturation
            sharpness_to_set = current_sharpness

        # --- Create Video Configuration with ALL controls ---
        config = picam2.create_video_configuration(
            main={"size": (target_width, target_height), "format": "RGB888"},
            controls={
                # Core controls
                "FrameRate": target_fps,
                "NoiseReductionMode": noise_reduction_mode_to_set,

                # Auto Exposure Controls
                "AeEnable": True, # Assuming AE should generally be enabled
                "AeExposureMode": ae_mode_to_set,
                "AeMeteringMode": metering_mode_to_set,
                # "ExposureTime": # Only if AeEnable is False or AeExposureMode is Custom
                # "AnalogueGain": # Only if AeEnable is False

                # Auto White Balance Controls
                "AwbEnable": True, # Assuming AWB should generally be enabled
                "AwbMode": awb_mode_to_set,
                # "ColourGains": # Only if AwbEnable is False

                # Image Adjustment Controls
                "Brightness": brightness_to_set,
                "Contrast": contrast_to_set,
                "Saturation": saturation_to_set,
                "Sharpness": sharpness_to_set,
                # Other possible controls: ExposureValue (EV compensation), ColourCorrectionMatrix, ScalerCrop etc.
            }
        )
        logging.info(f"Configuring Picamera2 with: main={config['main']}, controls={config['controls']}")

        picam2.configure(config)
        time.sleep(0.5) # Give driver time to settle configuration

        # --- Verify Applied Configuration ---
        new_config = picam2.camera_configuration()
        if not new_config:
            logging.warning("Could not get camera configuration after applying.")
        else:
            applied_controls = new_config.get('controls', {})
            applied_main = new_config.get('main', {})
            logging.info(f"Applied Config: main={applied_main}, controls={applied_controls}")
            # Optional: Log differences if needed (e.g., FrameRate, AwbMode)
            if applied_controls.get('FrameRate') != target_fps:
                 logging.warning(f"Camera driver adjusted FrameRate from {target_fps} to {applied_controls.get('FrameRate')}")
            # ... add checks for other modes if necessary ...


        logging.info("Configuration successful!")

        # --- Start Camera ---
        picam2.start()
        logging.info("Camera started")
        # Allow more time for sensor settings (like AWB, AE) to stabilize
        time.sleep(2.0)

        # --- Log Final Running Configuration ---
        actual_config = picam2.camera_configuration()
        if not actual_config:
                raise RuntimeError("Failed to get camera configuration after start.")

        actual_format = actual_config.get('main', {})
        actual_w = actual_format.get('size', (0,0))[0]
        actual_h = actual_format.get('size', (0,0))[1]
        actual_fmt_str = actual_format.get('format', 'Unknown')
        actual_fps = actual_config.get('controls', {}).get('FrameRate', 'N/A')
        actual_awb = actual_config.get('controls', {}).get('AwbMode', 'N/A')
        # ... get other actual settings if desired for logging ...
        logging.info(f"Picamera2 initialized. Actual main stream: {actual_w}x{actual_h} {actual_fmt_str} @ {actual_fps} fps. AWB Mode: {actual_awb}")

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

# --- Capture Loop Function ---
def capture_and_process_loop():
    global output_frame, is_recording, last_error, picam2, switch, digital_recording_active
    global current_resolution_index, reconfigure_resolution_index, last_battery_read_time
    global video_writers, recording_paths, config_lock 
    global undistort_active, undistort_map1, undistort_map2, last_prepared_undistort_size # Added undistortion vars

    logging.info("Starting frame capture loop...")
    consecutive_error_count = 0
    max_consecutive_errors = 15
    # Removed last_prepared_undistort_size init from here, handled in global scope

    # --- Initial Camera Setup ---
    width, height = get_current_resolution()
    if not initialize_camera(width, height):
        logging.error("Initial camera setup failed. Capture thread cannot start.")
        return 

    # --- Prepare initial undistort maps if active ---
    if undistort_active:
        if prepare_undistort_maps(width, height):
            last_prepared_undistort_size = (width, height)
        else:
            logging.error("Failed initial undistort map preparation. Disabling feature.")
            undistort_active = False # Disable if prep fails

    # --- Main Loop ---
    loop_iteration = 0 
    while not shutdown_event.is_set():
        loop_iteration += 1
        logging.debug(f"--- Capture Loop Iteration {loop_iteration} Start ---") 
        loop_start_time = time.monotonic()

        try:
            # --- Check for Reconfiguration Request ---
            target_index = -1
            logging.debug(f"[{loop_iteration}] Checking reconfigure_resolution_index...") 
            reconfig_check_value = None 
            logging.debug(f"[{loop_iteration}] Attempting to acquire config_lock for reconfig check.")
            with config_lock: 
                 logging.debug(f"[{loop_iteration}] Acquired config_lock for reconfig check.") 
                 reconfig_check_value = reconfigure_resolution_index 
            logging.debug(f"[{loop_iteration}] Released config_lock. reconfigure_resolution_index = {reconfig_check_value}") 

            if reconfig_check_value is not None:
                logging.debug(f"[{loop_iteration}] Attempting to acquire config_lock to clear reconfig index.")
                with config_lock:
                    logging.debug(f"[{loop_iteration}] Acquired config_lock to clear reconfig index.")
                    if reconfigure_resolution_index is not None: 
                        target_index = reconfigure_resolution_index
                        reconfigure_resolution_index = None 
                    else:
                         target_index = -1 
                logging.debug(f"[{loop_iteration}] Released config_lock after clearing reconfig index. Target index set to {target_index}")
            else:
                 target_index = -1


            # --- Handle Reconfiguration ---
            if target_index != -1:
                logging.info(f"[{loop_iteration}] --- Reconfiguring resolution to index {target_index} ---")
                physical_switch_on_before = (switch is not None and switch.is_pressed)
                digital_switch_on_before = digital_recording_active
                should_be_recording_after = physical_switch_on_before or digital_switch_on_before
                was_actually_recording = is_recording

                if was_actually_recording:
                    logging.info(f"[{loop_iteration}] Stopping recording for reconfiguration...")
                    stop_recording()

                new_width, new_height = SUPPORTED_RESOLUTIONS[target_index]
                if initialize_camera(new_width, new_height): 
                    with config_lock:
                        current_resolution_index = target_index 
                    logging.info(f"[{loop_iteration}] --- Reconfiguration successful to {new_width}x{new_height} ---")
                    
                    # Re-prepare undistort maps if active 
                    if undistort_active:
                        logging.info(f"[{loop_iteration}] Re-preparing undistort maps for new resolution.")
                        if prepare_undistort_maps(new_width, new_height):
                            last_prepared_undistort_size = (new_width, new_height)
                        else:
                             logging.error("Failed undistort map preparation after reconfig. Disabling feature.")
                             undistort_active = False 
                    else: # Ensure maps are cleared if undistort is not active
                         undistort_map1 = None
                         undistort_map2 = None
                         last_prepared_undistort_size = None

                    # Resume recording if needed
                    if should_be_recording_after:
                        logging.info(f"[{loop_iteration}] Resuming recording after successful reconfiguration...")
                        time.sleep(1.0) 
                        if not start_recording():
                            logging.error(f"[{loop_iteration}] Failed to restart recording after reconfiguration!")
                else:
                     # Reconfiguration failed! Attempt to restore previous resolution
                     logging.error(f"[{loop_iteration}] !!! Failed reconfigure to index {target_index}. Attempting to restore previous resolution... !!!")
                     prev_width, prev_height = get_current_resolution() 
                     if not initialize_camera(prev_width, prev_height):
                        logging.critical(f"[{loop_iteration}] !!! Failed to restore previous camera resolution after failed reconfig. Stopping service. !!!")
                        last_error = "Camera failed fatally during reconfig restore."
                        shutdown_event.set(); break 
                     else:
                        logging.info(f"[{loop_iteration}] Successfully restored previous camera resolution.")
                        # Ensure maps are correct/cleared after failed reconfig too
                        if undistort_active:
                             prev_w_restored, prev_h_restored = get_current_resolution() # Should match prev_width, prev_height
                             if last_prepared_undistort_size != (prev_w_restored, prev_h_restored):
                                  logging.info(f"[{loop_iteration}] Re-preparing undistort maps for restored resolution.")
                                  if prepare_undistort_maps(prev_w_restored, prev_h_restored):
                                       last_prepared_undistort_size = (prev_w_restored, prev_h_restored)
                                  else:
                                       logging.error("Failed map prep after failed reconfig. Disabling feature.")
                                       undistort_active = False
                        else:
                             undistort_map1 = None
                             undistort_map2 = None
                             last_prepared_undistort_size = None
                        # Try restarting recording if it was on before
                        if should_be_recording_after:
                            logging.info(f"[{loop_iteration}] Attempting recording restart with restored resolution...")
                            time.sleep(1.0)
                            if not start_recording():
                                logging.error(f"[{loop_iteration}] Failed to restart recording after failed reconfig and restore.")


                logging.info(f"[{loop_iteration}] --- Finished handling reconfiguration request ---")
                continue 

            # --- Check if Camera is Available (if not reconfiguring) ---
            logging.debug(f"[{loop_iteration}] Checking camera availability (picam2 object)...") 
            is_cam_available = False
            cam_status_details = "Unknown"
            if picam2 is not None:
                logging.debug(f"[{loop_iteration}] picam2 object exists.")
                try:
                    if picam2.started:
                        logging.debug(f"[{loop_iteration}] picam2.started is True.")
                        is_cam_available = True
                        cam_status_details = "Exists and Started"
                    else:
                         logging.warning(f"[{loop_iteration}] picam2 object exists BUT picam2.started is False.")
                         cam_status_details = "Exists but Not Started"
                except Exception as cam_check_e:
                     logging.error(f"[{loop_iteration}] Error checking picam2.started: {cam_check_e}")
                     cam_status_details = f"Exists but Error checking started: {cam_check_e}"
            else:
                 logging.error(f"[{loop_iteration}] picam2 object is None!")
                 cam_status_details = "picam2 is None"
            logging.debug(f"[{loop_iteration}] Camera status details: {cam_status_details}")

            if not is_cam_available:
                if not last_error: last_error = "Picamera2 became unavailable unexpectedly."
                logging.error(f"[{loop_iteration}] Camera unavailable based on check: {last_error}. Attempting reinitialization...")
                width, height = get_current_resolution()
                if initialize_camera(width, height):
                    logging.info(f"[{loop_iteration}] Camera re-initialized successfully after unexpected stop.")
                    last_error = None; consecutive_error_count = 0
                     # Re-prepare maps after successful re-init if needed
                    if undistort_active:
                         if prepare_undistort_maps(width, height):
                              last_prepared_undistort_size = (width, height)
                         else:
                              logging.error("Failed map prep after re-init. Disabling feature.")
                              undistort_active = False
                else:
                    logging.error(f"[{loop_iteration}] Camera re-initialization failed: {last_error}. Stopping capture loop.")
                    shutdown_event.set(); break
                continue 
            else:
                 logging.debug(f"[{loop_iteration}] Camera check passed.") 

            # --- Capture Frame ---
            logging.debug(f"[{loop_iteration}] Attempting to capture frame...") 
            frame_bgr = picam2.capture_array("main")
            logging.debug(f"[{loop_iteration}] capture_array call completed (frame is {'None' if frame_bgr is None else 'Valid'}).") 

            if frame_bgr is None:
                logging.warning(f"[{loop_iteration}] Failed to capture frame (capture_array returned None). Retrying...")
                consecutive_error_count += 1
                if consecutive_error_count > max_consecutive_errors:
                    last_error = f"Failed capture {max_consecutive_errors} consecutive times. Assuming camera failure."
                    logging.error(last_error); shutdown_event.set(); break
                time.sleep(0.1); continue 
            else:
                 if consecutive_error_count > 0:
                      logging.info(f"[{loop_iteration}] Recovered frame grab after {consecutive_error_count} errors.")
                 consecutive_error_count = 0
            
            # --- Apply Undistortion if Active --- 
            processed_frame = frame_bgr 
            if undistort_active and undistort_map1 is not None and undistort_map2 is not None:
                 current_frame_size = (processed_frame.shape[1], processed_frame.shape[0]) # W, H
                 if current_frame_size == last_prepared_undistort_size:
                     logging.debug(f"[{loop_iteration}] Applying undistortion remap...")
                     remap_start_time = time.monotonic()
                     processed_frame = cv2.remap(frame_bgr, undistort_map1, undistort_map2, cv2.INTER_LINEAR)
                     remap_duration = (time.monotonic() - remap_start_time) * 1000
                     logging.debug(f"[{loop_iteration}] Remap completed in {remap_duration:.1f} ms.")
                 else:
                      logging.warning(f"[{loop_iteration}] Undistort active but maps size {last_prepared_undistort_size} != frame size {current_frame_size}. Skipping undistort.")

            # --- Check Recording Trigger ---
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

            # --- Start/Stop Recording Logic ---
            if should_be_recording and not is_recording:
                log_msg = "Physical switch ON" if physical_switch_on else ""
                if digital_switch_on: log_msg += (" / " if log_msg else "") + "Digital trigger ON"
                logging.info(f"Recording trigger active ({log_msg}) - initiating start.")
                if not start_recording():
                    logging.error("Attempt to start recording failed.")
            elif not should_be_recording and is_recording:
                logging.info("Recording trigger(s) OFF - initiating stop.")
                stop_recording()

            # --- Write Frame if Recording ---
            if is_recording:
                if not video_writers:
                    logging.warning("Inconsistent state: is_recording=True, but no video writers. Forcing stop.")
                    last_error = "Rec stopped: writer list was empty."
                    stop_recording() 
                else:
                    write_errors = 0
                    for i, writer in enumerate(video_writers):
                        try:
                            logging.debug(f"Writing frame to writer {i}") 
                            writer.write(processed_frame) # Use potentially undistorted frame
                        except Exception as e:
                            path_str = recording_paths[i] if i < len(recording_paths) else f"Writer {i}"
                            logging.error(f"!!! Failed to write frame to {path_str}: {e}")
                            write_errors += 1
                            last_error = f"Frame write error: {os.path.basename(path_str)}"
                    if write_errors > 0 and write_errors == len(video_writers):
                         logging.error("All active video writers failed to write frame. Stopping recording.")
                         last_error = "Rec stopped: All writers failed write."
                         stop_recording() 

            # --- Update Output Frame for Streaming ---
            with frame_lock:
                output_frame = processed_frame.copy() # Use potentially undistorted frame

            # --- Periodic Battery Check ---
            if ina219_sensor and (loop_start_time - last_battery_read_time > BATTERY_READ_INTERVAL):
                    read_battery_level()
                    last_battery_read_time = loop_start_time

        except Exception as e:
            logging.exception(f"!!! [{loop_iteration}] Unexpected Error in capture loop: {e}") 
            last_error = f"Capture Loop Error: {e}"
            consecutive_error_count += 1
            if consecutive_error_count > max_consecutive_errors / 2: 
                logging.error(f"[{loop_iteration}] Too many consecutive errors ({consecutive_error_count}). Signaling shutdown.")
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
# --- End capture_and_process_loop ---


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

@app.route("/")
def index():
    # <<< Update global list >>>
    global last_error, digital_recording_active, battery_percentage, config_lock
    global current_awb_mode, AVAILABLE_AWB_MODES, DEFAULT_AWB_MODE_NAME
    global current_ae_mode, AVAILABLE_AE_MODES, DEFAULT_AE_MODE_NAME
    global current_metering_mode, AVAILABLE_METERING_MODES, DEFAULT_METERING_MODE_NAME
    global current_noise_reduction_mode, AVAILABLE_NOISE_REDUCTION_MODES, DEFAULT_NOISE_REDUCTION_MODE_NAME
    global current_brightness, MIN_BRIGHTNESS, MAX_BRIGHTNESS, STEP_BRIGHTNESS
    global current_contrast, MIN_CONTRAST, MAX_CONTRAST, STEP_CONTRAST
    global current_saturation, MIN_SATURATION, MAX_SATURATION, STEP_SATURATION
    global current_sharpness, MIN_SHARPNESS, MAX_SHARPNESS, STEP_SHARPNESS
    global undistort_active 

    current_w, current_h = get_current_resolution()
    resolution_text = f"{current_w}x{current_h}"
    err_msg = last_error if last_error else ""

    # <<< Get initial state for all controls >>>
    with config_lock:
        digital_rec_state_initial = digital_recording_active
        batt_perc_initial = battery_percentage
        undistort_state_initial = undistort_active 

        try: current_awb_mode_name_initial = current_awb_mode.name
        except AttributeError: current_awb_mode_name_initial = DEFAULT_AWB_MODE_NAME
        try: current_ae_mode_name_initial = current_ae_mode.name
        except AttributeError: current_ae_mode_name_initial = DEFAULT_AE_MODE_NAME
        try: current_metering_mode_name_initial = current_metering_mode.name
        except AttributeError: current_metering_mode_name_initial = DEFAULT_METERING_MODE_NAME
        try: current_noise_reduction_mode_name_initial = current_noise_reduction_mode.name
        except AttributeError: current_noise_reduction_mode_name_initial = DEFAULT_NOISE_REDUCTION_MODE_NAME

        brightness_initial = current_brightness
        contrast_initial = current_contrast
        saturation_initial = current_saturation
        sharpness_initial = current_sharpness

    batt_text_initial = f"{batt_perc_initial:.1f}" if batt_perc_initial is not None else "--"

    # --- Helper Function to Build Dropdown Options ---
    def build_options(available_modes, current_mode_name):
        options_html = ""
        for mode_name in available_modes:
            selected_attr = ' selected' if mode_name == current_mode_name else ''
            options_html += f'<option value="{mode_name}"{selected_attr}>{mode_name}</option>'
        return options_html

    # --- Build HTML Option Strings ---
    awb_options_html = build_options(AVAILABLE_AWB_MODES, current_awb_mode_name_initial)
    ae_options_html = build_options(AVAILABLE_AE_MODES, current_ae_mode_name_initial)
    metering_options_html = build_options(AVAILABLE_METERING_MODES, current_metering_mode_name_initial)
    noise_reduction_options_html = build_options(AVAILABLE_NOISE_REDUCTION_MODES, current_noise_reduction_mode_name_initial)

    # --- Calculate URL for video feed ---
    video_feed_url = url_for('video_feed')

    # --- Define HTML Template (Standard String, NOT f-string) ---
    html_template = """
    <!DOCTYPE html>
    <html>
    <head>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Pi Camera Stream & Record</title>
        {% raw %}
        <style>
            /* --- CSS Styles (remain the same as before) --- */
            body { font-family: sans-serif; line-height: 1.4; margin: 1em; background-color: #f0f0f0;}
            .container { max-width: 960px; margin: 0 auto; background: #fff; padding: 15px; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
            h1 { text-align: center; color: #333; margin-bottom: 10px; }
            .grid-container { display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 20px; margin-bottom: 15px; }
            .status-panel, .controls-panel, .sliders-panel { background-color: #eef; padding: 15px; border-radius: 5px; box-sizing: border-box; } 
            .panel-title { font-weight: bold; margin-bottom: 10px; border-bottom: 1px solid #ccc; padding-bottom: 5px; }
            .status-grid { display: grid; grid-template-columns: auto 1fr; gap: 5px 10px; align-items: center; }
            .status-grid span:first-child { font-weight: bold; color: #555; text-align: right;}
            #status, #rec-status, #resolution, #battery-level, #awb-mode-status, #ae-mode-status, #metering-mode-status, #nr-mode-status, #undistort-status { color: #0056b3; font-weight: normal; overflow-wrap: break-word; } 
            #rec-status.active { color: #D83B01; font-weight: bold;}
            #undistort-status.active { color: #1E88E5; font-weight: bold; } 
            .main-controls { display: flex; justify-content: center; align-items: center; flex-wrap: wrap; gap: 10px; margin-bottom: 10px; }
            .main-controls button { padding: 10px 15px; margin: 5px; font-size: 1em; cursor: pointer; border-radius: 5px; border: 1px solid #ccc; background-color: #e9e9e9; transition: background-color 0.2s, border-color 0.2s; }
            .main-controls button:hover:not(:disabled) { background-color: #dcdcdc; border-color: #bbb; }
            .mode-controls { display: grid; grid-template-columns: auto 1fr; gap: 8px 10px; align-items: center; }
            .mode-controls label { font-weight: normal; color: #444; text-align: right; font-size: 0.9em;}
            .mode-controls select { padding: 5px 8px; font-size: 0.9em; border-radius: 4px; border: 1px solid #ccc; width: 100%; box-sizing: border-box; }
            .mode-controls select:hover:not(:disabled) { border-color: #bbb; background-color: #f9f9f9; }
            .slider-controls { display: grid; grid-template-columns: auto 1fr auto; gap: 5px 10px; align-items: center; margin-bottom: 8px; }
            .slider-controls label { font-weight: normal; color: #444; text-align: right; font-size: 0.9em;}
            .slider-controls input[type=range] { width: 100%; margin: 0; padding: 0; cursor: pointer; box-sizing: border-box; } 
            .slider-controls span { font-size: 0.9em; color: #0056b3; min-width: 35px; text-align: right; } 
            #error { color: red; margin-top: 15px; white-space: pre-wrap; font-weight: bold; min-height: 1.2em; text-align: center; background-color: #ffebeb; border: 1px solid red; padding: 8px; border-radius: 4px; display: none; }
            img#stream { display: block; margin: 15px auto; border: 1px solid black; max-width: 100%; height: auto; background-color: #ddd; box-sizing: border-box;} 
            button#btn-record.recording-active { background-color: #ff4d4d; color: white; border-color: #ff1a1a; }
            button#btn-record.recording-active:hover:not(:disabled) { background-color: #e60000; }
            button#btn-record.recording-inactive { background-color: #4CAF50; color: white; border-color: #367c39;}
            button#btn-record.recording-inactive:hover:not(:disabled) { background-color: #45a049; }
            button#btn-undistort.undistort-active { background-color: #2196F3; color: white; border-color: #0b7dda; }
            button#btn-undistort.undistort-inactive { background-color: #eee; color: #333; border-color: #ccc; }
            button#btn-undistort.undistort-active:hover:not(:disabled) { background-color: #0b7dda; }
            button#btn-undistort.undistort-inactive:hover:not(:disabled) { background-color: #ddd; }
            button#btn-powerdown { background-color: #f44336; color: white; border-color: #d32f2f;}
            button#btn-powerdown:hover:not(:disabled) { background-color: #c62828; }
            button:disabled, select:disabled, input[type=range]:disabled { background-color: #cccccc !important; cursor: not-allowed !important; border-color: #999 !important; color: #666 !important; opacity: 0.7; }
            input[type=range]:disabled::-webkit-slider-thumb { background: #999; }
            input[type=range]:disabled::-moz-range-thumb { background: #999; }
        </style>
        {% endraw %}
    </head>
    <body>
        <div class="container">
            <h1>Pi Camera Stream & Record</h1>

             <div class="main-controls">
                 <button onclick="changeResolution('down')" id="btn-down" title="Decrease resolution">&laquo; Lower Res</button>
                 <button onclick="toggleRecording()" id="btn-record" class="recording-inactive" title="Toggle recording via web interface">Start Rec (Web)</button>
                 <button onclick="toggleUndistort()" id="btn-undistort" class="{{ 'undistort-active' if undistort_state_initial else 'undistort-inactive' }}" title="Toggle Fisheye Undistortion (Performance Cost!)">{{ 'Undistort ON' if undistort_state_initial else 'Undistort OFF' }}</button>
                 <button onclick="changeResolution('up')" id="btn-up" title="Increase resolution">Higher Res &raquo;</button>
                 <button onclick="powerDown()" id="btn-powerdown" title="Gracefully stop service and reboot Pi">Power Down</button>
            </div>

            <div class="grid-container">
                <div class="status-panel">
                     <div class="panel-title">Current Status</div>
                     <div class="status-grid">
                        <span>Sys Status:</span> <span id="status">Initializing...</span>
                        <span>Recording:</span> <span id="rec-status">OFF</span>
                        <span>Resolution:</span> <span id="resolution">{{ resolution_text }}</span>
                        <span>Battery:</span> <span id="battery-level">{{ batt_text_initial }}%</span>
                        <hr style="grid-column: 1 / -1; border-top: 1px dashed #bbb; border-bottom: none; margin: 5px 0;">
                        <span>AWB Mode:</span> <span id="awb-mode-status">{{ current_awb_mode_name_initial }}</span>
                        <span>AE Mode:</span> <span id="ae-mode-status">{{ current_ae_mode_name_initial }}</span>
                        <span>Metering:</span> <span id="metering-mode-status">{{ current_metering_mode_name_initial }}</span>
                        <span>Noise Red.:</span> <span id="nr-mode-status">{{ current_noise_reduction_mode_name_initial }}</span>
                        <span>Undistortion:</span> <span id="undistort-status" class="{{ 'active' if undistort_state_initial else '' }}">{{ 'ON' if undistort_state_initial else 'OFF' }}</span>
                    </div>
                 </div>

                 <div class="controls-panel">
                     <div class="panel-title">Mode Controls</div>
                     <div class="mode-controls">
                         <label for="awb-select">AWB Mode:</label>
                         <select id="awb-select" onchange="changeCameraControl('AwbMode', this.value)" title="Select Auto White Balance Mode">
                             {{ awb_options_html | safe }}
                         </select>
                         <label for="ae-select">Exposure Mode:</label>
                         <select id="ae-select" onchange="changeCameraControl('AeExposureMode', this.value)" title="Select Auto Exposure Mode">
                             {{ ae_options_html | safe }}
                         </select>
                        <label for="metering-select">Metering Mode:</label>
                         <select id="metering-select" onchange="changeCameraControl('AeMeteringMode', this.value)" title="Select AE Metering Mode">
                             {{ metering_options_html | safe }}
                         </select>
                         <label for="nr-select">Noise Reduction:</label>
                         <select id="nr-select" onchange="changeCameraControl('NoiseReductionMode', this.value)" title="Select Noise Reduction Mode">
                             {{ noise_reduction_options_html | safe }}
                         </select>
                     </div>
                     </div>

                 <div class="sliders-panel">
                     <div class="panel-title">Image Adjustments</div>
                     <div class="slider-controls">
                         <label for="brightness-slider">Brightness:</label>
                         <input type="range" id="brightness-slider" min="{{ MIN_BRIGHTNESS }}" max="{{ MAX_BRIGHTNESS }}" step="{{ STEP_BRIGHTNESS }}" value="{{ brightness_initial }}" oninput="updateSliderValue(this.id, this.value)" onchange="changeCameraControl('Brightness', this.value)" title="Adjust Brightness">
                         <span id="brightness-slider-value">{{ "%.1f" | format(brightness_initial) }}</span>
                         <label for="contrast-slider">Contrast:</label>
                         <input type="range" id="contrast-slider" min="{{ MIN_CONTRAST }}" max="{{ MAX_CONTRAST }}" step="{{ STEP_CONTRAST }}" value="{{ contrast_initial }}" oninput="updateSliderValue(this.id, this.value)" onchange="changeCameraControl('Contrast', this.value)" title="Adjust Contrast">
                         <span id="contrast-slider-value">{{ "%.1f" | format(contrast_initial) }}</span>
                         <label for="saturation-slider">Saturation:</label>
                         <input type="range" id="saturation-slider" min="{{ MIN_SATURATION }}" max="{{ MAX_SATURATION }}" step="{{ STEP_SATURATION }}" value="{{ saturation_initial }}" oninput="updateSliderValue(this.id, this.value)" onchange="changeCameraControl('Saturation', this.value)" title="Adjust Saturation">
                         <span id="saturation-slider-value">{{ "%.1f" | format(saturation_initial) }}</span>
                         <label for="sharpness-slider">Sharpness:</label>
                         <input type="range" id="sharpness-slider" min="{{ MIN_SHARPNESS }}" max="{{ MAX_SHARPNESS }}" step="{{ STEP_SHARPNESS }}" value="{{ sharpness_initial }}" oninput="updateSliderValue(this.id, this.value)" onchange="changeCameraControl('Sharpness', this.value)" title="Adjust Sharpness">
                         <span id="sharpness-slider-value">{{ "%.1f" | format(sharpness_initial) }}</span>
                     </div>
                      </div>
            </div> <div id="error" {% if err_msg %}style="display: block;"{% endif %}>{{ err_msg }}</div>
            
            <img id="stream" src="{{ video_feed_url }}" alt="Loading stream..."
                   onerror="handleStreamError()" onload="handleStreamLoad()">
                   
        </div>

        <script>
            // --- JavaScript Section (Includes Resolution Timeout Fix) ---
            let currentDigitalRecordState = {{ 'true' if digital_rec_state_initial else 'false' }};
            const videoFeedUrlBase = "{{ video_feed_url }}"; 

            // Get Element References
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
            const awbStatusElement = document.getElementById('awb-mode-status');
            const aeStatusElement = document.getElementById('ae-mode-status');
            const meteringStatusElement = document.getElementById('metering-mode-status');
            const nrStatusElement = document.getElementById('nr-mode-status');
            const btnUndistort = document.getElementById('btn-undistort'); 
            const undistortStatusElement = document.getElementById('undistort-status'); 
            const awbSelectElement = document.getElementById('awb-select');
            const aeSelectElement = document.getElementById('ae-select');
            const meteringSelectElement = document.getElementById('metering-select');
            const nrSelectElement = document.getElementById('nr-select');
            const brightnessSlider = document.getElementById('brightness-slider');
            const contrastSlider = document.getElementById('contrast-slider');
            const saturationSlider = document.getElementById('saturation-slider');
            const sharpnessSlider = document.getElementById('sharpness-slider');
            const brightnessValueSpan = document.getElementById('brightness-slider-value');
            const contrastValueSpan = document.getElementById('contrast-slider-value');
            const saturationValueSpan = document.getElementById('saturation-slider-value');
            const sharpnessValueSpan = document.getElementById('sharpness-slider-value');

            // State Variables 
            let isChangingResolution = false;
            let isTogglingRecording = false;
            let isChangingControl = false; 
            let isTogglingUndistort = false; 
            let isPoweringDown = false;
            let statusUpdateInterval = null; 
            let streamErrorTimeout = null;

            // --- UI Update Functions ---
            function updateRecordButtonState() { /* (Same) */ }
            function updateUndistortButtonState(isActive) { /* (Same) */ }
            function updateStatus() { /* (Same - includes undistort update) */ }
            function updateControlUI(controlKey, newValue, statusEl, controlEl, valueSpanEl = null) { /* (Same) */ }

            function disableControls(poweringDown = false) { /* (Includes btnUndistort) */
                console.log(`disableControls called (poweringDown=${poweringDown})`); 
                [btnUp, btnDown, btnRecord, btnUndistort, btnPowerdown, 
                 awbSelectElement, aeSelectElement, meteringSelectElement, nrSelectElement,
                 brightnessSlider, contrastSlider, saturationSlider, sharpnessSlider
                ].forEach(el => { if(el) el.disabled = true; }); 
                if(poweringDown) { document.body.style.opacity = '0.7'; }
            }

            function enableControls() { /* (Includes btnUndistort and log) */
                 console.log("enableControls called"); 
                 if (!isPoweringDown) {
                    [btnUp, btnDown, btnRecord, btnUndistort, btnPowerdown, 
                     awbSelectElement, aeSelectElement, meteringSelectElement, nrSelectElement,
                     brightnessSlider, contrastSlider, saturationSlider, sharpnessSlider
                    ].forEach(el => { if(el) el.disabled = false; }); 
                    document.body.style.opacity = '1';
                 } else {
                    console.log("enableControls skipped: isPoweringDown is true.");
                 }
            }

            // --- Action Functions ---
            function changeResolution(direction) { /* (Includes isTogglingUndistort check and timeout logic) */
                if (isChangingResolution || isTogglingRecording || isChangingControl || isTogglingUndistort || isPoweringDown) return;
                
                isChangingResolution = true; 
                disableControls(); 
                statusElement.textContent = 'Changing resolution... Please wait.'; 
                errorElement.textContent = ''; errorElement.style.display = 'none';
                
                const cleanupResolutionChange = (isSuccess = false) => {
                    console.log(`CleanupResolutionChange called (success=${isSuccess}). Current isChangingResolution = ${isChangingResolution}`);
                    if (isChangingResolution) { 
                        isChangingResolution = false; 
                        enableControls(); 
                        setTimeout(updateStatus, 500); 
                    }
                };

                const resolutionTimeoutId = setTimeout(() => {
                    console.warn("Resolution change timeout reached. Forcing cleanup."); 
                    console.log("Attempting cleanupResolutionChange from timeout..."); 
                    cleanupResolutionChange(false); 
                 }, 8000); // 8 second timeout

                fetch(`/set_resolution/${direction}`, { method: 'POST' })
                    .then(response => response.json().then(data => ({ status: response.status, body: data })))
                    .then(({ status, body }) => {
                        if (status === 200 && body.success) {
                            statusElement.textContent = 'Resolution change initiated. Reloading stream...';
                            resolutionElement.textContent = body.new_resolution;
                            // <<< No width/height set on img tag here >>>
                            console.log("Resolution change request successful, forcing stream reload...");
                            streamImage.src = videoFeedUrlBase + "?" + Date.now(); 
                            // Rely on timeout for cleanup
                        } else {
                            errorElement.textContent = `Error changing resolution: ${body.message || 'Unknown error.'}`; 
                            errorElement.style.display = 'block'; 
                            statusElement.textContent = 'Resolution change failed.';
                            clearTimeout(resolutionTimeoutId); 
                            cleanupResolutionChange(false); 
                        }
                    })
                    .catch(err => {
                         console.error("Network error sending resolution change:", err); 
                         errorElement.textContent = `Network error changing resolution: ${err.message}`; 
                         errorElement.style.display = 'block'; 
                         statusElement.textContent = 'Resolution change failed (Network).';
                         clearTimeout(resolutionTimeoutId); 
                         cleanupResolutionChange(false); 
                     });
            }

            function toggleRecording() { /* (Includes isTogglingUndistort check) */ }
            function changeCameraControl(controlName, controlValue) { /* (Includes isTogglingUndistort check) */ }
            function toggleUndistort() { /* (Includes isTogglingUndistort check) */ }
            function updateSliderValue(sliderId, value) { /* (Same) */ }
            function powerDown() { /* (Includes isTogglingUndistort check) */ }

            // --- Stream Handling ---
            function handleStreamError() { /* (Same) */ }
            function handleStreamLoad() { /* (Same) */ }

            // --- Initialization ---
            document.addEventListener('DOMContentLoaded', () => { /* (Same simple version) */
                console.log("DOMContentLoaded event fired.");
                updateRecordButtonState();
                updateStatus(); 
                if (!statusUpdateInterval) {
                    statusUpdateInterval = setInterval(() => {
                        if (!isChangingResolution && !isTogglingRecording && !isChangingControl && !isTogglingUndistort && !isPoweringDown) {
                            updateStatus();
                        }
                    }, 5000); 
                 }
                 if(!isPoweringDown) { enableControls(); } 
            });
            window.addEventListener('beforeunload', () => { /* (Same) */ });

        </script>
    </body>
    </html>
    """

    # Render the template
    return render_template_string(html_template,
                                   # Pass all necessary variables ...
                                   resolution_text=resolution_text,
                                   err_msg=err_msg, 
                                   digital_rec_state_initial=digital_rec_state_initial,
                                   batt_text_initial=batt_text_initial,
                                   undistort_state_initial=undistort_state_initial, 
                                   current_awb_mode_name_initial=current_awb_mode_name_initial, awb_options_html=awb_options_html, 
                                   current_ae_mode_name_initial=current_ae_mode_name_initial, ae_options_html=ae_options_html, 
                                   current_metering_mode_name_initial=current_metering_mode_name_initial, metering_options_html=metering_options_html, 
                                   current_noise_reduction_mode_name_initial=current_noise_reduction_mode_name_initial, noise_reduction_options_html=noise_reduction_options_html, 
                                   brightness_initial=brightness_initial, MIN_BRIGHTNESS=MIN_BRIGHTNESS, MAX_BRIGHTNESS=MAX_BRIGHTNESS, STEP_BRIGHTNESS=STEP_BRIGHTNESS, 
                                   contrast_initial=contrast_initial, MIN_CONTRAST=MIN_CONTRAST, MAX_CONTRAST=MAX_CONTRAST, STEP_CONTRAST=STEP_CONTRAST, 
                                   saturation_initial=saturation_initial, MIN_SATURATION=MIN_SATURATION, MAX_SATURATION=MAX_SATURATION, STEP_SATURATION=STEP_SATURATION, 
                                   sharpness_initial=sharpness_initial, MIN_SHARPNESS=MIN_SHARPNESS, MAX_SHARPNESS=MAX_SHARPNESS, STEP_SHARPNESS=STEP_SHARPNESS, 
                                   video_feed_url=video_feed_url 
                                  )
# --- End index Route ---


@app.route('/set_camera_control', methods=['POST'])
def set_camera_control():
    global picam2, last_error, config_lock
    # Add globals for all controllable variables here
    global current_awb_mode, current_ae_mode, current_metering_mode, current_noise_reduction_mode
    global current_brightness, current_contrast, current_saturation, current_sharpness

    if not picam2 or not picam2.started:
        return jsonify({'success': False, 'message': 'Camera not available.'}), 503

    control_name = None # Initialize for error logging
    try:
        data = request.get_json()
        if not data or 'control' not in data or 'value' not in data:
             logging.warning("/set_camera_control called without 'control' or 'value'.")
             return jsonify({'success': False, 'message': 'Missing control or value in request.'}), 400

        control_name = data['control']
        control_value = data['value']
        logging.info(f"Web request: Set control '{control_name}' to '{control_value}'")

        control_dict_to_set = {}
        update_success = False
        error_msg = None
        new_value_logged = control_value # Default for logging

        with config_lock:
            # --- Handle Mode Controls (Dropdowns) ---
            if control_name == 'AwbMode':
                # <<< V3 Change: Use __members__.get() to retrieve enum member >>>
                enum_val = controls.AwbModeEnum.__members__.get(control_value)
                if enum_val is not None: # Check if lookup was successful
                    control_dict_to_set[control_name] = enum_val
                    control_dict_to_set["AwbEnable"] = True # Ensure enabled when setting mode
                    current_awb_mode = enum_val
                    update_success = True
                else: error_msg = f"Invalid AwbMode value: {control_value}"

            elif control_name == 'AeExposureMode':
                 # <<< V3 Change: Use __members__.get() >>>
                 enum_val = controls.AeExposureModeEnum.__members__.get(control_value)
                 if enum_val is not None:
                     control_dict_to_set[control_name] = enum_val
                     control_dict_to_set["AeEnable"] = True
                     current_ae_mode = enum_val
                     update_success = True
                 else: error_msg = f"Invalid AeExposureMode value: {control_value}"

            elif control_name == 'AeMeteringMode':
                 # <<< V3 Change: Use __members__.get() >>>
                 enum_val = controls.AeMeteringModeEnum.__members__.get(control_value)
                 if enum_val is not None:
                     control_dict_to_set[control_name] = enum_val
                     control_dict_to_set["AeEnable"] = True
                     current_metering_mode = enum_val
                     update_success = True
                 else: error_msg = f"Invalid AeMeteringMode value: {control_value}"

            elif control_name == 'NoiseReductionMode':
                 # <<< V3 Change: Use __members__.get() >>>
                 enum_val = controls.draft.NoiseReductionModeEnum.__members__.get(control_value)
                 if enum_val is not None:
                     control_dict_to_set[control_name] = enum_val
                     current_noise_reduction_mode = enum_val
                     update_success = True
                 else: error_msg = f"Invalid NoiseReductionMode value: {control_value}"

            # --- Handle Numeric Controls (Sliders) ---
            elif control_name == 'Brightness':
                 try:
                     val = float(control_value)
                     val = max(MIN_BRIGHTNESS, min(MAX_BRIGHTNESS, val)) # Clamp
                     control_dict_to_set[control_name] = val
                     current_brightness = val
                     update_success = True
                     new_value_logged = f"{val:.2f}" # Format for logging
                 except ValueError: error_msg = "Invalid numeric value for Brightness"
            elif control_name == 'Contrast':
                 try:
                     val = float(control_value)
                     val = max(MIN_CONTRAST, min(MAX_CONTRAST, val)) # Clamp
                     control_dict_to_set[control_name] = val
                     current_contrast = val
                     update_success = True
                     new_value_logged = f"{val:.2f}"
                 except ValueError: error_msg = "Invalid numeric value for Contrast"
            elif control_name == 'Saturation':
                 try:
                     val = float(control_value)
                     val = max(MIN_SATURATION, min(MAX_SATURATION, val)) # Clamp
                     control_dict_to_set[control_name] = val
                     current_saturation = val
                     update_success = True
                     new_value_logged = f"{val:.2f}"
                 except ValueError: error_msg = "Invalid numeric value for Saturation"
            elif control_name == 'Sharpness':
                 try:
                     val = float(control_value)
                     val = max(MIN_SHARPNESS, min(MAX_SHARPNESS, val)) # Clamp
                     control_dict_to_set[control_name] = val
                     current_sharpness = val
                     update_success = True
                     new_value_logged = f"{val:.2f}"
                 except ValueError: error_msg = "Invalid numeric value for Sharpness"

            # --- Unknown Control ---
            else:
                error_msg = f"Unknown control name: {control_name}"

            # --- Apply if valid ---
            if update_success:
                # <<< Apply the controls to the camera >>>
                picam2.set_controls(control_dict_to_set)
                last_error = None # Clear potential previous errors
                logging.info(f"Successfully set {control_name} to: {new_value_logged}")
                # Give controls time to settle (especially AWB/AE)
                if control_name in ['AwbMode', 'AeExposureMode', 'AeMeteringMode', 'Brightness']:
                     time.sleep(0.5)
                else:
                     time.sleep(0.1) # Shorter delay for others
                return jsonify({'success': True, 'message': f'{control_name} set to {new_value_logged}.'})
            else:
                 # Failed validation before trying to set
                 logging.error(f"Failed validation for control '{control_name}': {error_msg}")
                 return jsonify({'success': False, 'message': error_msg}), 400

    except Exception as e:
        # Error occurred during the process, possibly in set_controls
        logging.error(f"Error setting control '{control_name}': {e}", exc_info=True)
        last_error = f"Control Set Error ({control_name or 'Unknown'}): {e}"
        # Return the specific error message if possible
        return jsonify({'success': False, 'message': f'Failed to set {control_name or "control"}: {e}'}), 500
    
@app.route("/video_feed")
def video_feed():
    logging.info("Client connected to video feed.")
    return Response(generate_stream_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route("/status")
def status():
    global last_error, digital_recording_active, is_recording, battery_percentage, config_lock
    global video_writers, recording_paths, undistort_active # Added undistort_active
    global current_awb_mode, current_ae_mode, current_metering_mode, current_noise_reduction_mode
    global current_brightness, current_contrast, current_saturation, current_sharpness

    status_text = "Streaming"
    rec_stat_detail = ""
    current_w, current_h = get_current_resolution()

    # Initialize values to return
    status_data = {
        'is_recording': False,
        'digital_recording_active': False,
        'resolution': f"{current_w}x{current_h}",
        'status_text': status_text,
        'error': "",
        'active_recordings': [],
        'battery_percent': None,
        'awb_mode': "Unknown",
        'ae_mode': "Unknown",
        'metering_mode': "Unknown",
        'noise_reduction_mode': "Unknown",
        'brightness': DEFAULT_BRIGHTNESS,
        'contrast': DEFAULT_CONTRAST,
        'saturation': DEFAULT_SATURATION,
        'sharpness': DEFAULT_SHARPNESS,
        'undistort_active': False # Add default
    }

    with config_lock:
        status_data['digital_recording_active'] = digital_recording_active
        status_data['battery_percent'] = battery_percentage
        status_data['is_recording'] = is_recording
        status_data['active_recordings'] = list(recording_paths)

        # Safely get current control values 
        try: status_data['awb_mode'] = current_awb_mode.name
        except AttributeError: pass 
        try: status_data['ae_mode'] = current_ae_mode.name
        except AttributeError: pass
        try: status_data['metering_mode'] = current_metering_mode.name
        except AttributeError: pass
        try: status_data['noise_reduction_mode'] = current_noise_reduction_mode.name
        except AttributeError: pass # Will fail if draft controls aren't available

        status_data['brightness'] = current_brightness
        status_data['contrast'] = current_contrast
        status_data['saturation'] = current_saturation
        status_data['sharpness'] = current_sharpness
        status_data['undistort_active'] = undistort_active # Read global state

    # Recording Status Text 
    if status_data['is_recording']:
        if status_data['active_recordings']:
            rec_stat_detail = f" (Recording to {len(status_data['active_recordings'])} USB(s))"
        else:
            rec_stat_detail = " (ERROR: Recording active but no paths!)"
            logging.warning("Status check found is_recording=True but recording_paths is empty.")
            if not last_error: last_error = "Inconsistent State: Recording active but no paths."
        status_data['status_text'] += rec_stat_detail

    # Error Handling & Auto-Clear 
    err_msg = last_error if last_error else ""
    if output_frame is not None and err_msg and ("Init Error" in err_msg or "unavailable" in err_msg or "capture" in err_msg):
            logging.info("Auto-clearing previous camera/capture error as frames are being received.")
            last_error = None; err_msg = ""
    if status_data['battery_percent'] is not None and err_msg and ("Battery Monitor" in err_msg or "INA219" in err_msg or "I2C Error" in err_msg):
            logging.info("Auto-clearing previous battery monitor error as a reading was successful.")
            last_error = None; err_msg = ""
    if undistort_map1 is not None and status_data['undistort_active'] and err_msg and "Undistort" in err_msg:
            logging.info("Auto-clearing previous undistort error as maps seem prepared now.")
            last_error = None; err_msg = ""

    status_data['error'] = err_msg

    return jsonify(status_data)
# --- End /status Route ---


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

@app.route('/toggle_undistort', methods=['POST'])
def toggle_undistort():
    global undistort_active, undistort_map1, undistort_map2, last_prepared_undistort_size
    global config_lock, last_error

    new_state = False
    message = "Undistortion toggled."
    success = True
    status_code = 200

    with config_lock:
        undistort_active = not undistort_active
        new_state = undistort_active
        logging.info(f"Undistortion toggled via web UI to: {'ON' if new_state else 'OFF'}")

        if new_state:
            # Enabling - need to prepare maps for current resolution
            if picam2 and picam2.started:
                 current_w, current_h = get_current_resolution() # Use helper under lock? No, read global index only
                 res_index = current_resolution_index
                 current_w, current_h = SUPPORTED_RESOLUTIONS[res_index]

                 logging.info("Enabling undistortion, preparing maps...")
                 if prepare_undistort_maps(current_w, current_h):
                     last_prepared_undistort_size = (current_w, current_h)
                     message = "Undistortion enabled. Maps prepared."
                 else:
                     message = "Undistortion enabled BUT failed to prepare maps! Feature disabled."
                     logging.error(message)
                     last_error = "Undistort map prep failed."
                     undistort_active = False # Revert state
                     new_state = False
                     success = False
                     status_code = 500
            else:
                 message = "Undistortion enabled, but maps will be prepared when camera starts/reconfigures."
                 # Clear potentially stale maps
                 undistort_map1 = None
                 undistort_map2 = None
                 last_prepared_undistort_size = None
        else:
             # Disabling - clear maps
             undistort_map1 = None
             undistort_map2 = None
             last_prepared_undistort_size = None
             message = "Undistortion disabled."

        if success and last_error and "Undistort" in last_error:
             last_error = None # Clear previous undistort errors on success

    return jsonify({'success': success, 'undistort_active': new_state, 'message': message}), status_code

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