# -*- coding: utf-8 -*-
"""
config.py

Configuration constants for the Pi Camera Stream & Record application.
"""
import os
from libcamera import controls
from picamera2 import Picamera2 # Needed for Picamera2.load_tuning_file

# --- Camera Configuration ---
SUPPORTED_RESOLUTIONS = [
    (640, 480),      # VGA
    (1280, 720),     # 720p HD
    (1640, 1232),    # Custom Pi Cam V2 mode
    (1920, 1080),    # 1080p FHD
    (3280, 2464)     # Max Native Resolution IMX219
]
DEFAULT_RESOLUTION_INDEX = 1 # 1280x720
# Note: Actual FRAME_RATE might be adjusted by the camera driver based on resolution.
# The get_target_fps function in camera_manager.py determines the requested rate.
DEFAULT_FRAME_RATE = 30 # Default target FPS, used if specific logic doesn't override

# --- Recording Configuration ---
RECORDING_FORMAT = "mp4v" # Codec for OpenCV VideoWriter
RECORDING_EXTENSION = ".mp4"

# --- File Paths ---
USB_BASE_PATH = "/media/hecke/" # CHANGE this if your mount point base is different
# Determine script directory to find toggle script and potential tuning files
try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    # Fallback if __file__ is not defined (e.g., interactive mode)
    SCRIPT_DIR = os.path.abspath('.')
    print(f"Warning: Could not automatically determine script directory. Using current directory: {SCRIPT_DIR}")

TOGGLE_SCRIPT_NAME = "toggle.sh"
TOGGLE_SCRIPT_PATH = os.path.join(SCRIPT_DIR, TOGGLE_SCRIPT_NAME)

# --- NoIR Camera Tuning ---
USE_NOIR_TUNING = True # Set to False to use default tuning
NOIR_TUNING_FILE_PATH = "/usr/share/libcamera/ipa/rpi/pisp/imx219_noir.json"
# Load tuning data if enabled and file exists
CAMERA_TUNING = None
if USE_NOIR_TUNING:
    if os.path.exists(NOIR_TUNING_FILE_PATH):
        try:
            # Use Picamera2's utility function to load the tuning file
            CAMERA_TUNING = Picamera2.load_tuning_file(NOIR_TUNING_FILE_PATH)
            print(f"Successfully loaded NoIR tuning file: {NOIR_TUNING_FILE_PATH}")
        except Exception as e:
            print(f"Error loading NoIR tuning file '{NOIR_TUNING_FILE_PATH}': {e}. Using default tuning.")
            CAMERA_TUNING = None
    else:
        print(f"NoIR tuning file not found at {NOIR_TUNING_FILE_PATH}. Using default tuning.")
else:
     print("Using default camera tuning (NoIR tuning disabled).")


# --- Web Server Configuration ---
WEB_PORT = 8000

# --- GPIO Configuration ---
SWITCH_GPIO_PIN = 17          # BCM Pin number for the physical switch, Set to None to disable
SWITCH_BOUNCE_TIME = 0.1      # Debounce time in seconds for the switch

# --- Battery Monitor (INA219) Configuration ---
INA219_I2C_ADDRESS = 0x41     # I2C address of the INA219 sensor, Set to None to disable
INA219_I2C_BUS = 1            # I2C bus number (usually 1 for Raspberry Pi)
BATTERY_READ_INTERVAL = 30.0  # How often to read the battery level (seconds)
BATTERY_MAX_VOLTAGE = 12.6    # Voltage when fully charged (e.g., 3S LiPo)
BATTERY_MIN_VOLTAGE = 9.0     # Voltage when empty (e.g., 3S LiPo cut-off)
# INA219 Calibration (Choose one appropriate for your setup)
# These values depend on the shunt resistor value and expected current/voltage ranges.
# The default is 16V/5A (using a 0.01 Ohm shunt, common on Adafruit boards)
# If using a different shunt or range, adjust calibration values and LSBs accordingly.
INA219_CALIBRATION_CONFIG = "16V_5A" # Options: "16V_5A", "32V_2A" (or add custom)

# --- Servo Configuration ---
SERVO_ENABLED = True          # Set to False to disable servo control entirely
SERVO_PWM_CHIP = 0            # PWM Chip number (e.g., 0 for pwmchip0)
SERVO_PWM_CHANNEL = 0         # PWM Channel number (e.g., 0 for pwm0)
SERVO_GPIO_PIN = 12           # The GPIO pin associated with the PWM (for reference/logging)

# Servo Timing Configuration (adjust for your specific servo)
SERVO_PERIOD_NS = 20000000    # 20ms (50Hz) period in nanoseconds
SERVO_MIN_DUTY_NS = 500000    # 0.5ms pulse width min (for angle 0) - Adjust if needed
SERVO_MAX_DUTY_NS = 2500000   # 2.5ms pulse width max (for angle 180) - Adjust if needed
SERVO_CENTER_ANGLE = 90       # Initial angle for the servo on startup
SERVO_MIN_ANGLE = 0           # Minimum angle limit
SERVO_MAX_ANGLE = 180         # Maximum angle limit

# --- Camera Control Defaults & Ranges ---

# Auto White Balance (AWB)
AVAILABLE_AWB_MODES = list(controls.AwbModeEnum.__members__.keys())
DEFAULT_AWB_MODE_NAME = "Auto"
if DEFAULT_AWB_MODE_NAME not in AVAILABLE_AWB_MODES:
    DEFAULT_AWB_MODE_NAME = AVAILABLE_AWB_MODES[0] if AVAILABLE_AWB_MODES else "Auto"

# Auto Exposure (AE)
AVAILABLE_AE_MODES = list(controls.AeExposureModeEnum.__members__.keys())
DEFAULT_AE_MODE_NAME = "Normal"
if DEFAULT_AE_MODE_NAME not in AVAILABLE_AE_MODES:
    DEFAULT_AE_MODE_NAME = AVAILABLE_AE_MODES[0] if AVAILABLE_AE_MODES else "Normal"

# AE Metering Mode
AVAILABLE_METERING_MODES = list(controls.AeMeteringModeEnum.__members__.keys())
DEFAULT_METERING_MODE_NAME = "CentreWeighted"
if DEFAULT_METERING_MODE_NAME not in AVAILABLE_METERING_MODES:
    DEFAULT_METERING_MODE_NAME = AVAILABLE_METERING_MODES[0] if AVAILABLE_METERING_MODES else "CentreWeighted"

# Noise Reduction Mode (Using draft controls)
# Note: controls.draft might change in future libcamera/picamera2 versions
try:
    AVAILABLE_NOISE_REDUCTION_MODES = list(controls.draft.NoiseReductionModeEnum.__members__.keys())
    DEFAULT_NOISE_REDUCTION_MODE_NAME = "Fast"
    if DEFAULT_NOISE_REDUCTION_MODE_NAME not in AVAILABLE_NOISE_REDUCTION_MODES:
        DEFAULT_NOISE_REDUCTION_MODE_NAME = AVAILABLE_NOISE_REDUCTION_MODES[0] if AVAILABLE_NOISE_REDUCTION_MODES else "Off" # Fallback
except AttributeError:
    print("Warning: controls.draft.NoiseReductionModeEnum not found. Noise reduction control disabled.")
    AVAILABLE_NOISE_REDUCTION_MODES = ["Off"] # Provide a dummy list
    DEFAULT_NOISE_REDUCTION_MODE_NAME = "Off"

# Brightness
DEFAULT_BRIGHTNESS = 0.0
MIN_BRIGHTNESS = -1.0
MAX_BRIGHTNESS = 1.0
STEP_BRIGHTNESS = 0.1

# Contrast
DEFAULT_CONTRAST = 1.0
MIN_CONTRAST = 0.0
MAX_CONTRAST = 2.0 # Allow a bit more range
STEP_CONTRAST = 0.1

# Saturation
DEFAULT_SATURATION = 1.0
MIN_SATURATION = 0.0
MAX_SATURATION = 2.0 # Allow a bit more range
STEP_SATURATION = 0.1

# Sharpness
DEFAULT_SHARPNESS = 1.0
MIN_SHARPNESS = 0.0
MAX_SHARPNESS = 2.0 # Allow a bit more range
STEP_SHARPNESS = 0.1

# --- Error Handling ---
MAX_CONSECUTIVE_CAPTURE_ERRORS = 15 # How many capture errors before trying to restart/shutdown

# --- Logging Configuration ---
LOG_LEVEL = "INFO" # Options: DEBUG, INFO, WARNING, ERROR, CRITICAL
LOG_FORMAT = '%(asctime)s - %(levelname)s - [%(threadName)s] - %(message)s'
LOG_DATE_FORMAT = '%Y-%m-%d %H:%M:%S'

