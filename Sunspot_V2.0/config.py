# -*- coding: utf-8 -*-
"""
config.py

Configuration constants for the Pi Camera Stream & Record application.
Handles settings for dual cameras (HQ and IMX219), audio, hardware, etc.
"""
import os
import logging
from libcamera import controls
from picamera2 import Picamera2 # Needed for Picamera2.load_tuning_file

# --- Camera Configuration ---

# Camera Identifiers (Ensure these match physical connections)
CAM0_ID = 0 # Primary camera (HQ) - for recording and main stream view
CAM1_ID = 1 # Secondary camera (IMX219) - for secondary stream view

# --- Camera 0 (HQ Camera) Settings ---
# Resolutions format: (width, height, frame_rate)
CAM0_RESOLUTIONS = [
    (1332, 990, 120.0),   # High Frame Rate
    (2028, 1080, 50.0),   # High Res / High FPS
    (2028, 1520, 40.0),   # High Res / Medium FPS
    (1920, 1080, 30.0),   # Standard 1080p
    # Add other HQ modes if needed, ensure they are supported by the sensor/libcamera
]
CAM0_DEFAULT_RESOLUTION_INDEX = 3 # Default to 1920x1080 @ 30fps
CAM0_RECORDING_FORMAT = "mp4v"    # Codec for OpenCV VideoWriter for HQ camera
CAM0_RECORDING_EXTENSION = ".mp4"
CAM0_USE_TUNING_FILE = False      # HQ camera generally doesn't need specific tuning like NoIR
CAM0_TUNING_FILE_PATH = None      # No tuning file path needed if not used

# --- Camera 1 (IMX219 NoIR) Settings ---
CAM1_RESOLUTION = (1920, 1080) # Fixed resolution for secondary stream
CAM1_FRAME_RATE = 30.0       # Fixed frame rate for secondary stream
CAM1_USE_NOIR_TUNING = True  # Enable NoIR tuning for the IMX219
CAM1_NOIR_TUNING_FILE_PATH = "/usr/share/libcamera/ipa/rpi/pisp/imx219_noir.json" # Path to NoIR tuning

# Load tuning data for Cam 1 if enabled and file exists
CAM1_TUNING = None
if CAM1_USE_NOIR_TUNING:
    if os.path.exists(CAM1_NOIR_TUNING_FILE_PATH):
        try:
            CAM1_TUNING = Picamera2.load_tuning_file(CAM1_NOIR_TUNING_FILE_PATH)
            logging.info(f"Successfully loaded NoIR tuning file for Cam1: {CAM1_NOIR_TUNING_FILE_PATH}")
        except Exception as e:
            logging.error(f"Error loading NoIR tuning file '{CAM1_NOIR_TUNING_FILE_PATH}': {e}. Using default tuning for Cam1.")
            CAM1_TUNING = None
    else:
        logging.warning(f"NoIR tuning file not found at {CAM1_NOIR_TUNING_FILE_PATH}. Using default tuning for Cam1.")
else:
     logging.info("Using default camera tuning for Cam1 (NoIR tuning disabled).")

# --- Combined Stream Configuration ---
# How to arrange the two camera views in the web stream
# Options: "VERTICAL" (Cam0 above Cam1), "HORIZONTAL" (Cam0 left of Cam1)
STREAM_LAYOUT = "VERTICAL"
STREAM_BORDER_SIZE = 5 # Pixels between camera views in the combined stream
STREAM_BORDER_COLOR = (100, 100, 100) # BGR color for the border

# --- File Paths ---
USB_BASE_PATH = "/media/hecke/" # CHANGE this if your mount point base is different
# Determine script directory to find toggle script and potential tuning files
try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    # Fallback if __file__ is not defined (e.g., interactive mode)
    SCRIPT_DIR = os.path.abspath('.')
    logging.warning(f"Warning: Could not automatically determine script directory. Using current directory: {SCRIPT_DIR}")

TOGGLE_SCRIPT_NAME = "toggle.sh"
TOGGLE_SCRIPT_PATH = os.path.join(SCRIPT_DIR, TOGGLE_SCRIPT_NAME)

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
SERVO_MIN_ANGLE = 0           # Minimum angle limit for UI slider
SERVO_MAX_ANGLE = 180         # Maximum angle limit for UI slider

# --- Camera Control Defaults & Ranges (Common Controls) ---

# ISO Control (Replaces AWB)
# Map UI display names (e.g., "100") to AnalogueGain float values
# Adjust these values based on camera capabilities and testing
AVAILABLE_ISO_SETTINGS = {
    "Auto": 0.0,   # Special value for Auto ISO (handled by AE algorithm)
    "100": 1.0,
    "200": 2.0,
    "400": 4.0,
    "800": 8.0,
    "1600": 16.0,
    # Add more if supported and needed
}
DEFAULT_ISO_NAME = "Auto"
if DEFAULT_ISO_NAME not in AVAILABLE_ISO_SETTINGS:
    DEFAULT_ISO_NAME = list(AVAILABLE_ISO_SETTINGS.keys())[0] # Fallback to first entry
DEFAULT_ANALOGUE_GAIN = AVAILABLE_ISO_SETTINGS[DEFAULT_ISO_NAME]

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
    logging.warning("Warning: controls.draft.NoiseReductionModeEnum not found. Noise reduction control disabled.")
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

# --- Audio Configuration (Modification 2) ---
AUDIO_ENABLED = True                  # Set to False to disable audio recording
AUDIO_DEVICE_HINT = "USB"             # Hint to find the USB microphone (e.g., part of the name)
AUDIO_SAMPLE_RATE = 44100             # Sample rate in Hz
AUDIO_CHANNELS = 1                    # Number of audio channels (1 for mono, 2 for stereo)
AUDIO_FORMAT = 'int16'                # Sample format (e.g., 'float32', 'int16', 'int32')
AUDIO_BLOCK_SIZE = 1024               # Buffer size for audio stream
AUDIO_TEMP_EXTENSION = ".wav"         # Temporary file format for raw audio
FFMPEG_PATH = "/usr/bin/ffmpeg"       # Path to ffmpeg executable (adjust if needed)
# FFMPEG_LOG_LEVEL = "warning"        # Log level for ffmpeg process ('quiet', 'info', 'warning', 'error')
FFMPEG_LOG_LEVEL = "error"            # Keep ffmpeg quiet unless there's an error
AUDIO_MUX_TIMEOUT = 60                # Max seconds to wait for ffmpeg muxing process

# --- Error Handling ---
MAX_CONSECUTIVE_CAPTURE_ERRORS = 15 # How many capture errors before trying to restart/shutdown

# --- Logging Configuration ---
LOG_LEVEL = "DEBUG" # Options: DEBUG, INFO, WARNING, ERROR, CRITICAL
LOG_FORMAT = '%(asctime)s - %(levelname)s - [%(threadName)s] - %(message)s'
LOG_DATE_FORMAT = '%Y-%m-%d %H:%M:%S'

# --- Sanity Checks ---
if not isinstance(CAM0_RESOLUTIONS, list) or not CAM0_RESOLUTIONS:
    logging.critical("Config Error: CAM0_RESOLUTIONS must be a non-empty list.")
    # Potentially raise an error or exit here
if not all(isinstance(res, tuple) and len(res) == 3 for res in CAM0_RESOLUTIONS):
     logging.critical("Config Error: Each item in CAM0_RESOLUTIONS must be a tuple of (width, height, fps).")

if not isinstance(CAM1_RESOLUTION, tuple) or len(CAM1_RESOLUTION) != 2:
     logging.critical("Config Error: CAM1_RESOLUTION must be a tuple of (width, height).")

if not isinstance(AVAILABLE_ISO_SETTINGS, dict) or not AVAILABLE_ISO_SETTINGS:
     logging.critical("Config Error: AVAILABLE_ISO_SETTINGS must be a non-empty dictionary.")

