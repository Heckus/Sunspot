# -*- coding: utf-8 -*-
"""
config.py

Configuration constants for the Pi Camera Stream & Record application.
Handles settings for multiple cameras, hardware, and the web UI.

**Modification:** Reverted AUDIO_MUX_RECODE_TIMEOUT_MULTIPLIER to 1 (as video is copied again).
"""
import os
from libcamera import controls
from picamera2 import Picamera2 # Needed for Picamera2.load_tuning_file

# --- General ---
LOG_LEVEL = "INFO" # Options: DEBUG, INFO, WARNING, ERROR, CRITICAL
LOG_FORMAT = '%(asctime)s - %(levelname)s - [%(threadName)s:%(lineno)d] - %(message)s' # Added line number
LOG_DATE_FORMAT = '%Y-%m-%d %H:%M:%S'

# --- File Paths ---
# Determine script directory to find toggle script and potential tuning files
try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    # Fallback if __file__ is not defined (e.g., interactive mode)
    SCRIPT_DIR = os.path.abspath('.')
    print(f"Warning: Could not automatically determine script directory. Using current directory: {SCRIPT_DIR}")

USB_BASE_PATH = "/media/hecke/" # CHANGE this if your mount point base is different
TOGGLE_SCRIPT_NAME = "toggle.sh"
TOGGLE_SCRIPT_PATH = os.path.join(SCRIPT_DIR, TOGGLE_SCRIPT_NAME)

# --- Web Server Configuration ---
WEB_PORT = 8000

# --- Error Handling ---
MAX_CONSECUTIVE_CAPTURE_ERRORS = 15 # How many capture errors before trying to restart/shutdown

# ===========================================================
# === Camera Enable Flags ===
# ===========================================================
# Set ENABLE_CAM1 to False to run in single-camera mode (only Cam0)
ENABLE_CAM1 = False 

# ===========================================================
# === Camera 0 (Primary - e.g., HQ Camera) Configuration ===
# ===========================================================
CAM0_ID = 0 # Physical camera number (usually 0 for the first camera)

# Define available resolutions and TARGET frame rates for Cam0
# Format: (Width, Height, Target_FPS)
CAM0_RESOLUTIONS = [
    (640, 480, 30.0),      # VGA
    (1332, 990, 120.0),     # 720p HD
    (1920, 1080, 50.0),    # 1080p FHD
    (2028, 1080, 50.0),    # Specific HQ mode (adjust FPS if needed)
    (2028, 1520, 40.0),    # Specific HQ mode (adjust FPS if needed)
    (4056, 3040, 10.0)     # Max Native Resolution IMX477 (Max FPS ~10)
]
CAM0_DEFAULT_RESOLUTION_INDEX = 2 # Default to 1920x1080

# --- Cam0 Tuning ---
# Set to None to use default tuning, or provide path to a valid JSON tuning file
# Example: CAM0_TUNING_FILE_PATH = "/path/to/your/imx477_tuning.json"
CAM0_TUNING_FILE_PATH = None
# Load tuning data if path is provided and file exists
CAM0_TUNING = None
if CAM0_TUNING_FILE_PATH:
    if os.path.exists(CAM0_TUNING_FILE_PATH):
        try:
            CAM0_TUNING = Picamera2.load_tuning_file(CAM0_TUNING_FILE_PATH)
            print(f"Successfully loaded Cam0 tuning file: {CAM0_TUNING_FILE_PATH}")
        except Exception as e:
            print(f"Error loading Cam0 tuning file '{CAM0_TUNING_FILE_PATH}': {e}. Using default tuning.")
            CAM0_TUNING = None # Ensure it's None on error
    else:
        print(f"Cam0 tuning file not found at {CAM0_TUNING_FILE_PATH}. Using default tuning.")
else:
     print("Using default camera tuning for Cam0 (no tuning file specified).")

# --- Cam0 Recording Configuration ---
CAM0_RECORDING_FORMAT = "mp4v" # Codec for OpenCV VideoWriter (e.g., "mp4v", "XVID")
CAM0_RECORDING_EXTENSION = ".mp4"

# ===========================================================
# === Camera 1 (Secondary - e.g., IMX219 NoIR) Configuration ===
# ===========================================================
# These settings are only used if ENABLE_CAM1 is True
CAM1_ID = 1 # Physical camera number (usually 1 for the second camera)

# Fixed resolution and frame rate for Cam1 (used for combined stream)
CAM1_RESOLUTION = (640, 480) # Lower resolution suitable for secondary stream
CAM1_FRAME_RATE = 30.0

# --- Cam1 Flipping ---
CAM1_VFLIP = False # Set to True if Cam1 image is vertically flipped
CAM1_HFLIP = False # Set to True if Cam1 image is horizontally flipped

# --- Cam1 Tuning (NoIR Example) ---
CAM1_USE_NOIR_TUNING = True # Set to False to use default tuning for Cam1
CAM1_NOIR_TUNING_FILE_PATH = "/usr/share/libcamera/ipa/rpi/pisp/imx219_noir.json"
# Load tuning data if enabled and file exists
CAM1_TUNING = None
if ENABLE_CAM1 and CAM1_USE_NOIR_TUNING: # Only load if Cam1 is enabled
    if os.path.exists(CAM1_NOIR_TUNING_FILE_PATH):
        try:
            CAM1_TUNING = Picamera2.load_tuning_file(CAM1_NOIR_TUNING_FILE_PATH)
            print(f"Successfully loaded Cam1 NoIR tuning file: {CAM1_NOIR_TUNING_FILE_PATH}")
        except Exception as e:
            print(f"Error loading Cam1 NoIR tuning file '{CAM1_NOIR_TUNING_FILE_PATH}': {e}. Using default tuning.")
            CAM1_TUNING = None # Ensure it's None on error
    else:
        print(f"Cam1 NoIR tuning file not found at {CAM1_NOIR_TUNING_FILE_PATH}. Using default tuning.")
elif ENABLE_CAM1:
     print("Using default camera tuning for Cam1 (NoIR tuning disabled).")
else:
    print("Cam1 is disabled. Skipping Cam1 tuning file load.")


# ===========================================================
# === Combined Stream Configuration ===
# ===========================================================
# These settings are only used if ENABLE_CAM1 is True
STREAM_BORDER_SIZE = 5 # Pixels between Cam0 and Cam1 in combined stream
STREAM_BORDER_COLOR = (64, 64, 64) # BGR color for the border

# ===========================================================
# === Common Camera Control Defaults & Ranges ===
# ===========================================================
# These apply to BOTH cameras (if Cam1 enabled) when changed via UI/API

# --- Auto White Balance (AWB) ---
AVAILABLE_AWB_MODES = list(controls.AwbModeEnum.__members__.keys())
DEFAULT_AWB_MODE_NAME = "Auto"
if DEFAULT_AWB_MODE_NAME not in AVAILABLE_AWB_MODES:
    DEFAULT_AWB_MODE_NAME = AVAILABLE_AWB_MODES[0] if AVAILABLE_AWB_MODES else "Auto" # Fallback

# --- ISO / Analogue Gain ---
AVAILABLE_ISO_SETTINGS = {
    "Auto": 0.0,
    "100": 1.0,
    "200": 2.0,
    "400": 4.0,
    "800": 8.0,
    "1600": 16.0,
}
DEFAULT_ISO_NAME = "Auto"
DEFAULT_ANALOGUE_GAIN = AVAILABLE_ISO_SETTINGS.get(DEFAULT_ISO_NAME, 0.0)

# --- Auto Exposure (AE) ---
AVAILABLE_AE_MODES = list(controls.AeExposureModeEnum.__members__.keys())
DEFAULT_AE_MODE_NAME = "Normal"
if DEFAULT_AE_MODE_NAME not in AVAILABLE_AE_MODES:
    DEFAULT_AE_MODE_NAME = AVAILABLE_AE_MODES[0] if AVAILABLE_AE_MODES else "Normal"

# --- AE Metering Mode ---
AVAILABLE_METERING_MODES = list(controls.AeMeteringModeEnum.__members__.keys())
DEFAULT_METERING_MODE_NAME = "CentreWeighted"
if DEFAULT_METERING_MODE_NAME not in AVAILABLE_METERING_MODES:
    DEFAULT_METERING_MODE_NAME = AVAILABLE_METERING_MODES[0] if AVAILABLE_METERING_MODES else "CentreWeighted"

# --- Noise Reduction Mode ---
try:
    AVAILABLE_NOISE_REDUCTION_MODES = list(controls.draft.NoiseReductionModeEnum.__members__.keys())
    DEFAULT_NOISE_REDUCTION_MODE_NAME = "Fast"
    if DEFAULT_NOISE_REDUCTION_MODE_NAME not in AVAILABLE_NOISE_REDUCTION_MODES:
        DEFAULT_NOISE_REDUCTION_MODE_NAME = AVAILABLE_NOISE_REDUCTION_MODES[0] if AVAILABLE_NOISE_REDUCTION_MODES else "Off"
except AttributeError:
    print("Warning: controls.draft.NoiseReductionModeEnum not found. Noise reduction control disabled.")
    AVAILABLE_NOISE_REDUCTION_MODES = ["Off"]
    DEFAULT_NOISE_REDUCTION_MODE_NAME = "Off"

# --- Manual Adjustments (Sliders) ---
DEFAULT_BRIGHTNESS = 0.0
MIN_BRIGHTNESS = -1.0
MAX_BRIGHTNESS = 1.0
STEP_BRIGHTNESS = 0.1

DEFAULT_CONTRAST = 1.0
MIN_CONTRAST = 0.0
MAX_CONTRAST = 2.0
STEP_CONTRAST = 0.1

DEFAULT_SATURATION = 1.0
MIN_SATURATION = 0.0
MAX_SATURATION = 2.0
STEP_SATURATION = 0.1

DEFAULT_SHARPNESS = 1.0
MIN_SHARPNESS = 0.0
MAX_SHARPNESS = 2.0
STEP_SHARPNESS = 0.1


# ===========================================================
# === Audio Configuration ===
# ===========================================================
AUDIO_ENABLED = True
AUDIO_DEVICE_HINT = "USB"
AUDIO_SAMPLE_RATE = 44100
AUDIO_CHANNELS = 1
AUDIO_FORMAT = 'int16'
AUDIO_BLOCK_SIZE = 1024
AUDIO_TEMP_EXTENSION = ".wav"
AUDIO_MUX_TIMEOUT = 60
AUDIO_MUX_RECODE_TIMEOUT_MULTIPLIER = 1 # Reverted to 1 (no re-encode)
FFMPEG_PATH = "/usr/bin/ffmpeg"
FFMPEG_LOG_LEVEL = "error"


# ===========================================================
# === Hardware Configuration ===
# ===========================================================
SWITCH_GPIO_PIN = 17
SWITCH_BOUNCE_TIME = 0.1

INA219_I2C_ADDRESS = 0x41
INA219_I2C_BUS = 1
BATTERY_READ_INTERVAL = 30.0
BATTERY_MAX_VOLTAGE = 12.6
BATTERY_MIN_VOLTAGE = 9.0
INA219_CALIBRATION_CONFIG = "16V_5A"

SERVO_ENABLED = True
SERVO_PWM_CHIP = 0
SERVO_PWM_CHANNEL = 0
SERVO_GPIO_PIN = 12
SERVO_PERIOD_NS = 20000000
SERVO_MIN_DUTY_NS = 500000
SERVO_MAX_DUTY_NS = 2500000
SERVO_CENTER_ANGLE = 90
SERVO_MIN_ANGLE = 0
SERVO_MAX_ANGLE = 180

# --- Deprecated / Old Config Values ---
# (Removed for clarity)
