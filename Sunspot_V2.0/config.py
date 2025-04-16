# -*- coding: utf-8 -*-
"""
config.py

Configuration constants for the Pi Camera Stream & Record application.
Handles settings for the camera, hardware, and the web UI.
Refactored for single-camera (Cam0) operation.
Includes settings for smooth servo movement.
"""
import os
import math # Import math if needed for calculations here, otherwise import in hardware_manager
from libcamera import controls
from picamera2 import Picamera2 # Needed for Picamera2.load_tuning_file

# --- General ---
LOG_LEVEL = "INFO" # Options: DEBUG, INFO, WARNING, ERROR, CRITICAL
LOG_FORMAT = '%(asctime)s - %(levelname)s - [%(threadName)s:%(lineno)d] - %(message)s'
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
# === Camera Configuration ===
# ===========================================================
CAMERA_ID = 0 # Physical camera number

# Define available resolutions and TARGET frame rates for the camera
# Format: (Width, Height, Target_FPS)
CAMERA_RESOLUTIONS = [
    (640, 480, 30.0),      # VGA
    (1332, 990, 120.0),     # 720p HD (Check if sensor supports this rate)
    (1920, 1080, 30.0),    # 1080p FHD (Adjusted FPS for stability)
    (2028, 1080, 50.0),    # Specific HQ mode (adjust FPS if needed)
    (2028, 1520, 40.0),    # Specific HQ mode (adjust FPS if needed)
    (4056, 3040, 10.0)     # Max Native Resolution IMX477 (Max FPS ~10)
]
DEFAULT_RESOLUTION_INDEX = 2 # Default to 1920x1080 @ 30fps

# --- Camera Tuning ---
# Set to None to use default tuning, or provide path to a valid JSON tuning file
# Example: TUNING_FILE_PATH = "/path/to/your/imx477_tuning.json"
TUNING_FILE_PATH = None
# Load tuning data if path is provided and file exists
TUNING = None
if TUNING_FILE_PATH:
    if os.path.exists(TUNING_FILE_PATH):
        try:
            TUNING = Picamera2.load_tuning_file(TUNING_FILE_PATH)
            print(f"Successfully loaded camera tuning file: {TUNING_FILE_PATH}")
        except Exception as e:
            print(f"Error loading camera tuning file '{TUNING_FILE_PATH}': {e}. Using default tuning.")
            TUNING = None # Ensure it's None on error
    else:
        print(f"Camera tuning file not found at {TUNING_FILE_PATH}. Using default tuning.")
else:
     print("Using default camera tuning (no tuning file specified).")

# --- Camera Recording Configuration ---
RECORDING_FORMAT = "mp4v" # Codec for OpenCV VideoWriter (e.g., "mp4v", "XVID")
RECORDING_EXTENSION = ".mp4"

# ===========================================================
# === Camera Control Defaults & Ranges ===
# ===========================================================

# --- Auto White Balance (AWB) ---
AVAILABLE_AWB_MODES = list(controls.AwbModeEnum.__members__.keys())
DEFAULT_AWB_MODE_NAME = "Auto"
if DEFAULT_AWB_MODE_NAME not in AVAILABLE_AWB_MODES:
    DEFAULT_AWB_MODE_NAME = AVAILABLE_AWB_MODES[0] if AVAILABLE_AWB_MODES else "Auto" # Fallback

# --- ISO / Analogue Gain ---
AVAILABLE_ISO_SETTINGS = {
    "Auto": 0.0, # 0.0 typically means Auto Gain in libcamera controls
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
AUDIO_DEVICE_HINT = "USB" # Hint to find the correct audio input device
AUDIO_SAMPLE_RATE = 44100 # Sample rate in Hz
AUDIO_CHANNELS = 1 # Number of audio channels (1 for mono, 2 for stereo)
AUDIO_FORMAT = 'int16' # Data type for audio samples (e.g., 'int16', 'float32')
AUDIO_BLOCK_SIZE = 1024 # Number of frames per buffer in audio callback
AUDIO_TEMP_EXTENSION = ".wav" # Extension for temporary audio file
AUDIO_MUX_TIMEOUT = 60 # Timeout in seconds for the ffmpeg muxing process
# Multiplier for mux timeout (kept at 1 as video is copied, not re-encoded)
AUDIO_MUX_RECODE_TIMEOUT_MULTIPLIER = 1
FFMPEG_PATH = "/usr/bin/ffmpeg" # Path to ffmpeg executable
FFMPEG_LOG_LEVEL = "error" # ffmpeg log level (e.g., "quiet", "error", "warning", "info", "debug")


# ===========================================================
# === Hardware Configuration ===
# ===========================================================
SWITCH_GPIO_PIN = 17 # GPIO pin for the physical recording switch (BCM numbering)
SWITCH_BOUNCE_TIME = 0.1 # Debounce time for the switch in seconds

# --- Battery Monitor (INA219) ---
INA219_I2C_ADDRESS = 0x41 # I2C address of the INA219 sensor
INA219_I2C_BUS = 1 # I2C bus number (usually 1 on Raspberry Pi)
BATTERY_READ_INTERVAL = 30.0 # How often to read battery level (seconds)
BATTERY_MAX_VOLTAGE = 12.6 # Voltage considered 100%
BATTERY_MIN_VOLTAGE = 9.0 # Voltage considered 0%
INA219_CALIBRATION_CONFIG = "16V_5A" # Predefined calibration setting in hardware_manager.py

# --- Servo Motor ---
SERVO_ENABLED = True # Set to False to disable servo control
SERVO_PWM_CHIP = 0 # PWM chip number (check /sys/class/pwm/)
SERVO_PWM_CHANNEL = 0 # PWM channel number on the chip
SERVO_GPIO_PIN = 12 # GPIO pin associated with the PWM channel (for reference/logging)
SERVO_PERIOD_NS = 20000000 # PWM period in nanoseconds (20ms = 50Hz)
SERVO_MIN_DUTY_NS = 500000 # Duty cycle for minimum angle (0.5ms)
SERVO_MAX_DUTY_NS = 2500000 # Duty cycle for maximum angle (2.5ms)
SERVO_CENTER_ANGLE = 90 # Angle corresponding to center position
SERVO_MIN_ANGLE = 0 # Minimum controllable angle
SERVO_MAX_ANGLE = 180 # Maximum controllable angle

# --- Servo Smooth Movement ---
SERVO_SMOOTH_MOVE = True # Enable smooth movement logic
SERVO_SMOOTH_MOVE_STEPS = 25 # Number of steps for smooth movement
SERVO_SMOOTH_MOVE_DELAY = 0.015 # Delay between steps in seconds (adjust for speed)

# --- Deprecated / Old Config Values ---
# (Removed for clarity)
