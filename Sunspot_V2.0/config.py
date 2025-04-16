# -*- coding: utf-8 -*-
"""
config.py

Configuration constants for the Pi Camera Stream & Record application.
Handles settings for multiple cameras, hardware, and the web UI.
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
    (1280, 720, 30.0),     # 720p HD
    (1920, 1080, 30.0),    # 1080p FHD
    (2028, 1080, 30.0),    # Specific HQ mode (adjust FPS if needed)
    (2028, 1520, 25.0),    # Specific HQ mode (adjust FPS if needed)
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

# --- ISO / Analogue Gain ---
# Define available ISO settings and their corresponding AnalogueGain values
# Gain 0.0 usually means Auto ISO for picamera2
AVAILABLE_ISO_SETTINGS = {
    "Auto": 0.0,
    "100": 1.0,
    "200": 2.0,
    "400": 4.0,
    "800": 8.0,
    "1600": 16.0,
    # Add more if supported/needed
}
DEFAULT_ISO_NAME = "Auto"
DEFAULT_ANALOGUE_GAIN = AVAILABLE_ISO_SETTINGS.get(DEFAULT_ISO_NAME, 0.0) # Default to Auto gain

# --- Auto Exposure (AE) ---
AVAILABLE_AE_MODES = list(controls.AeExposureModeEnum.__members__.keys())
DEFAULT_AE_MODE_NAME = "Normal"
if DEFAULT_AE_MODE_NAME not in AVAILABLE_AE_MODES:
    DEFAULT_AE_MODE_NAME = AVAILABLE_AE_MODES[0] if AVAILABLE_AE_MODES else "Normal" # Fallback

# --- AE Metering Mode ---
AVAILABLE_METERING_MODES = list(controls.AeMeteringModeEnum.__members__.keys())
DEFAULT_METERING_MODE_NAME = "CentreWeighted"
if DEFAULT_METERING_MODE_NAME not in AVAILABLE_METERING_MODES:
    DEFAULT_METERING_MODE_NAME = AVAILABLE_METERING_MODES[0] if AVAILABLE_METERING_MODES else "CentreWeighted" # Fallback

# --- Noise Reduction Mode (Using draft controls) ---
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

# --- Manual Adjustments (Sliders) ---
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


# ===========================================================
# === Audio Configuration ===
# ===========================================================
AUDIO_ENABLED = True          # Set to False to disable audio recording entirely
AUDIO_DEVICE_HINT = "USB"     # Part of the name of the desired USB audio device (e.g., "USB", "Microphone", "C-Media")
AUDIO_SAMPLE_RATE = 44100     # Sample rate in Hz (e.g., 44100, 48000)
AUDIO_CHANNELS = 1            # Number of audio channels (1 for mono, 2 for stereo)
AUDIO_FORMAT = 'int16'        # Data type for audio samples ('int16', 'float32', etc.) - must be supported by sounddevice/soundfile
AUDIO_BLOCK_SIZE = 1024       # Number of frames per buffer processed by sounddevice callback (adjust for performance/latency)
AUDIO_TEMP_EXTENSION = ".wav" # Temporary file extension for raw audio
AUDIO_MUX_TIMEOUT = 60        # Timeout in seconds for the ffmpeg muxing process (copying)
AUDIO_MUX_RECODE_TIMEOUT_MULTIPLIER = 3 # Multiply timeout by this when re-encoding video in ffmpeg
FFMPEG_PATH = "/usr/bin/ffmpeg" # Path to the ffmpeg executable
FFMPEG_LOG_LEVEL = "error"    # ffmpeg log level ('quiet', 'panic', 'fatal', 'error', 'warning', 'info', 'verbose', 'debug')


# ===========================================================
# === Hardware Configuration ===
# ===========================================================

# --- GPIO Switch ---
SWITCH_GPIO_PIN = 17          # BCM Pin number for the physical switch, Set to None to disable
SWITCH_BOUNCE_TIME = 0.1      # Debounce time in seconds for the switch

# --- Battery Monitor (INA219) ---
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

# --- Deprecated / Old Config Values (Keep for reference or remove later) ---
# USE_NOIR_TUNING = True # Replaced by CAM1_USE_NOIR_TUNING
# NOIR_TUNING_FILE_PATH = "/usr/share/libcamera/ipa/rpi/pisp/imx219_noir.json" # Replaced by CAM1_NOIR_TUNING_FILE_PATH
# CAMERA_TUNING = None # Replaced by CAM0_TUNING / CAM1_TUNING
# SUPPORTED_RESOLUTIONS = [...] # Replaced by CAM0_RESOLUTIONS
# DEFAULT_RESOLUTION_INDEX = 1 # Replaced by CAM0_DEFAULT_RESOLUTION_INDEX
# DEFAULT_FRAME_RATE = 30 # Used within CAM0_RESOLUTIONS now
# RECORDING_FORMAT = "mp4v" # Replaced by CAM0_RECORDING_FORMAT
# RECORDING_EXTENSION = ".mp4" # Replaced by CAM0_RECORDING_EXTENSION
# AVAILABLE_AWB_MODES = list(controls.AwbModeEnum.__members__.keys()) # AWB not currently implemented in UI/controls
# DEFAULT_AWB_MODE_NAME = "Auto" # AWB not currently implemented
