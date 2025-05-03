# -*- coding: utf-8 -*-
"""
config.py

Configuration constants for the Pi Camera Stream & Record application.
Handles settings for multiple cameras, hardware, and the web UI.

**Modification:** Reverted CAM0_RECORDING_FORMAT back to 'mp4v' as 'avc1'
                  failed to initialize with the available OpenCV/FFmpeg backend.
**Modification 2:** Added VIDEO_START_DELAY_SECONDS for A/V sync adjustment.
**Modification 3:** Removed VIDEO_START_DELAY_SECONDS, added AUDIO_START_DELAY_SECONDS.
**Modification 4 (User Request):** Disabled audio, removed audio-specific config.
**Modification 5 (User Request):** Added placeholder for camera intrinsic calibration data.
**Modification 6 (User Request):** Removed cv2.VideoWriter specific config for recording format/extension.
"""
import os
from libcamera import controls
from picamera2 import Picamera2 # Needed for Picamera2.load_tuning_file
import numpy as np # Added for camera intrinsic data

# --- General ---
LOG_LEVEL = "INFO" # Options: DEBUG, INFO, WARNING, ERROR, CRITICAL
LOG_FORMAT = '%(asctime)s - %(levelname)s - [%(threadName)s:%(lineno)d] - %(message)s'
LOG_DATE_FORMAT = '%Y-%m-%d %H:%M:%S'

# --- File Paths ---
try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    SCRIPT_DIR = os.path.abspath('.')
    print(f"Warning: Could not automatically determine script directory. Using current directory: {SCRIPT_DIR}")

USB_BASE_PATH = "/media/hecke/" # CHANGE this if your mount point base is different
TOGGLE_SCRIPT_NAME = "toggle.sh"
TOGGLE_SCRIPT_PATH = os.path.join(SCRIPT_DIR, TOGGLE_SCRIPT_NAME)

# --- Web Server Configuration ---
WEB_PORT = 8000

# --- Error Handling ---
MAX_CONSECUTIVE_CAPTURE_ERRORS = 15

# ===========================================================
# === Camera Enable Flags ===
# ===========================================================
ENABLE_CAM1 = False # Keeping secondary camera disabled as per original file

# ===========================================================
# === Camera 0 (Primary - e.g., HQ Camera) Configuration ===
# ===========================================================
CAM0_ID = 0

# Define available resolutions and TARGET frame rates for Cam0
# Format: (Width, Height, Target_FPS)
# *** Ensure the FPS value here is the rate the camera is configured to capture at ***
CAM0_RESOLUTIONS = [
    (640, 480, 30.0),      # VGA @ 30 FPS
    (1332, 990, 60.0),     # Custom @ 60 FPS (May stress Pi 5 depending on encoding load)
    (1920, 1080, 40),    # 1080p FHD @ 25 FPS
    (2028, 1080, 30.0),    # Specific HQ mode @ 50 FPS
    (2028, 1520, 30.0),    # Specific HQ mode @ 40 FPS
    (4056, 3040, 10.0)     # Max Res @ 10 FPS
]
CAM0_DEFAULT_RESOLUTION_INDEX = 2 # Default to 1920x1080 @ Target 25 FPS

# --- Cam0 Tuning ---
CAM0_TUNING_FILE_PATH = None
CAM0_TUNING = None
if CAM0_TUNING_FILE_PATH:
    if os.path.exists(CAM0_TUNING_FILE_PATH):
        try:
            CAM0_TUNING = Picamera2.load_tuning_file(CAM0_TUNING_FILE_PATH)
            print(f"Successfully loaded Cam0 tuning file: {CAM0_TUNING_FILE_PATH}")
        except Exception as e:
            print(f"Error loading Cam0 tuning file '{CAM0_TUNING_FILE_PATH}': {e}. Using default tuning.")
            CAM0_TUNING = None
    else:
        print(f"Cam0 tuning file not found at {CAM0_TUNING_FILE_PATH}. Using default tuning.")
else:
     print("Using default camera tuning for Cam0 (no tuning file specified).")

# --- Cam0 Recording Configuration ---
# REMOVED CAM0_RECORDING_FORMAT = "mp4v"
# REMOVED CAM0_RECORDING_EXTENSION = ".mp4"
# These will be handled by Picamera2's encoder/muxer now.

# ===========================================================
# === Camera 1 (Secondary - e.g., IMX219 NoIR) Configuration ===
# ===========================================================
# Cam1 remains disabled, config kept for potential future use but not active
CAM1_ID = 1
CAM1_RESOLUTION = (640, 480)
CAM1_FRAME_RATE = 30.0
CAM1_VFLIP = False
CAM1_HFLIP = False
CAM1_USE_NOIR_TUNING = True
CAM1_NOIR_TUNING_FILE_PATH = "/usr/share/libcamera/ipa/rpi/pisp/imx219_noir.json"
CAM1_TUNING = None
if ENABLE_CAM1 and CAM1_USE_NOIR_TUNING:
    if os.path.exists(CAM1_NOIR_TUNING_FILE_PATH):
        try: CAM1_TUNING = Picamera2.load_tuning_file(CAM1_NOIR_TUNING_FILE_PATH); print(f"Successfully loaded Cam1 NoIR tuning file: {CAM1_NOIR_TUNING_FILE_PATH}")
        except Exception as e: print(f"Error loading Cam1 NoIR tuning file '{CAM1_NOIR_TUNING_FILE_PATH}': {e}. Using default tuning."); CAM1_TUNING = None
    else: print(f"Cam1 NoIR tuning file not found at {CAM1_NOIR_TUNING_FILE_PATH}. Using default tuning.")
elif ENABLE_CAM1: print("Using default camera tuning for Cam1 (NoIR tuning disabled).")
else: print("Cam1 is disabled. Skipping Cam1 tuning file load.")


# ===========================================================
# === Combined Stream Configuration ===
# ===========================================================
STREAM_BORDER_SIZE = 5
STREAM_BORDER_COLOR = (64, 64, 64) # Gray

# ===========================================================
# === Common Camera Control Defaults & Ranges ===
# ===========================================================
AVAILABLE_AWB_MODES = list(controls.AwbModeEnum.__members__.keys())
DEFAULT_AWB_MODE_NAME = "Auto"
if DEFAULT_AWB_MODE_NAME not in AVAILABLE_AWB_MODES: DEFAULT_AWB_MODE_NAME = AVAILABLE_AWB_MODES[0] if AVAILABLE_AWB_MODES else "Auto"

AVAILABLE_ISO_SETTINGS = {"Auto": 0.0, "100": 1.0, "200": 2.0, "400": 4.0, "800": 8.0, "1600": 16.0}
DEFAULT_ISO_NAME = "Auto"
DEFAULT_ANALOGUE_GAIN = AVAILABLE_ISO_SETTINGS.get(DEFAULT_ISO_NAME, 0.0)

AVAILABLE_AE_MODES = list(controls.AeExposureModeEnum.__members__.keys())
DEFAULT_AE_MODE_NAME = "Normal"
if DEFAULT_AE_MODE_NAME not in AVAILABLE_AE_MODES: DEFAULT_AE_MODE_NAME = AVAILABLE_AE_MODES[0] if AVAILABLE_AE_MODES else "Normal"

AVAILABLE_METERING_MODES = list(controls.AeMeteringModeEnum.__members__.keys())
DEFAULT_METERING_MODE_NAME = "CentreWeighted"
if DEFAULT_METERING_MODE_NAME not in AVAILABLE_METERING_MODES: DEFAULT_METERING_MODE_NAME = AVAILABLE_METERING_MODES[0] if AVAILABLE_METERING_MODES else "CentreWeighted"

try:
    AVAILABLE_NOISE_REDUCTION_MODES = list(controls.draft.NoiseReductionModeEnum.__members__.keys())
    DEFAULT_NOISE_REDUCTION_MODE_NAME = "Fast"
    if DEFAULT_NOISE_REDUCTION_MODE_NAME not in AVAILABLE_NOISE_REDUCTION_MODES: DEFAULT_NOISE_REDUCTION_MODE_NAME = AVAILABLE_NOISE_REDUCTION_MODES[0] if AVAILABLE_NOISE_REDUCTION_MODES else "Off"
except AttributeError:
    print("Warning: controls.draft.NoiseReductionModeEnum not found. Noise reduction control disabled.")
    AVAILABLE_NOISE_REDUCTION_MODES = ["Off"]; DEFAULT_NOISE_REDUCTION_MODE_NAME = "Off"

DEFAULT_BRIGHTNESS = 0.0; MIN_BRIGHTNESS = -1.0; MAX_BRIGHTNESS = 1.0; STEP_BRIGHTNESS = 0.1
DEFAULT_CONTRAST = 1.0; MIN_CONTRAST = 0.0; MAX_CONTRAST = 2.0; STEP_CONTRAST = 0.1
DEFAULT_SATURATION = 1.0; MIN_SATURATION = 0.0; MAX_SATURATION = 2.0; STEP_SATURATION = 0.1
DEFAULT_SHARPNESS = 1.0; MIN_SHARPNESS = 0.0; MAX_SHARPNESS = 2.0; STEP_SHARPNESS = 0.1

# ===========================================================
# === Audio Configuration (DISABLED) ===
# ===========================================================
AUDIO_ENABLED = False # Requirement 1: Disable audio completely

# ===========================================================
# === Computer Vision (CV) Configuration ===
# ===========================================================
CAMERA_INTRINSIC_MATRIX = np.array([
    [1000.0,    0.0, 640.0], # [fx, 0, cx]
    [   0.0, 1000.0, 480.0], # [0, fy, cy]
    [   0.0,    0.0,   1.0]  # [0, 0,  1]
]) # Example for 1280x960 assumed principal point, adjust fx, fy based on calibration
CAMERA_DISTORTION_COEFFS = np.array([0.0, 0.0, 0.0, 0.0, 0.0]) # Example: [k1, k2, p1, p2, k3], adjust based on calibration

CV_PROCESSING_ENABLED = True # Flag to enable/disable CV processing pipeline
CV_INPUT_SCALING_FACTOR = 0.5 # Example: Process CV on frames scaled down by 50%
CV_APPLY_GRAYSCALE = False # Example: Flag for potential grayscale preprocessing

# ===========================================================
# === Recording Configuration ===
# ===========================================================
# Recording uses Picamera2's H264Encoder by default now.
# Target FPS/Resolution enforcement handled in camera_manager using camera controls
# based on CAM0_RESOLUTIONS.
RECORDING_VIDEO_BITRATE = 10000000 # Example: 10 Mbps for H.264 encoding (adjust as needed)

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

SERVO_SMOOTH_MOVE = True
SERVO_SMOOTH_MOVE_STEPS = 20
SERVO_SMOOTH_MOVE_DELAY = 0.01
# -----------------------------------------------------------
