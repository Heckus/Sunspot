# Code/Config.py
# config.py

import os
import numpy as np

# --- General ---
LOG_LEVEL = "INFO"  # Options: DEBUG, INFO, WARNING, ERROR, CRITICAL
LOG_FORMAT = '%(asctime)s - %(levelname)s - [%(threadName)s:%(lineno)d] - %(message)s'
LOG_DATE_FORMAT = '%Y-%m-%d %H:%M:%S'

# --- File Paths ---
try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    SCRIPT_DIR = os.path.abspath('.')
    print(f"Warning: Could not determine script directory. Using current directory: {SCRIPT_DIR}")

CALIBRATION_DATA_FILE = os.path.join(SCRIPT_DIR, "camera_calibration_data.npz")
CALIBRATION_IMAGE_DIR = os.path.join(SCRIPT_DIR, "calibration_images")

# --- Web Server Configuration ---
WEB_UI_ENABLED = True
WEB_PORT = 8000
MAX_CONSECUTIVE_CAPTURE_ERRORS = 15
# For WebUi.py MJPEG stream
WEB_STREAM_JPEG_QUALITY = 75 # 0-100, higher is better quality, larger size
WEB_STREAM_MAX_FPS = 20      # Cap FPS for web stream to reduce load

# --- Camera Configuration ---
CAMERA_INDEX = 0
CAM_REQUESTED_WIDTH = 1920
CAM_REQUESTED_HEIGHT = 1080
CAM_REQUESTED_FPS = 20.0

CAMERA_INTRINSIC_MTX = np.array([[1000.0, 0.0, CAM_REQUESTED_WIDTH/2],
                                 [0.0, 1000.0, CAM_REQUESTED_HEIGHT/2],
                                 [0.0, 0.0, 1.0]], dtype=np.float32)
CAMERA_DIST_COEFFS = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)

# --- Checkerboard Configuration (for camera_calibration.py) ---
CHECKERBOARD_DIMS = (10, 7)
CHECKERBOARD_INNER_CORNERS_WIDTH = 10
CHECKERBOARD_INNER_CORNERS_HEIGHT = 7
SQUARE_SIZE_MM = 20.0
NUM_CALIBRATION_IMAGES = 30

# --- World Coordinate System (MODIFIED FOR 3D) ---
# Define a 1x1x1 meter cube for full 3D extrinsic calibration.
BOX_WIDTH_M = 1.0
BOX_DEPTH_M = 1.0
BOX_HEIGHT_M = 1.0 # Changed from 0.05 to 1.0

# The 8 corners of the cube. The order is important for calibration.
# Bottom face (Z=0), then top face (Z=BOX_HEIGHT_M)
WORLD_BOX_CORNERS_M = np.array([
    # Bottom face (Z=0) - Click these first
    [0.0, 0.0, 0.0],                # 1. Origin (Bottom-Front-Left)
    [BOX_WIDTH_M, 0.0, 0.0],        # 2. X-axis (Bottom-Front-Right)
    [BOX_WIDTH_M, BOX_DEPTH_M, 0.0],# 3. X-Y (Bottom-Back-Right)
    [0.0, BOX_DEPTH_M, 0.0],        # 4. Y-axis (Bottom-Back-Left)
    # Top face (Z=BOX_HEIGHT_M) - Click these second
    [0.0, 0.0, BOX_HEIGHT_M],                # 5. Top-Front-Left
    [BOX_WIDTH_M, 0.0, BOX_HEIGHT_M],        # 6. Top-Front-Right
    [BOX_WIDTH_M, BOX_DEPTH_M, BOX_HEIGHT_M],# 7. Top-Back-Right
    [0.0, BOX_DEPTH_M, BOX_HEIGHT_M]         # 8. Top-Back-Left
], dtype=np.float32)


# --- Volleyball Detection (CV_FUNCTIONS) ---
# HSV Color Ranges for Mikasa Volleyball (Yellow and Blue)
# These values are CRITICAL and need to be tuned based on your specific ball and lighting.
# Use a tool to pick HSV values from your image (volleyball.jpg) if these don't work.
# Hue: 0-179 (OpenCV), Saturation: 0-255, Value: 0-255

# For Yellow in volleyball.jpg: (Adjust these by testing)
LOWER_YELLOW_HSV = np.array([20, 120, 120]) # Hue: 20-35, Sat: >120, Val: >120
UPPER_YELLOW_HSV = np.array([35, 255, 255])

# For Blue in volleyball.jpg: (Adjust these by testing)
LOWER_BLUE_HSV = np.array([95, 100, 40])  # Hue: 95-125, Sat: >100, Val: 40-200 (it's a dark blue)
UPPER_BLUE_HSV = np.array([125, 255, 220]) # Allow slightly brighter value if highlights occur

# Morphological operation kernel size
MORPH_KERNEL_SIZE = (5, 5) # ( dość mały kernel, aby nie usunąć małych detekcji)

# Region of Interest (ROI) for ball detection
BALL_DETECTION_ROI = None # Example: (100, 200, 1000, 600) -> (x, y, width, height)

# Minimum and maximum expected ball radius in pixels (for filtering detections)
# These are highly dependent on camera distance and ball size.
# Measure the ball's radius in pixels from your test images if possible.
MIN_BALL_RADIUS_PX = 15
MAX_BALL_RADIUS_PX = 200
MIN_BALL_CONTOUR_AREA = np.pi * (MIN_BALL_RADIUS_PX**2) * 0.6 # Area can be less than perfect circle
MAX_BALL_CONTOUR_AREA = np.pi * (MAX_BALL_RADIUS_PX**2) * 1.4

VOLLEYBALL_RADIUS_M = 0.105 # Approx. 10.5 cm

# --- 3D Visualization (Three.js in WebUI) ---
# Camera position: X (center), Y (in front), Z (elevated)
# Look at the center of the box volume.
# These values are in meters.
VIS_CAMERA_POSITION_THREEJS_X_M = BOX_WIDTH_M / 2
VIS_CAMERA_POSITION_THREEJS_Y_M = BOX_DEPTH_M / 2 - BOX_DEPTH_M * 1.8
VIS_CAMERA_POSITION_THREEJS_Z_M = BOX_HEIGHT_M / 2 + max(BOX_WIDTH_M, BOX_DEPTH_M) * 1.5

VIS_CAMERA_LOOKAT_THREEJS_X_M = BOX_WIDTH_M / 2
VIS_CAMERA_LOOKAT_THREEJS_Y_M = BOX_DEPTH_M / 2
VIS_CAMERA_LOOKAT_THREEJS_Z_M = BOX_HEIGHT_M / 2

# These will be passed as lists/arrays to JavaScript
VIS_CAMERA_POSITION_THREEJS = [VIS_CAMERA_POSITION_THREEJS_X_M, VIS_CAMERA_POSITION_THREEJS_Y_M, VIS_CAMERA_POSITION_THREEJS_Z_M]
VIS_CAMERA_LOOKAT_THREEJS = [VIS_CAMERA_LOOKAT_THREEJS_X_M, VIS_CAMERA_LOOKAT_THREEJS_Y_M, VIS_CAMERA_LOOKAT_THREEJS_Z_M]


# --- Hardware Manager ---
BATTERY_MONITOR_ENABLED = False
SWITCH_GPIO_PIN = None
SWITCH_BOUNCE_TIME = 0.1
INA219_I2C_ADDRESS = 0x40
INA219_I2C_BUS = 1
BATTERY_READ_INTERVAL = 30.0
BATTERY_MAX_VOLTAGE = 12.6
BATTERY_MIN_VOLTAGE = 9.0
INA219_CALIBRATION_CONFIG = "16V_5A"
SERVO_ENABLED = False
# ... (other servo configs if ever used)

# --- Main Loop Configuration ---
MAIN_LOOP_MAX_FPS = 30
DISPLAY_2D_FEED = True

# --- Sanity Checks & Derived Configs ---
if CHECKERBOARD_INNER_CORNERS_WIDTH <= 0 or CHECKERBOARD_INNER_CORNERS_HEIGHT <= 0:
    raise ValueError("Checkerboard inner corner dimensions must be positive.")
if SQUARE_SIZE_MM <= 0:
    raise ValueError("Checkerboard square size must be positive.")

valid_log_levels = ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]
if LOG_LEVEL.upper() not in valid_log_levels:
    print(f"Warning: Invalid LOG_LEVEL '{LOG_LEVEL}'. Defaulting to INFO.")
    LOG_LEVEL = "INFO"

print(f"Config loaded. Calibration data expected at: {CALIBRATION_DATA_FILE}")
