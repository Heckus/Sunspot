# Code/Config.py
# config.py

import os
import numpy as np

# --- General ---
LOG_LEVEL = "INFO"
LOG_FORMAT = '%(asctime)s - %(levelname)s - [%(threadName)s:%(lineno)d] - %(message)s'
LOG_DATE_FORMAT = '%Y-%m-%d %H:%M:%S'

# --- File Paths ---
try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
except NameError:
    SCRIPT_DIR = os.path.abspath('.')

CALIBRATION_DATA_FILE = os.path.join(SCRIPT_DIR, "camera_calibration_data.npz")
CALIBRATION_IMAGE_DIR = os.path.join(SCRIPT_DIR, "calibration_images")

# --- NEW: YOLO Model Configuration ---
# Path to your trained YOLO model file.
# Place your 'yolo.pt' file in the same directory as this script.
YOLO_MODEL_PATH = os.path.join(SCRIPT_DIR, "../ComputerVisionResearch/version2/models/models_ball2.pt")
# Confidence threshold for detection. Detections below this score will be ignored.
YOLO_CONFIDENCE_THRESHOLD = 0.65


# --- Web Server Configuration ---
WEB_UI_ENABLED = True
WEB_PORT = 8000
MAX_CONSECUTIVE_CAPTURE_ERRORS = 15
WEB_STREAM_JPEG_QUALITY = 75
WEB_STREAM_MAX_FPS = 20

# --- Camera Configuration ---
CAMERA_INDEX = 0
CAM_REQUESTED_WIDTH = 1920
CAM_REQUESTED_HEIGHT = 1080
CAM_REQUESTED_FPS = 10.0

# --- Checkerboard Configuration (for camera_calibration.py) ---
CHECKERBOARD_DIMS = (10, 7)
SQUARE_SIZE_MM = 20.0
NUM_CALIBRATION_IMAGES = 30

# --- World Coordinate System (For 3D Calibration) ---
# We define all 8 corners here, but the calibration script in main.py will only use 6.
BOX_WIDTH_M = 1.0
BOX_DEPTH_M = 1.0
BOX_HEIGHT_M = 1.0

WORLD_BOX_CORNERS_M = np.array([
    # Bottom face (Z=0)
    [0.0, 0.0, 0.0],                # 1. Origin (Bottom-Front-Left)
    [BOX_WIDTH_M, 0.0, 0.0],        # 2. X-axis (Bottom-Front-Right)
    [BOX_WIDTH_M, BOX_DEPTH_M, 0.0],# 3. X-Y (Bottom-Back-Right)
    [0.0, BOX_DEPTH_M, 0.0],        # 4. Y-axis (Bottom-Back-Left)
    # Top face (Z=BOX_HEIGHT_M)
    [0.0, 0.0, BOX_HEIGHT_M],                # Corresponds to point 1
    [BOX_WIDTH_M, 0.0, BOX_HEIGHT_M],        # Corresponds to point 2
    [BOX_WIDTH_M, BOX_DEPTH_M, BOX_HEIGHT_M],# Corresponds to point 3
    [0.0, BOX_DEPTH_M, BOX_HEIGHT_M]         # Corresponds to point 4
], dtype=np.float32)

# --- Volleyball Properties ---
# These are still crucial for the 3D position estimation from radius
VOLLEYBALL_RADIUS_M = 0.105

# --- 3D Visualization (Three.js in WebUI) ---
VIS_CAMERA_POSITION_THREEJS_X_M = BOX_WIDTH_M / 2
VIS_CAMERA_POSITION_THREEJS_Y_M = BOX_DEPTH_M / 2 - BOX_DEPTH_M * 1.8
VIS_CAMERA_POSITION_THREEJS_Z_M = BOX_HEIGHT_M / 2 + max(BOX_WIDTH_M, BOX_DEPTH_M) * 1.5
VIS_CAMERA_LOOKAT_THREEJS_X_M = BOX_WIDTH_M / 2
VIS_CAMERA_LOOKAT_THREEJS_Y_M = BOX_DEPTH_M / 2
VIS_CAMERA_LOOKAT_THREEJS_Z_M = BOX_HEIGHT_M / 2
VIS_CAMERA_POSITION_THREEJS = [VIS_CAMERA_POSITION_THREEJS_X_M, VIS_CAMERA_POSITION_THREEJS_Y_M, VIS_CAMERA_POSITION_THREEJS_Z_M]
VIS_CAMERA_LOOKAT_THREEJS = [VIS_CAMERA_LOOKAT_THREEJS_X_M, VIS_CAMERA_LOOKAT_THREEJS_Y_M, VIS_CAMERA_LOOKAT_THREEJS_Z_M]

# --- Main Loop Configuration ---
DISPLAY_2D_FEED = True

print(f"Config loaded. YOLO model expected at: {YOLO_MODEL_PATH}")
