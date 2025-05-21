# config.py

import os
import numpy as np
from libcamera import controls # For potential future use if directly interfacing with libcamera controls

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

# Path to save/load camera calibration data
CALIBRATION_DATA_FILE = os.path.join(SCRIPT_DIR, "camera_calibration_data.npz")
# Path for saving calibration images (if camera_calibration.py is run from the same root)
CALIBRATION_IMAGE_DIR = os.path.join(SCRIPT_DIR, "calibration_images")

# --- Web Server Configuration ---
WEB_UI_ENABLED = True # Enable or disable the web UI
WEB_PORT = 8000
MAX_CONSECUTIVE_CAPTURE_ERRORS = 15 # From sample, might be useful

# --- Camera Configuration ---
CAMERA_INDEX = 0  # OpenCV camera index

# Default resolution and FPS for the camera, can be overridden by loaded calibration
CAM_REQUESTED_WIDTH = 1920 # From PDF, RPi HQ camera supports various resolutions
CAM_REQUESTED_HEIGHT = 1080
CAM_REQUESTED_FPS = 30.0

# Camera intrinsic parameters (to be loaded from CALIBRATION_DATA_FILE)
# These are just placeholders, actual values will be loaded.
CAMERA_INTRINSIC_MTX = np.array([[1000.0, 0.0, CAM_REQUESTED_WIDTH/2],
                                 [0.0, 1000.0, CAM_REQUESTED_HEIGHT/2],
                                 [0.0, 0.0, 1.0]], dtype=np.float32)
CAMERA_DIST_COEFFS = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)

# --- Checkerboard Configuration (for camera_calibration.py) ---
# Number of inner corners per row and column (e.g., a 9x6 board has 8x5 inner corners)
CHECKERBOARD_DIMS = (10, 7) # (width-1, height-1) as per PDF (e.g. 7,4) -> (8,5 squares), (7,4 inner corners)
                            # The PDF mentions (7,4) inner corners.
                            # The sample calibration script uses (10,7) inner corners.
                            # Let's use the PDF's example: (7,4) inner corners means an 8x5 board.
CHECKERBOARD_INNER_CORNERS_WIDTH = 10 # Number of inner corners along width
CHECKERBOARD_INNER_CORNERS_HEIGHT = 7 # Number of inner corners along height
SQUARE_SIZE_MM = 20.0  # Size of a chessboard square in millimeters (as per PDF example)
NUM_CALIBRATION_IMAGES = 20 # Number of images to capture for calibration (as per PDF)

# --- World Coordinate System ---
# Dimensions of the physical box
BOX_WIDTH_M = 2.0  # meters
BOX_DEPTH_M = 2.0  # meters (assuming Y-axis is depth into the scene from camera's perspective of the ground)
BOX_HEIGHT_M = 0.05 # Small height for visualizing the box base/tape thickness, as per PDF

# Defined 3D coordinates of the box corners on the ground (Z=0)
# Origin at Bottom-Front-Left, X along width, Y along depth
WORLD_BOX_CORNERS_M = np.array([
    [0.0, 0.0, 0.0],         # Corner 1 (Origin: Bottom-Front-Left)
    [BOX_WIDTH_M, 0.0, 0.0], # Corner 2 (Bottom-Front-Right)
    [BOX_WIDTH_M, BOX_DEPTH_M, 0.0], # Corner 3 (Bottom-Back-Right)
    [0.0, BOX_DEPTH_M, 0.0]  # Corner 4 (Bottom-Back-Left)
], dtype=np.float32)

# --- Volleyball Detection (CV_FUNCTIONS) ---
# HSV Color Ranges for Mikasa Volleyball (Yellow and Blue)
# These need to be tuned empirically. Values are (Hue, Saturation, Value)
# Hue: 0-179 (in OpenCV), Saturation: 0-255, Value: 0-255
LOWER_YELLOW_HSV = np.array([20, 100, 100]) # Example
UPPER_YELLOW_HSV = np.array([30, 255, 255]) # Example
LOWER_BLUE_HSV = np.array([100, 150, 50])   # Example
UPPER_BLUE_HSV = np.array([140, 255, 255])  # Example

# Morphological operation kernel size
MORPH_KERNEL_SIZE = (5, 5)

# Region of Interest (ROI) for ball detection (optional, can be set dynamically or manually)
# Format: (x_start, y_start, width, height) in pixels of the undistorted frame.
# If None, the whole frame is processed.
BALL_DETECTION_ROI = None # Example: (100, 200, 1000, 600)

# Minimum and maximum expected ball radius in pixels (for filtering detections)
# This is highly dependent on camera distance and ball size.
MIN_BALL_RADIUS_PX = 10 # Example
MAX_BALL_RADIUS_PX = 150 # Example
MIN_BALL_CONTOUR_AREA = np.pi * (MIN_BALL_RADIUS_PX**2) * 0.5 # Allow for some non-circularity
MAX_BALL_CONTOUR_AREA = np.pi * (MAX_BALL_RADIUS_PX**2) * 1.5

# Physical radius of the volleyball in meters
VOLLEYBALL_RADIUS_M = 0.105  # Approx. 10.5 cm, as per PDF

# --- 3D Visualization (WEB_UI using Open3D) ---
VIS_WINDOW_TITLE = "Real-Time 3D Volleyball Tracker"
VIS_WINDOW_WIDTH = 1024
VIS_WINDOW_HEIGHT = 768

# Colors for 3D objects (R, G, B, values 0.0 to 1.0)
VIS_BOX_COLOR = [0.5, 0.5, 0.5]  # Grey for the box
VIS_VOLLEYBALL_COLOR = [1.0, 0.84, 0.0] # Yellow for the ball (as per PDF)
VIS_AXES_ENABLED = True # Draw world coordinate axes

# Initial camera viewpoint for Open3D visualizer
# These are example values and might need adjustment based on setup
VIS_CAMERA_LOOKAT = [BOX_WIDTH_M / 2, BOX_DEPTH_M / 2, 0.0]  # Look at the center of the box ground
VIS_CAMERA_FRONT = [-0.7, -0.7, -0.5]  # Vector from lookat point to camera
VIS_CAMERA_UP = [0.0, 0.0, 1.0]      # Z-up
VIS_CAMERA_ZOOM = 0.5

# --- Hardware Manager (Example from SampleCode, adapt as needed) ---
# This project description does not heavily focus on these, but placeholders can be kept.
# If using Raspberry Pi specific hardware features like GPIOs for triggers or battery monitor.
BATTERY_MONITOR_ENABLED = False # Assuming not a primary focus for this PDF.
SWITCH_GPIO_PIN = None       # Example: 17 if a physical switch is used.
SWITCH_BOUNCE_TIME = 0.1

# If INA219 is used (as in sample code)
INA219_I2C_ADDRESS = 0x40 # Default I2C address for INA219, 0x41 in sample
INA219_I2C_BUS = 1
BATTERY_READ_INTERVAL = 30.0 # Seconds
BATTERY_MAX_VOLTAGE = 12.6
BATTERY_MIN_VOLTAGE = 9.0
INA219_CALIBRATION_CONFIG = "16V_5A" # As per sample

# Servo control (not mentioned in PDF for volleyball tracker, but in sample)
SERVO_ENABLED = False
SERVO_PWM_CHIP = 0
SERVO_PWM_CHANNEL = 0
SERVO_GPIO_PIN = 12 # GPIO pin for PWM control
SERVO_PERIOD_NS = 20000000  # 20ms = 50Hz
SERVO_MIN_DUTY_NS = 500000   # 0.5ms pulse for 0 degrees (example)
SERVO_MAX_DUTY_NS = 2500000  # 2.5ms pulse for 180 degrees (example)
SERVO_CENTER_ANGLE = 90
SERVO_MIN_ANGLE = 0
SERVO_MAX_ANGLE = 180
SERVO_SMOOTH_MOVE = False # Keep False if not needed
SERVO_SMOOTH_MOVE_STEPS = 20
SERVO_SMOOTH_MOVE_DELAY = 0.01

# --- Main Loop Configuration ---
MAIN_LOOP_MAX_FPS = 30 # Cap the main processing loop FPS if needed, 0 for no cap.
DISPLAY_2D_FEED = True # Whether to show the 2D camera feed with overlays using cv2.imshow()

# --- Sanity Checks & Derived Configs ---
if CHECKERBOARD_INNER_CORNERS_WIDTH <= 0 or CHECKERBOARD_INNER_CORNERS_HEIGHT <= 0:
    raise ValueError("Checkerboard inner corner dimensions must be positive.")
if SQUARE_SIZE_MM <= 0:
    raise ValueError("Checkerboard square size must be positive.")

# Check if log level is valid
valid_log_levels = ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]
if LOG_LEVEL.upper() not in valid_log_levels:
    print(f"Warning: Invalid LOG_LEVEL '{LOG_LEVEL}'. Defaulting to INFO.")
    LOG_LEVEL = "INFO"

print(f"Config loaded. Calibration data expected at: {CALIBRATION_DATA_FILE}")