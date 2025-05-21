# camera_manager.py

import cv2
import logging
import time
import numpy as np

# Configuration (can be moved to a separate Config.py later)
CAMERA_INDEX = 0  # Default camera index
DEFAULT_WIDTH = 1920 # Default resolution width
DEFAULT_HEIGHT = 1080 # Default resolution height
DEFAULT_FPS = 30.0 # Default FPS

class CameraManager:
    """
    Manages the Raspberry Pi HQ Camera, including initialization,
    Configuration, and frame capture for the 3D modeling project.
    """

    def __init__(self, camera_index=CAMERA_INDEX, width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT, fps=DEFAULT_FPS):
        """
        Initializes the CameraManager.
        Args:
            camera_index (int): The index of the camera to use.
            width (int): The desired width of the camera frames.
            height (int): The desired height of the camera frames.
            fps (float): The desired frames per second.
        """
        self.camera_index = camera_index
        self.cap = None
        self.is_initialized = False
        self.last_error = None
        self.frame_width = width
        self.frame_height = height
        self.fps = fps
        self.mtx = None # To be loaded from calibration
        self.dist = None # To be loaded from calibration
        self.new_camera_mtx = None # For undistortion
        self.roi = None # For undistortion

        self.measured_fps_avg = None
        self.last_capture_time = None

        logging.info(f"CameraManager initialized for camera index {self.camera_index} "
                     f"at {self.frame_width}x{self.frame_height} @ {self.fps}fps.")

    def load_calibration_data(self, calibration_file_path):
        """
        Loads camera intrinsic matrix and distortion coefficients from a .npz file.
        Args:
            calibration_file_path (str): Path to the .npz file containing calibration data.
        Returns:
            bool: True if loading was successful, False otherwise.
        """
        try:
            data = np.load(calibration_file_path)
            self.mtx = data['camera_matrix']
            self.dist = data['dist_coeffs']
            logging.info(f"Successfully loaded camera calibration data from {calibration_file_path}")
            logging.info(f"Camera Matrix (mtx):\n{self.mtx}") #
            logging.info(f"Distortion Coefficients (dist):\n{self.dist}") #

            h, w = self.frame_height, self.frame_width
            self.new_camera_mtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))
            if self.new_camera_mtx is not None:
                 logging.info(f"Calculated new optimal camera matrix:\n{self.new_camera_mtx}")
                 logging.info(f"Calculated ROI for undistortion: {self.roi}")
            else:
                 logging.warning("Failed to calculate new optimal camera matrix. Undistortion might be suboptimal.")
            return True
        except FileNotFoundError:
            self.last_error = f"Calibration file not found: {calibration_file_path}"
            logging.error(self.last_error)
            return False
        except KeyError as e:
            self.last_error = f"Calibration file {calibration_file_path} is missing key: {e}. Expected 'camera_matrix' and 'dist_coeffs'."
            logging.error(self.last_error)
            return False
        except Exception as e:
            self.last_error = f"Error loading calibration data from {calibration_file_path}: {e}"
            logging.error(self.last_error, exc_info=True)
            return False

    def initialize_camera(self):
        """
        Initializes the camera using OpenCV's VideoCapture.
        Returns:
            bool: True if initialization was successful, False otherwise.
        """
        logging.info(f"Attempting to initialize camera (ID: {self.camera_index}) "
                     f"at {self.frame_width}x{self.frame_height} @ {self.fps}fps...")

        if self.cap is not None and self.cap.isOpened():
            logging.info("Camera already initialized. Releasing first.")
            self.cap.release()
            self.is_initialized = False # Ensure state is reset

        try:
            self.cap = cv2.VideoCapture(self.camera_index) #
            if not self.cap.isOpened():
                self.last_error = f"Cannot open camera with index {self.camera_index}. Check if it's connected and not in use by another application."
                logging.error(self.last_error)
                self.is_initialized = False
                return False

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)

            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

            logging.info(f"Camera {self.camera_index} initialized (isOpened: {self.cap.isOpened()}).")
            logging.info(f"Requested: {self.frame_width}x{self.frame_height} @ {self.fps:.2f}fps")
            logging.info(f"Actual:    {actual_width}x{actual_height} @ {actual_fps:.2f}fps")

            if actual_fps <= 0:
                logging.warning(f"Camera reported an actual FPS of {actual_fps:.2f}. "
                                "This might indicate an issue with FPS reporting or camera configuration. "
                                "The application will proceed, but frame rate might be unstable or not as expected.")

            if abs(actual_width - self.frame_width) > 10 or abs(actual_height - self.frame_height) > 10 :
                logging.warning(f"Actual resolution {actual_width}x{actual_height} "
                                f"differs from requested {self.frame_width}x{self.frame_height}. "
                                "Updating internal dimensions.")
                self.frame_width = actual_width
                self.frame_height = actual_height
                if self.mtx is not None and self.dist is not None:
                    h, w = self.frame_height, self.frame_width
                    self.new_camera_mtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))
                    logging.info("Re-calculated new_camera_mtx and ROI due to resolution change.")

            self.is_initialized = True
            self.last_error = None
            self.last_capture_time = None
            self.measured_fps_avg = None
            return True

        except Exception as e:
            self.last_error = f"Failed to initialize camera {self.camera_index}: {e}"
            logging.error(self.last_error, exc_info=True)
            if self.cap is not None:
                self.cap.release()
            self.is_initialized = False
            return False

    def capture_frame(self, undistort=True):
        """
        Captures a single frame from the camera.
        Args:
            undistort (bool): Whether to undistort the frame using loaded calibration data.
                              Requires calibration data to be loaded first.
        Returns:
            numpy.ndarray: The captured (and optionally undistorted) frame, or None if capture fails.
        """
        if not self.is_initialized or not self.cap or not self.cap.isOpened(): # Added self.cap check
            self.last_error = "Cannot capture frame, camera not initialized or not opened."
            logging.warning(self.last_error) # Potentially verbose, but useful for debugging current issues
            return None

        ret, frame = self.cap.read() #

        if not ret or frame is None:
            self.last_error = "Failed to read frame from camera. 'ret' was False or frame was None."
            logging.warning(self.last_error)
            return None
        
        self.last_error = None # Clear last error if frame read is successful

        current_time = time.monotonic()
        if self.last_capture_time is not None:
            time_diff = current_time - self.last_capture_time
            if time_diff > 0.0001: 
                instant_fps = 1.0 / time_diff
                alpha = 0.05 
                self.measured_fps_avg = (alpha * instant_fps + (1 - alpha) * self.measured_fps_avg) \
                                        if self.measured_fps_avg is not None else instant_fps
        self.last_capture_time = current_time

        if undistort:
            if self.mtx is not None and self.dist is not None and self.new_camera_mtx is not None:
                undistorted_frame = cv2.undistort(frame, self.mtx, self.dist, None, self.new_camera_mtx) #
                return undistorted_frame
            elif self.mtx is not None and self.dist is not None:
                logging.warning("Attempting undistortion without pre-calculated new_camera_mtx. Results may be suboptimal.")
                return cv2.undistort(frame, self.mtx, self.dist, None, self.mtx)
            else:
                # logging.debug("Cannot undistort: Camera calibration data not loaded.")
                return frame 
        else:
            return frame


    def get_camera_properties(self):
        """
        Returns a dictionary of current camera properties.
        """
        if not self.is_initialized or not self.cap or not self.cap.isOpened(): # Added self.cap check
            return {
                'is_initialized': False,
                'error': self.last_error or "Camera not initialized or not opened"
            }
        return {
            'is_initialized': True,
            'camera_index': self.camera_index,
            'frame_width': int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            'frame_height': int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            'fps': self.cap.get(cv2.CAP_PROP_FPS),
            'measured_fps': self.measured_fps_avg,
            'backend_name': self.cap.getBackendName() if hasattr(self.cap, 'getBackendName') else 'N/A', # Check if method exists
            'mtx_loaded': self.mtx is not None,
            'dist_loaded': self.dist is not None,
            'last_error': self.last_error
        }

    def shutdown(self):
        """
        Releases the camera resource.
        """
        logging.info("Shutting down CameraManager...")
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
            logging.info(f"Camera {self.camera_index} released.")
        self.is_initialized = False
        self.cap = None # Ensure cap is None after release
        logging.info("CameraManager shutdown complete.")

if __name__ == '__main__':
    # Ensure Config.py is accessible if this block uses it
    # For example, by setting PYTHONPATH or having Config.py in the same directory
    try:
        import Config as main_config # Assuming Config.py is in the same directory or accessible
        log_level = getattr(logging, main_config.LOG_LEVEL.upper(), logging.INFO)
        log_format = main_config.LOG_FORMAT
        log_date_format = main_config.LOG_DATE_FORMAT
    except ImportError:
        log_level = logging.INFO
        log_format='%(asctime)s - %(levelname)s - %(message)s'
        log_date_format='%Y-%m-%d %H:%M:%S'
        main_config = None # Indicate config is not loaded for example
        logging.warning("Config.py not found for example usage, using default logging settings.")

    logging.basicConfig(level=log_level, format=log_format, datefmt=log_date_format)


    # Example Usage
    # Use default width/height/fps or from a loaded Config if available
    cam_width = DEFAULT_WIDTH
    cam_height = DEFAULT_HEIGHT
    cam_fps = DEFAULT_FPS
    calib_file_path = "camera_calibration_data.npz" # Default

    if main_config:
        cam_width = main_config.CAM_REQUESTED_WIDTH
        cam_height = main_config.CAM_REQUESTED_HEIGHT
        cam_fps = main_config.CAM_REQUESTED_FPS
        calib_file_path = main_config.CALIBRATION_DATA_FILE


    cam_manager = CameraManager(width=cam_width, height=cam_height, fps=cam_fps)

    if cam_manager.load_calibration_data(calib_file_path): #
        logging.info("Calibration data loaded successfully for example usage.")
    else:
        logging.warning(f"Failed to load calibration data for example. Check path: {calib_file_path}")


    if cam_manager.initialize_camera():
        cv2.namedWindow("Camera Stream", cv2.WINDOW_NORMAL)
        frame_count = 0
        start_time = time.time()

        while True:
            frame = cam_manager.capture_frame(undistort=True) #

            if frame is not None:
                frame_count += 1
                cv2.imshow("Camera Stream", frame)
                props = cam_manager.get_camera_properties()
                if props.get('measured_fps') is not None:
                    print(f"Measured FPS: {props['measured_fps']:.2f}", end='\r')
            else:
                logging.error(f"Failed to capture frame in example loop. Last error: {cam_manager.last_error}")
                # If camera init failed and we still got here, or if read fails, break.
                if not cam_manager.is_initialized or "Cannot open camera" in (cam_manager.last_error or ""):
                    break 
                time.sleep(0.5) # Wait a bit if it's a transient read error

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        # To clear the FPS print from the console line
        print(" " * 50, end='\r')

        end_time = time.time()
        elapsed_time = end_time - start_time
        avg_fps_loop = frame_count / elapsed_time if elapsed_time > 0 else 0
        logging.info(f"\nExample loop finished. Captured {frame_count} frames in {elapsed_time:.2f}s. Average Loop FPS: {avg_fps_loop:.2f}")

        cam_manager.shutdown()
        cv2.destroyAllWindows()
    else:
        logging.error(f"Failed to initialize camera. Error: {cam_manager.last_error}")

    logging.info("Example finished.")