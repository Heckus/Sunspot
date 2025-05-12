# camera_manager.py

import cv2
import logging
import time
import numpy as np

# Configuration (can be moved to a separate Config.py later)
CAMERA_INDEX = 0  # Default camera index
DEFAULT_WIDTH = 1920 # Default resolution width [cite: 71]
DEFAULT_HEIGHT = 1080 # Default resolution height [cite: 71]
DEFAULT_FPS = 30.0 # Default FPS [cite: 71]

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
        self.mtx = None # To be loaded from calibration [cite: 18, 22]
        self.dist = None # To be loaded from calibration [cite: 18, 22]
        self.new_camera_mtx = None # For undistortion [cite: 122]
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
            logging.info(f"Camera Matrix (mtx):\n{self.mtx}") # [cite: 67]
            logging.info(f"Distortion Coefficients (dist):\n{self.dist}") # [cite: 67]

            # Pre-calculate optimal new camera matrix and ROI for undistortion [cite: 122]
            # This is done once after loading calibration data
            h, w = self.frame_height, self.frame_width # Assuming these are the dimensions used for calibration
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

        try:
            self.cap = cv2.VideoCapture(self.camera_index) # [cite: 120]
            if not self.cap.isOpened():
                self.last_error = f"Cannot open camera with index {self.camera_index}"
                logging.error(self.last_error)
                self.is_initialized = False
                return False

            # Attempt to set resolution and FPS
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)

            # Verify actual settings
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

            logging.info(f"Camera {self.camera_index} initialized.")
            logging.info(f"Requested: {self.frame_width}x{self.frame_height} @ {self.fps:.2f}fps")
            logging.info(f"Actual:    {actual_width}x{actual_height} @ {actual_fps:.2f}fps")

            # Update internal state if actual dimensions differ significantly
            # Some cameras might not support all requested resolutions/fps
            if abs(actual_width - self.frame_width) > 10 or abs(actual_height - self.frame_height) > 10 :
                logging.warning(f"Actual resolution {actual_width}x{actual_height} "
                                f"differs from requested {self.frame_width}x{self.frame_height}. "
                                "Updating internal dimensions.")
                self.frame_width = actual_width
                self.frame_height = actual_height
                # If dimensions change, new_camera_mtx and roi should be recalculated if calibration data is loaded
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
        if not self.is_initialized or not self.cap.isOpened():
            self.last_error = "Cannot capture frame, camera not initialized."
            # logging.warning(self.last_error) # Potentially too verbose
            return None

        ret, frame = self.cap.read() # [cite: 120]

        if not ret or frame is None:
            self.last_error = "Failed to read frame from camera."
            logging.warning(self.last_error)
            return None

        # Calculate FPS
        current_time = time.monotonic()
        if self.last_capture_time is not None:
            time_diff = current_time - self.last_capture_time
            if time_diff > 0.0001: # Avoid division by zero
                instant_fps = 1.0 / time_diff
                alpha = 0.05 # Smoothing factor
                self.measured_fps_avg = (alpha * instant_fps + (1 - alpha) * self.measured_fps_avg) \
                                        if self.measured_fps_avg is not None else instant_fps
        self.last_capture_time = current_time

        if undistort:
            if self.mtx is not None and self.dist is not None and self.new_camera_mtx is not None:
                # Undistort the frame [cite: 122]
                undistorted_frame = cv2.undistort(frame, self.mtx, self.dist, None, self.new_camera_mtx) # [cite: 123]

                # Optionally crop the image based on ROI from getOptimalNewCameraMatrix
                # if self.roi is not None:
                #     x, y, w, h = self.roi
                #     if w > 0 and h > 0: # Check if ROI is valid
                #        undistorted_frame = undistorted_frame[y:y+h, x:x+w]
                return undistorted_frame
            elif self.mtx is not None and self.dist is not None:
                # Fallback if new_camera_mtx wasn't calculated (e.g. error during load_calibration)
                logging.warning("Attempting undistortion without new_camera_mtx. Results may be suboptimal.")
                return cv2.undistort(frame, self.mtx, self.dist, None, self.mtx)
            else:
                # logging.debug("Cannot undistort: Camera calibration data not loaded.")
                return frame # Return original frame if no calibration data
        else:
            return frame


    def get_camera_properties(self):
        """
        Returns a dictionary of current camera properties.
        """
        if not self.is_initialized or not self.cap.isOpened():
            return {
                'is_initialized': False,
                'error': self.last_error or "Camera not initialized"
            }
        return {
            'is_initialized': True,
            'camera_index': self.camera_index,
            'frame_width': int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            'frame_height': int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            'fps': self.cap.get(cv2.CAP_PROP_FPS),
            'measured_fps': self.measured_fps_avg,
            'backend_name': self.cap.getBackendName(),
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
        self.cap = None
        logging.info("CameraManager shutdown complete.")

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    # Example Usage
    cam_manager = CameraManager()

    # Example: Load calibration data (replace with your actual file path)
    # Make sure 'camera_calibration_data.npz' exists from your calibration script
    # This file should contain 'camera_matrix' and 'dist_coeffs'
    calibration_file = "camera_calibration_data.npz" # As per camera_calibration.py [cite: 1]
    if cam_manager.load_calibration_data(calibration_file): # [cite: 68]
        logging.info("Calibration data loaded successfully for example usage.")
    else:
        logging.warning(f"Failed to load calibration data for example. Check path: {calibration_file}")


    if cam_manager.initialize_camera():
        cv2.namedWindow("Camera Stream", cv2.WINDOW_NORMAL)
        frame_count = 0
        start_time = time.time()

        while True:
            # Capture frame (with undistortion if calibration data was loaded)
            frame = cam_manager.capture_frame(undistort=True) # [cite: 26, 122]

            if frame is not None:
                frame_count += 1
                # Display the frame
                cv2.imshow("Camera Stream", frame)

                # Display FPS (calculated within capture_frame)
                props = cam_manager.get_camera_properties()
                if props.get('measured_fps') is not None:
                    print(f"Measured FPS: {props['measured_fps']:.2f}", end='\r')

            else:
                logging.error("Failed to capture frame in example loop.")
                break

            # Press 'q' to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        end_time = time.time()
        elapsed_time = end_time - start_time
        avg_fps_loop = frame_count / elapsed_time if elapsed_time > 0 else 0
        logging.info(f"\nExample loop finished. Captured {frame_count} frames in {elapsed_time:.2f}s. Average Loop FPS: {avg_fps_loop:.2f}")

        cam_manager.shutdown()
        cv2.destroyAllWindows()
    else:
        logging.error(f"Failed to initialize camera. Error: {cam_manager.last_error}")

    logging.info("Example finished.")