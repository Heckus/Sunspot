# camera_manager.py

import cv2 # Still needed for undistortion, color conversion etc.
import logging
import time
import numpy as np
from picamera2 import Picamera2 # Using Picamera2 library
# from libcamera import controls # Can be added later if more fine-grained controls are needed

# Import project's Config for default values
import Config as main_project_config

class CameraManager:
    """
    Manages a single camera using the Picamera2 library,
    including initialization, configuration, and frame capture.
    """

    def __init__(self, camera_index=main_project_config.CAMERA_INDEX,
                 width=main_project_config.CAM_REQUESTED_WIDTH,
                 height=main_project_config.CAM_REQUESTED_HEIGHT,
                 fps=main_project_config.CAM_REQUESTED_FPS):
        """
        Initializes the CameraManager.
        Args:
            camera_index (int): The index of the camera to use (for Picamera2, often 0 or 1).
            width (int): The desired width of the camera frames.
            height (int): The desired height of the camera frames.
            fps (float): The desired frames per second.
        """
        self.camera_index = camera_index
        self.picam = None
        self.is_initialized = False
        self.last_error = None
        self.requested_width = width
        self.requested_height = height
        self.requested_fps = fps

        # Actual dimensions and FPS will be updated after camera configuration
        self.frame_width = width
        self.frame_height = height
        self.fps = fps

        self.mtx = None # To be loaded from calibration
        self.dist = None # To be loaded from calibration
        self.new_camera_mtx = None # For undistortion
        self.roi = None # For undistortion

        self.measured_fps_avg = None
        self.last_capture_time = None

        logging.info(f"CameraManager (Picamera2) initialized for camera index {self.camera_index} "
                     f"requesting {self.requested_width}x{self.requested_height} @ {self.requested_fps}fps.")

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
            logging.info(f"Camera Matrix (mtx):\n{self.mtx}")
            logging.info(f"Distortion Coefficients (dist):\n{self.dist}")

            h, w = self.frame_height, self.frame_width
            if w > 0 and h > 0: # Ensure width and height are valid before calculating
                self.new_camera_mtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))
                if self.new_camera_mtx is not None:
                     logging.info(f"Calculated new optimal camera matrix (initial):\n{self.new_camera_mtx}")
                     logging.info(f"Calculated ROI for undistortion (initial): {self.roi}")
                else:
                     logging.warning("Failed to calculate new optimal camera matrix (initial) - likely due to w/h being 0.")
            else:
                logging.warning("Initial frame width/height is 0, skipping initial calculation of new_camera_mtx and ROI.")
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
        Initializes the camera using Picamera2.
        Returns:
            bool: True if initialization was successful, False otherwise.
        """
        logging.info(f"Attempting to initialize Picamera2 (ID: {self.camera_index}) "
                     f"requesting {self.requested_width}x{self.requested_height} @ {self.requested_fps}fps...")

        if self.picam is not None:
            logging.info("Picamera2 instance already exists. Stopping and closing first.")
            try:
                if self.picam.started:
                    self.picam.stop()
                self.picam.close()
            except Exception as e:
                logging.warning(f"Error stopping/closing existing Picamera2 instance: {e}")
            self.picam = None
            self.is_initialized = False

        try:
            self.picam = Picamera2(camera_num=self.camera_index)
            # Picamera2 will select a sensor mode close to the request.
            # Request "BGR888" format to get a 3-channel BGR numpy array directly.
            video_config_params = {
                "main": {"size": (self.requested_width, self.requested_height), "format": "BGR888"}, # CHANGED
                "controls": {"FrameRate": self.requested_fps}
            }
            
            cam_config = self.picam.create_video_configuration(**video_config_params)
            logging.info(f"Picamera2 generated video config (requesting BGR888): {cam_config}")
            self.picam.configure(cam_config)
            
            # Update actual frame dimensions and FPS from the configured stream
            self.frame_width = self.picam.camera_configuration()['main']['size'][0]
            self.frame_height = self.picam.camera_configuration()['main']['size'][1]
            actual_format = self.picam.camera_configuration()['main']['format']
            
            try:
                self.fps = self.picam.camera_configuration()['controls']['FrameRate']
            except KeyError:
                logging.warning(f"Could not read 'FrameRate' from controls, using requested FPS: {self.requested_fps}")
                self.fps = self.requested_fps # Fallback

            logging.info(f"Picamera2 configured. Actual stream: {self.frame_width}x{self.frame_height} @ ~{self.fps:.2f}fps, Format: {actual_format}")
            if actual_format != "BGR888":
                logging.warning(f"Requested BGR888 format, but actual format is {actual_format}. Color conversion might still be needed or unexpected.")


            self.picam.start()
            logging.info(f"Picamera2 (ID: {self.camera_index}) started successfully.")
            
            if (self.frame_width != self.requested_width or self.frame_height != self.requested_height or \
               (self.new_camera_mtx is None and self.mtx is not None)) and \
               (self.mtx is not None and self.dist is not None):
                logging.info("Actual resolution differs from requested or new_camera_mtx not set. Re-calculating optimal new camera matrix and ROI.")
                h, w = self.frame_height, self.frame_width
                self.new_camera_mtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))
                if self.new_camera_mtx is not None:
                    logging.info(f"Re-calculated new optimal camera matrix:\n{self.new_camera_mtx}")
                    logging.info(f"Re-calculated ROI for undistortion: {self.roi}")
                else:
                    logging.warning("Failed to re-calculate new optimal camera matrix after resolution change.")

            self.is_initialized = True
            self.last_error = None
            self.last_capture_time = None
            self.measured_fps_avg = None
            return True

        except Exception as e:
            self.last_error = f"Failed to initialize Picamera2 {self.camera_index}: {e}"
            logging.error(self.last_error, exc_info=True)
            if self.picam is not None:
                try:
                    if self.picam.started: self.picam.stop()
                    self.picam.close()
                except Exception as close_err:
                    logging.error(f"Error closing picam after failed init: {close_err}")
            self.picam = None
            self.is_initialized = False
            return False

    def capture_frame(self, undistort=True):
        """
        Captures a single frame from the Picamera2.
        Args:
            undistort (bool): Whether to undistort the frame using loaded calibration data.
        Returns:
            numpy.ndarray: The captured (and optionally undistorted) BGR frame, or None if capture fails.
        """
        if not self.is_initialized or not self.picam or not self.picam.started:
            self.last_error = "Cannot capture frame, Picamera2 not initialized or not started."
            # logging.warning(self.last_error) # This can be very spammy
            return None

        try:
            # capture_array("main") returns a BGR NumPy array if configured with "BGR888"
            frame_bgr = self.picam.capture_array("main") # RENAMED from frame_rgb
            if frame_bgr is None:
                self.last_error = "Picamera2 capture_array returned None."
                logging.warning(self.last_error)
                return None
            
            # NO LONGER NEEDED IF CAMERA PROVIDES BGR888 AND capture_array RESPECTS IT:
            # frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            self.last_error = None # Clear error if capture successful

        except Exception as e:
            self.last_error = f"Error capturing frame from Picamera2: {e}"
            logging.error(self.last_error, exc_info=True)
            return None
        
        current_time = time.monotonic()
        if self.last_capture_time is not None:
            time_diff = current_time - self.last_capture_time
            if time_diff > 1e-4: 
                instant_fps = 1.0 / time_diff
                alpha = 0.05 
                self.measured_fps_avg = (alpha * instant_fps + (1 - alpha) * self.measured_fps_avg) \
                                        if self.measured_fps_avg is not None else instant_fps
        self.last_capture_time = current_time

        if undistort:
            if self.mtx is not None and self.dist is not None:
                # Ensure new_camera_mtx is used if available, otherwise use original mtx for undistortion map
                current_undistort_mtx = self.new_camera_mtx if self.new_camera_mtx is not None else self.mtx
                undistorted_frame = cv2.undistort(frame_bgr, self.mtx, self.dist, None, current_undistort_mtx)
                # Cropping logic can be refined based on how getOptimalNewCameraMatrix's alpha parameter is used.
                # If alpha=0, ROI gives the valid pixel region. If alpha=1, all source pixels are retained.
                # For now, not applying ROI crop unless specifically intended.
                # if self.roi is not None and self.new_camera_mtx is not None:
                #     x, y, w, h = self.roi
                #     if w > 0 and h > 0:
                #         undistorted_frame = undistorted_frame[y:y+h, x:x+w]
                return undistorted_frame
            else:
                # logging.debug("Cannot undistort: Camera calibration data (mtx, dist) not loaded.") # Can be spammy
                return frame_bgr 
        else:
            return frame_bgr

    def get_camera_properties(self):
        """
        Returns a dictionary of current camera properties.
        """
        if not self.is_initialized or not self.picam:
            return {
                'is_initialized': False,
                'error': self.last_error or "Picamera2 not initialized"
            }
        
        actual_format = "N/A"
        try:
            actual_format = self.picam.camera_configuration()['main']['format']
        except Exception:
            pass


        return {
            'is_initialized': True,
            'camera_index': self.camera_index,
            'frame_width': self.frame_width,
            'frame_height': self.frame_height,
            'target_fps': self.fps, 
            'measured_fps': self.measured_fps_avg,
            'backend_name': 'Picamera2',
            'stream_format': actual_format,
            'mtx_loaded': self.mtx is not None,
            'dist_loaded': self.dist is not None,
            'last_error': self.last_error
        }

    def shutdown(self):
        """
        Releases the Picamera2 resource.
        """
        logging.info("Shutting down CameraManager (Picamera2)...")
        if self.picam is not None:
            try:
                if self.picam.started:
                    self.picam.stop()
                    logging.info(f"Picamera2 (ID: {self.camera_index}) stopped.")
                self.picam.close()
                logging.info(f"Picamera2 (ID: {self.camera_index}) closed.")
            except Exception as e:
                logging.error(f"Error during Picamera2 shutdown: {e}", exc_info=True)
        self.is_initialized = False
        self.picam = None
        logging.info("CameraManager (Picamera2) shutdown complete.")

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    
    # --- Create a dummy Config.py for standalone testing if it doesn't exist ---
    if not os.path.exists("Config.py"):
        print("Creating dummy Config.py for testing CameraManager.py standalone.")
        with open("Config.py", "w") as f:
            f.write("CAMERA_INDEX = 0\n")
            f.write("CAM_REQUESTED_WIDTH = 1280\n") # Default to a common resolution
            f.write("CAM_REQUESTED_HEIGHT = 720\n")
            f.write("CAM_REQUESTED_FPS = 30\n")
            f.write("CALIBRATION_DATA_FILE = 'camera_calibration_data.npz'\n") # Ensure this file exists or loading will fail
            f.write("LOG_LEVEL = 'INFO'\n") # Add other necessary Config vars if your main needs them
    try:
        import Config as main_project_config # Re-import in case it was just created
    except ImportError:
        print("Failed to import Config.py. Please ensure it exists or CameraManager is used within a project structure.")
        exit()
    # --- End dummy Config.py creation ---


    cam_manager = CameraManager(
        camera_index=main_project_config.CAMERA_INDEX,
        width=main_project_config.CAM_REQUESTED_WIDTH,
        height=main_project_config.CAM_REQUESTED_HEIGHT,
        fps=main_project_config.CAM_REQUESTED_FPS
    )

    # Try to load calibration data, but don't make it fatal for the example
    if os.path.exists(main_project_config.CALIBRATION_DATA_FILE):
        if cam_manager.load_calibration_data(main_project_config.CALIBRATION_DATA_FILE):
            logging.info("Calibration data loaded successfully for example usage.")
        else:
            logging.warning(f"Failed to load calibration data for example from: {main_project_config.CALIBRATION_DATA_FILE}")
    else:
        logging.warning(f"Calibration file {main_project_config.CALIBRATION_DATA_FILE} not found. Proceeding without undistortion for example.")


    if cam_manager.initialize_camera():
        cv2.namedWindow("Picamera2 Stream (BGR)", cv2.WINDOW_NORMAL)
        frame_count = 0
        start_time_loop = time.monotonic()

        while True:
            bgr_frame = cam_manager.capture_frame(undistort=True) 

            if bgr_frame is not None:
                frame_count += 1
                cv2.imshow("Picamera2 Stream (BGR)", bgr_frame) # Expects BGR

                props = cam_manager.get_camera_properties()
                if props.get('measured_fps') is not None:
                    print(f"Format: {props.get('stream_format')} | Measured FPS: {props['measured_fps']:.2f}    ", end='\r')
            else:
                logging.error(f"Failed to capture frame in example loop. Last error: {cam_manager.last_error}")
                if not cam_manager.is_initialized:
                    logging.error("Camera became uninitialized. Exiting example loop.")
                    break
                time.sleep(0.1) 

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'): 
                logging.info("Re-initializing camera by user request (r key)...")
                cam_manager.shutdown() # Full shutdown before re-init
                time.sleep(0.5) # Give it a moment
                if cam_manager.initialize_camera():
                     logging.info("Camera re-initialized successfully.")
                else:
                     logging.error("Failed to re-initialize camera.")
                     break # Exit if re-init fails

        print(" " * 70, end='\r') 

        end_time_loop = time.monotonic()
        elapsed_time = end_time_loop - start_time_loop
        avg_fps_loop = frame_count / elapsed_time if elapsed_time > 0 else 0
        logging.info(f"\nExample loop finished. Captured {frame_count} frames in {elapsed_time:.2f}s. Average Loop FPS: {avg_fps_loop:.2f}")

        cam_manager.shutdown()
        cv2.destroyAllWindows()
    else:
        logging.error(f"Failed to initialize Picamera2. Error: {cam_manager.last_error}")

    logging.info("CameraManager (Picamera2) example finished.")