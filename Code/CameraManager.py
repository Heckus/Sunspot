# camera_manager.py

import cv2
import logging
import time
import numpy as np
from picamera2 import Picamera2 

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
        self.camera_index = camera_index
        self.picam = None
        self.is_initialized = False
        self.last_error = None
        self.requested_width = width
        self.requested_height = height
        self.requested_fps = fps

        self.frame_width = width
        self.frame_height = height
        self.fps = fps

        self.mtx = None 
        self.dist = None 
        self.new_camera_mtx = None 
        self.roi = None 

        self.measured_fps_avg = None
        self.last_capture_time = None
        self._frame_counter_for_fps = 0
        self._fps_calc_start_time = None


        logging.info(f"CameraManager (Picamera2) initialized for camera index {self.camera_index} "
                     f"requesting {self.requested_width}x{self.requested_height} @ {self.requested_fps}fps.")

    def load_calibration_data(self, calibration_file_path):
        try:
            data = np.load(calibration_file_path)
            self.mtx = data['camera_matrix']
            self.dist = data['dist_coeffs']
            # Try to get image_size if saved by calibration script
            if 'image_size' in data:
                calib_w, calib_h = data['image_size']
                logging.info(f"Calibration data is for resolution: {calib_w}x{calib_h}")
                # If current requested res is different, it might be an issue, but proceed.
                # new_camera_mtx should ideally be calculated with the *actual* operating resolution.
            
            logging.info(f"Successfully loaded camera calibration data from {calibration_file_path}")
            logging.debug(f"Camera Matrix (mtx):\n{self.mtx}")
            logging.debug(f"Distortion Coefficients (dist):\n{self.dist}")

            # Defer calculation of new_camera_mtx until actual resolution is known post-initialization
            # because self.frame_width/height might change.
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

    def _calculate_optimal_new_camera_matrix(self):
        """Calculates new_camera_mtx and roi based on current frame_width/height."""
        if self.mtx is not None and self.dist is not None and \
           self.frame_width > 0 and self.frame_height > 0:
            logging.info(f"Calculating new optimal camera matrix for resolution {self.frame_width}x{self.frame_height}.")
            self.new_camera_mtx, self.roi = cv2.getOptimalNewCameraMatrix(
                self.mtx, self.dist, 
                (self.frame_width, self.frame_height), 
                1, # Alpha = 1 means all pixels are retained. Use 0 for cropping to valid pixels only.
                (self.frame_width, self.frame_height)
            )
            if self.new_camera_mtx is not None:
                 logging.info(f"Calculated new optimal camera matrix:\n{self.new_camera_mtx}")
                 logging.debug(f"Calculated ROI for undistortion: {self.roi}")
            else:
                 logging.warning("Failed to calculate new optimal camera matrix.")
        else:
            logging.warning("Cannot calculate new optimal camera matrix: Missing intrinsics or valid frame dimensions.")


    def initialize_camera(self):
        logging.info(f"Attempting to initialize Picamera2 (ID: {self.camera_index}) "
                     f"requesting {self.requested_width}x{self.requested_height} @ {self.requested_fps}fps...")

        if self.picam is not None: # Should not happen if properly shut down
            logging.info("Picamera2 instance already exists. Shutting down first.")
            self.shutdown()

        try:
            self.picam = Picamera2(camera_num=self.camera_index)
            
            # It's good practice to list available formats/modes if exact config fails
            # sensor_modes = self.picam.sensor_modes
            # logging.debug(f"Available sensor modes: {sensor_modes}")

            video_config_params = {
                "main": {"size": (self.requested_width, self.requested_height), "format": "BGR888"},
                "controls": {"FrameRate": self.requested_fps, "NoiseReductionMode": 1}, # Added NoiseReductionMode
                "queue": True # Use queue for smoother capture
            }
            
            cam_config = self.picam.create_video_configuration(**video_config_params)
            logging.info(f"Picamera2 generated video config (requesting BGR888): {cam_config}")
            self.picam.configure(cam_config)
            
            # Update actual frame dimensions and FPS from the configured stream
            current_config = self.picam.camera_configuration()
            self.frame_width = current_config['main']['size'][0]
            self.frame_height = current_config['main']['size'][1]
            actual_format = current_config['main']['format']
            
            try:
                self.fps = current_config['controls']['FrameRate']
            except KeyError:
                logging.warning(f"Could not read 'FrameRate' from picam2 controls after configure. Using requested FPS: {self.requested_fps}")
                self.fps = self.requested_fps 

            logging.info(f"Picamera2 configured. Actual stream: {self.frame_width}x{self.frame_height} @ ~{self.fps:.2f}fps, Format: {actual_format}")
            if actual_format != "BGR888":
                logging.warning(f"Requested BGR888 format, but actual format is {actual_format}. Color issues might occur if not handled.")

            self.picam.start()
            logging.info(f"Picamera2 (ID: {self.camera_index}) started successfully.")
            
            # Now that actual resolution is known, calculate new_camera_mtx
            self._calculate_optimal_new_camera_matrix()

            self.is_initialized = True
            self.last_error = None
            self.last_capture_time = time.monotonic()
            self._fps_calc_start_time = time.monotonic()
            self._frame_counter_for_fps = 0
            return True

        except Exception as e:
            self.last_error = f"Failed to initialize Picamera2 {self.camera_index}: {e}"
            logging.error(self.last_error, exc_info=True)
            if self.picam is not None:
                try:
                    if hasattr(self.picam, 'started') and self.picam.started: self.picam.stop()
                    self.picam.close()
                except Exception as close_err:
                    logging.error(f"Error closing picam after failed init: {close_err}")
            self.picam = None
            self.is_initialized = False
            return False

    def capture_frame(self, undistort=True):
        if not self.is_initialized or not self.picam or not (hasattr(self.picam, 'started') and self.picam.started):
            self.last_error = "Cannot capture frame, Picamera2 not initialized or not started."
            return None

        try:
            # Picamera2 with "BGR888" format is expected to return a 3-channel BGR numpy array.
            # If it were RGBA (e.g. from "RGBA8888"), then capture_array() might give 4 channels.
            frame_data = self.picam.capture_array("main") 
            
            if frame_data is None:
                self.last_error = "Picamera2 capture_array returned None."
                logging.warning(self.last_error)
                return None

            # *** Explicit Color Conversion Attempt ***
            # If picam2's "BGR888" is actually providing RGB in the numpy array order
            # then this conversion will fix it for OpenCV's BGR expectation.
            # If it's truly BGR, this will swap R and B, making colors wrong in a different way.
            # This is a key line for diagnosing the color problem.
            frame_bgr = cv2.cvtColor(frame_data, cv2.COLOR_RGB2BGR)
            # If you find this line *causes* the color swap, it means frame_data was already BGR.
            # If this line *fixes* the skin tones, it means frame_data was RGB.

            self.last_error = None 
        except Exception as e:
            self.last_error = f"Error capturing/processing frame from Picamera2: {e}"
            logging.error(self.last_error, exc_info=True)
            return None
        
        # FPS Calculation
        self._frame_counter_for_fps += 1
        current_time = time.monotonic()
        elapsed_time_fps_calc = current_time - (self._fps_calc_start_time if self._fps_calc_start_time else current_time)
        if elapsed_time_fps_calc >= 1.0: # Update measured_fps_avg about once per second
            self.measured_fps_avg = self._frame_counter_for_fps / elapsed_time_fps_calc
            self._frame_counter_for_fps = 0
            self._fps_calc_start_time = current_time
        self.last_capture_time = current_time


        if undistort:
            if self.mtx is not None and self.dist is not None:
                # Use new_camera_mtx if available and calculated, else original mtx
                undist_mtx = self.new_camera_mtx if self.new_camera_mtx is not None else self.mtx
                try:
                    undistorted_frame = cv2.undistort(frame_bgr, self.mtx, self.dist, None, undist_mtx)
                    # Optional: crop with ROI if alpha=0 was used for getOptimalNewCameraMatrix
                    # if self.roi is not None and undist_mtx is self.new_camera_mtx: # only crop if new_camera_mtx was used for undistortion
                    #    x, y, w, h = self.roi
                    #    if w > 0 and h > 0:
                    #        undistorted_frame = undistorted_frame[y:y+h, x:x+w]
                    return undistorted_frame
                except cv2.error as cv_err:
                    logging.error(f"OpenCV error during undistortion: {cv_err}. Returning non-undistorted frame.")
                    return frame_bgr # Fallback
            else:
                return frame_bgr 
        else:
            return frame_bgr

    def get_camera_properties(self):
        if not self.is_initialized or not self.picam:
            return {
                'is_initialized': False,
                'error': self.last_error or "Picamera2 not initialized"
            }
        
        actual_format = "N/A"
        target_fps_val = self.fps # This should be from config after picam.configure
        try:
            # Re-check config if possible, but self.fps should be source of truth post-init
            cam_config = self.picam.camera_configuration()
            actual_format = cam_config['main']['format']
            target_fps_val = cam_config['controls'].get('FrameRate', self.fps)
        except Exception:
            pass # Stick to stored values if re-check fails

        return {
            'is_initialized': True,
            'camera_index': self.camera_index,
            'frame_width': self.frame_width,
            'frame_height': self.frame_height,
            'target_fps': target_fps_val, 
            'measured_fps': self.measured_fps_avg,
            'backend_name': 'Picamera2',
            'stream_format': actual_format,
            'mtx_loaded': self.mtx is not None,
            'dist_loaded': self.dist is not None,
            'last_error': self.last_error
        }

    def shutdown(self):
        logging.info("Shutting down CameraManager (Picamera2)...")
        self.is_initialized = False # Set early to prevent race conditions
        if self.picam is not None:
            try:
                if hasattr(self.picam, 'started') and self.picam.started:
                    self.picam.stop()
                    logging.info(f"Picamera2 (ID: {self.camera_index}) stopped.")
                self.picam.close()
                logging.info(f"Picamera2 (ID: {self.camera_index}) closed.")
            except Exception as e:
                logging.error(f"Error during Picamera2 shutdown: {e}", exc_info=True)
        self.picam = None
        logging.info("CameraManager (Picamera2) shutdown complete.")

if __name__ == '__main__':
    import os # Required for dummy Config creation below
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - [%(threadName)s:%(lineno)d] - %(message)s')
    
    # --- Create a dummy Config.py for standalone testing if it doesn't exist ---
    # This is simplified, ensure your actual Config.py has all necessary attributes.
    if not os.path.exists("Config.py"):
        print("Creating dummy Config.py for testing CameraManager.py standalone.")
        with open("Config.py", "w") as f:
            f.write("CAMERA_INDEX = 0\n")
            f.write("CAM_REQUESTED_WIDTH = 1280\n") 
            f.write("CAM_REQUESTED_HEIGHT = 720\n")
            f.write("CAM_REQUESTED_FPS = 30.0\n") # Use float for FPS
            f.write("CALIBRATION_DATA_FILE = 'camera_calibration_data.npz'\n") 
            f.write("LOG_LEVEL = 'INFO'\n")
            f.write("LOG_FORMAT = '%(asctime)s - %(levelname)s - [%(threadName)s:%(lineno)d] - %(message)s'\n")
            f.write("LOG_DATE_FORMAT = '%Y-%m-%d %H:%M:%S'\n")
    try:
        # Ensure Config is reloaded if it was just created or if this script is run multiple times
        import importlib
        if 'Config' in locals() or 'Config' in globals():
            importlib.reload(main_project_config) # Use the alias if already imported
        else:
            import Config as main_project_config

    except ImportError:
        print("Failed to import Config.py. Please ensure it exists.")
        exit()
    # --- End dummy Config.py creation ---

    cam_manager = CameraManager(
        camera_index=main_project_config.CAMERA_INDEX,
        width=main_project_config.CAM_REQUESTED_WIDTH,
        height=main_project_config.CAM_REQUESTED_HEIGHT,
        fps=main_project_config.CAM_REQUESTED_FPS
    )

    # For testing, create a dummy calibration file if it doesn't exist
    dummy_calib_file = main_project_config.CALIBRATION_DATA_FILE
    if not os.path.exists(dummy_calib_file):
        logging.info(f"Creating dummy calibration file: {dummy_calib_file}")
        dummy_mtx = np.array([[1000, 0, 640],[0,1000,360],[0,0,1]], dtype=float)
        dummy_dist = np.array([[0.0,0,0,0,0]], dtype=float) # No distortion
        dummy_image_size = (1280, 720) # W, H
        np.savez(dummy_calib_file, camera_matrix=dummy_mtx, dist_coeffs=dummy_dist, image_size=dummy_image_size)

    if cam_manager.load_calibration_data(main_project_config.CALIBRATION_DATA_FILE):
        logging.info("Calibration data loaded for example.")
    else:
        logging.warning(f"Failed to load calibration data for example from: {main_project_config.CALIBRATION_DATA_FILE}")

    if cam_manager.initialize_camera():
        cv2.namedWindow("Picamera2 Stream Test", cv2.WINDOW_NORMAL)
        frame_count_display = 0
        start_time_loop = time.monotonic()

        while frame_count_display < 150 : # Run for 150 frames
            # Test with undistort=True to exercise that path
            processed_frame = cam_manager.capture_frame(undistort=True) 

            if processed_frame is not None:
                frame_count_display += 1
                # Add FPS text to the displayed frame
                props = cam_manager.get_camera_properties()
                measured_fps_text = f"CamFPS: {props.get('measured_fps', 0.0):.2f}"
                cv2.putText(processed_frame, measured_fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.imshow("Picamera2 Stream Test", processed_frame)
            else:
                logging.error(f"Failed to capture frame in example. Last error: {cam_manager.last_error}")
                if not cam_manager.is_initialized:
                    logging.error("Camera became uninitialized. Exiting example.")
                    break
                time.sleep(0.1) 

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'): 
                logging.info("Re-initializing camera by user request (r key)...")
                # cam_manager.shutdown() # Full shutdown before re-init
                # time.sleep(0.5) 
                if cam_manager.initialize_camera(): # initialize_camera now handles prior shutdown
                     logging.info("Camera re-initialized successfully.")
                else:
                     logging.error("Failed to re-initialize camera.")
                     break 
        
        end_time_loop = time.monotonic()
        elapsed_time = end_time_loop - start_time_loop
        avg_fps_loop = frame_count_display / elapsed_time if elapsed_time > 0 else 0
        logging.info(f"\nExample loop finished. Displayed {frame_count_display} frames in {elapsed_time:.2f}s. Avg Loop FPS: {avg_fps_loop:.2f}")

        cam_manager.shutdown()
        cv2.destroyAllWindows()
    else:
        logging.error(f"Failed to initialize Picamera2. Error: {cam_manager.last_error}")

    logging.info("CameraManager (Picamera2) example finished.")