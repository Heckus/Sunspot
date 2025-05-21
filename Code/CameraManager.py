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
        self.picam = None # Changed from self.cap
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

            # Initial calculation based on requested/current frame dimensions.
            # This will be re-calculated if actual dimensions change after camera init.
            h, w = self.frame_height, self.frame_width
            self.new_camera_mtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w, h), 1, (w, h))
            if self.new_camera_mtx is not None:
                 logging.info(f"Calculated new optimal camera matrix (initial):\n{self.new_camera_mtx}")
                 logging.info(f"Calculated ROI for undistortion (initial): {self.roi}")
            else:
                 logging.warning("Failed to calculate new optimal camera matrix (initial).")
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
            # Format "RGB888" gives a 3-channel RGB numpy array.
            # Controls can be added here, e.g. for exposure, gain, if needed.
            # For now, primarily setting size and framerate.
            video_config_params = {
                "main": {"size": (self.requested_width, self.requested_height), "format": "RGB888"},
                "controls": {"FrameRate": self.requested_fps}
                # Add other controls like "AnalogueGain", "ExposureTime" if needed from main_project_config
            }
            
            # Example of adding more controls if they were in your project's Config.py
            # if hasattr(main_project_config, 'CAMERA_ANALOGUE_GAIN'):
            #     video_config_params["controls"]["AnalogueGain"] = main_project_config.CAMERA_ANALOGUE_GAIN
            # if hasattr(main_project_config, 'CAMERA_EXPOSURE_TIME'):
            #      video_config_params["controls"]["ExposureTime"] = main_project_config.CAMERA_EXPOSURE_TIME


            cam_config = self.picam.create_video_configuration(**video_config_params)
            logging.info(f"Picamera2 generated video config: {cam_config}")
            self.picam.configure(cam_config)
            
            # Update actual frame dimensions and FPS from the configured stream
            self.frame_width = self.picam.camera_configuration()['main']['size'][0]
            self.frame_height = self.picam.camera_configuration()['main']['size'][1]
            # FrameRate might not always be perfectly what was requested or easily readable back for exact FPS.
            # Picamera2 aims to match it. For simplicity, we'll use the configured main stream's FPS target
            # or rely on measured FPS. Actual control 'FrameRate' might be available.
            try:
                self.fps = self.picam.camera_configuration()['controls']['FrameRate']
            except KeyError:
                logging.warning(f"Could not read 'FrameRate' from controls, using requested FPS: {self.requested_fps}")
                self.fps = self.requested_fps # Fallback

            logging.info(f"Picamera2 configured. Actual stream: {self.frame_width}x{self.frame_height} @ ~{self.fps:.2f}fps, Format: {self.picam.camera_configuration()['main']['format']}")

            self.picam.start()
            logging.info(f"Picamera2 (ID: {self.camera_index}) started successfully.")
            
            # If resolution changed from requested, and calibration data is loaded, recalculate optimal matrix
            if (self.frame_width != self.requested_width or self.frame_height != self.requested_height) and \
               (self.mtx is not None and self.dist is not None):
                logging.info("Actual resolution differs from requested. Re-calculating optimal new camera matrix and ROI.")
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
            self.measured_fps_avg = None # Reset measured FPS
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
            logging.warning(self.last_error)
            return None

        try:
            # capture_array("main") returns an RGB NumPy array with the main stream
            frame_rgb = self.picam.capture_array("main")
            if frame_rgb is None:
                self.last_error = "Picamera2 capture_array returned None."
                logging.warning(self.last_error)
                return None
            
            # Convert RGB to BGR as OpenCV functions (and potentially WebUI) expect BGR
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_BGR2RGB)
            self.last_error = None # Clear error if capture successful

        except Exception as e:
            self.last_error = f"Error capturing frame from Picamera2: {e}"
            logging.error(self.last_error, exc_info=True)
            return None
        
        # --- Calculate measured FPS (optional but useful) ---
        current_time = time.monotonic()
        if self.last_capture_time is not None:
            time_diff = current_time - self.last_capture_time
            if time_diff > 0.0001: 
                instant_fps = 1.0 / time_diff
                alpha = 0.05 
                self.measured_fps_avg = (alpha * instant_fps + (1 - alpha) * self.measured_fps_avg) \
                                        if self.measured_fps_avg is not None else instant_fps
        self.last_capture_time = current_time
        # --- End FPS Calculation ---

        if undistort:
            if self.mtx is not None and self.dist is not None:
                current_undistort_mtx = self.new_camera_mtx if self.new_camera_mtx is not None else self.mtx
                undistorted_frame = cv2.undistort(frame_bgr, self.mtx, self.dist, None, current_undistort_mtx)
                # Optionally crop using self.roi if new_camera_mtx was calculated with alpha=0 or a specific ROI
                # if self.roi is not None and self.new_camera_mtx is not None: # Example: if alpha=0 for getOptimal...
                #     x, y, w, h = self.roi
                #     if w > 0 and h > 0:
                #         undistorted_frame = undistorted_frame[y:y+h, x:x+w]
                return undistorted_frame
            else:
                logging.debug("Cannot undistort: Camera calibration data (mtx, dist) not loaded.")
                return frame_bgr # Return BGR frame if no calibration data
        else:
            return frame_bgr # Return BGR frame

    def get_camera_properties(self):
        """
        Returns a dictionary of current camera properties.
        """
        if not self.is_initialized or not self.picam:
            return {
                'is_initialized': False,
                'error': self.last_error or "Picamera2 not initialized"
            }
        
        # Try to get current control values if picam is active
        actual_fps_from_controls = self.fps # Use the one set during/after configure
        try:
            if self.picam.started: # Some controls are only available when started
                 # This might be heavy to call frequently; using stored self.fps is preferred for target FPS
                 # control_values = self.picam.camera_controls 
                 # actual_fps_from_controls = control_values.get('FrameRate', self.fps)
                 pass # For now, avoid frequent calls to camera_controls in get_properties
        except Exception:
            pass # Ignore if controls can't be read

        return {
            'is_initialized': True,
            'camera_index': self.camera_index,
            'frame_width': self.frame_width,
            'frame_height': self.frame_height,
            'fps': actual_fps_from_controls, # Target/configured FPS
            'measured_fps': self.measured_fps_avg,
            'backend_name': 'Picamera2', # Explicitly state backend
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
    # Setup basic logging for the example
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    # Example Usage
    # These would normally come from your project's Config.py
    # For this standalone test, using defaults or hardcoded values.
    # Make sure main_project_config.CALIBRATION_DATA_FILE points to your actual calibration file.
    
    # Use parameters from the project's Config.py
    cam_manager = CameraManager(
        camera_index=main_project_config.CAMERA_INDEX,
        width=main_project_config.CAM_REQUESTED_WIDTH,
        height=main_project_config.CAM_REQUESTED_HEIGHT,
        fps=main_project_config.CAM_REQUESTED_FPS
    )

    if cam_manager.load_calibration_data(main_project_config.CALIBRATION_DATA_FILE):
        logging.info("Calibration data loaded successfully for example usage.")
    else:
        logging.warning(f"Failed to load calibration data for example. Check path: {main_project_config.CALIBRATION_DATA_FILE}")

    if cam_manager.initialize_camera():
        cv2.namedWindow("Picamera2 Stream", cv2.WINDOW_NORMAL)
        frame_count = 0
        start_time_loop = time.monotonic() # Use monotonic for duration

        while True:
            # Capture BGR frame, optionally undistorted
            bgr_frame = cam_manager.capture_frame(undistort=True) 

            if bgr_frame is not None:
                frame_count += 1
                cv2.imshow("Picamera2 Stream", bgr_frame)

                props = cam_manager.get_camera_properties()
                if props.get('measured_fps') is not None:
                    print(f"Measured FPS: {props['measured_fps']:.2f}", end='\r')
            else:
                logging.error(f"Failed to capture frame in example loop. Last error: {cam_manager.last_error}")
                if not cam_manager.is_initialized: # If camera became uninitialized, break
                    break
                time.sleep(0.1) # Wait a bit on error

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'): # Example to re-initialize camera
                logging.info("Re-initializing camera by user request (r key)...")
                cam_manager.initialize_camera()


        # Clear the FPS print from the console line
        print(" " * 50, end='\r') 

        end_time_loop = time.monotonic()
        elapsed_time = end_time_loop - start_time_loop
        avg_fps_loop = frame_count / elapsed_time if elapsed_time > 0 else 0
        logging.info(f"\nExample loop finished. Captured {frame_count} frames in {elapsed_time:.2f}s. Average Loop FPS: {avg_fps_loop:.2f}")

        cam_manager.shutdown()
        cv2.destroyAllWindows()
    else:
        logging.error(f"Failed to initialize Picamera2. Error: {cam_manager.last_error}")

    logging.info("CameraManager (Picamera2) example finished.")