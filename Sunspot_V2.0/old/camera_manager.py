# -*- coding: utf-8 -*-
"""
camera_manager.py

Manages the Picamera2 camera, including initialization, configuration,
frame capture, and video recording.
"""

import os
import time
import datetime
import logging
import threading
import cv2
from picamera2 import Picamera2
from libcamera import controls

# Import configuration constants
import config

# Helper function to determine target FPS based on resolution
def get_target_fps(width, height):
    """Determines the target FPS based on resolution."""
    # Prioritize specific high-resolution cases first
    if width >= 3280: # Max resolution (e.g., 3280x2464)
        return 15.0
    elif width == 1920 and height == 1080: # 1080p
        return 30.0
    # Group lower resolutions that can likely handle higher FPS
    elif width <= 1640: # Includes (1640, 1232), (1280, 720), (640, 480) etc.
        return 40.0 # Request higher FPS for lower resolutions
    else: # Default fallback if resolution doesn't match specific cases
        return config.DEFAULT_FRAME_RATE

# Helper function to find writable USB mounts
def get_usb_mounts():
    """
    Finds writable directories under the configured USB base path.

    Returns:
        list: A list of full paths to writable USB mount points.
    """
    mounts = []
    base_path = config.USB_BASE_PATH
    logging.debug(f"Checking for USB mounts under: {base_path}")
    if not os.path.isdir(base_path):
        logging.warning(f"USB base path '{base_path}' does not exist or is not a directory.")
        return mounts

    try:
        for item in os.listdir(base_path):
            path = os.path.join(base_path, item)
            # Check if it's a directory and we have write access
            if os.path.isdir(path) and os.access(path, os.W_OK):
                # Perform a quick write test for extra certainty (e.g., handles permissions issues)
                test_file = os.path.join(path, f".write_test_{os.getpid()}")
                try:
                    with open(test_file, 'w') as f:
                        f.write('test')
                    os.remove(test_file)
                    mounts.append(path)
                    logging.debug(f"Found writable mount: {path}")
                except Exception as write_err:
                    logging.warning(f"Directory {path} appears mounted but test write failed: {write_err}")
            elif os.path.isdir(path):
                logging.debug(f"Directory {path} found but is not writable.")

        if not mounts:
            logging.debug("No writable USB mounts found.")
    except Exception as e:
        logging.error(f"Error finding USB mounts in {base_path}: {e}")

    return mounts


class CameraManager:
    """Handles all camera operations and state."""

    def __init__(self):
        """Initializes the CameraManager."""
        self.picam2 = None
        self.is_initialized = False
        self.last_error = None

        # State variables
        self.current_resolution_index = config.DEFAULT_RESOLUTION_INDEX
        self.output_frame = None # Stores the latest captured frame
        self.is_recording = False
        self.video_writers = [] # List of OpenCV VideoWriter objects
        self.recording_paths = [] # List of corresponding file paths

        # Locks for thread safety
        self.frame_lock = threading.Lock() # Protects access to output_frame
        self.config_lock = threading.Lock() # Protects access to camera state/config changes
        self.recording_lock = threading.Lock() # Protects access to recording state/writers

        # --- Initialize Camera Controls to Defaults from Config ---
        # Store the actual libcamera control objects/values
        try:
            self.current_awb_mode = controls.AwbModeEnum.__members__[config.DEFAULT_AWB_MODE_NAME]
        except KeyError:
            logging.error(f"Default AWB mode '{config.DEFAULT_AWB_MODE_NAME}' not found! Falling back.")
            self.current_awb_mode = controls.AwbModeEnum.Auto # Safe fallback
        try:
             self.current_ae_mode = controls.AeExposureModeEnum.__members__[config.DEFAULT_AE_MODE_NAME]
        except KeyError:
            logging.error(f"Default AE mode '{config.DEFAULT_AE_MODE_NAME}' not found! Falling back.")
            self.current_ae_mode = controls.AeExposureModeEnum.Normal
        try:
            self.current_metering_mode = controls.AeMeteringModeEnum.__members__[config.DEFAULT_METERING_MODE_NAME]
        except KeyError:
            logging.error(f"Default Metering mode '{config.DEFAULT_METERING_MODE_NAME}' not found! Falling back.")
            self.current_metering_mode = controls.AeMeteringModeEnum.CentreWeighted
        try:
            # Noise reduction is in 'draft', handle potential absence
            self.current_noise_reduction_mode = controls.draft.NoiseReductionModeEnum.__members__[config.DEFAULT_NOISE_REDUCTION_MODE_NAME]
        except (AttributeError, KeyError):
            logging.warning(f"Default Noise Reduction mode '{config.DEFAULT_NOISE_REDUCTION_MODE_NAME}' not found or draft controls unavailable. Setting to Off.")
            self.current_noise_reduction_mode = controls.draft.NoiseReductionModeEnum.Off # Assuming 'Off' exists

        self.current_brightness = config.DEFAULT_BRIGHTNESS
        self.current_contrast = config.DEFAULT_CONTRAST
        self.current_saturation = config.DEFAULT_SATURATION
        self.current_sharpness = config.DEFAULT_SHARPNESS

        logging.info("CameraManager initialized with default control settings.")


    def initialize_camera(self, resolution_index=None):
        """
        Initializes or re-initializes the Picamera2 instance with the specified resolution index.
        If index is None, uses the current internal state.

        Args:
            resolution_index (int, optional): The index in config.SUPPORTED_RESOLUTIONS. Defaults to None.

        Returns:
            bool: True if initialization was successful, False otherwise.
        """
        with self.config_lock: # Ensure exclusive access during initialization/reconfiguration
            if resolution_index is not None:
                if 0 <= resolution_index < len(config.SUPPORTED_RESOLUTIONS):
                    self.current_resolution_index = resolution_index
                else:
                    logging.error(f"Invalid resolution index {resolution_index} requested. Using current index {self.current_resolution_index}.")
                    # Keep the current index

            target_width, target_height = self.get_current_resolution() # Get dimensions based on the (potentially updated) index
            logging.info(f"Attempting to initialize camera at index {self.current_resolution_index} ({target_width}x{target_height})...")

            # --- Stop/Close existing instance ---
            if self.picam2 is not None:
                try:
                    if self.picam2.started:
                        logging.info("Stopping existing Picamera2 instance...")
                        self.picam2.stop()
                    logging.info("Closing existing Picamera2 instance...")
                    self.picam2.close()
                    logging.info("Previous Picamera2 instance stopped and closed.")
                except Exception as e:
                    logging.warning(f"Error stopping/closing previous Picamera2 instance: {e}")
                finally:
                    self.picam2 = None
                    self.is_initialized = False
                time.sleep(0.5) # Give time for resources to release

            # --- Create and Configure New Instance ---
            try:
                # Pass tuning data loaded in config.py
                self.picam2 = Picamera2(tuning=config.CAMERA_TUNING)
                logging.info("Picamera2 object created.")

                target_fps = get_target_fps(target_width, target_height)
                logging.info(f"Targeting FPS: {target_fps:.1f} for resolution {target_width}x{target_height}")

                # --- Create Video Configuration with ALL current controls ---
                # Access state variables directly as we hold the config_lock
                controls_to_set = {
                    "FrameRate": target_fps,
                    "NoiseReductionMode": self.current_noise_reduction_mode,
                    "AeEnable": True, # Assume AE should generally be enabled
                    "AeExposureMode": self.current_ae_mode,
                    "AeMeteringMode": self.current_metering_mode,
                    "AwbEnable": True, # Assume AWB should generally be enabled
                    "AwbMode": self.current_awb_mode,
                    "Brightness": self.current_brightness,
                    "Contrast": self.current_contrast,
                    "Saturation": self.current_saturation,
                    "Sharpness": self.current_sharpness,
                }
                # Remove None controls if any draft mode failed init
                controls_to_set = {k: v for k, v in controls_to_set.items() if v is not None}


                cam_config = self.picam2.create_video_configuration(
                    main={"size": (target_width, target_height), "format": "RGB888"}, # Format needed by OpenCV
                    controls=controls_to_set
                )
                logging.info(f"Configuring Picamera2 with: main={cam_config['main']}, controls={cam_config['controls']}")

                self.picam2.configure(cam_config)
                time.sleep(0.5) # Give driver time to settle configuration

                # --- Verify Applied Configuration ---
                # (Optional but recommended)
                new_config = self.picam2.camera_configuration()
                if new_config:
                    applied_controls = new_config.get('controls', {})
                    applied_main = new_config.get('main', {})
                    logging.info(f"Verified Applied Config: main={applied_main}, controls={applied_controls}")
                    # Log differences if FrameRate was adjusted by driver
                    actual_fps = applied_controls.get('FrameRate')
                    if actual_fps and abs(actual_fps - target_fps) > 0.1:
                         logging.warning(f"Camera driver adjusted FrameRate from requested {target_fps:.1f} to {actual_fps:.1f}")
                else:
                     logging.warning("Could not get camera configuration after applying.")


                logging.info("Configuration successful!")

                # --- Start Camera ---
                self.picam2.start()
                logging.info("Camera started")
                # Allow more time for sensor settings (like AWB, AE) to stabilize
                time.sleep(2.0)

                # --- Log Final Running Configuration ---
                actual_config = self.picam2.camera_configuration()
                if not actual_config:
                    raise RuntimeError("Failed to get camera configuration after start.")

                actual_format = actual_config.get('main', {})
                actual_w = actual_format.get('size', (0,0))[0]
                actual_h = actual_format.get('size', (0,0))[1]
                actual_fmt_str = actual_format.get('format', 'Unknown')
                actual_fps = actual_config.get('controls', {}).get('FrameRate', 'N/A')
                actual_awb = actual_config.get('controls', {}).get('AwbMode', 'N/A')
                logging.info(f"Picamera2 initialized. Actual stream: {actual_w}x{actual_h} {actual_fmt_str} @ {actual_fps} fps. AWB Mode: {actual_awb}")

                self.is_initialized = True
                self.last_error = None
                return True

            except Exception as e:
                logging.error(f"!!! Failed to initialize Picamera2 at {target_width}x{target_height}: {e}", exc_info=True)
                self.last_error = f"Picamera2 Init Error ({target_width}x{target_height}): {e}"
                if self.picam2 is not None:
                    try:
                        # Ensure cleanup even on partial failure
                        if self.picam2.started: self.picam2.stop()
                        self.picam2.close()
                    except Exception as close_e:
                        logging.error(f"Error closing picam2 after init failure: {close_e}")
                self.picam2 = None
                self.is_initialized = False
                return False

    def get_current_resolution(self):
        """Returns the current resolution tuple (width, height) based on the index."""
        # No lock needed here if current_resolution_index is only written inside config_lock
        # in initialize_camera
        try:
            return config.SUPPORTED_RESOLUTIONS[self.current_resolution_index]
        except IndexError:
            logging.error(f"Invalid internal resolution index {self.current_resolution_index}. Falling back to default.")
            # Fallback safely
            safe_default_index = max(0, min(len(config.SUPPORTED_RESOLUTIONS) - 1, config.DEFAULT_RESOLUTION_INDEX))
            return config.SUPPORTED_RESOLUTIONS[safe_default_index]

    def apply_camera_controls(self, controls_dict):
        """
        Applies a dictionary of controls to the running camera.

        Args:
            controls_dict (dict): Dictionary of control names and values to set.
                                  Values should be the correct type (e.g., enum members, floats).

        Returns:
            bool: True if controls were applied successfully, False otherwise.
        """
        if not self.is_initialized or not self.picam2 or not self.picam2.started:
            logging.error("Cannot apply controls: Camera not initialized or started.")
            self.last_error = "Control Apply Error: Camera not ready."
            return False

        logging.debug(f"Applying controls: {controls_dict}")
        try:
            with self.config_lock: # Protect access while setting controls
                self.picam2.set_controls(controls_dict)
                # Update internal state after successful application
                for key, value in controls_dict.items():
                    if key == 'AwbMode': self.current_awb_mode = value
                    elif key == 'AeExposureMode': self.current_ae_mode = value
                    elif key == 'AeMeteringMode': self.current_metering_mode = value
                    elif key == 'NoiseReductionMode': self.current_noise_reduction_mode = value
                    elif key == 'Brightness': self.current_brightness = value
                    elif key == 'Contrast': self.current_contrast = value
                    elif key == 'Saturation': self.current_saturation = value
                    elif key == 'Sharpness': self.current_sharpness = value
                    # Add other controls if needed
                logging.debug("Controls applied successfully.")
                self.last_error = None # Clear previous control errors
            # Give controls time to settle, especially AWB/AE
            if any(k in controls_dict for k in ['AwbMode', 'AeExposureMode', 'AeMeteringMode', 'Brightness']):
                time.sleep(0.5)
            else:
                time.sleep(0.1)
            return True
        except Exception as e:
            logging.error(f"!!! Error applying camera controls: {e}", exc_info=True)
            self.last_error = f"Control Apply Error: {e}"
            return False

    def get_camera_state(self):
        """Returns a dictionary containing the current camera state and control values."""
        with self.config_lock: # Ensure consistent read of state
            state = {
                'is_initialized': self.is_initialized,
                'resolution_index': self.current_resolution_index,
                'resolution_wh': self.get_current_resolution(),
                'is_recording': self.is_recording, # Read directly, recording lock protects writes
                'recording_paths': list(self.recording_paths), # Read directly, recording lock protects writes
                'last_error': self.last_error,
                # Control values (convert enums to names for status reporting)
                'awb_mode': self.current_awb_mode.name if hasattr(self.current_awb_mode, 'name') else str(self.current_awb_mode),
                'ae_mode': self.current_ae_mode.name if hasattr(self.current_ae_mode, 'name') else str(self.current_ae_mode),
                'metering_mode': self.current_metering_mode.name if hasattr(self.current_metering_mode, 'name') else str(self.current_metering_mode),
                'noise_reduction_mode': self.current_noise_reduction_mode.name if hasattr(self.current_noise_reduction_mode, 'name') else str(self.current_noise_reduction_mode),
                'brightness': self.current_brightness,
                'contrast': self.current_contrast,
                'saturation': self.current_saturation,
                'sharpness': self.current_sharpness,
            }
        return state

    def start_recording(self):
        """
        Starts recording video to all available/writable USB drives.

        Returns:
            bool: True if recording started successfully on at least one drive, False otherwise.
        """
        with self.recording_lock: # Ensure exclusive access to recording state
            if self.is_recording:
                logging.warning("Start recording called, but already recording.")
                return True # Indicate it's already in the desired state

            if not self.is_initialized or not self.picam2 or not self.picam2.started:
                 logging.error("Cannot start recording, camera is not available.")
                 self.last_error = "Camera not available for recording."
                 return False

            logging.info("Attempting to start recording...")
            usb_drives = get_usb_mounts()
            if not usb_drives:
                logging.warning(f"Cannot start recording: No writable USB drives found in {config.USB_BASE_PATH}.")
                self.last_error = f"Cannot start recording: No writable USB drives found"
                return False

            self.video_writers.clear()
            self.recording_paths.clear()
            success_count = 0
            start_error = None

            try:
                # --- Get ACTUAL configuration ---
                cam_config = self.picam2.camera_configuration()
                if not cam_config:
                    raise RuntimeError("Failed to get camera configuration before recording start.")

                main_stream_config = cam_config.get('main', {})
                controls_config = cam_config.get('controls', {})

                width = main_stream_config.get('size', (0,0))[0]
                height = main_stream_config.get('size', (0,0))[1]

                # --- Use ACTUAL FPS from camera controls ---
                actual_fps = controls_config.get('FrameRate')
                if actual_fps is None:
                    logging.error("!!! Could not determine actual FrameRate from camera config. Falling back.")
                    # Fallback logic: Use the function based on current resolution
                    current_w_fb, current_h_fb = self.get_current_resolution()
                    actual_fps = get_target_fps(current_w_fb, current_h_fb)
                    self.last_error = "Rec FPS Fallback: Couldn't read actual FPS."

                # Ensure FPS is a valid float for VideoWriter
                try:
                    fps_for_writer = float(actual_fps)
                    if fps_for_writer <= 0: raise ValueError("Invalid FPS value <= 0")
                except (TypeError, ValueError) as fps_err:
                    logging.error(f"!!! Invalid FPS value obtained ({actual_fps}). Defaulting to 15 FPS. Error: {fps_err}")
                    fps_for_writer = 15.0
                    self.last_error = f"Rec FPS Error: Invalid value {actual_fps}"

                if width <= 0 or height <= 0:
                    raise ValueError(f"Invalid camera dimensions obtained: {width}x{height}")

                logging.info(f"Starting recording with dimensions: {width}x{height} @ {fps_for_writer:.2f} fps")

                fourcc = cv2.VideoWriter_fourcc(*config.RECORDING_FORMAT)
                timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

                for drive_path in usb_drives:
                    try:
                        filename = f"recording_{timestamp}_{width}x{height}{config.RECORDING_EXTENSION}"
                        full_path = os.path.join(drive_path, filename)
                        writer = cv2.VideoWriter(full_path, fourcc, fps_for_writer, (width, height))
                        if not writer.isOpened():
                            raise IOError(f"Failed to open VideoWriter for path: {full_path}")
                        self.video_writers.append(writer)
                        self.recording_paths.append(full_path)
                        logging.info(f"Successfully started recording to: {full_path}")
                        success_count += 1
                    except Exception as e:
                        logging.error(f"!!! Failed to create VideoWriter for drive {drive_path}: {e}", exc_info=True)
                        if start_error is None: # Record the first error encountered
                            start_error = f"Failed writer {os.path.basename(drive_path)}: {e}"

                if success_count > 0:
                    self.is_recording = True
                    logging.info(f"Recording successfully started on {success_count} drive(s).")
                    if start_error and success_count < len(usb_drives):
                         self.last_error = f"Partial Rec Start Fail: {start_error}"
                    elif not start_error: # Clear previous recording errors on full success
                         if self.last_error and ("Recording" in self.last_error or "writers" in self.last_error or "USB" in self.last_error or "sync" in self.last_error or "Rec FPS" in self.last_error):
                             logging.info(f"Clearing previous recording error: '{self.last_error}'")
                             self.last_error = None
                    return True
                else:
                    # Failed to start on ANY drive
                    self.is_recording = False
                    logging.error("Failed to start recording on ANY USB drive.")
                    self.last_error = f"Recording Start Failed: {start_error or 'No writers opened'}"
                    # Clean up any potentially opened (but failed) writers
                    for writer in self.video_writers:
                        try: writer.release()
                        except: pass
                    self.video_writers.clear()
                    self.recording_paths.clear()
                    return False

            except Exception as e:
                logging.error(f"!!! Critical error during recording setup: {e}", exc_info=True)
                self.last_error = f"Recording Setup Error: {e}"
                # Ensure cleanup even if setup fails midway
                self.stop_recording() # Call internal stop which handles locks/state
                return False

    def stop_recording(self):
        """Stops recording, releases video writers, and syncs the filesystem."""
        with self.recording_lock: # Ensure exclusive access
            if not self.is_recording:
                # If called when not recording, ensure lists are clear just in case
                if self.video_writers or self.recording_paths:
                     logging.warning("stop_recording called but not recording; clearing potentially stale writer lists.")
                     self.video_writers.clear()
                     self.recording_paths.clear()
                return

            logging.info("Stopping recording...")
            self.is_recording = False # Set flag immediately
            released_count = 0
            # Copy lists to avoid modifying while iterating (though lock should prevent issues)
            writers_to_release = list(self.video_writers)
            paths_recorded = list(self.recording_paths)
            self.video_writers.clear()
            self.recording_paths.clear()

        # Release writers outside the lock to avoid holding it during potentially long I/O
        logging.info(f"Releasing {len(writers_to_release)} video writer(s)...")
        for i, writer in enumerate(writers_to_release):
            path = paths_recorded[i] if i < len(paths_recorded) else f"Unknown Path (Writer Index {i})"
            try:
                writer.release()
                logging.info(f"Successfully released writer for: {path}")
                released_count += 1
            except Exception as e:
                logging.error(f"Error releasing VideoWriter for {path}: {e}", exc_info=True)
                # Potentially set last_error here?

        if released_count > 0:
            logging.info("Syncing filesystem to ensure data is written to USB drives...")
            try:
                sync_start_time = time.monotonic()
                # os.sync() # Python's sync - might be sufficient
                os.system('sync') # Call external sync command for robustness
                sync_duration = time.monotonic() - sync_start_time
                logging.info(f"Filesystem sync completed in {sync_duration:.2f} seconds.")
                # time.sleep(0.5) # Optional extra safety sleep
            except Exception as e:
                logging.error(f"!!! Failed to execute 'sync' command: {e}", exc_info=True)
                self.last_error = "Filesystem sync failed after recording."
        else:
            logging.warning("No video writers were successfully released, skipping filesystem sync.")

        logging.info(f"Recording fully stopped. Released {released_count} writer(s) out of {len(writers_to_release)} attempted.")


    def capture_frame(self):
        """
        Captures a frame from the camera and updates the internal output_frame.
        Also writes the frame to video writers if recording is active.

        Returns:
            numpy.ndarray or None: The captured frame (BGR format) or None if capture failed.
        """
        if not self.is_initialized or not self.picam2 or not self.picam2.started:
            # logging.debug("Capture skipped: Camera not ready.") # Too verbose for regular logging
            return None

        frame_bgr = None
        try:
            # Capture the frame (main stream configured as RGB888)
            frame_bgr = self.picam2.capture_array("main")
            if frame_bgr is None:
                 logging.warning("capture_array returned None.")
                 # Don't set last_error here immediately, might be transient
                 return None

            # --- Update Output Frame for Streaming ---
            with self.frame_lock:
                self.output_frame = frame_bgr.copy() # Make a copy for the streaming thread

            # --- Write Frame if Recording ---
            # Check recording status outside the recording_lock initially for performance
            if self.is_recording:
                with self.recording_lock:
                    # Double-check state after acquiring lock
                    if not self.is_recording: return frame_bgr # Recording stopped between checks

                    if not self.video_writers:
                        # Inconsistent state check
                        logging.warning("Inconsistent state: is_recording=True, but no video writers. Forcing stop.")
                        self.last_error = "Rec stopped: writer list was empty."
                        # Need to release the lock before calling stop_recording
                        force_stop = True
                    else:
                        force_stop = False
                        write_errors = 0
                        for i, writer in enumerate(self.video_writers):
                            try:
                                # logging.debug(f"Writing frame to writer {i}") # Too verbose
                                writer.write(frame_bgr)
                            except Exception as e:
                                path_str = self.recording_paths[i] if i < len(self.recording_paths) else f"Writer {i}"
                                logging.error(f"!!! Failed to write frame to {path_str}: {e}")
                                write_errors += 1
                                self.last_error = f"Frame write error: {os.path.basename(path_str)}"
                                # Optional: Could remove the failing writer here, but complex

                        if write_errors > 0 and write_errors == len(self.video_writers):
                            # All writers failed
                            logging.error("All active video writers failed to write frame. Stopping recording.")
                            self.last_error = "Rec stopped: All writers failed write."
                            force_stop = True

                # Call stop_recording outside the lock if needed
                if force_stop:
                    self.stop_recording()

            return frame_bgr # Return the captured frame

        except Exception as e:
            logging.error(f"!!! Unexpected Error during frame capture/processing: {e}", exc_info=True)
            self.last_error = f"Capture Frame Error: {e}"
            # Ensure output_frame is cleared on critical error?
            with self.frame_lock:
                self.output_frame = None
            return None


    def get_latest_frame(self):
        """
        Returns the latest captured frame in a thread-safe manner.

        Returns:
            numpy.ndarray or None: A copy of the latest frame (BGR format) or None.
        """
        with self.frame_lock:
            if self.output_frame is not None:
                return self.output_frame.copy() # Return a copy
            else:
                return None

    def shutdown(self):
        """Stops recording, stops and closes the camera."""
        logging.info("CameraManager shutting down...")
        # Stop recording first (handles sync)
        if self.is_recording:
            self.stop_recording()

        # Stop and close camera
        if self.picam2:
            try:
                if self.picam2.started:
                    logging.info("Stopping Picamera2 instance...")
                    self.picam2.stop()
                logging.info("Closing Picamera2 instance...")
                self.picam2.close()
                logging.info("Picamera2 stopped and closed.")
            except Exception as e:
                logging.error(f"Error stopping/closing Picamera2 during shutdown: {e}")
            finally:
                self.picam2 = None
                self.is_initialized = False

        logging.info("CameraManager shutdown complete.")


# Example Usage (for testing purposes)
if __name__ == "__main__":
    logging.basicConfig(level=config.LOG_LEVEL, format=config.LOG_FORMAT, datefmt=config.LOG_DATE_FORMAT)
    logging.info("--- Camera Manager Test ---")

    cam_manager = CameraManager()

    try:
        # Initialize camera with default resolution
        if not cam_manager.initialize_camera():
            print(f"Camera initialization failed: {cam_manager.last_error}")
            exit()

        print("Camera initialized successfully.")
        print(f"Initial State: {cam_manager.get_camera_state()}")

        # --- Test Frame Capture ---
        print("\nCapturing a few frames...")
        for i in range(5):
            frame = cam_manager.capture_frame()
            if frame is not None:
                print(f"Frame {i+1} captured, shape: {frame.shape}")
                # Display frame (optional, requires GUI environment)
                # cv2.imshow("Test Frame", frame)
                # if cv2.waitKey(500) & 0xFF == ord('q'): break
            else:
                print(f"Frame {i+1} capture failed. Error: {cam_manager.last_error}")
            time.sleep(0.2)
        # cv2.destroyAllWindows()


        # --- Test Recording (requires USB drive) ---
        print("\nTesting recording (ensure a writable USB drive is mounted)...")
        if get_usb_mounts():
             if cam_manager.start_recording():
                 print("Recording started. Recording for 5 seconds...")
                 start_time = time.time()
                 while time.time() - start_time < 5:
                     frame = cam_manager.capture_frame() # Keep capturing
                     if frame is None:
                         print("Capture failed during recording test.")
                         break
                     time.sleep(0.05) # Simulate processing delay
                 print("Stopping recording...")
                 cam_manager.stop_recording()
                 print(f"Recording stopped. Check USB drive(s) for files in {config.USB_BASE_PATH}")
                 print(f"Final state: {cam_manager.get_camera_state()}")
             else:
                 print(f"Failed to start recording. Error: {cam_manager.last_error}")
        else:
             print("No writable USB drives found, skipping recording test.")


        # --- Test Control Change ---
        print("\nTesting control change (Brightness)...")
        initial_state = cam_manager.get_camera_state()
        print(f"Initial Brightness: {initial_state.get('brightness')}")
        new_brightness = initial_state.get('brightness', config.DEFAULT_BRIGHTNESS) + 0.2
        if new_brightness > config.MAX_BRIGHTNESS: new_brightness -= 0.4 # Adjust if already high

        if cam_manager.apply_camera_controls({'Brightness': new_brightness}):
             print(f"Brightness set to {new_brightness:.2f}. Capturing frame...")
             time.sleep(1) # Allow change to take effect
             frame = cam_manager.capture_frame()
             if frame is not None: print("Frame captured after brightness change.")
             final_state = cam_manager.get_camera_state()
             print(f"Final Brightness in state: {final_state.get('brightness')}")
        else:
             print(f"Failed to set brightness. Error: {cam_manager.last_error}")


        # --- Test Reconfiguration ---
        print("\nTesting resolution change...")
        current_index = cam_manager.get_camera_state()['resolution_index']
        next_index = (current_index + 1) % len(config.SUPPORTED_RESOLUTIONS)
        if next_index == current_index: next_index = (current_index - 1 + len(config.SUPPORTED_RESOLUTIONS)) % len(config.SUPPORTED_RESOLUTIONS) # Go down if only one res

        print(f"Changing resolution index from {current_index} to {next_index}...")
        if cam_manager.initialize_camera(next_index):
            print("Resolution change successful.")
            print(f"New State: {cam_manager.get_camera_state()}")
            print("Capturing frame at new resolution...")
            frame = cam_manager.capture_frame()
            if frame is not None: print(f"Frame captured, shape: {frame.shape}")
        else:
            print(f"Resolution change failed. Error: {cam_manager.last_error}")


    except KeyboardInterrupt:
        print("\nExiting test.")
    except Exception as e:
        print(f"\nAn error occurred during test: {e}")
        logging.exception("Test failed")
    finally:
        print("Shutting down camera manager...")
        cam_manager.shutdown()
        print("--- Test Complete ---")

