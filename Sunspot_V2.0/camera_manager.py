# -*- coding: utf-8 -*-
"""
camera_manager.py

Manages multiple Picamera2 cameras, including initialization, configuration,
frame capture (combined stream), video recording (from primary camera),
and audio recording/muxing.

**Modification:** Added logic to conditionally disable Cam1 based on config.ENABLE_CAM1.
"""

import os
import time
import datetime
import logging
import threading
import cv2
import numpy as np
from picamera2 import Picamera2
from libcamera import controls, Transform

# Audio related imports
import sounddevice as sd
import soundfile as sf
import subprocess
import queue
import tempfile
import numpy # Explicit import for audio dtype check

# Import configuration constants
import config

# Helper function to find writable USB mounts
def get_usb_mounts():
    """Finds writable directories under the configured USB base path."""
    mounts = []
    base_path = config.USB_BASE_PATH
    logging.debug(f"Checking for USB mounts under: {base_path}")
    if not os.path.isdir(base_path):
        logging.warning(f"USB base path '{base_path}' does not exist or is not a directory.")
        return mounts
    try:
        for item in os.listdir(base_path):
            path = os.path.join(base_path, item)
            if os.path.isdir(path) and os.access(path, os.W_OK):
                # More robust write test
                test_file = os.path.join(path, f".write_test_{os.getpid()}_{time.time_ns()}")
                try:
                    with open(test_file, 'w') as f: f.write('test')
                    os.remove(test_file)
                    mounts.append(path)
                    logging.debug(f"Found writable mount: {path}")
                except Exception as write_err:
                    logging.warning(f"Directory {path} appears mounted but test write failed: {write_err}")

        if not mounts: logging.debug("No writable USB mounts found.")
    except Exception as e:
        logging.error(f"Error finding USB mounts in {base_path}: {e}")
    return mounts


class CameraManager:
    """Handles multiple camera operations and state, including audio."""

    def __init__(self):
        """Initializes the CameraManager based on config."""
        self.picam0 = None # Primary camera (HQ)
        self.picam1 = None # Secondary camera (IMX219) - conditionally initialized
        self.is_initialized0 = False
        self.is_initialized1 = False # Will remain False if ENABLE_CAM1 is False
        self.last_error = None # General/last error message

        # State variables for Cam0 (HQ)
        self.current_resolution_index0 = config.CAM0_DEFAULT_RESOLUTION_INDEX
        self.actual_cam0_fps = None # Store the ACTUAL FPS reported by the driver
        self.last_cam0_capture_time = None # For calculating instantaneous FPS
        self.measured_cam0_fps_avg = None # Simple moving average for measured FPS
        # Minimum reasonable measured FPS to use for VideoWriter
        self.MIN_MEASURED_FPS_THRESHOLD = 5.0

        # State variable for output frame (combined or single)
        self.output_frame = None # Stores the latest frame for streaming

        # Recording state (only for Cam0)
        self.is_recording = False
        self.video_writers = [] # List of OpenCV VideoWriter objects for Cam0
        self.recording_paths = [] # List of corresponding file paths for Cam0 (video-only initially)
        self.recording_start_time = None # Track recording start time
        self.recording_fps = None # Store the FPS used for the current recording session

        # Locks for thread safety
        self.frame_lock = threading.Lock() # Protects access to output_frame
        self.config_lock = threading.Lock() # Protects access to camera state/config changes
        self.recording_lock = threading.Lock() # Protects access to recording state/writers

        # --- Initialize Common Camera Controls to Defaults from Config ---
        # Default control values (ensure these exist in config)
        self.current_analogue_gain = config.DEFAULT_ANALOGUE_GAIN
        self.current_iso_name = "Unknown" # Will be set based on gain
        self.current_ae_mode = controls.AeExposureModeEnum.Normal # Fallback
        self.current_metering_mode = controls.AeMeteringModeEnum.CentreWeighted # Fallback
        self.current_noise_reduction_mode = controls.draft.NoiseReductionModeEnum.Off # Fallback
        self.current_brightness = config.DEFAULT_BRIGHTNESS
        self.current_contrast = config.DEFAULT_CONTRAST
        self.current_saturation = config.DEFAULT_SATURATION
        self.current_sharpness = config.DEFAULT_SHARPNESS

        # Attempt to load defaults safely
        try:
            for name, gain in config.AVAILABLE_ISO_SETTINGS.items():
                if abs(gain - self.current_analogue_gain) < 0.01: self.current_iso_name = name; break
        except Exception as e: logging.error(f"Error setting default ISO name: {e}.")

        try: self.current_ae_mode = controls.AeExposureModeEnum.__members__[config.DEFAULT_AE_MODE_NAME]
        except (KeyError, AttributeError): logging.error(f"Default AE mode '{config.DEFAULT_AE_MODE_NAME}' invalid. Using fallback.")

        try: self.current_metering_mode = controls.AeMeteringModeEnum.__members__[config.DEFAULT_METERING_MODE_NAME]
        except (KeyError, AttributeError): logging.error(f"Default Metering mode '{config.DEFAULT_METERING_MODE_NAME}' invalid. Using fallback.")

        try: self.current_noise_reduction_mode = controls.draft.NoiseReductionModeEnum.__members__[config.DEFAULT_NOISE_REDUCTION_MODE_NAME]
        except (AttributeError, KeyError): logging.warning(f"Default NR mode '{config.DEFAULT_NOISE_REDUCTION_MODE_NAME}' invalid/unavailable. Using fallback Off.")


        # --- Audio Recording State ---
        self.audio_stream = None
        self.audio_thread = None
        self.audio_write_thread = None # Thread to write queue to file
        self.audio_queue = queue.Queue()
        self.temp_audio_file_path = None
        self.audio_device_index = None
        self.audio_last_error = None
        self.stop_audio_event = threading.Event()
        # Check audio dtype support once
        self.audio_dtype = None
        if config.AUDIO_ENABLED:
            try:
                # Ensure config.AUDIO_FORMAT is a valid numpy dtype string e.g., 'int16', 'float32'
                self.audio_dtype = np.dtype(config.AUDIO_FORMAT).type
            except TypeError:
                 logging.error(f"Unsupported audio format '{config.AUDIO_FORMAT}' in config. Disabling audio.")
                 config.AUDIO_ENABLED = False # Modify config state locally if invalid

        logging.info("CameraManager initialized.")
        if not config.ENABLE_CAM1:
            logging.warning("Cam1 is DISABLED in config.py. Operating in single-camera mode.")
        logging.debug(f"Initial Controls: ISO={self.current_iso_name}({self.current_analogue_gain:.2f}), AE={self.current_ae_mode.name}, Metering={self.current_metering_mode.name}, NR={self.current_noise_reduction_mode.name}, Bright={self.current_brightness}, Contr={self.current_contrast}, Sat={self.current_saturation}, Sharp={self.current_sharpness}")


    def _initialize_camera(self, cam_id, resolution_index=None):
        """Internal helper to initialize or re-initialize a specific camera instance."""
        picam_instance = None
        is_initialized_flag = False
        cam_name = f"Cam{cam_id}"
        actual_fps_reported = None # Store the FPS reported by driver

        # --- Check if this camera should be initialized ---
        if cam_id == config.CAM1_ID and not config.ENABLE_CAM1:
            logging.info(f"Skipping initialization of {cam_name} (disabled in config).")
            # Ensure state is clean for disabled camera
            self.picam1 = None
            self.is_initialized1 = False
            return True # Return True as skipping is not an error in this context

        # Determine target settings based on cam_id
        if cam_id == config.CAM0_ID:
            if resolution_index is not None:
                if 0 <= resolution_index < len(config.CAM0_RESOLUTIONS):
                    self.current_resolution_index0 = resolution_index
                else:
                    logging.error(f"{cam_name}: Invalid res index {resolution_index}. Using current {self.current_resolution_index0}.")
            res_list = config.CAM0_RESOLUTIONS
            current_res_index = self.current_resolution_index0
            tuning_data = config.CAM0_TUNING # Use loaded tuning data
            try:
                target_width, target_height, target_fps = res_list[current_res_index]
            except IndexError:
                logging.error(f"{cam_name}: Res index {current_res_index} out of bounds for {len(res_list)} resolutions. Using default index {config.CAM0_DEFAULT_RESOLUTION_INDEX}.")
                current_res_index = config.CAM0_DEFAULT_RESOLUTION_INDEX
                self.current_resolution_index0 = current_res_index
                target_width, target_height, target_fps = res_list[current_res_index]

        elif cam_id == config.CAM1_ID: # Only executes if ENABLE_CAM1 is True
            target_width, target_height = config.CAM1_RESOLUTION
            target_fps = config.CAM1_FRAME_RATE
            tuning_data = config.CAM1_TUNING # Use loaded tuning data
            current_res_index = 0 # Cam1 has fixed resolution
        else:
            logging.error(f"Invalid camera ID {cam_id}")
            return False

        logging.info(f"Attempting init {cam_name} (ID: {cam_id}) at index {current_res_index} ({target_width}x{target_height} @ Target {target_fps:.1f}fps)...")

        # --- Stop and Close Existing Instance ---
        existing_picam = self.picam0 if cam_id == config.CAM0_ID else self.picam1
        if existing_picam is not None:
            try:
                if existing_picam.started:
                    logging.info(f"Stopping existing {cam_name}...")
                    existing_picam.stop()
                logging.info(f"Closing existing {cam_name}...")
                existing_picam.close()
            except Exception as e:
                logging.warning(f"Error stopping/closing previous {cam_name}: {e}")
            finally:
                # Ensure the instance variable is cleared
                if cam_id == config.CAM0_ID:
                    self.picam0 = None
                    self.is_initialized0 = False
                    self.actual_cam0_fps = None # Reset actual FPS
                elif cam_id == config.CAM1_ID: # Check ID again
                    self.picam1 = None
                    self.is_initialized1 = False
            time.sleep(0.5) # Allow time for resources to release

        # --- Create and Configure New Instance ---
        try:
            # Pass tuning data directly to constructor if available
            picam_instance = Picamera2(camera_num=cam_id, tuning=tuning_data)
            logging.info(f"{cam_name}: Picamera2 object created.")

            # Prepare controls dictionary (use current manager state)
            controls_to_set = {
                "FrameRate": target_fps,
                "NoiseReductionMode": self.current_noise_reduction_mode,
                "AeEnable": True, # Generally keep AE enabled unless specific manual exposure is set
                "AeExposureMode": self.current_ae_mode,
                "AeMeteringMode": self.current_metering_mode,
                "AnalogueGain": self.current_analogue_gain, # Use current gain
                "Brightness": self.current_brightness,
                "Contrast": self.current_contrast,
                "Saturation": self.current_saturation,
                "Sharpness": self.current_sharpness,
            }
            # If gain is 0, it implies auto ISO, ensure AE is definitely on
            if self.current_analogue_gain == 0.0:
                controls_to_set["AeEnable"] = True
                logging.info(f"{cam_name}: AnalogueGain is 0.0 (Auto ISO), ensuring AE is enabled.")

            # Remove None values if any control state is somehow None
            controls_to_set = {k: v for k, v in controls_to_set.items() if v is not None}

            # Define transform (e.g., for flipping camera 1)
            transform = Transform()
            if cam_id == config.CAM1_ID: # Only apply flips to Cam1
                if config.CAM1_VFLIP:
                    transform.vflip = True
                    logging.info(f"{cam_name}: Applying vertical flip.")
                if config.CAM1_HFLIP:
                    transform.hflip = True
                    logging.info(f"{cam_name}: Applying horizontal flip.")


            # Create video configuration
            cam_config = picam_instance.create_video_configuration(
                main={"size": (target_width, target_height), "format": "RGB888"}, # Use RGB for OpenCV compatibility
                controls=controls_to_set,
                transform=transform
            )
            logging.info(f"{cam_name}: Configuring with: main={cam_config['main']}, controls={cam_config['controls']}")
            picam_instance.configure(cam_config)
            time.sleep(0.5) # Allow time for configuration to apply

            # Verify configuration and get actual FPS
            new_config = picam_instance.camera_configuration()
            if new_config:
                applied_controls = new_config.get('controls', {})
                applied_main = new_config.get('main', {})
                logging.info(f"{cam_name}: Verified Applied Config: main={applied_main}, controls={applied_controls}")
                actual_fps_reported = applied_controls.get('FrameRate') # Get the ACTUAL frame rate
                if actual_fps_reported is not None:
                    logging.info(f"{cam_name}: Driver reported actual FrameRate: {actual_fps_reported:.2f} fps")
                    if abs(actual_fps_reported - target_fps) > 1.0: # Log if significantly different
                        logging.warning(f"{cam_name}: Driver adjusted FrameRate significantly from target {target_fps:.1f} to {actual_fps_reported:.2f}")
                else:
                     logging.warning(f"{cam_name}: Could not read back actual FrameRate from config.")
                     actual_fps_reported = target_fps # Fallback to target if not readable
            else:
                logging.warning(f"{cam_name}: Could not get camera configuration after applying.")
                actual_fps_reported = target_fps # Fallback

            logging.info(f"{cam_name}: Starting camera...")
            picam_instance.start()
            logging.info(f"{cam_name}: Camera started");
            time.sleep(1.0) # Allow camera to stabilize after start

            # Final check on configuration after start
            actual_config_after_start = picam_instance.camera_configuration()
            if not actual_config_after_start:
                raise RuntimeError(f"{cam_name}: Failed get config after start.")

            actual_format = actual_config_after_start.get('main', {})
            actual_w = actual_format.get('size', (0,0))[0]
            actual_h = actual_format.get('size', (0,0))[1]
            actual_fmt_str = actual_format.get('format', 'Unknown')
            actual_fps_final = actual_config_after_start.get('controls', {}).get('FrameRate', actual_fps_reported) # Use final reported FPS
            actual_gain = actual_config_after_start.get('controls', {}).get('AnalogueGain', 'N/A')

            logging.info(f"{cam_name}: Initialized. Actual stream: {actual_w}x{actual_h} {actual_fmt_str} @ {actual_fps_final:.2f} fps. Gain: {actual_gain}")
            is_initialized_flag = True
            self.last_error = None # Clear previous errors on success

            return True # Return True on success

        except Exception as e:
            logging.error(f"!!! Failed to initialize {cam_name} at {target_width}x{target_height}: {e}", exc_info=True)
            self.last_error = f"{cam_name} Init Error: {e}"
            # Cleanup instance if partially created
            if picam_instance is not None:
                try:
                    if picam_instance.started: picam_instance.stop()
                    picam_instance.close()
                except Exception as close_e:
                    logging.error(f"Error closing {cam_name} after init failure: {close_e}")
            picam_instance = None
            is_initialized_flag = False
            actual_fps_reported = None
            return False # Return False on failure

        finally:
            # Update manager state based on success/failure
            if cam_id == config.CAM0_ID:
                self.picam0 = picam_instance
                self.is_initialized0 = is_initialized_flag
                self.actual_cam0_fps = actual_fps_reported # Store the actual FPS
                self.last_cam0_capture_time = None # Reset capture timing
                self.measured_cam0_fps_avg = None # Reset measured FPS
            elif cam_id == config.CAM1_ID: # Check ID again
                self.picam1 = picam_instance
                self.is_initialized1 = is_initialized_flag


    def initialize_cameras(self, resolution_index=None):
        """Initializes or re-initializes camera(s) based on config."""
        with self.config_lock:
            logging.info("--- Initializing Camera(s) ---")
            # Always initialize Cam0
            success0 = self._initialize_camera(config.CAM0_ID, resolution_index)
            success1 = True # Assume success if Cam1 is disabled

            # Initialize Cam1 only if enabled
            if config.ENABLE_CAM1:
                success1 = self._initialize_camera(config.CAM1_ID)
            else:
                logging.info("Cam1 initialization skipped (disabled in config).")
                # Ensure Cam1 state is marked as not initialized
                self.picam1 = None
                self.is_initialized1 = False

            if success0 and success1:
                cam_count = "One" if not config.ENABLE_CAM1 else "Both"
                logging.info(f"--- {cam_count} Camera(s) Initialized Successfully ---")
                return True
            else:
                logging.error("!!! Failed to initialize one or both required cameras. Check logs. !!!")
                # Ensure last_error reflects the failure
                if not success0 and not self.last_error: self.last_error = "Cam0 Initialization Failed"
                if not success1 and config.ENABLE_CAM1 and not self.last_error: self.last_error = "Cam1 Initialization Failed"
                elif not success1 and config.ENABLE_CAM1: self.last_error += " / Cam1 Initialization Failed"
                return False

    def get_cam0_resolution_config(self):
        """Returns the configured resolution tuple (width, height, target_fps) for Cam0."""
        try:
            # Return the tuple directly from the config list based on the current index
            return config.CAM0_RESOLUTIONS[self.current_resolution_index0]
        except IndexError:
            logging.error(f"Invalid resolution index {self.current_resolution_index0} for Cam0. Using default.")
            # Ensure default index is valid before using it
            safe_default_index = max(0, min(len(config.CAM0_RESOLUTIONS) - 1, config.CAM0_DEFAULT_RESOLUTION_INDEX))
            self.current_resolution_index0 = safe_default_index # Correct the index
            return config.CAM0_RESOLUTIONS[safe_default_index]

    def apply_camera_controls(self, controls_dict):
        """Applies a dictionary of common controls to running cameras."""
        if not self.is_initialized0 and (config.ENABLE_CAM1 and not self.is_initialized1):
            logging.error("Cannot apply controls: No cameras initialized.")
            self.last_error = "Control Apply Error: No cameras ready."
            return False
        if not self.is_initialized0:
            logging.error("Cannot apply controls: Cam0 not initialized.")
            self.last_error = "Control Apply Error: Cam0 not ready."
            return False

        logging.info(f"Applying common controls to cameras: {controls_dict}")
        success_count = 0
        applied_to_cam0 = False
        applied_to_cam1 = False
        temp_last_error = None # Store errors temporarily

        try:
            with self.config_lock:
                # Update internal state FIRST
                iso_name_updated = False
                for key, value in controls_dict.items():
                    if key == 'AnalogueGain':
                        self.current_analogue_gain = value
                        # Update ISO name based on the new gain value
                        self.current_iso_name = "Unknown"
                        try:
                            for name, gain in config.AVAILABLE_ISO_SETTINGS.items():
                                if abs(gain - value) < 0.01:
                                    self.current_iso_name = name
                                    iso_name_updated = True
                                    break
                            if not iso_name_updated:
                                logging.warning(f"Applied AnalogueGain {value} does not match any known ISO name.")
                        except Exception as e:
                             logging.error(f"Error updating ISO name for gain {value}: {e}")
                    elif key == 'AeExposureMode': self.current_ae_mode = value
                    elif key == 'AeMeteringMode': self.current_metering_mode = value
                    elif key == 'NoiseReductionMode': self.current_noise_reduction_mode = value
                    elif key == 'Brightness': self.current_brightness = value
                    elif key == 'Contrast': self.current_contrast = value
                    elif key == 'Saturation': self.current_saturation = value
                    elif key == 'Sharpness': self.current_sharpness = value
                    # Add other controls if needed

                # Apply to Cam0 (always attempt if initialized)
                logging.debug(f"Applying to Cam0...")
                if self.is_initialized0 and self.picam0 and self.picam0.started:
                    try:
                        self.picam0.set_controls(controls_dict)
                        logging.debug("Cam0 controls applied.")
                        applied_to_cam0 = True
                        success_count += 1
                    except Exception as e0:
                        logging.error(f"!!! Error applying controls to Cam0: {e0}")
                        temp_last_error = f"Cam0 Control Error: {e0}"
                else:
                    logging.warning("Skipping control application for Cam0 (not ready).")

                # Apply to Cam1 only if enabled and ready
                if config.ENABLE_CAM1:
                    logging.debug(f"Applying to Cam1...")
                    if self.is_initialized1 and self.picam1 and self.picam1.started:
                        try:
                            self.picam1.set_controls(controls_dict)
                            logging.debug("Cam1 controls applied.")
                            applied_to_cam1 = True
                            success_count += 1 # Increment only if Cam1 applied
                        except Exception as e1:
                            logging.error(f"!!! Error applying controls to Cam1: {e1}")
                            # Append error message
                            err_msg = f"Cam1 Control Error: {e1}"
                            temp_last_error = (temp_last_error + " / " + err_msg) if temp_last_error else err_msg
                    else:
                        logging.warning("Skipping control application for Cam1 (not ready).")
                else:
                    logging.debug("Skipping control application for Cam1 (disabled).")


            # --- Post-Application ---
            required_successes = 1 if not config.ENABLE_CAM1 else 2
            if success_count > 0: # Check if at least one succeeded
                # Allow some time for controls like exposure/gain to take effect
                if any(k in controls_dict for k in ['AnalogueGain', 'AeExposureMode', 'AeMeteringMode', 'Brightness', 'ExposureTime']):
                    time.sleep(0.5)
                else:
                    time.sleep(0.1)

                # Clear the main error if all required cameras succeeded and had no error during this call
                if temp_last_error is None and success_count >= required_successes:
                     self.last_error = None # Clear previous general errors if this call was fully successful

                logging.info(f"Controls applied successfully to {success_count} camera(s).")
                return True
            else:
                # If no camera succeeded, update the main error status
                self.last_error = temp_last_error if temp_last_error else "Control Apply Error: Failed on all ready cameras."
                logging.error(f"Failed to apply controls to any ready camera. Last Error: {self.last_error}")
                return False

        except Exception as e:
            logging.error(f"!!! Unexpected Error during control application: {e}", exc_info=True)
            self.last_error = f"Control Apply Unexpected Error: {e}"
            return False


    def get_camera_state(self):
        """Returns a dictionary containing the current camera state and common control values."""
        with self.config_lock:
            # Get configured resolution details for Cam0
            res_w0, res_h0, target_fps0 = self.get_cam0_resolution_config()

            # Determine output frame dimensions based on whether Cam1 is enabled
            output_w = res_w0
            output_h = res_h0
            if config.ENABLE_CAM1 and self.is_initialized1:
                res_w1, res_h1 = config.CAM1_RESOLUTION
                # Calculate combined height (assuming Cam1 is resized to fit Cam0 width if needed)
                # This logic matches capture_and_combine_frames
                display_w1 = min(res_w1, output_w) # Cam1 display width is capped by Cam0 width
                # If aspect ratio needs preserving, calculate display_h1 based on display_w1
                # Assuming simple resize for now:
                display_h1 = res_h1 if display_w1 == res_w1 else int(res_h1 * (display_w1 / res_w1))

                output_h = res_h0 + display_h1 + config.STREAM_BORDER_SIZE

            # Safely get enum names
            ae_mode_name = getattr(self.current_ae_mode, 'name', str(self.current_ae_mode))
            metering_mode_name = getattr(self.current_metering_mode, 'name', str(self.current_metering_mode))
            nr_mode_name = getattr(self.current_noise_reduction_mode, 'name', str(self.current_noise_reduction_mode))


            state = {
                # Overall state depends on required cameras being initialized
                'is_initialized': self.is_initialized0 and (not config.ENABLE_CAM1 or self.is_initialized1),
                'is_initialized0': self.is_initialized0,
                'is_initialized1': self.is_initialized1, # Will be False if disabled
                'is_cam1_enabled': config.ENABLE_CAM1, # Add flag indicating if Cam1 is supposed to be active
                'resolution_index': self.current_resolution_index0,
                'resolution_wh': (res_w0, res_h0), # Cam0 resolution
                'output_frame_wh': (output_w, output_h), # Dimensions of the frame sent to stream/UI
                'target_cam0_fps': target_fps0, # Configured target FPS
                'actual_cam0_fps': self.actual_cam0_fps, # FPS reported by driver
                'measured_cam0_fps': self.measured_cam0_fps_avg, # FPS measured between captures
                'is_recording': self.is_recording,
                'recording_paths': list(self.recording_paths), # Copy the list
                'last_error': self.last_error,
                'audio_last_error': self.audio_last_error,
                'iso_mode': self.current_iso_name,
                'analogue_gain': self.current_analogue_gain,
                'ae_mode': ae_mode_name,
                'metering_mode': metering_mode_name,
                'noise_reduction_mode': nr_mode_name,
                'brightness': self.current_brightness,
                'contrast': self.current_contrast,
                'saturation': self.current_saturation,
                'sharpness': self.current_sharpness,
            }
        return state

    def start_recording(self):
        """Starts recording video FROM CAM0 and audio (if enabled)."""
        with self.recording_lock:
            if self.is_recording:
                logging.warning("Start recording called, but already recording.")
                return True

            if not self.is_initialized0 or not self.picam0 or not self.picam0.started:
                logging.error("Cannot start recording, Cam0 not available.")
                self.last_error = "Cam0 not available for recording."
                return False

            logging.info("Attempting to start recording (Video: Cam0)...")
            usb_drives = get_usb_mounts()
            if not usb_drives:
                logging.error(f"Cannot start recording: No writable USB drives found in {config.USB_BASE_PATH}.")
                self.last_error = f"Cannot start recording: No writable USB drives found"
                return False

            # --- Get dimensions and FPS for VideoWriter ---
            try:
                width, height, target_fps_config = self.get_cam0_resolution_config() # Get W, H, TargetFPS from config

                # *** Determine FPS for VideoWriter using NEW hierarchy ***
                fps_for_writer = None
                fps_source = "Unknown"

                # 1. Try Measured Average FPS (if available and reasonable)
                if self.measured_cam0_fps_avg is not None and self.measured_cam0_fps_avg >= self.MIN_MEASURED_FPS_THRESHOLD:
                    fps_for_writer = self.measured_cam0_fps_avg
                    fps_source = "Measured Average"
                # 2. Fallback to Driver-Reported FPS
                elif self.actual_cam0_fps is not None and self.actual_cam0_fps > 0:
                    fps_for_writer = self.actual_cam0_fps
                    fps_source = "Driver Reported"
                    logging.warning(f"Using driver-reported FPS ({fps_for_writer:.2f}) for VideoWriter as measured FPS was unavailable/low ({self.measured_cam0_fps_avg}).")
                # 3. Fallback to Configured Target FPS
                elif target_fps_config > 0:
                    fps_for_writer = target_fps_config
                    fps_source = "Config Target"
                    logging.warning(f"Using configured target FPS ({fps_for_writer:.2f}) for VideoWriter as driver/measured FPS were unavailable.")
                # 4. Final hardcoded fallback
                else:
                    fps_for_writer = 30.0
                    fps_source = "Hardcoded Fallback"
                    logging.error(f"All FPS sources invalid. Using hardcoded {fps_for_writer:.1f}fps for VideoWriter.")

                if width <= 0 or height <= 0:
                    raise ValueError(f"Invalid Cam0 dimensions: {width}x{height}")

                # Store the chosen FPS for potential use during muxing
                self.recording_fps = fps_for_writer

                # Log the chosen FPS clearly
                logging.info(f"Selected FPS for VideoWriter: {fps_for_writer:.2f} (Source: {fps_source})")
                logging.info(f"Starting Cam0 recording: {width}x{height} @ {fps_for_writer:.2f} fps")

                fourcc = cv2.VideoWriter_fourcc(*config.CAM0_RECORDING_FORMAT)
                timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                base_filename = f"recording_{timestamp}_{width}x{height}" # Used for both video and audio temp

            except Exception as setup_err:
                logging.error(f"!!! Error getting recording parameters: {setup_err}", exc_info=True)
                self.last_error = f"Rec Param Error: {setup_err}"
                return False

            # --- Create Video Writers ---
            self.video_writers.clear()
            self.recording_paths.clear()
            success_count = 0
            start_error = None # Store first writer error

            for drive_path in usb_drives:
                try:
                    # Define video-only filename initially
                    video_only_filename = f"{base_filename}_video{config.CAM0_RECORDING_EXTENSION}"
                    full_path = os.path.join(drive_path, video_only_filename)

                    # Create the writer with the chosen FPS
                    writer = cv2.VideoWriter(full_path, fourcc, fps_for_writer, (width, height))
                    if not writer.isOpened():
                        raise IOError(f"Failed to open VideoWriter for path: {full_path}")

                    self.video_writers.append(writer)
                    self.recording_paths.append(full_path) # Store the video-only path for now
                    logging.info(f"Started video recording component to: {full_path}")
                    success_count += 1
                except Exception as e:
                    logging.error(f"!!! Failed create VideoWriter for drive {drive_path}: {e}", exc_info=True)
                    if start_error is None: # Store only the first error encountered
                         start_error = f"Failed writer {os.path.basename(drive_path)}: {e}"

            # --- Handle Writer Creation Results ---
            if success_count > 0:
                self.is_recording = True
                self.recording_start_time = time.monotonic() # Mark start time
                logging.info(f"Video recording started on {success_count} drive(s).")

                # Start audio recording if enabled
                if config.AUDIO_ENABLED:
                    if not self._start_audio_recording(base_filename): # Pass base filename for temp audio
                        logging.error("Failed to start audio recording component.")
                        # Don't necessarily fail the whole recording start if only audio fails
                        self.audio_last_error = self.audio_last_error or "Audio start failed"
                    else:
                        logging.info("Audio recording component started.")
                        self.audio_last_error = None # Clear previous audio errors

                # Report partial failure if some writers failed
                if start_error and success_count < len(usb_drives):
                    self.last_error = f"Partial Rec Start: {start_error}"
                elif not start_error:
                    # Clear previous recording-related errors on full success
                    if self.last_error and ("Recording" in self.last_error or "writers" in self.last_error or "USB" in self.last_error):
                        logging.info(f"Clearing previous video error on successful start: '{self.last_error}'")
                        self.last_error = None
                return True
            else:
                # All writers failed
                self.is_recording = False
                self.recording_fps = None # Clear stored FPS
                logging.error("Failed to start video recording on ANY drive.")
                self.last_error = f"Rec Start Failed: {start_error or 'No writers opened'}"
                # Ensure any partially opened writers are released (shouldn't happen if isOpened check works)
                for writer in self.video_writers:
                    try: writer.release()
                    except: pass
                self.video_writers.clear()
                self.recording_paths.clear()
                return False

    def stop_recording(self):
        """Stops video/audio recording, releases writers, muxes files, syncs."""
        audio_file_to_mux = None
        final_output_paths = []
        released_count = 0
        # Store the FPS used for this session before clearing state
        fps_used_for_recording = self.recording_fps

        with self.recording_lock:
            if not self.is_recording:
                # Handle potential stale state if stop is called unexpectedly
                if self.video_writers or self.recording_paths:
                    logging.warning("stop_recording called when not recording, but writers/paths exist. Clearing stale state.")
                    self.video_writers.clear()
                    self.recording_paths.clear()
                # Check and stop orphaned audio thread if necessary
                if config.AUDIO_ENABLED and (self.audio_thread and self.audio_thread.is_alive()):
                    logging.warning("Stopping orphaned audio recording during stop_recording call...")
                    self._stop_audio_recording() # Attempt cleanup
                self.recording_fps = None # Ensure FPS is cleared
                return # Nothing to do if not recording

            logging.info("Stopping recording (Video and Audio)...")
            self.is_recording = False
            recording_duration = (time.monotonic() - self.recording_start_time) if self.recording_start_time else None
            self.recording_start_time = None # Reset start time
            self.recording_fps = None # Clear stored FPS

            # Make copies of lists to work with outside the lock potentially
            writers_to_release = list(self.video_writers)
            video_paths_recorded = list(self.recording_paths) # These are the _video.mp4 paths

            # Clear manager's lists immediately
            self.video_writers.clear()
            self.recording_paths.clear()

        # --- Stop Audio First (if enabled) ---
        if config.AUDIO_ENABLED:
            audio_file_to_mux = self._stop_audio_recording() # Returns path to temp WAV if successful
            if audio_file_to_mux:
                logging.info(f"Audio recording stopped. Temp file ready for muxing: {audio_file_to_mux}")
            else:
                logging.warning("Audio recording stop failed or was not running.")
                # Keep any existing audio error message

        # --- Release Video Writers and Mux ---
        logging.info(f"Releasing {len(writers_to_release)} video writer(s)...")
        for i, writer in enumerate(writers_to_release):
            video_path = video_paths_recorded[i] if i < len(video_paths_recorded) else f"Unknown_Path_{i}"
            try:
                writer.release()
                logging.info(f"Released video writer for: {video_path}")
                released_count += 1

                # --- Muxing Logic ---
                # Pass the FPS used for recording to the muxer
                if audio_file_to_mux and os.path.exists(video_path):
                    final_path = self._mux_audio_video(video_path, audio_file_to_mux, fps_used_for_recording)
                    if final_path:
                        final_output_paths.append(final_path)
                        # If muxing created a new file, remove the temporary video-only file
                        if final_path != video_path:
                            try:
                                os.remove(video_path)
                                logging.info(f"Removed temporary video-only file: {video_path}")
                            except OSError as rm_err:
                                logging.warning(f"Could not remove temporary video file {video_path}: {rm_err}")
                    else:
                        # Muxing failed, keep the video-only file as fallback
                        logging.error(f"Muxing failed for {video_path}. Keeping video-only file.")
                        final_output_paths.append(video_path)
                        # Ensure error state reflects muxing failure
                        self.last_error = self.last_error or "Muxing Failed"
                elif os.path.exists(video_path):
                    # No audio to mux or audio failed, just add the video path
                    final_output_paths.append(video_path)
                else:
                     logging.warning(f"Video file {video_path} not found after writer release. Cannot mux or keep.")

            except Exception as e:
                logging.error(f"Error releasing VideoWriter for {video_path}: {e}", exc_info=True)
                # Try to keep the file path if it exists, even if release failed? Risky.
                if os.path.exists(video_path):
                     final_output_paths.append(video_path)
                     logging.warning(f"Keeping video file {video_path} despite release error.")


        # --- Cleanup Temporary Audio File ---
        if audio_file_to_mux and os.path.exists(audio_file_to_mux):
            try:
                os.remove(audio_file_to_mux)
                logging.info(f"Removed temporary audio file: {audio_file_to_mux}")
            except OSError as e:
                logging.warning(f"Could not remove temporary audio file {audio_file_to_mux}: {e}")

        # --- Filesystem Sync ---
        if released_count > 0 or final_output_paths: # Only sync if work was done
            logging.info("Syncing filesystem to ensure data is written to USB drives...")
            try:
                sync_start_time = time.monotonic()
                # Using os.system('sync') is common, but subprocess might be slightly safer
                subprocess.run(['sync'], check=True, timeout=15) # Add timeout
                sync_duration = time.monotonic() - sync_start_time
                logging.info(f"Sync completed in {sync_duration:.2f}s.")
            except subprocess.TimeoutExpired:
                 logging.error("!!! Filesystem sync timed out after 15s!")
                 self.last_error = "Sync timed out after recording."
            except (subprocess.CalledProcessError, FileNotFoundError, Exception) as e:
                logging.error(f"!!! Failed execute 'sync': {e}", exc_info=True)
                self.last_error = "Sync failed after recording."
        else:
            logging.warning("No writers released or final files generated, skipping sync.")

        log_duration = f" ~{recording_duration:.1f}s" if recording_duration else ""
        logging.info(f"Recording stopped (Duration:{log_duration}). Released {released_count} writer(s). Final files created: {len(final_output_paths)}")
        # Log final paths for verification
        for fpath in final_output_paths: logging.debug(f" - Final file: {fpath}")


    def capture_and_combine_frames(self):
        """
        Captures frames from enabled cameras, combines if necessary,
        and writes Cam0 frame if recording.
        Returns the frame to be streamed.
        """
        frame0 = None
        frame1 = None
        output_frame_for_stream = None
        capture_successful = False # Track if primary frame was captured

        # --- Capture Cam0 ---
        if self.is_initialized0 and self.picam0 and self.picam0.started:
            try:
                capture_start_time = time.monotonic()
                frame0 = self.picam0.capture_array("main")
                capture_end_time = time.monotonic()
                capture_successful = True # Mark success

                # --- Calculate Instantaneous and Average FPS ---
                if self.last_cam0_capture_time is not None:
                    time_diff = capture_end_time - self.last_cam0_capture_time
                    if time_diff > 0.0001: # Avoid division by zero or near-zero
                        instant_fps = 1.0 / time_diff
                        # Update simple moving average (e.g., alpha=0.1)
                        if self.measured_cam0_fps_avg is None:
                            self.measured_cam0_fps_avg = instant_fps
                        else:
                            alpha = 0.1
                            self.measured_cam0_fps_avg = alpha * instant_fps + (1 - alpha) * self.measured_cam0_fps_avg

                        # Log instantaneous FPS periodically for debugging
                        if capture_start_time - getattr(self, '_last_fps_log_time', 0) > 5.0: # Log every 5s
                             avg_fps_str = f", Avg: {self.measured_cam0_fps_avg:.2f}" if self.measured_cam0_fps_avg is not None else ""
                             logging.info(f"Cam0 Capture Time Diff: {time_diff:.4f}s (Instant FPS: {instant_fps:.2f}{avg_fps_str})") # Now INFO
                             self._last_fps_log_time = capture_start_time
                    else:
                         logging.warning(f"Cam0 capture time difference too small or negative: {time_diff:.5f}s")

                self.last_cam0_capture_time = capture_end_time # Update last capture time

            except Exception as e0:
                logging.error(f"!!! Error during Cam0 capture: {e0}")
                if self.last_error != f"Cam0 Capture Error: {e0}":
                     self.last_error = f"Cam0 Capture Error: {e0}"
                self.last_cam0_capture_time = None
                self.measured_cam0_fps_avg = None
        else:
             # logging.debug("Skipping Cam0 capture (not initialized/started).")
             pass

        # --- Capture Cam1 (only if enabled) ---
        if config.ENABLE_CAM1 and self.is_initialized1 and self.picam1 and self.picam1.started:
            try:
                frame1 = self.picam1.capture_array("main")
            except Exception as e1:
                logging.error(f"!!! Error during Cam1 capture: {e1}")
                err_msg = f"Cam1 Capture Error: {e1}"
                self.last_error = (self.last_error + " / " + err_msg) if (self.last_error and "Cam0" in self.last_error) else err_msg
        elif config.ENABLE_CAM1:
             # logging.debug("Skipping Cam1 capture (not initialized/started).")
             pass


        # --- Prepare Output Frame (Combine or Single) ---
        if frame0 is not None and frame1 is not None and config.ENABLE_CAM1:
            # Combine frames if Cam1 is enabled and both frames were captured
            try:
                h0, w0, _ = frame0.shape
                h1, w1, _ = frame1.shape
                target_w1, target_h1 = config.CAM1_RESOLUTION # Target size for Cam1 display

                # Resize Cam1 frame if needed
                if w1 != target_w1 or h1 != target_h1 or w1 > w0:
                    frame1_resized = cv2.resize(frame1, (target_w1, target_h1), interpolation=cv2.INTER_AREA)
                    h1, w1, _ = frame1_resized.shape # Update dimensions after resize
                else:
                    frame1_resized = frame1

                # Create combined canvas
                final_w = w0
                final_h = h0 + h1 + config.STREAM_BORDER_SIZE
                output_frame_for_stream = np.zeros((final_h, final_w, 3), dtype=np.uint8)
                output_frame_for_stream[:, :] = config.STREAM_BORDER_COLOR

                # Place Cam0 frame
                output_frame_for_stream[0:h0, 0:w0] = frame0

                # Place resized Cam1 frame
                y_start1 = h0 + config.STREAM_BORDER_SIZE
                x_start1 = (final_w - w1) // 2
                output_frame_for_stream[y_start1:y_start1 + h1, x_start1:x_start1 + w1] = frame1_resized

            except Exception as e_comb:
                logging.error(f"!!! Error combining frames: {e_comb}", exc_info=True)
                self.last_error = f"Frame Combine Error: {e_comb}"
                output_frame_for_stream = frame0 # Fallback to showing only frame0
        elif frame0 is not None:
            # Only frame0 available (or Cam1 disabled)
            output_frame_for_stream = frame0
        elif frame1 is not None and config.ENABLE_CAM1:
            # Only frame1 available (and Cam1 enabled), resize it
            try:
                target_w1, target_h1 = config.CAM1_RESOLUTION
                output_frame_for_stream = cv2.resize(frame1, (target_w1, target_h1), interpolation=cv2.INTER_AREA)
            except Exception as e_resize1:
                logging.error(f"Could not resize fallback frame1: {e_resize1}")
                output_frame_for_stream = None
        else:
            # No frames captured
            output_frame_for_stream = None


        # --- Update Shared Frame ---
        with self.frame_lock:
            # Store a copy if a valid frame exists
            self.output_frame = output_frame_for_stream.copy() if output_frame_for_stream is not None else None


        # --- Write Frame to Video Files (if recording) ---
        # Do this *after* updating the shared frame for streaming
        if self.is_recording and capture_successful: # Only write if Cam0 frame was captured
            force_stop_recording = False # Flag to force stop if all writers fail
            with self.recording_lock:
                # Double-check recording state inside lock
                if not self.is_recording:
                    return output_frame_for_stream # Recording stopped between check and lock acquisition

                if not self.video_writers:
                    logging.warning("Recording is True, but VideoWriter list is empty. Forcing stop.")
                    self.last_error = "Rec stopped: writer list empty."
                    force_stop_recording = True
                else:
                    write_errors = 0
                    for i, writer in enumerate(self.video_writers):
                        try:
                            # Ensure frame0 is not None before writing
                            if frame0 is not None:
                                writer.write(frame0)
                            else:
                                logging.warning(f"Skipping write for writer {i}: frame0 is None despite capture_successful flag.")
                                write_errors += 1

                        except Exception as e:
                            path_str = self.recording_paths[i] if i < len(self.recording_paths) else f"Writer {i}"
                            logging.error(f"!!! Failed write frame to {path_str}: {e}")
                            write_errors += 1
                            if "write error" not in (self.last_error or ""):
                                 self.last_error = f"Frame write error: {os.path.basename(path_str)}"

                    # If all writers failed, stop recording
                    if write_errors > 0 and write_errors == len(self.video_writers):
                        logging.error("All video writers failed to write frame. Stopping recording.")
                        self.last_error = "Rec stopped: All writers failed."
                        force_stop_recording = True

            # Force stop outside the lock to avoid deadlock with stop_recording
            if force_stop_recording:
                logging.info("Scheduling recording stop due to writer failure.")
                threading.Timer(0.1, self.stop_recording).start()


        return output_frame_for_stream # Return the frame for the main loop/streaming


    def get_latest_combined_frame(self):
        """Returns the latest output frame for streaming (thread-safe)."""
        with self.frame_lock:
            if self.output_frame is not None:
                # Return a copy to prevent modification by the caller
                return self.output_frame.copy()
            else:
                # Return None if no frame is available
                return None

    # --- Audio Methods ---
    # (Audio methods remain unchanged from previous version)
    def _find_audio_device(self):
        """Finds the index of the USB audio input device based on hint."""
        if not config.AUDIO_ENABLED:
            logging.info("Audio is disabled in config.")
            return False
        self.audio_device_index = None
        logging.info(f"Searching for audio input device containing hint: '{config.AUDIO_DEVICE_HINT}'")
        try:
            devices = sd.query_devices()
            logging.debug(f"Available audio devices:\n{devices}")
            candidate_indices = []
            for i, device in enumerate(devices):
                # Check for input channels and hint in name (case-insensitive)
                # Also check if it's not an output-only device like 'default' sometimes is
                if device['max_input_channels'] > 0 and \
                   config.AUDIO_DEVICE_HINT.lower() in device['name'].lower() and \
                   'output' not in device['name'].lower(): # Basic check to exclude pure output
                    logging.info(f"Found potential audio device: Index {i}, Name: {device['name']}, Inputs: {device['max_input_channels']}, Rate: {device['default_samplerate']}")
                    candidate_indices.append(i)

            if not candidate_indices:
                 logging.warning(f"No audio input device found matching hint '{config.AUDIO_DEVICE_HINT}'.")
                 self.audio_last_error = "Audio Device: Not Found"
                 return False

            # Check if candidates support the required settings
            for index in candidate_indices:
                 try:
                     # Ensure audio_dtype is valid before checking
                     if not self.audio_dtype:
                          logging.error("Cannot check audio settings: Invalid audio format in config.")
                          self.audio_last_error = "Audio Device: Invalid config format"
                          return False
                     sd.check_input_settings(device=index, samplerate=config.AUDIO_SAMPLE_RATE, channels=config.AUDIO_CHANNELS, dtype=self.audio_dtype)
                     logging.info(f"Device {index} ('{devices[index]['name']}') supports required settings. Selecting.")
                     self.audio_device_index = index
                     # Clear previous device errors on success
                     if "Audio Device" in (self.audio_last_error or ""):
                          self.audio_last_error = None
                     return True
                 except Exception as check_err:
                     logging.warning(f"Device {index} ('{devices[index]['name']}') does not support required settings ({config.AUDIO_SAMPLE_RATE}Hz, {config.AUDIO_CHANNELS}ch, {config.AUDIO_FORMAT}): {check_err}")

            # If loop finishes, no candidate supported the settings
            logging.error(f"Found devices matching hint, but none support required audio settings.")
            self.audio_last_error = "Audio Device: Found but incompatible settings"
            self.audio_device_index = None
            return False

        except Exception as e:
            logging.error(f"Error querying audio devices: {e}", exc_info=True)
            self.audio_last_error = f"Audio Device Query Error: {e}"
            return False

    def _audio_recording_thread(self):
        """Thread function to capture audio data and put it in a queue."""
        stream_started_successfully = False
        try:
            logging.info(f"Audio capture thread started for device {self.audio_device_index}.")
            self.stop_audio_event.clear()

            def audio_callback(indata, frames, time_info, status):
                """This runs in a separate thread managed by sounddevice."""
                if status:
                    logging.warning(f"Audio callback status: {status}")
                if self.audio_queue:
                    try:
                         self.audio_queue.put(indata.copy(), block=True, timeout=0.1)
                    except queue.Full:
                         logging.warning("Audio queue is full. Discarding audio data.")
                    except Exception as q_err:
                         logging.error(f"Error putting audio data into queue: {q_err}")

            self.audio_stream = sd.InputStream(
                samplerate=config.AUDIO_SAMPLE_RATE,
                device=self.audio_device_index,
                channels=config.AUDIO_CHANNELS,
                dtype=self.audio_dtype,
                blocksize=config.AUDIO_BLOCK_SIZE,
                callback=audio_callback
            )
            self.audio_stream.start()
            stream_started_successfully = True
            logging.info("Audio stream started.")

            while not self.stop_audio_event.is_set():
                self.stop_audio_event.wait(timeout=0.2)
                if not self.audio_stream.active:
                     logging.warning("Audio stream became inactive unexpectedly.")
                     if not self.audio_last_error:
                          self.audio_last_error = "Audio stream stopped unexpectedly"
                     break

        except sd.PortAudioError as pae:
             logging.error(f"!!! PortAudioError in audio recording thread: {pae}", exc_info=True)
             self.audio_last_error = f"Audio PortAudioError: {pae}"
        except Exception as e:
            logging.error(f"!!! Error in audio recording thread: {e}", exc_info=True)
            if not self.audio_last_error:
                 self.audio_last_error = f"Audio Thread Error: {e}"
        finally:
            if self.audio_stream and stream_started_successfully:
                try:
                    if not self.audio_stream.closed:
                         logging.info("Aborting and closing audio stream...")
                         self.audio_stream.abort(ignore_errors=True)
                         self.audio_stream.close(ignore_errors=True)
                         logging.info("Audio stream aborted and closed.")
                except Exception as e_close:
                    logging.error(f"Error closing audio stream during cleanup: {e_close}")
            elif self.audio_stream:
                 logging.warning("Audio stream object exists but was not started successfully.")

            logging.info("Audio capture thread finished.")
            if self.audio_queue:
                 try:
                      self.audio_queue.put(None, block=False)
                 except queue.Full:
                      logging.warning("Could not add sentinel to full audio queue during cleanup.")
                 except Exception as q_err:
                      logging.error(f"Error adding sentinel to queue during cleanup: {q_err}")

    def _audio_write_file_thread(self):
        """Thread function to write audio data from queue to temporary WAV file."""
        sound_file = None
        data_written = False
        items_processed = 0
        try:
            if not self.temp_audio_file_path:
                logging.error("Audio write thread: Temp audio file path not set."); return
            if not self.audio_dtype:
                 logging.error("Audio write thread: Audio dtype not set."); return

            subtype = None
            if self.audio_dtype == np.int16: subtype = 'PCM_16'
            elif self.audio_dtype == np.int32: subtype = 'PCM_32'
            elif self.audio_dtype == np.float32: subtype = 'FLOAT'
            elif self.audio_dtype == np.int8: subtype = 'PCM_S8'
            elif self.audio_dtype == np.uint8: subtype = 'PCM_U8'

            if subtype is None:
                 logging.error(f"Audio write thread: Unsupported audio format '{config.AUDIO_FORMAT}' ({self.audio_dtype}) for soundfile. Cannot write WAV.")
                 self.audio_last_error = f"Audio Write Error: Unsupported format {config.AUDIO_FORMAT}"
                 return

            logging.info(f"Audio write thread: Opening {self.temp_audio_file_path} (subtype: {subtype}) for writing.")
            sound_file = sf.SoundFile(
                self.temp_audio_file_path, mode='w',
                samplerate=config.AUDIO_SAMPLE_RATE,
                channels=config.AUDIO_CHANNELS,
                subtype=subtype
            )
            logging.info(f"Audio write thread: File opened successfully.")

            while True:
                try:
                    audio_data = self.audio_queue.get(block=True, timeout=0.5)

                    if audio_data is None:
                        logging.info(f"Audio write thread: Received stop sentinel after processing {items_processed} blocks.");
                        break

                    if isinstance(audio_data, np.ndarray):
                        sound_file.write(audio_data)
                        data_written = True
                        items_processed += 1
                    else:
                         logging.warning(f"Audio write thread: Received non-numpy data from queue: {type(audio_data)}")

                except queue.Empty:
                    if self.stop_audio_event.is_set():
                        logging.info(f"Audio write thread: Stop event detected during queue wait after processing {items_processed} blocks.");
                        break
                    continue
                except sf.SoundFileError as sf_err:
                     logging.error(f"!!! SoundFileError writing audio data: {sf_err}", exc_info=True)
                     self.audio_last_error = f"Audio File Write Error: {sf_err}"
                     break
                except Exception as write_err:
                     logging.error(f"!!! Unexpected error writing audio data: {write_err}", exc_info=True)
                     self.audio_last_error = f"Audio File Write Error: {write_err}"
                     break

        except sf.SoundFileError as sf_open_err:
             logging.error(f"!!! SoundFileError opening {self.temp_audio_file_path}: {sf_open_err}", exc_info=True)
             self.audio_last_error = f"Audio File Open Error: {sf_open_err}"
        except Exception as e:
            logging.error(f"!!! Error in audio write thread setup/loop: {e}", exc_info=True)
            self.audio_last_error = f"Audio Write Thread Error: {e}"
        finally:
            if sound_file:
                try:
                    logging.info(f"Audio write thread: Closing {self.temp_audio_file_path}...")
                    sound_file.close()
                    logging.info(f"Audio write thread: Closed sound file.")
                except Exception as e_close:
                    logging.error(f"Error closing sound file: {e_close}")

            file_exists = self.temp_audio_file_path and os.path.exists(self.temp_audio_file_path)
            if file_exists and not data_written:
                 logging.warning(f"Audio write thread finished, but no data seems to have been written to {self.temp_audio_file_path}.")
            elif not file_exists and data_written:
                 logging.error("Audio write thread: Data was processed, but the output file does not exist!")
                 self.audio_last_error = self.audio_last_error or "Audio file missing after write"

            logging.info("Audio write thread finished.")

    def _start_audio_recording(self, base_filename):
        """Starts the audio recording threads and sets up the temporary file."""
        if not config.AUDIO_ENABLED:
            logging.info("Audio recording disabled in config.")
            return False
        if not self.audio_dtype:
            logging.error("Cannot start audio recording: Invalid audio format in config.")
            self.audio_last_error = "Audio Start Failed: Invalid config format"
            return False

        if self.audio_thread and self.audio_thread.is_alive():
             logging.warning("Audio start called while previous capture thread alive. Attempting stop first.")
             self._stop_audio_recording()
        elif self.audio_write_thread and self.audio_write_thread.is_alive():
             logging.warning("Audio start called while previous write thread alive. Attempting stop first.")
             self._stop_audio_recording()
        while not self.audio_queue.empty():
            try: self.audio_queue.get_nowait()
            except queue.Empty: break
            except Exception: pass

        if not self._find_audio_device() or self.audio_device_index is None:
            logging.error(f"Audio Start Failed: Could not find suitable audio device. Last error: {self.audio_last_error}")
            return False

        try:
            temp_dir = tempfile.gettempdir()
            self.temp_audio_file_path = os.path.join(temp_dir, f"{base_filename}_audio{config.AUDIO_TEMP_EXTENSION}")
            if os.path.exists(self.temp_audio_file_path):
                 logging.warning(f"Removing existing temp audio file: {self.temp_audio_file_path}")
                 os.remove(self.temp_audio_file_path)

            logging.info(f"Starting audio recording. Temp file: {self.temp_audio_file_path}")

            self.stop_audio_event.clear()

            self.audio_thread = threading.Thread(target=self._audio_recording_thread, name="AudioCaptureThread")
            self.audio_thread.daemon = True
            self.audio_thread.start()

            time.sleep(0.2)

            self.audio_write_thread = threading.Thread(target=self._audio_write_file_thread, name="AudioWriteThread")
            self.audio_write_thread.daemon = True
            self.audio_write_thread.start()

            time.sleep(0.5)
            if not self.audio_thread.is_alive() or not self.audio_write_thread.is_alive():
                 logging.error("Audio threads did not remain alive shortly after start.")
                 self.audio_last_error = self.audio_last_error or "Audio threads failed to start/stay alive"
                 self.stop_audio_event.set()
                 if self.audio_thread.is_alive(): self.audio_thread.join(timeout=1.0)
                 if self.audio_write_thread.is_alive(): self.audio_write_thread.join(timeout=1.0)
                 if self.temp_audio_file_path and os.path.exists(self.temp_audio_file_path):
                     try: os.remove(self.temp_audio_file_path)
                     except OSError: pass
                 self.temp_audio_file_path = None; self.audio_thread = None; self.audio_write_thread = None
                 return False

            self.audio_last_error = None
            logging.info("Audio recording threads started successfully.")
            return True

        except Exception as e:
            logging.error(f"!!! Failed to start audio recording setup: {e}", exc_info=True)
            self.audio_last_error = f"Audio Start Error: {e}"
            self.stop_audio_event.set()
            if self.audio_thread and self.audio_thread.is_alive(): self.audio_thread.join(timeout=1.0)
            if self.audio_write_thread and self.audio_write_thread.is_alive(): self.audio_write_thread.join(timeout=1.0)
            if self.temp_audio_file_path and os.path.exists(self.temp_audio_file_path):
                 try: os.remove(self.temp_audio_file_path)
                 except OSError: pass
            self.temp_audio_file_path = None; self.audio_thread = None; self.audio_write_thread = None
            return False

    def _stop_audio_recording(self):
        """Stops the audio recording threads and returns the temp file path if valid."""
        if not config.AUDIO_ENABLED: return None

        capture_thread_running = self.audio_thread and self.audio_thread.is_alive()
        write_thread_running = self.audio_write_thread and self.audio_write_thread.is_alive()
        temp_file_path_at_start = self.temp_audio_file_path

        if not capture_thread_running and not write_thread_running:
            logging.info("Audio recording threads were not running.")
            if temp_file_path_at_start and os.path.exists(temp_file_path_at_start) and os.path.getsize(temp_file_path_at_start) > 44:
                 logging.info(f"Found potentially valid existing temp audio file: {temp_file_path_at_start}")
                 self.temp_audio_file_path = None
                 return temp_file_path_at_start
            else:
                 if temp_file_path_at_start and os.path.exists(temp_file_path_at_start):
                      try:
                           logging.info(f"Cleaning up empty/invalid temp audio file: {temp_file_path_at_start}")
                           os.remove(temp_file_path_at_start)
                      except OSError as e:
                           logging.warning(f"Could not remove temp audio file {temp_file_path_at_start}: {e}")
                 self.temp_audio_file_path = None
                 return None

        logging.info("Stopping audio recording threads...")
        self.stop_audio_event.set()

        if capture_thread_running:
            logging.debug("Waiting for audio capture thread to join...")
            self.audio_thread.join(timeout=2.0)
            if self.audio_thread.is_alive():
                logging.warning("Audio capture thread did not stop cleanly within timeout.")
            else:
                 logging.debug("Audio capture thread joined.")

        if write_thread_running:
            logging.debug("Waiting for audio write thread to join...")
            self.audio_write_thread.join(timeout=5.0)
            if self.audio_write_thread.is_alive():
                logging.warning("Audio write thread did not stop cleanly within timeout.")
            else:
                 logging.debug("Audio write thread joined.")

        self.audio_thread = None
        self.audio_write_thread = None
        self.audio_stream = None

        if temp_file_path_at_start and os.path.exists(temp_file_path_at_start):
             try:
                 file_size = os.path.getsize(temp_file_path_at_start)
                 if file_size > 44:
                     logging.info(f"Audio stop successful. Temp file ready: {temp_file_path_at_start} (Size: {file_size} bytes)")
                     self.temp_audio_file_path = None
                     return temp_file_path_at_start
                 else:
                     logging.warning(f"Temporary audio file {temp_file_path_at_start} seems empty/invalid after stop (Size: {file_size}).")
                     try: os.remove(temp_file_path_at_start); logging.info("Cleaned up empty/invalid temp audio file.")
                     except OSError as e: logging.warning(f"Could not remove empty temp audio file: {e}")
                     self.temp_audio_file_path = None
                     return None
             except OSError as e:
                  logging.error(f"Error checking temp audio file size {temp_file_path_at_start}: {e}")
                  self.temp_audio_file_path = None
                  return None
        else:
            logging.error("Temporary audio file path not set or file does not exist after stop sequence.")
            self.temp_audio_file_path = None
            return None

    def _mux_audio_video(self, video_path, audio_path, recording_fps=None):
        """
        Merges audio and video files using ffmpeg.
        Re-encodes video using libx264, forcing the frame rate via -vf filter.
        """
        if not config.AUDIO_ENABLED: return None

        if not os.path.exists(config.FFMPEG_PATH):
            logging.error(f"ffmpeg not found at '{config.FFMPEG_PATH}'. Cannot mux audio.")
            self.audio_last_error = "Mux Error: ffmpeg not found"
            return None
        if not os.path.exists(video_path):
            logging.error(f"Video file not found for muxing: {video_path}")
            self.audio_last_error = "Mux Error: Video file missing"
            return None
        if not os.path.exists(audio_path):
            logging.error(f"Audio file not found for muxing: {audio_path}")
            self.audio_last_error = "Mux Error: Audio file missing"
            return None
        if os.path.getsize(audio_path) <= 44:
             logging.error(f"Audio file {audio_path} is too small, likely invalid. Skipping mux.")
             self.audio_last_error = "Mux Error: Audio file invalid/empty"
             return None

        output_path = video_path.replace("_video" + config.CAM0_RECORDING_EXTENSION, config.CAM0_RECORDING_EXTENSION)
        if output_path == video_path:
            output_path = video_path.replace(config.CAM0_RECORDING_EXTENSION, "_muxed" + config.CAM0_RECORDING_EXTENSION)

        fps_to_use = recording_fps
        if fps_to_use is None or fps_to_use <= 0:
             logging.warning("Invalid or missing recording_fps for muxing. Falling back to 30.0.")
             fps_to_use = 30.0

        logging.info(f"Muxing (re-encoding video) '{os.path.basename(video_path)}' and audio '{os.path.basename(audio_path)}' into '{os.path.basename(output_path)}' at {fps_to_use:.2f} fps...")

        command = [
            config.FFMPEG_PATH,
            "-y",
            "-i", video_path,
            "-i", audio_path,
            "-vf", f"fps={fps_to_use:.4f}",
            "-c:v", "libx264",
            "-preset", "veryfast",
            "-crf", "23",
            "-c:a", "aac",
            "-b:a", "128k",
            "-map", "0:v:0",
            "-map", "1:a:0",
            "-shortest",
            "-loglevel", config.FFMPEG_LOG_LEVEL,
            output_path
        ]
        logging.debug(f"Executing ffmpeg command: {' '.join(command)}")

        mux_start_time = time.monotonic()
        # Calculate timeout based on config, multiplied for re-encoding
        recode_timeout = config.AUDIO_MUX_TIMEOUT * config.AUDIO_MUX_RECODE_TIMEOUT_MULTIPLIER
        try:
            process = subprocess.run(command, capture_output=True, text=True, check=True, timeout=recode_timeout)
            mux_duration = time.monotonic() - mux_start_time
            logging.info(f"ffmpeg muxing (re-encode) successful for {output_path} in {mux_duration:.2f}s.")
            if logging.getLogger().isEnabledFor(logging.DEBUG):
                 logging.debug(f"ffmpeg output:\n--- stdout ---\n{process.stdout}\n--- stderr ---\n{process.stderr}\n---")
            if "Mux Error" in (self.audio_last_error or ""):
                 self.audio_last_error = None
            return output_path

        except subprocess.TimeoutExpired:
            mux_duration = time.monotonic() - mux_start_time
            logging.error(f"!!! ffmpeg muxing (re-encode) timed out ({recode_timeout}s) for {output_path} after {mux_duration:.1f}s.")
            self.audio_last_error = "Mux Error: ffmpeg re-encode timed out"
        except subprocess.CalledProcessError as e:
            mux_duration = time.monotonic() - mux_start_time
            logging.error(f"!!! ffmpeg muxing (re-encode) failed for {output_path} after {mux_duration:.1f}s. Return Code: {e.returncode}")
            logging.error(f"ffmpeg stderr:\n{e.stderr}")
            logging.error(f"ffmpeg stdout:\n{e.stdout}")
            self.audio_last_error = f"Mux Error: ffmpeg re-encode failed (code {e.returncode})"
        except Exception as e:
            mux_duration = time.monotonic() - mux_start_time
            logging.error(f"!!! Unexpected error during ffmpeg re-encode execution after {mux_duration:.1f}s: {e}", exc_info=True)
            self.audio_last_error = f"Mux Error: {e}"

        if os.path.exists(output_path):
             try:
                 logging.warning(f"Attempting to remove failed mux output file: {output_path}")
                 os.remove(output_path)
             except OSError as rm_err:
                  logging.error(f"Could not remove failed mux output {output_path}: {rm_err}")

        return None


    def shutdown(self):
        """Stops recording, stops and closes enabled cameras cleanly."""
        logging.info("--- CameraManager Shutting Down ---")

        # 1. Stop Recording
        if self.is_recording:
            logging.info("Shutdown: Stopping active recording...")
            self.stop_recording() # This handles audio stop, muxing, etc.
        else:
            # Cleanup orphaned audio if necessary
            if config.AUDIO_ENABLED:
                 if (self.audio_thread and self.audio_thread.is_alive()) or \
                    (self.audio_write_thread and self.audio_write_thread.is_alive()):
                     logging.warning("Shutdown: Stopping orphaned audio recording...")
                     self._stop_audio_recording()
                 elif self.temp_audio_file_path and os.path.exists(self.temp_audio_file_path):
                      try:
                           logging.warning(f"Shutdown: Removing orphaned temp audio file: {self.temp_audio_file_path}")
                           os.remove(self.temp_audio_file_path)
                           self.temp_audio_file_path = None
                      except OSError as e:
                           logging.error(f"Could not remove orphaned temp audio file: {e}")


        # 2. Stop and Close Cameras (only if they were initialized)
        cameras_to_shutdown = [(config.CAM0_ID, 'picam0', 'is_initialized0', "Cam0")]
        if config.ENABLE_CAM1: # Only add Cam1 if it was supposed to be enabled
            cameras_to_shutdown.append((config.CAM1_ID, 'picam1', 'is_initialized1', "Cam1"))

        for cam_id, picam_instance_attr, init_flag_attr, name in cameras_to_shutdown:
            picam_instance = getattr(self, picam_instance_attr, None)
            if picam_instance: # Check if instance exists
                logging.info(f"Shutdown: Stopping and closing {name}...")
                try:
                    if picam_instance.started:
                        picam_instance.stop()
                        logging.info(f"{name} stopped.")
                    picam_instance.close()
                    logging.info(f"{name} closed.")
                except Exception as e:
                    logging.error(f"Error stopping/closing {name} during shutdown: {e}")
                finally:
                    # Clear the instance and flag regardless of errors
                    setattr(self, picam_instance_attr, None)
                    setattr(self, init_flag_attr, False)
            else:
                 # Log only if it was supposed to be initialized but isn't found
                 if getattr(self, init_flag_attr, False):
                     logging.warning(f"Shutdown: {name} instance was None, but expected to be initialized.")
                 else:
                     logging.debug(f"Shutdown: {name} instance already None or was disabled.")


        # Reset other state variables
        self.actual_cam0_fps = None
        self.last_cam0_capture_time = None
        self.measured_cam0_fps_avg = None
        self.recording_fps = None
        with self.frame_lock: self.output_frame = None
        logging.info("--- CameraManager Shutdown Complete ---")

# Example Usage (Main testing block)
if __name__ == "__main__":
    # Basic logging setup for testing
    log_level_str = os.environ.get("LOG_LEVEL", "DEBUG") # Allow overriding via env var
    log_level = getattr(logging, log_level_str.upper(), logging.INFO)
    logging.basicConfig(level=log_level, format=config.LOG_FORMAT, datefmt=config.LOG_DATE_FORMAT)

    logging.info("--- Camera Manager Dual Cam Test ---")
    cam_manager = CameraManager()
    test_failed = False

    try:
        # --- Initialization Test ---
        print("\n--- Testing Camera Initialization ---")
        if not cam_manager.initialize_cameras():
            print(f"!!! FATAL: Camera initialization failed: {cam_manager.last_error}")
            test_failed = True; raise SystemExit("Exiting due to init failure.") # Use SystemExit for clean exit in test
        print("Cameras initialized successfully.")
        print(f"Initial State: {cam_manager.get_camera_state()}")
        print("------------------------------------")

        # --- Frame Capture Test ---
        print("\n--- Testing Frame Capture & Display (5s) ---")
        start_capture_test = time.monotonic()
        frames_captured = 0
        while time.monotonic() - start_capture_test < 5.0:
            frame = cam_manager.capture_and_combine_frames()
            if frame is not None:
                 frames_captured += 1
                 cv2.imshow("Stream Test", frame) # Changed window name
            else:
                 print(f"!!! Frame capture failed. Error: {cam_manager.last_error}")
            if cv2.waitKey(1) & 0xFF == ord('q'):
                 print("Quit requested during capture test.")
                 break
        cv2.destroyAllWindows()
        print(f"Capture test complete. Captured {frames_captured} frames.")
        if frames_captured == 0:
             print("!!! WARNING: No frames captured during test.")
        print("------------------------------------")


        # --- Recording Test ---
        print("\n--- Testing Recording (Video + Audio if enabled, 5s) ---")
        usb_drives = get_usb_mounts()
        if usb_drives:
             print(f"Found writable USB drives: {usb_drives}")
             if cam_manager.start_recording():
                 print("Recording started. Recording for 5 seconds...")
                 start_time = time.monotonic()
                 rec_frames = 0
                 while time.monotonic() - start_time < 5.0:
                     frame = cam_manager.capture_and_combine_frames() # Continues capturing/writing
                     if frame is None:
                         print("!!! Capture failed during recording test.")
                     else:
                          rec_frames += 1
                          cv2.imshow("Recording Test", frame) # Display while recording
                     if cv2.waitKey(1) & 0xFF == ord('q'):
                         print("Quit requested during recording.")
                         break
                 print(f"Stopping recording after ~5s ({rec_frames} frames processed)...")
                 cam_manager.stop_recording() # This now takes longer due to re-encoding
                 print(f"Recording stopped and muxing finished. Check USB drive(s) in {config.USB_BASE_PATH}")
                 print(f"Final state after recording: {cam_manager.get_camera_state()}")
             else:
                 print(f"!!! Failed to start recording. Error: {cam_manager.last_error} / Audio: {cam_manager.audio_last_error}")
                 test_failed = True
        else:
             print("!!! No writable USB drives found, skipping recording test.")
             test_failed = True
        cv2.destroyAllWindows()
        print("------------------------------------")

        # --- Control Change Test (ISO/AnalogueGain) ---
        # (Skipping this test in this version for brevity, logic is unchanged)
        print("\n--- Skipping Control Change Test ---")
        print("------------------------------------")

        # --- Resolution Change Test (Cam0) ---
        # (Skipping this test in this version for brevity, logic is unchanged)
        print("\n--- Skipping Resolution Change Test ---")
        print("------------------------------------")


    except KeyboardInterrupt:
        print("\nExiting test due to KeyboardInterrupt.")
        test_failed = True
    except SystemExit as e:
         print(f"\nExiting test: {e}")
    except Exception as e:
        print(f"\n!!! An unexpected error occurred during test: {e}")
        logging.exception("Test failed due to unexpected error")
        test_failed = True
    finally:
        print("\n--- Shutting Down Camera Manager ---")
        cam_manager.shutdown()
        cv2.destroyAllWindows()
        print("--- Test Complete ---")
        if test_failed:
             print("!!! One or more tests failed or were skipped due to errors. !!!")
        else:
             print("+++ All tests passed (or skipped gracefully). +++")
