# -*- coding: utf-8 -*-
"""
camera_manager.py

Manages multiple Picamera2 cameras, including initialization, configuration,
frame capture (combined stream), and video recording (from primary camera).

**Modification 9 (User Request):** Removed all audio recording and processing components.
**Modification 10 (User Request):** Ensured camera manager uses designated FPS/resolution from config
                    for recording (VideoWriter initialization) and capture (camera controls).
**Modification 11 (User Request):** Modified frame capture to make the raw frame from Cam0
                    available for external CV processing via a new getter method.
**Modification 12 (User Request):** Replaced cv2.VideoWriter recording with Picamera2's
                    native start_recording API to fix sped-up video issue.
"""

import os
import time
import datetime
import logging
import threading
import numpy as np # Used for frame manipulation
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder # Use Picamera2's H.264 encoder
# from picamera2.outputs import FileOutput # Might need if start_recording isn't sufficient
import shutil # For copying recorded files
from libcamera import controls, Transform
import subprocess # Only needed for 'sync' command now

# Import configuration constants
import config

# Helper function to find writable USB mounts (unchanged)
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
    """Handles multiple camera operations and state (video-only, Picamera2 recording)."""

    def __init__(self):
        """Initializes the CameraManager based on config (Picamera2 recording)."""
        self.picam0 = None # Primary camera (HQ)
        self.picam1 = None # Secondary camera (IMX219) - conditionally initialized
        self.picam0_encoder = None # Stores the Picamera2 encoder instance
        self.is_initialized0 = False
        self.is_initialized1 = False
        self.last_error = None

        # State variables for Cam0 (HQ)
        self.current_resolution_index0 = config.CAM0_DEFAULT_RESOLUTION_INDEX
        self.actual_cam0_fps = None
        self.last_cam0_capture_time = None
        self.measured_cam0_fps_avg = None

        # State variable for output frame (combined or single) for streaming
        self.output_frame = None
        # State variable for raw frame output for CV processing
        self.latest_raw_frame0 = None

        # --- Recording State (Using Picamera2 API) ---
        self.is_recording = False
        self.recording_target_fps = None # Store the TARGET FPS used for the current recording (for info)
        self.primary_recording_path = None # Path where Picamera2 is actively recording
        self.target_recording_paths = [] # List of all final destination paths (across USBs)
        self.recording_start_time_monotonic = None

        # REMOVED: recording_frame_queue, recording_thread, stop_recording_event, video_writers

        # Locks for thread safety
        self.frame_lock = threading.Lock() # Protects access to output_frame (for streaming)
        self.raw_frame_lock = threading.Lock() # Protects access to latest_raw_frame0 (for CV)
        self.config_lock = threading.Lock() # Protects access to camera state/config changes
        self.recording_lock = threading.Lock() # Protects access to recording state variables

        # --- Initialize Common Camera Controls to Defaults from Config ---
        # (Control initialization unchanged)
        self.current_analogue_gain = config.DEFAULT_ANALOGUE_GAIN
        self.current_iso_name = "Unknown"
        # ... (rest of control initializations unchanged) ...
        self.current_ae_mode = controls.AeExposureModeEnum.Normal
        self.current_metering_mode = controls.AeMeteringModeEnum.CentreWeighted
        self.current_noise_reduction_mode = controls.draft.NoiseReductionModeEnum.Off
        self.current_brightness = config.DEFAULT_BRIGHTNESS
        self.current_contrast = config.DEFAULT_CONTRAST
        self.current_saturation = config.DEFAULT_SATURATION
        self.current_sharpness = config.DEFAULT_SHARPNESS
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


        logging.info("CameraManager initialized (Using Picamera2 Recording API, Audio Disabled).")
        if not config.ENABLE_CAM1:
            logging.warning("Cam1 is DISABLED in config.py. Operating in single-camera mode.")
        logging.debug(f"Initial Controls: ISO={self.current_iso_name}({self.current_analogue_gain:.2f}), AE={self.current_ae_mode.name}, Metering={self.current_metering_mode.name}, NR={self.current_noise_reduction_mode.name}, Bright={self.current_brightness}, Contr={self.current_contrast}, Sat={self.current_saturation}, Sharp={self.current_sharpness}")


    def _initialize_camera(self, cam_id, resolution_index=None):
        """Internal helper to initialize or re-initialize a specific camera instance.
           Stops active Picamera2 recording if needed."""
        picam_instance = None
        is_initialized_flag = False
        cam_name = f"Cam{cam_id}"
        actual_fps_reported = None

        # --- Check if this camera should be initialized ---
        if cam_id == config.CAM1_ID and not config.ENABLE_CAM1:
             # ... (unchanged skip logic) ...
            logging.info(f"Skipping initialization of {cam_name} (disabled in config).")
            self.picam1 = None
            self.is_initialized1 = False
            return True


        # Determine target settings based on cam_id
        if cam_id == config.CAM0_ID:
            # ... (logic to get target_width, target_height, target_fps from config unchanged) ...
            if resolution_index is not None:
                if 0 <= resolution_index < len(config.CAM0_RESOLUTIONS):
                    self.current_resolution_index0 = resolution_index
                else:
                    logging.error(f"{cam_name}: Invalid res index {resolution_index}. Using current {self.current_resolution_index0}.")
            res_list = config.CAM0_RESOLUTIONS
            current_res_index = self.current_resolution_index0
            tuning_data = config.CAM0_TUNING
            try:
                target_width, target_height, target_fps = res_list[current_res_index]
            except IndexError:
                logging.error(f"{cam_name}: Res index {current_res_index} out of bounds for {len(res_list)} resolutions. Using default index {config.CAM0_DEFAULT_RESOLUTION_INDEX}.")
                current_res_index = config.CAM0_DEFAULT_RESOLUTION_INDEX
                self.current_resolution_index0 = current_res_index
                target_width, target_height, target_fps = res_list[current_res_index]

        elif cam_id == config.CAM1_ID:
             # ... (logic for Cam1 unchanged) ...
            target_width, target_height = config.CAM1_RESOLUTION
            target_fps = config.CAM1_FRAME_RATE # Use Cam1 specific FPS
            tuning_data = config.CAM1_TUNING
            current_res_index = 0 # Cam1 has fixed resolution
        else:
            logging.error(f"Invalid camera ID {cam_id}")
            return False

        logging.info(f"Attempting init {cam_name} (ID: {cam_id}) at index {current_res_index} ({target_width}x{target_height} @ Target {target_fps:.1f}fps)...")

        # --- Stop and Close Existing Instance ---
        existing_picam = self.picam0 if cam_id == config.CAM0_ID else self.picam1
        if existing_picam is not None:
            try:
                # *** CRITICAL: Stop Picamera2 recording before stopping camera ***
                if cam_id == config.CAM0_ID and self.is_recording:
                    logging.warning(f"Stopping active recording on {cam_name} due to re-initialization.")
                    self.stop_recording() # Use the proper stop method

                if existing_picam.started:
                    logging.info(f"Stopping existing {cam_name}...")
                    existing_picam.stop() # Stop the camera stream
                logging.info(f"Closing existing {cam_name}...")
                existing_picam.close()
            except Exception as e:
                logging.warning(f"Error stopping/closing previous {cam_name}: {e}")
            finally:
                if cam_id == config.CAM0_ID:
                    self.picam0 = None
                    self.is_initialized0 = False
                    self.actual_cam0_fps = None
                    self.picam0_encoder = None # Clear encoder ref
                elif cam_id == config.CAM1_ID:
                    self.picam1 = None
                    self.is_initialized1 = False
            time.sleep(0.5)

        # --- Create and Configure New Instance ---
        try:
            picam_instance = Picamera2(camera_num=cam_id, tuning=tuning_data)
            logging.info(f"{cam_name}: Picamera2 object created.")

            # Setup controls, including FrameRate (unchanged logic)
            # ... (controls setup unchanged) ...
            controls_to_set = {
                "FrameRate": target_fps, # Explicitly set target frame rate
                "NoiseReductionMode": self.current_noise_reduction_mode,
                "AeEnable": True, # Default to AE enabled
                "AeExposureMode": self.current_ae_mode,
                "AeMeteringMode": self.current_metering_mode,
                "AnalogueGain": self.current_analogue_gain,
                "Brightness": self.current_brightness,
                "Contrast": self.current_contrast,
                "Saturation": self.current_saturation,
                "Sharpness": self.current_sharpness,
            }
            if self.current_analogue_gain == 0.0:
                controls_to_set["AeEnable"] = True # Auto ISO needs AE enabled
            controls_to_set = {k: v for k, v in controls_to_set.items() if v is not None}
            transform = Transform()
            if cam_id == config.CAM1_ID: # Apply flips only to Cam1 if enabled
                if config.CAM1_VFLIP: transform.vflip = True
                if config.CAM1_HFLIP: transform.hflip = True


            # Configure video stream (main purpose is capture for stream/CV now)
            cam_config = picam_instance.create_video_configuration(
                main={"size": (target_width, target_height), "format": "RGB888"}, # RGB for CV/stream
                controls=controls_to_set,
                transform=transform
            )
            logging.info(f"{cam_name}: Configuring with: main={cam_config['main']}, controls={cam_config['controls']}")
            picam_instance.configure(cam_config)
            time.sleep(0.5)

            # Verify applied configuration (unchanged logic)
            # ... (verification and logging unchanged) ...
            new_config = picam_instance.camera_configuration()
            if new_config:
                applied_controls = new_config.get('controls', {})
                applied_main = new_config.get('main', {})
                logging.info(f"{cam_name}: Verified Applied Config: main={applied_main}, controls={applied_controls}")
                actual_fps_reported = applied_controls.get('FrameRate')
                if actual_fps_reported is not None:
                    logging.info(f"{cam_name}: Driver reported actual FrameRate: {actual_fps_reported:.2f} fps")
                    if abs(actual_fps_reported - target_fps) > 1.0: # Log significant deviation
                        logging.warning(f"{cam_name}: Driver adjusted FrameRate significantly from target {target_fps:.1f} to {actual_fps_reported:.2f}")
                else:
                     logging.warning(f"{cam_name}: Could not read back actual FrameRate from config.")
                     actual_fps_reported = target_fps # Assume target if readback fails
            else:
                logging.warning(f"{cam_name}: Could not get camera configuration after applying.")
                actual_fps_reported = target_fps # Assume target if config read fails


            logging.info(f"{cam_name}: Starting camera...")
            picam_instance.start() # Start capture stream
            logging.info(f"{cam_name}: Camera started");
            time.sleep(1.0)

            # Final check and log (unchanged logic)
            # ... (final check and log unchanged) ...
            actual_config_after_start = picam_instance.camera_configuration()
            if not actual_config_after_start:
                raise RuntimeError(f"{cam_name}: Failed get config after start.")
            actual_format = actual_config_after_start.get('main', {})
            actual_w = actual_format.get('size', (0,0))[0]
            actual_h = actual_format.get('size', (0,0))[1]
            actual_fmt_str = actual_format.get('format', 'Unknown')
            actual_fps_final = actual_config_after_start.get('controls', {}).get('FrameRate', actual_fps_reported)
            actual_gain = actual_config_after_start.get('controls', {}).get('AnalogueGain', 'N/A')
            logging.info(f"{cam_name}: Initialized. Actual stream: {actual_w}x{actual_h} {actual_fmt_str} @ {actual_fps_final:.2f} fps. Gain: {actual_gain}")

            is_initialized_flag = True
            self.last_error = None
            return True

        except Exception as e:
            # ... (error handling unchanged) ...
            logging.error(f"!!! Failed to initialize {cam_name} at {target_width}x{target_height}: {e}", exc_info=True)
            self.last_error = f"{cam_name} Init Error: {e}"
            if picam_instance is not None:
                try:
                    if picam_instance.started: picam_instance.stop()
                    picam_instance.close()
                except Exception as close_e:
                    logging.error(f"Error closing {cam_name} after init failure: {close_e}")
            picam_instance = None
            is_initialized_flag = False
            actual_fps_reported = None
            return False

        finally:
            # Update manager state (unchanged logic)
            # ... (state update unchanged) ...
            if cam_id == config.CAM0_ID:
                self.picam0 = picam_instance
                self.is_initialized0 = is_initialized_flag
                self.actual_cam0_fps = actual_fps_reported
                self.last_cam0_capture_time = None
                self.measured_cam0_fps_avg = None
            elif cam_id == config.CAM1_ID:
                self.picam1 = picam_instance
                self.is_initialized1 = is_initialized_flag

    def initialize_cameras(self, resolution_index=None):
        """Initializes or re-initializes camera(s) based on config."""
        # Uses the updated _initialize_camera which handles stopping recording
        # ... (logic unchanged) ...
        with self.config_lock:
            logging.info("--- Initializing Camera(s) ---")
            success0 = self._initialize_camera(config.CAM0_ID, resolution_index)
            success1 = True # Assume success if Cam1 is disabled

            if config.ENABLE_CAM1: # Only initialize Cam1 if enabled
                success1 = self._initialize_camera(config.CAM1_ID)
            else:
                logging.info("Cam1 initialization skipped (disabled in config).")
                self.picam1 = None # Ensure Cam1 instance is None if disabled
                self.is_initialized1 = False

            if success0 and success1:
                cam_count = "One" if not config.ENABLE_CAM1 else "Both"
                logging.info(f"--- {cam_count} Camera(s) Initialized Successfully ---")
                return True
            else:
                logging.error("!!! Failed to initialize one or both required cameras. Check logs. !!!")
                if not success0 and not self.last_error: self.last_error = "Cam0 Initialization Failed"
                if not success1 and config.ENABLE_CAM1 and not self.last_error: self.last_error = "Cam1 Initialization Failed"
                elif not success1 and config.ENABLE_CAM1: self.last_error += " / Cam1 Initialization Failed"
                return False


    def get_cam0_resolution_config(self):
        """Returns the configured resolution tuple (width, height, target_fps) for Cam0."""
        # ... (unchanged logic) ...
        try:
            return config.CAM0_RESOLUTIONS[self.current_resolution_index0]
        except IndexError:
            logging.error(f"Invalid resolution index {self.current_resolution_index0} for Cam0. Using default.")
            safe_default_index = max(0, min(len(config.CAM0_RESOLUTIONS) - 1, config.CAM0_DEFAULT_RESOLUTION_INDEX))
            self.current_resolution_index0 = safe_default_index
            return config.CAM0_RESOLUTIONS[safe_default_index]


    def apply_camera_controls(self, controls_dict):
        """Applies a dictionary of common controls to running cameras."""
        # ... (unchanged logic) ...
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
        temp_last_error = None

        try:
            with self.config_lock:
                # Update internal state FIRST
                iso_name_updated = False
                for key, value in controls_dict.items():
                    if key == 'AnalogueGain':
                        self.current_analogue_gain = value
                        self.current_iso_name = "Unknown"
                        try:
                            for name, gain in config.AVAILABLE_ISO_SETTINGS.items():
                                if abs(gain - value) < 0.01: self.current_iso_name = name; iso_name_updated = True; break
                            if not iso_name_updated: logging.warning(f"Applied AnalogueGain {value} does not match any known ISO name.")
                        except Exception as e: logging.error(f"Error updating ISO name for gain {value}: {e}")
                    elif key == 'AeExposureMode': self.current_ae_mode = value
                    elif key == 'AeMeteringMode': self.current_metering_mode = value
                    elif key == 'NoiseReductionMode': self.current_noise_reduction_mode = value
                    elif key == 'Brightness': self.current_brightness = value
                    elif key == 'Contrast': self.current_contrast = value
                    elif key == 'Saturation': self.current_saturation = value
                    elif key == 'Sharpness': self.current_sharpness = value

                # Apply to Cam0
                logging.debug(f"Applying to Cam0...")
                if self.is_initialized0 and self.picam0 and self.picam0.started:
                    try: self.picam0.set_controls(controls_dict); applied_to_cam0 = True; success_count += 1
                    except Exception as e0: logging.error(f"!!! Error applying controls to Cam0: {e0}"); temp_last_error = f"Cam0 Control Error: {e0}"
                else: logging.warning("Skipping control application for Cam0 (not ready).")

                # Apply to Cam1 (only if enabled)
                if config.ENABLE_CAM1:
                    logging.debug(f"Applying to Cam1...")
                    if self.is_initialized1 and self.picam1 and self.picam1.started:
                        try: self.picam1.set_controls(controls_dict); applied_to_cam1 = True; success_count += 1
                        except Exception as e1: logging.error(f"!!! Error applying controls to Cam1: {e1}"); err_msg = f"Cam1 Control Error: {e1}"; temp_last_error = (temp_last_error + " / " + err_msg) if temp_last_error else err_msg
                    else: logging.warning("Skipping control application for Cam1 (not ready).")
                else: logging.debug("Skipping control application for Cam1 (disabled).")

            # --- Post-Application ---
            required_successes = 1 if not config.ENABLE_CAM1 else 2
            if success_count > 0:
                if any(k in controls_dict for k in ['AnalogueGain', 'AeExposureMode', 'AeMeteringMode', 'Brightness', 'ExposureTime']): time.sleep(0.5)
                else: time.sleep(0.1)
                if temp_last_error is None and success_count >= required_successes: self.last_error = None
                logging.info(f"Controls applied successfully to {success_count} camera(s).")
                return True
            else:
                self.last_error = temp_last_error if temp_last_error else "Control Apply Error: Failed on all ready cameras."
                logging.error(f"Failed to apply controls to any ready camera. Last Error: {self.last_error}")
                return False

        except Exception as e:
            logging.error(f"!!! Unexpected Error during control application: {e}", exc_info=True)
            self.last_error = f"Control Apply Unexpected Error: {e}"
            return False


    def get_camera_state(self):
        """Returns a dictionary containing the current camera state."""
        with self.config_lock:
            # ... (logic to determine output frame dimensions unchanged) ...
            res_w0, res_h0, target_fps0 = self.get_cam0_resolution_config()
            output_w = res_w0
            output_h = res_h0
            if config.ENABLE_CAM1 and self.is_initialized1: # Adjust if Cam1 enabled
                res_w1, res_h1 = config.CAM1_RESOLUTION
                display_w1 = min(res_w1, output_w); display_h1 = res_h1 if display_w1 == res_w1 else int(res_h1 * (display_w1 / res_w1))
                output_h = res_h0 + display_h1 + config.STREAM_BORDER_SIZE

            ae_mode_name = getattr(self.current_ae_mode, 'name', str(self.current_ae_mode))
            metering_mode_name = getattr(self.current_metering_mode, 'name', str(self.current_metering_mode))
            nr_mode_name = getattr(self.current_noise_reduction_mode, 'name', str(self.current_noise_reduction_mode))

            # Report target_recording_paths as recording_paths
            state = {
                'is_initialized': self.is_initialized0 and (not config.ENABLE_CAM1 or self.is_initialized1),
                'is_initialized0': self.is_initialized0,
                'is_initialized1': self.is_initialized1,
                'is_cam1_enabled': config.ENABLE_CAM1,
                'resolution_index': self.current_resolution_index0,
                'resolution_wh': (res_w0, res_h0),
                'output_frame_wh': (output_w, output_h),
                'target_cam0_fps': target_fps0,
                'actual_cam0_fps': self.actual_cam0_fps,
                'measured_cam0_fps': self.measured_cam0_fps_avg,
                'recording_target_fps': self.recording_target_fps,
                'is_recording': self.is_recording,
                'recording_paths': list(self.target_recording_paths), # Report target paths
                'last_error': self.last_error,
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

    # ===========================================================
    # === Recording Methods (Using Picamera2 API) ===
    # ===========================================================

    def start_recording(self):
        """Starts recording video FROM CAM0 using Picamera2's start_recording."""
        with self.recording_lock:
            if self.is_recording:
                logging.warning("Start recording called, but already recording.")
                return True

            # Check if primary camera is ready
            if not self.is_initialized0 or not self.picam0 or not self.picam0.started:
                logging.error("Cannot start recording, Cam0 not available.")
                self.last_error = "Cam0 not available for recording."
                return False

            logging.info("Attempting to start video recording (using Picamera2 API)...")
            usb_drives = get_usb_mounts()
            if not usb_drives:
                logging.error(f"Cannot start recording: No writable USB drives found in {config.USB_BASE_PATH}.")
                self.last_error = f"Cannot start recording: No writable USB drives found"
                return False

            # --- Get dimensions and TARGET FPS from config ---
            try:
                width, height, target_fps = self.get_cam0_resolution_config()
                if target_fps <= 0:
                     logging.error(f"Invalid TARGET FPS ({target_fps}) in config. Cannot record.")
                     self.last_error = "Rec Param Error: Invalid Target FPS in config"
                     return False
                if width <= 0 or height <= 0:
                    raise ValueError(f"Invalid Cam0 dimensions: {width}x{height}")
                self.recording_target_fps = target_fps # Store for info state
                logging.info(f"Recording parameters: {width}x{height} @ Target {target_fps:.2f} fps")

                # --- Create Filename and Paths ---
                timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                # Using .mp4 extension, H264Encoder typically handles muxing internally
                base_filename = f"recording_{timestamp}_{width}x{height}.mp4"

                # Use first USB drive for primary recording target
                self.primary_recording_path = os.path.join(usb_drives[0], base_filename)
                # Store all desired final paths for later copying
                self.target_recording_paths = [os.path.join(drive, base_filename) for drive in usb_drives]
                logging.info(f"Primary recording path: {self.primary_recording_path}")
                if len(self.target_recording_paths) > 1:
                    logging.info(f"Target paths for duplication: {self.target_recording_paths[1:]}")

            except Exception as setup_err:
                logging.error(f"!!! Error setting up recording parameters/paths: {setup_err}", exc_info=True)
                self.last_error = f"Rec Setup Error: {setup_err}"
                self.recording_target_fps = None; self.primary_recording_path = None; self.target_recording_paths = []
                return False

            # --- Start Picamera2 Recording ---
            try:
                # Create the encoder instance (e.g., H.264)
                # Adjust bitrate as needed via config
                self.picam0_encoder = H264Encoder(bitrate=config.RECORDING_VIDEO_BITRATE)
                logging.info(f"Using H264Encoder with bitrate {config.RECORDING_VIDEO_BITRATE}")

                # Start recording directly using the camera instance
                self.picam0.start_recording(self.picam0_encoder, self.primary_recording_path)

                self.is_recording = True # Set flag AFTER starting successfully
                self.recording_start_time_monotonic = time.monotonic()
                logging.info(f"Picamera2 recording started to {self.primary_recording_path}.")

                # Clear previous errors on successful start
                if self.last_error and ("Recording" in self.last_error or "writer" in self.last_error or "USB" in self.last_error or "sync" in self.last_error):
                    logging.info(f"Clearing previous recording error on successful start: '{self.last_error}'")
                    self.last_error = None
                return True

            except Exception as start_err:
                logging.error(f"!!! Failed to start Picamera2 recording: {start_err}", exc_info=True)
                self.last_error = f"Picamera2 Start Rec Error: {start_err}"
                self.is_recording = False # Ensure flag is false on error
                self.picam0_encoder = None # Clear encoder ref
                self.recording_target_fps = None
                self.primary_recording_path = None
                self.target_recording_paths = []
                return False


    def stop_recording(self):
        """Stops Picamera2 recording, copies file to other USB drives, and syncs."""
        primary_path = None; target_paths = []; start_time_rec = None; stop_success = False
        with self.recording_lock:
            if not self.is_recording:
                 # If the internal flag says not recording, trust it and exit.
                 # Cleanup of potentially orphaned Picamera2 recordings handled elsewhere (init/shutdown).
                 logging.debug("stop_recording called when not self.is_recording.")
                 self.primary_recording_path = None; self.target_recording_paths = []; self.recording_target_fps = None; self.picam0_encoder = None
                 return

            logging.info("Stopping Picamera2 video recording...")
            self.is_recording = False # Set flag early
            recording_stop_time_monotonic = time.monotonic()
            # Store paths and times needed outside the lock
            primary_path = self.primary_recording_path
            target_paths = list(self.target_recording_paths)
            start_time_rec = self.recording_start_time_monotonic
            # Clear state variables within lock
            self.primary_recording_path = None
            self.target_recording_paths = []
            self.recording_target_fps = None
            self.picam0_encoder = None
            # Assume we intend to stop successfully if we passed the initial check
            stop_success = True

        # --- Stop Picamera2 Recording (outside lock) ---
        if stop_success: # Only try to stop if the internal flag indicated recording
            try:
                if self.picam0: # Check if picam0 object exists
                    # REMOVED check for self.picam0.recording
                    # Directly call stop_recording() - Picamera2 should handle if not recording,
                    # or raise an error caught by the except block.
                    logging.debug("Attempting self.picam0.stop_recording()")
                    self.picam0.stop_recording()
                    logging.info(f"Picamera2 recording stopped successfully for {primary_path}.")
                    # stop_success remains True
                else:
                    # Should not happen if is_recording was true, but handle defensively
                    logging.warning("Picamera2 instance (picam0) was None during stop sequence, cannot call stop_recording().")
                    stop_success = False # Can't be successful if instance is gone
            except Exception as stop_err:
                 # Catch any error during stop_recording, including potential library errors
                 # if called when not actually recording (though it should ideally handle this)
                 logging.error(f"!!! Error stopping Picamera2 recording: {stop_err}", exc_info=True)
                 self.last_error = f"Picamera2 Stop Rec Error: {stop_err}"
                 stop_success = False # Mark as failed

        recording_duration = (recording_stop_time_monotonic - start_time_rec) if start_time_rec else None
        log_duration = f" ~{recording_duration:.1f}s" if recording_duration else ""; logging.info(f"Recording duration:{log_duration}")

        # --- Duplicate File to Other USB Drives ---
        copied_count = 0; copy_errors = 0
        if stop_success and primary_path and os.path.exists(primary_path) and len(target_paths) > 1:
            logging.info(f"Duplicating recorded file from {primary_path} to other target drives...")
            for target_path in target_paths:
                if target_path == primary_path: continue
                try:
                    dest_dir = os.path.dirname(target_path); os.makedirs(dest_dir, exist_ok=True) # Ensure dest dir exists
                    shutil.copy2(primary_path, target_path); logging.info(f"Successfully copied to: {target_path}"); copied_count += 1
                except Exception as copy_err: logging.error(f"!!! Failed copy to {target_path}: {copy_err}"); copy_errors += 1
            if copy_errors > 0: self.last_error = f"Rec Copy Error ({copy_errors} failed)"
        elif len(target_paths) <= 1: logging.debug("Only one target path, skipping duplication.")
        elif not stop_success: logging.warning("Skipping duplication because recording stop failed.")
        elif not primary_path or not os.path.exists(primary_path): logging.error(f"Skipping duplication: Primary file missing! Path: {primary_path}"); self.last_error = self.last_error or "Rec Error: Primary file missing"

        # --- Filesystem Sync ---
        final_files_exist = any(os.path.exists(p) for p in target_paths if p)
        if final_files_exist:
            logging.info("Syncing filesystem...")
            try: sync_start_time = time.monotonic(); subprocess.run(['sync'], check=True, timeout=15); sync_duration = time.monotonic() - sync_start_time; logging.info(f"Sync completed in {sync_duration:.2f}s.")
            except subprocess.TimeoutExpired: logging.error("!!! Filesystem sync timed out!"); self.last_error = "Sync timed out"
            except Exception as e: logging.error(f"!!! Failed execute 'sync': {e}"); self.last_error = "Sync failed"
        else: logging.warning("No final recording files found, skipping sync.")
        final_file_count = sum(1 for p in target_paths if p and os.path.exists(p)); logging.info(f"Recording stopped. Final files: {final_file_count}/{len(target_paths)}.")
        if final_file_count < len(target_paths): logging.warning(f"Failed save/copy to {len(target_paths) - final_file_count} locations.")


    def capture_and_combine_frames(self):
        """
        Captures frames from enabled cameras, combines if necessary (for streaming),
        updates the stream frame, and makes raw frame0 available for CV.
        (Recording is handled separately by Picamera2 API).
        """
        frame0 = None
        frame1 = None
        output_frame_for_stream = None
        capture_successful_cam0 = False
        frame0_timestamp = None

        # --- Capture Cam0 ---
        if self.is_initialized0 and self.picam0 and self.picam0.started:
            try:
                # Use capture_array which works even during recording
                frame0 = self.picam0.capture_array("main")
                frame0_timestamp = time.monotonic()
                capture_successful_cam0 = True

                # Update latest raw frame for CV (Requirement 4)
                with self.raw_frame_lock:
                    self.latest_raw_frame0 = frame0.copy()

                # Calculate Measured FPS (Informational)
                # ... (FPS calculation logic unchanged) ...
                if self.last_cam0_capture_time is not None:
                    time_diff = frame0_timestamp - self.last_cam0_capture_time
                    if time_diff > 0.0001: # Avoid division by zero or tiny values
                        instant_fps = 1.0 / time_diff
                        if self.measured_cam0_fps_avg is None: self.measured_cam0_fps_avg = instant_fps
                        else: alpha = 0.1; self.measured_cam0_fps_avg = alpha * instant_fps + (1 - alpha) * self.measured_cam0_fps_avg
                self.last_cam0_capture_time = frame0_timestamp

            except Exception as e0:
                # ... (Error handling unchanged) ...
                logging.error(f"!!! Error during Cam0 capture: {e0}")
                error_msg = f"Cam0 Capture Error: {e0}"
                if self.last_error != error_msg: self.last_error = error_msg
                self.last_cam0_capture_time = None; self.measured_cam0_fps_avg = None; frame0 = None; frame0_timestamp = None
                with self.raw_frame_lock: self.latest_raw_frame0 = None
        # else: Cam0 not ready

        # --- Capture Cam1 ---
        # ... (Unchanged logic) ...
        if config.ENABLE_CAM1 and self.is_initialized1 and self.picam1 and self.picam1.started:
            try: frame1 = self.picam1.capture_array("main")
            except Exception as e1: logging.error(f"!!! Error during Cam1 capture: {e1}"); err_msg = f"Cam1 Capture Error: {e1}"; self.last_error = (self.last_error + " / " + err_msg) if (self.last_error and "Cam0" in self.last_error) else err_msg; frame1 = None

        # --- Prepare Output Frame (Combine or Single) for Streaming ---
        # ... (Unchanged combination logic) ...
        if frame0 is not None and frame1 is not None and config.ENABLE_CAM1:
            try:
                h0, w0, _ = frame0.shape; h1, w1, _ = frame1.shape; target_w1, target_h1 = config.CAM1_RESOLUTION
                if w1 != target_w1 or h1 != target_h1 or w1 > w0:
                    if w1 > w0: scale = w0 / w1; target_w1_scaled = w0; target_h1_scaled = int(h1 * scale); frame1_resized = cv2.resize(frame1, (target_w1_scaled, target_h1_scaled), interpolation=cv2.INTER_AREA)
                    else: frame1_resized = cv2.resize(frame1, (target_w1, target_h1), interpolation=cv2.INTER_AREA)
                    h1, w1, _ = frame1_resized.shape
                else: frame1_resized = frame1
                final_w = w0; final_h = h0 + h1 + config.STREAM_BORDER_SIZE
                output_frame_for_stream = np.zeros((final_h, final_w, 3), dtype=np.uint8); output_frame_for_stream[:, :] = config.STREAM_BORDER_COLOR
                output_frame_for_stream[0:h0, 0:w0] = frame0
                y_start1 = h0 + config.STREAM_BORDER_SIZE; x_start1 = (final_w - w1) // 2
                output_frame_for_stream[y_start1:y_start1 + h1, x_start1:x_start1 + w1] = frame1_resized
            except Exception as e_comb: logging.error(f"!!! Error combining frames: {e_comb}", exc_info=True); self.last_error = f"Frame Combine Error: {e_comb}"; output_frame_for_stream = frame0
        elif frame0 is not None: output_frame_for_stream = frame0
        elif frame1 is not None and config.ENABLE_CAM1:
            try: target_w1, target_h1 = config.CAM1_RESOLUTION; output_frame_for_stream = cv2.resize(frame1, (target_w1, target_h1), interpolation=cv2.INTER_AREA)
            except Exception as e_resize1: logging.error(f"Could not resize fallback frame1: {e_resize1}"); output_frame_for_stream = None
        else: output_frame_for_stream = None

        # --- Update Shared Frame for Streaming ---
        with self.frame_lock:
            self.output_frame = output_frame_for_stream.copy() if output_frame_for_stream is not None else None

        # --- Put Frame into Recording Queue REMOVED ---
        # Recording is handled directly by Picamera2 API now

        return output_frame_for_stream


    def get_latest_frame(self):
        """Returns the latest COMBINED output frame for streaming (thread-safe)."""
        # ... (unchanged logic) ...
        with self.frame_lock:
            if self.output_frame is not None:
                return self.output_frame.copy() # Return a copy for thread safety
            else:
                return None


    def get_latest_raw_frame0(self):
        """Returns the latest RAW frame captured from Cam0 for CV processing (thread-safe)."""
        # ... (unchanged logic) ...
        with self.raw_frame_lock:
            if self.latest_raw_frame0 is not None:
                return self.latest_raw_frame0.copy() # Return a copy for thread safety
            else:
                return None


    def shutdown(self):
        """Stops recording (if active) and closes enabled cameras cleanly."""
        logging.info("--- CameraManager Shutting Down (Picamera2 Recording) ---")

        # 1. Stop Recording (uses the new stop_recording method)
        if self.is_recording:
            logging.info("Shutdown: Stopping active Picamera2 recording...")
            self.stop_recording()
        else:
            # Check if Picamera2 was recording without the flag set (shouldn't happen)
            with self.recording_lock: # Need lock to access picam0 safely
                 if self.picam0 and self.picam0.recording:
                     logging.warning("Shutdown: Picamera2 recording active but manager flag was false. Attempting stop.")
                     try: self.picam0.stop_recording()
                     except Exception as e: logging.error(f"Error stopping orphaned Picamera2 recording on shutdown: {e}")


        # 2. Stop and Close Camera Streams
        cameras_to_shutdown = []
        if self.picam0: cameras_to_shutdown.append((self.picam0, "Cam0"))
        if self.picam1 and config.ENABLE_CAM1: cameras_to_shutdown.append((self.picam1, "Cam1"))

        for picam_instance, name in cameras_to_shutdown:
            # ... (camera stopping/closing logic unchanged) ...
            logging.info(f"Shutdown: Stopping and closing {name}...")
            try:
                # Ensure recording is stopped before stopping stream
                if name == "Cam0" and picam_instance.recording:
                    logging.warning(f"Shutdown: Explicitly stopping recording again for {name} before closing.")
                    picam_instance.stop_recording()

                if picam_instance.started:
                    picam_instance.stop(); logging.info(f"{name} stopped.")
                picam_instance.close(); logging.info(f"{name} closed.")
            except Exception as e: logging.error(f"Error stopping/closing {name} during shutdown: {e}")


        # Reset camera instances and state flags
        self.picam0 = None; self.is_initialized0 = False
        self.picam1 = None; self.is_initialized1 = False
        self.picam0_encoder = None # Clear encoder ref

        # Reset other state variables
        # ... (state reset unchanged) ...
        self.actual_cam0_fps = None; self.last_cam0_capture_time = None; self.measured_cam0_fps_avg = None; self.recording_target_fps = None
        with self.frame_lock: self.output_frame = None
        with self.raw_frame_lock: self.latest_raw_frame0 = None
        # Ensure recording path lists are clear
        self.primary_recording_path = None; self.target_recording_paths = []

        logging.info("--- CameraManager Shutdown Complete ---")