# -*- coding: utf-8 -*-
"""
camera_manager.py

Manages multiple Picamera2 cameras, including initialization, configuration,
frame capture (combined stream), and video recording (from primary camera).

**Modification:** Overhauled recording logic to use a dedicated thread and queue
                  to strictly enforce the target FPS, aiming to fix A/V desync.
                  Uses frame duplication if capture lags behind target FPS.
**Modification 2:** Removed the '-r' (frame rate hint) flag from the ffmpeg muxing
                    command to let ffmpeg infer the rate, potentially improving A/V sync.
**Modification 3:** Calculate the actual average measured FPS during recording and use
                    that as the '-r' hint for ffmpeg muxing. Track timestamps during capture.
**Modification 4:** Removed frame duplication in the recording thread. The thread now
                    waits for actual frames. Removed '-r' hint from ffmpeg again.
**Modification 5:** Changed ffmpeg muxing to re-encode video (`-c:v libx264`) instead
                    of copying (`-c:v copy`), aiming for better sync by allowing
                    ffmpeg to adjust frame timing relative to audio. Removed `-shortest`.
**Modification 6:** Reverted ffmpeg muxing back to `-c:v copy` to reduce load and
                    address potential I/O errors caused by re-encoding.
                    Kept removal of `-shortest`.
**Modification 7:** Implemented VIDEO_START_DELAY_SECONDS from config to delay adding
                    video frames to the recording queue, attempting to align with audio.
**Modification 8:** Removed VIDEO_START_DELAY_SECONDS logic. Implemented AUDIO_START_DELAY_SECONDS
                    within the audio capture thread before starting the stream.
**Modification 9 (User Request):** Removed all audio recording and processing components.
**Modification 10 (User Request):** Ensured camera manager uses designated FPS/resolution from config
                    for recording (VideoWriter initialization) and capture (camera controls).
**Modification 11 (User Request):** Modified frame capture to make the raw frame from Cam0
                    available for external CV processing via a new getter method.
"""

import os
import time
import datetime
import logging
import threading
import cv2 # Used for video writing and frame manipulation
import numpy as np # Used for frame manipulation
from picamera2 import Picamera2
from libcamera import controls, Transform
import queue # Use queue for frame passing to recording thread
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
    """Handles multiple camera operations and state (now video-only)."""

    def __init__(self):
        """Initializes the CameraManager based on config (audio removed)."""
        self.picam0 = None # Primary camera (HQ)
        self.picam1 = None # Secondary camera (IMX219) - conditionally initialized
        self.is_initialized0 = False
        self.is_initialized1 = False # Will remain False if ENABLE_CAM1 is False
        self.last_error = None # General/last error message

        # State variables for Cam0 (HQ)
        self.current_resolution_index0 = config.CAM0_DEFAULT_RESOLUTION_INDEX
        self.actual_cam0_fps = None # Store the ACTUAL FPS reported by the driver (for info/loop timing)
        self.last_cam0_capture_time = None # For calculating instantaneous FPS
        self.measured_cam0_fps_avg = None # Simple moving average for measured FPS

        # State variable for output frame (combined or single) for streaming
        self.output_frame = None # Stores the latest frame for streaming
        # State variable for raw frame output for CV processing (Req 4)
        self.latest_raw_frame0 = None # Stores the latest raw frame from Cam0 for CV

        # --- Recording State (Video Only) ---
        self.is_recording = False
        self.recording_target_fps = None # Store the TARGET FPS used for the current recording
        self.video_writers = [] # List of OpenCV VideoWriter objects for Cam0
        self.recording_paths = [] # List of corresponding file paths for Cam0
        self.recording_frame_queue = None # Queue for passing frames to recording thread
        self.recording_thread = None # Dedicated thread for writing video frames
        self.stop_recording_event = threading.Event() # Event to signal recording thread to stop
        self.recording_start_time_monotonic = None # Track recording start time precisely

        # Locks for thread safety
        self.frame_lock = threading.Lock() # Protects access to output_frame (for streaming)
        self.raw_frame_lock = threading.Lock() # Protects access to latest_raw_frame0 (for CV)
        self.config_lock = threading.Lock() # Protects access to camera state/config changes
        self.recording_lock = threading.Lock() # Protects access to recording state variables

        # --- Initialize Common Camera Controls to Defaults from Config ---
        # (Control initialization unchanged)
        self.current_analogue_gain = config.DEFAULT_ANALOGUE_GAIN
        self.current_iso_name = "Unknown"
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

        # --- Audio Recording State REMOVED ---

        logging.info("CameraManager initialized (Audio Disabled).")
        if not config.ENABLE_CAM1:
            logging.warning("Cam1 is DISABLED in config.py. Operating in single-camera mode.")
        logging.debug(f"Initial Controls: ISO={self.current_iso_name}({self.current_analogue_gain:.2f}), AE={self.current_ae_mode.name}, Metering={self.current_metering_mode.name}, NR={self.current_noise_reduction_mode.name}, Bright={self.current_brightness}, Contr={self.current_contrast}, Sat={self.current_saturation}, Sharp={self.current_sharpness}")


    def _initialize_camera(self, cam_id, resolution_index=None):
        """Internal helper to initialize or re-initialize a specific camera instance.
           Ensures camera controls (including FrameRate) are set according to config."""
        picam_instance = None
        is_initialized_flag = False
        cam_name = f"Cam{cam_id}"
        actual_fps_reported = None # Store the FPS reported by driver

        # --- Check if this camera should be initialized ---
        if cam_id == config.CAM1_ID and not config.ENABLE_CAM1:
            logging.info(f"Skipping initialization of {cam_name} (disabled in config).")
            self.picam1 = None
            self.is_initialized1 = False
            return True

        # Determine target settings based on cam_id
        if cam_id == config.CAM0_ID:
            if resolution_index is not None:
                if 0 <= resolution_index < len(config.CAM0_RESOLUTIONS):
                    self.current_resolution_index0 = resolution_index
                else:
                    logging.error(f"{cam_name}: Invalid res index {resolution_index}. Using current {self.current_resolution_index0}.")
            res_list = config.CAM0_RESOLUTIONS
            current_res_index = self.current_resolution_index0
            tuning_data = config.CAM0_TUNING
            try:
                target_width, target_height, target_fps = res_list[current_res_index] # Get target FPS from config
            except IndexError:
                logging.error(f"{cam_name}: Res index {current_res_index} out of bounds for {len(res_list)} resolutions. Using default index {config.CAM0_DEFAULT_RESOLUTION_INDEX}.")
                current_res_index = config.CAM0_DEFAULT_RESOLUTION_INDEX
                self.current_resolution_index0 = current_res_index
                target_width, target_height, target_fps = res_list[current_res_index]

        elif cam_id == config.CAM1_ID: # Only executes if ENABLE_CAM1 is True
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
                if existing_picam.started:
                    logging.info(f"Stopping existing {cam_name}...")
                    existing_picam.stop()
                logging.info(f"Closing existing {cam_name}...")
                existing_picam.close()
            except Exception as e:
                logging.warning(f"Error stopping/closing previous {cam_name}: {e}")
            finally:
                if cam_id == config.CAM0_ID:
                    self.picam0 = None
                    self.is_initialized0 = False
                    self.actual_cam0_fps = None
                elif cam_id == config.CAM1_ID:
                    self.picam1 = None
                    self.is_initialized1 = False
            time.sleep(0.5)

        # --- Create and Configure New Instance ---
        try:
            picam_instance = Picamera2(camera_num=cam_id, tuning=tuning_data)
            logging.info(f"{cam_name}: Picamera2 object created.")

            # Ensure FrameRate is set based on the config (Requirement 2)
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
                logging.info(f"{cam_name}: AnalogueGain is 0.0 (Auto ISO), ensuring AE is enabled.")
            else:
                # If specific ISO is set, consider disabling AE? Depends on desired behavior.
                # For now, keep AE enabled, allowing it to adjust exposure time.
                # controls_to_set["AeEnable"] = False # Uncomment to fix exposure time
                pass

            controls_to_set = {k: v for k, v in controls_to_set.items() if v is not None}

            transform = Transform()
            if cam_id == config.CAM1_ID: # Apply flips only to Cam1 if enabled
                if config.CAM1_VFLIP: transform.vflip = True
                if config.CAM1_HFLIP: transform.hflip = True

            # Configure with specified resolution, format, and controls
            cam_config = picam_instance.create_video_configuration(
                main={"size": (target_width, target_height), "format": "RGB888"}, # Using RGB for CV compatibility
                controls=controls_to_set,
                transform=transform
            )
            logging.info(f"{cam_name}: Configuring with: main={cam_config['main']}, controls={cam_config['controls']}")
            picam_instance.configure(cam_config)
            time.sleep(0.5) # Allow config to apply

            # Verify applied configuration and actual frame rate
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
            picam_instance.start()
            logging.info(f"{cam_name}: Camera started");
            time.sleep(1.0) # Allow camera sensor to stabilize

            # Final check on configuration after start
            actual_config_after_start = picam_instance.camera_configuration()
            if not actual_config_after_start:
                raise RuntimeError(f"{cam_name}: Failed get config after start.")

            actual_format = actual_config_after_start.get('main', {})
            actual_w = actual_format.get('size', (0,0))[0]
            actual_h = actual_format.get('size', (0,0))[1]
            actual_fmt_str = actual_format.get('format', 'Unknown')
            # Read back FPS again after start, might differ slightly
            actual_fps_final = actual_config_after_start.get('controls', {}).get('FrameRate', actual_fps_reported)
            actual_gain = actual_config_after_start.get('controls', {}).get('AnalogueGain', 'N/A')

            logging.info(f"{cam_name}: Initialized. Actual stream: {actual_w}x{actual_h} {actual_fmt_str} @ {actual_fps_final:.2f} fps. Gain: {actual_gain}")
            is_initialized_flag = True
            self.last_error = None

            return True

        except Exception as e:
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
            # Update the corresponding camera instance and state
            if cam_id == config.CAM0_ID:
                self.picam0 = picam_instance
                self.is_initialized0 = is_initialized_flag
                self.actual_cam0_fps = actual_fps_reported # Store the reported FPS
                self.last_cam0_capture_time = None
                self.measured_cam0_fps_avg = None
            elif cam_id == config.CAM1_ID:
                self.picam1 = picam_instance
                self.is_initialized1 = is_initialized_flag


    def initialize_cameras(self, resolution_index=None):
        """Initializes or re-initializes camera(s) based on config."""
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
                # Set appropriate error messages
                if not success0 and not self.last_error: self.last_error = "Cam0 Initialization Failed"
                if not success1 and config.ENABLE_CAM1 and not self.last_error: self.last_error = "Cam1 Initialization Failed"
                elif not success1 and config.ENABLE_CAM1: self.last_error += " / Cam1 Initialization Failed"
                return False

    def get_cam0_resolution_config(self):
        """Returns the configured resolution tuple (width, height, target_fps) for Cam0."""
        try:
            # Return the currently selected resolution config tuple
            return config.CAM0_RESOLUTIONS[self.current_resolution_index0]
        except IndexError:
            logging.error(f"Invalid resolution index {self.current_resolution_index0} for Cam0. Using default.")
            # Fallback to default index, ensuring it's within bounds
            safe_default_index = max(0, min(len(config.CAM0_RESOLUTIONS) - 1, config.CAM0_DEFAULT_RESOLUTION_INDEX))
            self.current_resolution_index0 = safe_default_index
            return config.CAM0_RESOLUTIONS[safe_default_index]

    def apply_camera_controls(self, controls_dict):
        """Applies a dictionary of common controls to running cameras. (Unchanged logic)"""
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

                # Apply to Cam0
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

                # Apply to Cam1 (only if enabled)
                if config.ENABLE_CAM1:
                    logging.debug(f"Applying to Cam1...")
                    if self.is_initialized1 and self.picam1 and self.picam1.started:
                        try:
                            self.picam1.set_controls(controls_dict)
                            logging.debug("Cam1 controls applied.")
                            applied_to_cam1 = True
                            success_count += 1
                        except Exception as e1:
                            logging.error(f"!!! Error applying controls to Cam1: {e1}")
                            err_msg = f"Cam1 Control Error: {e1}"
                            temp_last_error = (temp_last_error + " / " + err_msg) if temp_last_error else err_msg
                    else:
                        logging.warning("Skipping control application for Cam1 (not ready).")
                else:
                    logging.debug("Skipping control application for Cam1 (disabled).")

            # --- Post-Application ---
            required_successes = 1 if not config.ENABLE_CAM1 else 2
            if success_count > 0:
                # Allow some time for controls like gain/exposure to settle
                if any(k in controls_dict for k in ['AnalogueGain', 'AeExposureMode', 'AeMeteringMode', 'Brightness', 'ExposureTime']):
                    time.sleep(0.5)
                else:
                    time.sleep(0.1)

                if temp_last_error is None and success_count >= required_successes:
                     self.last_error = None # Clear error if all succeeded

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
        """Returns a dictionary containing the current camera state (no audio state)."""
        with self.config_lock:
            res_w0, res_h0, target_fps0 = self.get_cam0_resolution_config()

            # Determine output frame dimensions (for streaming)
            output_w = res_w0
            output_h = res_h0
            if config.ENABLE_CAM1 and self.is_initialized1: # Adjust if Cam1 enabled
                res_w1, res_h1 = config.CAM1_RESOLUTION
                display_w1 = min(res_w1, output_w) # Scale cam1 width if needed
                display_h1 = res_h1 if display_w1 == res_w1 else int(res_h1 * (display_w1 / res_w1))
                output_h = res_h0 + display_h1 + config.STREAM_BORDER_SIZE # Stack vertically

            ae_mode_name = getattr(self.current_ae_mode, 'name', str(self.current_ae_mode))
            metering_mode_name = getattr(self.current_metering_mode, 'name', str(self.current_metering_mode))
            nr_mode_name = getattr(self.current_noise_reduction_mode, 'name', str(self.current_noise_reduction_mode))

            state = {
                'is_initialized': self.is_initialized0 and (not config.ENABLE_CAM1 or self.is_initialized1),
                'is_initialized0': self.is_initialized0,
                'is_initialized1': self.is_initialized1,
                'is_cam1_enabled': config.ENABLE_CAM1,
                'resolution_index': self.current_resolution_index0,
                'resolution_wh': (res_w0, res_h0),
                'output_frame_wh': (output_w, output_h), # Dimensions of the combined streaming frame
                'target_cam0_fps': target_fps0, # Target FPS from config for Cam0
                'actual_cam0_fps': self.actual_cam0_fps, # Actual reported by driver (for info)
                'measured_cam0_fps': self.measured_cam0_fps_avg, # Measured capture rate (for info)
                'recording_target_fps': self.recording_target_fps, # The target FPS used for current recording
                'is_recording': self.is_recording,
                'recording_paths': list(self.recording_paths), # Paths being written to
                'last_error': self.last_error,
                # 'audio_last_error': None, # Removed audio error
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
    # === Recording Methods (Video Only) ===
    # ===========================================================

    def start_recording(self):
        """Starts recording video FROM CAM0 using a dedicated thread."""
        with self.recording_lock:
            if self.is_recording:
                logging.warning("Start recording called, but already recording.")
                return True

            if not self.is_initialized0 or not self.picam0 or not self.picam0.started:
                logging.error("Cannot start recording, Cam0 not available.")
                self.last_error = "Cam0 not available for recording."
                return False

            logging.info("Attempting to start video recording (Cam0)...")
            usb_drives = get_usb_mounts()
            if not usb_drives:
                logging.error(f"Cannot start recording: No writable USB drives found in {config.USB_BASE_PATH}.")
                self.last_error = f"Cannot start recording: No writable USB drives found"
                return False

            # --- Get dimensions and TARGET FPS for VideoWriter ---
            try:
                width, height, target_fps = self.get_cam0_resolution_config()

                # --- CRITICAL: Use TARGET FPS from config (Requirement 2) ---
                if target_fps <= 0:
                     logging.error(f"Invalid TARGET FPS ({target_fps}) in config for resolution index {self.current_resolution_index0}. Cannot record.")
                     self.last_error = "Rec Param Error: Invalid Target FPS in config"
                     return False

                # Store the target FPS (used for VideoWriter init)
                self.recording_target_fps = target_fps
                logging.info(f"Using TARGET FPS for VideoWriter initialization: {self.recording_target_fps:.2f} fps")

                if width <= 0 or height <= 0:
                    raise ValueError(f"Invalid Cam0 dimensions: {width}x{height}")

                # Get codec and timestamp for filenames
                fourcc = cv2.VideoWriter_fourcc(*config.CAM0_RECORDING_FORMAT)
                timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                # Use final filename directly, no intermediate "_video" suffix needed
                base_filename = f"recording_{timestamp}_{width}x{height}{config.CAM0_RECORDING_EXTENSION}"

            except Exception as setup_err:
                logging.error(f"!!! Error getting recording parameters: {setup_err}", exc_info=True)
                self.last_error = f"Rec Param Error: {setup_err}"
                self.recording_target_fps = None
                return False

            # --- Create Video Writers ---
            temp_writers = []
            temp_paths = []
            success_count = 0
            start_error = None

            for drive_path in usb_drives:
                try:
                    full_path = os.path.join(drive_path, base_filename) # Use final filename

                    # Initialize VideoWriter with the TARGET FPS (Requirement 2)
                    # This sets the container metadata FPS.
                    writer = cv2.VideoWriter(full_path, fourcc, self.recording_target_fps, (width, height))
                    if not writer.isOpened():
                        raise IOError(f"Failed to open VideoWriter for path: {full_path}")

                    temp_writers.append(writer)
                    temp_paths.append(full_path)
                    logging.info(f"Initialized video writer for: {full_path}")
                    success_count += 1
                except Exception as e:
                    logging.error(f"!!! Failed create VideoWriter for drive {drive_path}: {e}", exc_info=True)
                    if start_error is None:
                         start_error = f"Failed writer {os.path.basename(drive_path)}: {e}"

            # --- Handle Writer Creation Results ---
            if success_count > 0:
                self.video_writers = temp_writers
                self.recording_paths = temp_paths

                # --- Setup Recording Thread ---
                # Queue size based on target FPS (e.g., 1 sec buffer or 10 frames min)
                queue_size = max(10, int(self.recording_target_fps))
                self.recording_frame_queue = queue.Queue(maxsize=queue_size)
                self.stop_recording_event.clear()

                self.recording_thread = threading.Thread(
                    target=self._recording_thread_loop,
                    name="VideoWriteThread"
                )
                self.recording_thread.daemon = True # Allow exit if main thread exits

                self.is_recording = True # Set flag before starting thread
                self.recording_start_time_monotonic = time.monotonic() # Record precise start time
                self.recording_thread.start()
                logging.info(f"Video recording thread started for {success_count} drive(s).")

                # --- Audio Start REMOVED ---

                # Handle partial success
                if start_error and success_count < len(usb_drives):
                    self.last_error = f"Partial Rec Start: {start_error}"
                elif not start_error:
                     # Clear previous recording-related errors on full success
                    if self.last_error and ("Recording" in self.last_error or "writer" in self.last_error or "USB" in self.last_error or "sync" in self.last_error):
                        logging.info(f"Clearing previous video error on successful start: '{self.last_error}'")
                        self.last_error = None
                return True
            else:
                # Failed to create any writers
                self.is_recording = False
                self.recording_target_fps = None
                logging.error("Failed to start video recording on ANY drive.")
                self.last_error = f"Rec Start Failed: {start_error or 'No writers opened'}"
                # Ensure any partially opened writers are released
                for writer in temp_writers:
                    try: writer.release()
                    except: pass
                return False

    def _recording_thread_loop(self):
        """Dedicated thread to write captured video frames as they arrive from the queue."""
        logging.info("Video recording thread loop starting (writing actual frames).")

        frames_written = 0
        last_log_time = time.monotonic()
        queue_timeouts = 0
        max_timeouts_before_warn = 10 # Log a warning if queue is empty for ~5 seconds

        while not self.stop_recording_event.is_set():
            frame_to_write = None

            try:
                # --- Get Frame from Queue (Blocking with Timeout) ---
                try:
                    # Wait up to 0.5 seconds for a frame (tuple of frame, timestamp)
                    # Timestamp is captured in capture_and_combine_frames but not used here directly
                    frame_data, _ = self.recording_frame_queue.get(block=True, timeout=0.5)

                    if frame_data is None: # Check for sentinel value
                         logging.info("Recording thread: Received stop sentinel.")
                         break
                    else:
                         frame_to_write = frame_data
                         queue_timeouts = 0 # Reset timeout counter on success

                except queue.Empty:
                    # Timeout waiting for frame - capture loop might be slow or stopped
                    queue_timeouts += 1
                    if queue_timeouts >= max_timeouts_before_warn:
                         logging.warning(f"Recording thread: Waited {queue_timeouts * 0.5:.1f}s for frame, queue empty. Capture loop lagging?")
                         queue_timeouts = 0 # Reset after warning
                    # Continue loop to check stop_recording_event
                    continue
                except Exception as q_err:
                     logging.error(f"Recording thread: Error getting frame from queue: {q_err}", exc_info=True)
                     time.sleep(0.1) # Avoid busy-looping on queue errors
                     continue

                # --- Write Frame to All Writers ---
                if frame_to_write is not None:
                    write_errors = 0
                    with self.recording_lock: # Access writers list under lock
                        # Make a copy of the list to avoid issues if modified during iteration (unlikely here)
                        current_writers = list(self.video_writers)

                    if not current_writers:
                        logging.warning("Recording thread: No video writers available, but received frame. Stopping?")
                        # This state shouldn't normally happen if stop_recording cleans up properly
                        continue

                    for i, writer in enumerate(current_writers):
                        try:
                            writer.write(frame_to_write)
                        except Exception as e:
                            # Try to get path for error message, fallback if index out of bounds
                            path_str = self.recording_paths[i] if i < len(self.recording_paths) else f"Writer {i}"
                            logging.error(f"!!! Recording thread: Failed write frame {frames_written+1} to {path_str}: {e}")
                            write_errors += 1
                            # Set last_error only if it's a new write error type
                            if "write error" not in (self.last_error or ""):
                                 self.last_error = f"Frame write error: {os.path.basename(path_str)}"

                    if write_errors == 0:
                        frames_written += 1
                    elif write_errors == len(current_writers):
                        # If all writers failed, stop the recording process
                        logging.error("!!! Recording thread: All video writers failed to write frame. Stopping recording.")
                        self.last_error = "Rec stopped: All writers failed."
                        self.stop_recording_event.set() # Signal stop
                        break

                # --- Periodic Logging ---
                current_time = time.monotonic()
                if current_time - last_log_time > 10.0: # Log stats every 10 seconds
                    qsize = self.recording_frame_queue.qsize() if self.recording_frame_queue else -1
                    logging.info(f"Rec Thread Stats: Written={frames_written}, QSize={qsize}")
                    last_log_time = current_time

            except Exception as loop_err:
                logging.exception(f"!!! Unexpected error in recording thread loop: {loop_err}")
                time.sleep(0.5) # Pause briefly after an unexpected error

        # --- Cleanup ---
        logging.info(f"Video recording thread loop finished. Total frames written by thread: {frames_written}.")
        # Note: Releasing writers is handled in stop_recording after thread join

    def stop_recording(self):
        """Stops video recording thread, releases writers, and syncs filesystem."""
        final_output_paths = [] # Store paths of successfully saved files
        released_count = 0

        with self.recording_lock:
            if not self.is_recording:
                # Handle potential stale state if stop is called unexpectedly
                if self.video_writers or self.recording_paths or (self.recording_thread and self.recording_thread.is_alive()):
                    logging.warning("stop_recording called when not recording, but resources exist. Attempting cleanup.")
                    if self.recording_thread and self.recording_thread.is_alive():
                        logging.warning("Signaling potentially orphaned recording thread to stop.")
                        self.stop_recording_event.set()
                        # Try putting sentinel in case it's blocked on get()
                        if self.recording_frame_queue:
                            try: self.recording_frame_queue.put_nowait((None, None)) # Frame is None for sentinel
                            except queue.Full: pass # Ignore if queue is full
                        self.recording_thread.join(timeout=2.0) # Wait briefly
                        if self.recording_thread.is_alive(): logging.error("Orphaned recording thread did not exit!")
                        self.recording_thread = None
                    # Clear lists even if thread join failed
                    self.video_writers.clear()
                    self.recording_paths.clear()
                # No audio thread to check
                self.recording_target_fps = None
                self.recording_frame_queue = None
                return

            logging.info("Stopping video recording...")
            self.is_recording = False # Set flag early
            recording_stop_time_monotonic = time.monotonic()
            recording_duration = (recording_stop_time_monotonic - self.recording_start_time_monotonic) if self.recording_start_time_monotonic else None
            self.recording_start_time_monotonic = None # Reset start time

            # --- Signal and Wait for Recording Thread ---
            if self.recording_thread and self.recording_thread.is_alive():
                logging.info("Signaling video recording thread to stop...")
                self.stop_recording_event.set()
                # Put sentinel in queue to unblock thread if waiting on get()
                if self.recording_frame_queue:
                    try:
                        # Put a tuple (None frame, None timestamp) as sentinel
                        self.recording_frame_queue.put_nowait((None, None))
                    except queue.Full:
                        logging.warning("Could not put stop sentinel in full recording queue.")
                    except Exception as e:
                         logging.error(f"Error putting sentinel in recording queue: {e}")

                logging.info("Waiting for video recording thread to finish...")
                self.recording_thread.join(timeout=5.0) # Wait up to 5 seconds
                if self.recording_thread.is_alive():
                    logging.error("!!! Video recording thread did not exit cleanly within timeout!")
                    # Proceed with cleanup anyway, but log the error
                else:
                    logging.info("Video recording thread joined successfully.")
            elif self.recording_thread:
                 logging.warning("Recording thread object existed but was not alive during stop.")
            else:
                 logging.warning("Recording thread object did not exist during stop sequence.")

            # Clear thread and queue references
            self.recording_thread = None
            self.recording_frame_queue = None

            # --- Get Lists for Cleanup (Still under lock) ---
            writers_to_release = list(self.video_writers)
            video_paths_recorded = list(self.recording_paths)

            # Clear manager's lists immediately
            self.video_writers.clear()
            self.recording_paths.clear()
            self.recording_target_fps = None # Reset target FPS

        # --- Stop Audio REMOVED ---

        # --- Release Video Writers (Outside recording lock) ---
        logging.info(f"Releasing {len(writers_to_release)} video writer(s)...")
        for i, writer in enumerate(writers_to_release):
            video_path = video_paths_recorded[i] if i < len(video_paths_recorded) else f"Unknown_Path_{i}"
            try:
                writer.release()
                logging.info(f"Released video writer for: {video_path}")
                released_count += 1

                # --- Muxing REMOVED ---
                # Since audio is gone, the video_path is the final output path
                if os.path.exists(video_path):
                     final_output_paths.append(video_path)
                else:
                     logging.warning(f"Video file {video_path} not found after writer release.")

            except Exception as e:
                logging.error(f"Error releasing VideoWriter for {video_path}: {e}", exc_info=True)
                # If release fails but file exists, still add it to potential output
                if os.path.exists(video_path):
                     final_output_paths.append(video_path)
                     logging.warning(f"Keeping video file {video_path} despite release error.")

        # --- Cleanup Temporary Audio File REMOVED ---

        # --- Filesystem Sync ---
        if released_count > 0 or final_output_paths:
            logging.info("Syncing filesystem to ensure data is written to USB drives...")
            try:
                sync_start_time = time.monotonic()
                # Use subprocess.run for sync
                subprocess.run(['sync'], check=True, timeout=15)
                sync_duration = time.monotonic() - sync_start_time
                logging.info(f"Sync completed in {sync_duration:.2f}s.")
            except subprocess.TimeoutExpired:
                 logging.error("!!! Filesystem sync timed out after 15s!")
                 self.last_error = "Sync timed out after recording."
            except (subprocess.CalledProcessError, FileNotFoundError, Exception) as e:
                # Log error if sync command fails
                logging.error(f"!!! Failed execute 'sync': {e}", exc_info=True)
                self.last_error = "Sync failed after recording."
        else:
            logging.warning("No writers released or final files generated, skipping sync.")

        log_duration = f" ~{recording_duration:.1f}s" if recording_duration else ""
        logging.info(f"Recording stopped (Duration:{log_duration}). Released {released_count} writer(s). Final files: {len(final_output_paths)}")
        for fpath in final_output_paths: logging.debug(f" - Final file: {fpath}")


    def capture_and_combine_frames(self):
        """
        Captures frames from enabled cameras, combines if necessary (for streaming),
        updates the stream frame, makes raw frame0 available for CV,
        and puts Cam0 frame into the recording queue if recording is active.
        """
        frame0 = None
        frame1 = None
        output_frame_for_stream = None
        capture_successful_cam0 = False # Specifically for Cam0 capture
        frame0_timestamp = None # Timestamp for the captured frame0

        # --- Capture Cam0 ---
        if self.is_initialized0 and self.picam0 and self.picam0.started:
            try:
                capture_time = time.monotonic() # Get time before capture
                frame0 = self.picam0.capture_array("main") # Capture in RGB format (set during init)
                frame0_timestamp = time.monotonic() # Get timestamp immediately after capture
                capture_successful_cam0 = True # Mark Cam0 capture as successful

                # --- Update latest raw frame for CV (Requirement 4) ---
                with self.raw_frame_lock:
                    # Store a copy so the CV thread gets a consistent frame
                    self.latest_raw_frame0 = frame0.copy()

                # --- Calculate Measured FPS (Informational) ---
                if self.last_cam0_capture_time is not None:
                    time_diff = frame0_timestamp - self.last_cam0_capture_time
                    if time_diff > 0.0001: # Avoid division by zero or tiny values
                        instant_fps = 1.0 / time_diff
                        # Use simple moving average for smoothing
                        if self.measured_cam0_fps_avg is None:
                            self.measured_cam0_fps_avg = instant_fps
                        else:
                            alpha = 0.1 # Smoothing factor
                            self.measured_cam0_fps_avg = alpha * instant_fps + (1 - alpha) * self.measured_cam0_fps_avg
                self.last_cam0_capture_time = frame0_timestamp # Store the timestamp of this capture

            except Exception as e0:
                logging.error(f"!!! Error during Cam0 capture: {e0}")
                # Set specific error message if it's different from the last one
                error_msg = f"Cam0 Capture Error: {e0}"
                if self.last_error != error_msg: self.last_error = error_msg
                self.last_cam0_capture_time = None # Reset time on error
                self.measured_cam0_fps_avg = None # Reset measured FPS
                frame0 = None # Ensure frame0 is None on error
                frame0_timestamp = None
                # Clear the raw frame on error too
                with self.raw_frame_lock:
                    self.latest_raw_frame0 = None
        # else: Cam0 not ready, frame0 remains None

        # --- Capture Cam1 (only if enabled and initialized) ---
        if config.ENABLE_CAM1 and self.is_initialized1 and self.picam1 and self.picam1.started:
            try:
                frame1 = self.picam1.capture_array("main")
            except Exception as e1:
                logging.error(f"!!! Error during Cam1 capture: {e1}")
                err_msg = f"Cam1 Capture Error: {e1}"
                # Append Cam1 error if Cam0 error also occurred, otherwise set it
                self.last_error = (self.last_error + " / " + err_msg) if (self.last_error and "Cam0" in self.last_error) else err_msg
                frame1 = None # Ensure frame1 is None on error
        # elif config.ENABLE_CAM1: pass # Cam1 enabled but not ready

        # --- Prepare Output Frame (Combine or Single) for Streaming ---
        # (Unchanged combination logic)
        if frame0 is not None and frame1 is not None and config.ENABLE_CAM1:
            # Combine frame0 and frame1 for streaming if both are available and Cam1 enabled
            try:
                h0, w0, _ = frame0.shape
                h1, w1, _ = frame1.shape
                target_w1, target_h1 = config.CAM1_RESOLUTION # Use configured target size for resizing

                # Resize frame1 if its dimensions don't match target or if wider than frame0
                if w1 != target_w1 or h1 != target_h1 or w1 > w0:
                    # Maintain aspect ratio if width needs scaling
                    if w1 > w0:
                        scale = w0 / w1
                        target_w1_scaled = w0
                        target_h1_scaled = int(h1 * scale)
                        frame1_resized = cv2.resize(frame1, (target_w1_scaled, target_h1_scaled), interpolation=cv2.INTER_AREA)
                    else:
                        # Resize to target Cam1 resolution if dimensions mismatch
                        frame1_resized = cv2.resize(frame1, (target_w1, target_h1), interpolation=cv2.INTER_AREA)
                    h1, w1, _ = frame1_resized.shape # Update dimensions after potential resize
                else:
                    frame1_resized = frame1 # Use original frame1

                # Create combined frame (Cam0 on top, Cam1 below with border)
                final_w = w0
                final_h = h0 + h1 + config.STREAM_BORDER_SIZE
                output_frame_for_stream = np.zeros((final_h, final_w, 3), dtype=np.uint8)
                output_frame_for_stream[:, :] = config.STREAM_BORDER_COLOR # Fill with border color
                output_frame_for_stream[0:h0, 0:w0] = frame0 # Place Cam0 frame
                # Center Cam1 frame horizontally below border
                y_start1 = h0 + config.STREAM_BORDER_SIZE
                x_start1 = (final_w - w1) // 2
                output_frame_for_stream[y_start1:y_start1 + h1, x_start1:x_start1 + w1] = frame1_resized

            except Exception as e_comb:
                logging.error(f"!!! Error combining frames: {e_comb}", exc_info=True)
                self.last_error = f"Frame Combine Error: {e_comb}"
                output_frame_for_stream = frame0 # Fallback to frame0 if combine fails
        elif frame0 is not None:
            # Use only frame0 if Cam1 disabled or frame1 unavailable
            output_frame_for_stream = frame0
        elif frame1 is not None and config.ENABLE_CAM1:
            # Fallback to showing frame1 if only it is available (and enabled)
            try:
                target_w1, target_h1 = config.CAM1_RESOLUTION
                output_frame_for_stream = cv2.resize(frame1, (target_w1, target_h1), interpolation=cv2.INTER_AREA)
            except Exception as e_resize1:
                logging.error(f"Could not resize fallback frame1: {e_resize1}")
                output_frame_for_stream = None # Set to None if resize fails
        else:
            # No frames captured
            output_frame_for_stream = None

        # --- Update Shared Frame for Streaming ---
        with self.frame_lock:
            # Store a copy for the streaming thread
            self.output_frame = output_frame_for_stream.copy() if output_frame_for_stream is not None else None

        # --- Put Frame into Recording Queue (if recording and Cam0 captured) ---
        if self.is_recording and capture_successful_cam0 and frame0 is not None and frame0_timestamp is not None:
            if self.recording_frame_queue:
                try:
                    # Put the raw frame0 and its timestamp into the queue (non-blocking)
                    # The recording thread will handle writing this frame
                    self.recording_frame_queue.put_nowait((frame0, frame0_timestamp))
                except queue.Full:
                    # Log if the queue is full, indicating recording thread can't keep up
                    logging.warning("Recording frame queue is full. Dropping captured frame.")
                except Exception as e:
                    logging.error(f"Error putting frame into recording queue: {e}")
            # else: Should not happen if recording is True, but log just in case
            #    logging.error("Inconsistent state: Recording active but queue not initialized.")

        # Return the frame prepared for streaming (combined or single)
        return output_frame_for_stream


    def get_latest_frame(self):
        """Returns the latest COMBINED output frame for streaming (thread-safe)."""
        with self.frame_lock:
            if self.output_frame is not None:
                return self.output_frame.copy() # Return a copy for thread safety
            else:
                return None

    def get_latest_raw_frame0(self):
        """Returns the latest RAW frame captured from Cam0 for CV processing (thread-safe). (Req 4)"""
        with self.raw_frame_lock:
            if self.latest_raw_frame0 is not None:
                return self.latest_raw_frame0.copy() # Return a copy for thread safety
            else:
                return None

    # --- Audio Methods REMOVED ---


    def shutdown(self):
        """Stops recording (if active) and closes enabled cameras cleanly."""
        logging.info("--- CameraManager Shutting Down (Video Only) ---")

        # 1. Stop Recording (handles video thread stop, writer release, sync)
        if self.is_recording:
            logging.info("Shutdown: Stopping active video recording...")
            self.stop_recording() # Call the simplified stop_recording
        else:
            # Cleanup potentially orphaned thread/writers if stop wasn't called properly
            with self.recording_lock: # Need lock to safely check/clear lists
                if self.recording_thread and self.recording_thread.is_alive():
                     logging.warning("Shutdown: Stopping orphaned video recording thread...")
                     self.stop_recording_event.set()
                     # Try putting sentinel
                     if self.recording_frame_queue:
                         try: self.recording_frame_queue.put_nowait((None, None))
                         except queue.Full: pass
                     self.recording_thread.join(timeout=2.0)
                     self.recording_thread = None
                # Release any writers that might still be held
                if self.video_writers:
                     logging.warning(f"Shutdown: Releasing {len(self.video_writers)} orphaned video writers.")
                     for writer in self.video_writers:
                        try: writer.release()
                        except Exception as e: logging.error(f"Error releasing orphaned writer: {e}")
                     self.video_writers.clear()
                     self.recording_paths.clear()

            # No audio cleanup needed

        # 2. Stop and Close Cameras
        cameras_to_shutdown = []
        if self.picam0: cameras_to_shutdown.append((self.picam0, "Cam0"))
        if self.picam1 and config.ENABLE_CAM1: cameras_to_shutdown.append((self.picam1, "Cam1"))

        for picam_instance, name in cameras_to_shutdown:
            logging.info(f"Shutdown: Stopping and closing {name}...")
            try:
                if picam_instance.started:
                    picam_instance.stop()
                    logging.info(f"{name} stopped.")
                picam_instance.close()
                logging.info(f"{name} closed.")
            except Exception as e:
                logging.error(f"Error stopping/closing {name} during shutdown: {e}")

        # Reset camera instances and state flags
        self.picam0 = None; self.is_initialized0 = False
        self.picam1 = None; self.is_initialized1 = False

        # Reset other state variables
        self.actual_cam0_fps = None
        self.last_cam0_capture_time = None
        self.measured_cam0_fps_avg = None
        self.recording_target_fps = None # Reset target FPS
        with self.frame_lock: self.output_frame = None
        with self.raw_frame_lock: self.latest_raw_frame0 = None # Clear raw frame

        logging.info("--- CameraManager Shutdown Complete ---")

# Example Usage Block Removed (as it involved audio/muxing previously)