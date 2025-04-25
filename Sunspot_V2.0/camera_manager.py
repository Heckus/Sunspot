# -*- coding: utf-8 -*-
"""
camera_manager.py

Manages multiple Picamera2 cameras, including initialization, configuration,
frame capture (combined stream), video recording (from primary camera),
and audio recording/muxing.

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
import queue # Use queue for frame passing
import tempfile
import numpy # Explicit import for audio dtype check

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
        self.actual_cam0_fps = None # Store the ACTUAL FPS reported by the driver (for info/loop timing)
        self.last_cam0_capture_time = None # For calculating instantaneous FPS
        self.measured_cam0_fps_avg = None # Simple moving average for measured FPS

        # State variable for output frame (combined or single)
        self.output_frame = None # Stores the latest frame for streaming

        # --- Recording State (Refactored) ---
        self.is_recording = False
        self.recording_target_fps = None # Store the TARGET FPS used for the current recording
        self.video_writers = [] # List of OpenCV VideoWriter objects for Cam0
        self.recording_paths = [] # List of corresponding file paths for Cam0 (video-only initially)
        self.recording_frame_queue = None # Queue for passing frames to recording thread
        self.recording_thread = None # Dedicated thread for writing video frames
        self.stop_recording_event = threading.Event() # Event to signal recording thread to stop
        self.recording_start_time_monotonic = None # Track recording start time precisely
        self.recording_frame_timestamps = [] # List to store capture timestamps during recording
        self.recording_actual_frame_count = 0 # Count actual frames captured during recording

        # Locks for thread safety
        self.frame_lock = threading.Lock() # Protects access to output_frame
        self.config_lock = threading.Lock() # Protects access to camera state/config changes
        self.recording_lock = threading.Lock() # Protects access to recording state variables (is_recording, paths, etc.)

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

        # --- Audio Recording State ---
        # (Audio state variables unchanged)
        self.audio_stream = None
        self.audio_thread = None
        self.audio_write_thread = None
        self.audio_queue = queue.Queue()
        self.temp_audio_file_path = None
        self.audio_device_index = None
        self.audio_last_error = None
        self.stop_audio_event = threading.Event()
        self.audio_dtype = None
        if config.AUDIO_ENABLED:
            try:
                self.audio_dtype = np.dtype(config.AUDIO_FORMAT).type
            except TypeError:
                 logging.error(f"Unsupported audio format '{config.AUDIO_FORMAT}' in config. Disabling audio.")
                 config.AUDIO_ENABLED = False

        logging.info("CameraManager initialized.")
        if not config.ENABLE_CAM1:
            logging.warning("Cam1 is DISABLED in config.py. Operating in single-camera mode.")
        logging.debug(f"Initial Controls: ISO={self.current_iso_name}({self.current_analogue_gain:.2f}), AE={self.current_ae_mode.name}, Metering={self.current_metering_mode.name}, NR={self.current_noise_reduction_mode.name}, Bright={self.current_brightness}, Contr={self.current_contrast}, Sat={self.current_saturation}, Sharp={self.current_sharpness}")


    def _initialize_camera(self, cam_id, resolution_index=None):
        """Internal helper to initialize or re-initialize a specific camera instance. (Unchanged)"""
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
                target_width, target_height, target_fps = res_list[current_res_index]
            except IndexError:
                logging.error(f"{cam_name}: Res index {current_res_index} out of bounds for {len(res_list)} resolutions. Using default index {config.CAM0_DEFAULT_RESOLUTION_INDEX}.")
                current_res_index = config.CAM0_DEFAULT_RESOLUTION_INDEX
                self.current_resolution_index0 = current_res_index
                target_width, target_height, target_fps = res_list[current_res_index]

        elif cam_id == config.CAM1_ID: # Only executes if ENABLE_CAM1 is True
            target_width, target_height = config.CAM1_RESOLUTION
            target_fps = config.CAM1_FRAME_RATE
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

            controls_to_set = {
                "FrameRate": target_fps,
                "NoiseReductionMode": self.current_noise_reduction_mode,
                "AeEnable": True,
                "AeExposureMode": self.current_ae_mode,
                "AeMeteringMode": self.current_metering_mode,
                "AnalogueGain": self.current_analogue_gain,
                "Brightness": self.current_brightness,
                "Contrast": self.current_contrast,
                "Saturation": self.current_saturation,
                "Sharpness": self.current_sharpness,
            }
            if self.current_analogue_gain == 0.0:
                controls_to_set["AeEnable"] = True
                logging.info(f"{cam_name}: AnalogueGain is 0.0 (Auto ISO), ensuring AE is enabled.")

            controls_to_set = {k: v for k, v in controls_to_set.items() if v is not None}

            transform = Transform()
            if cam_id == config.CAM1_ID:
                if config.CAM1_VFLIP: transform.vflip = True
                if config.CAM1_HFLIP: transform.hflip = True

            cam_config = picam_instance.create_video_configuration(
                main={"size": (target_width, target_height), "format": "RGB888"},
                controls=controls_to_set,
                transform=transform
            )
            logging.info(f"{cam_name}: Configuring with: main={cam_config['main']}, controls={cam_config['controls']}")
            picam_instance.configure(cam_config)
            time.sleep(0.5)

            new_config = picam_instance.camera_configuration()
            if new_config:
                applied_controls = new_config.get('controls', {})
                applied_main = new_config.get('main', {})
                logging.info(f"{cam_name}: Verified Applied Config: main={applied_main}, controls={applied_controls}")
                actual_fps_reported = applied_controls.get('FrameRate')
                if actual_fps_reported is not None:
                    logging.info(f"{cam_name}: Driver reported actual FrameRate: {actual_fps_reported:.2f} fps")
                    if abs(actual_fps_reported - target_fps) > 1.0:
                        logging.warning(f"{cam_name}: Driver adjusted FrameRate significantly from target {target_fps:.1f} to {actual_fps_reported:.2f}")
                else:
                     logging.warning(f"{cam_name}: Could not read back actual FrameRate from config.")
                     actual_fps_reported = target_fps
            else:
                logging.warning(f"{cam_name}: Could not get camera configuration after applying.")
                actual_fps_reported = target_fps

            logging.info(f"{cam_name}: Starting camera...")
            picam_instance.start()
            logging.info(f"{cam_name}: Camera started");
            time.sleep(1.0)

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
        """Initializes or re-initializes camera(s) based on config. (Unchanged)"""
        with self.config_lock:
            logging.info("--- Initializing Camera(s) ---")
            success0 = self._initialize_camera(config.CAM0_ID, resolution_index)
            success1 = True

            if config.ENABLE_CAM1:
                success1 = self._initialize_camera(config.CAM1_ID)
            else:
                logging.info("Cam1 initialization skipped (disabled in config).")
                self.picam1 = None
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
        """Returns the configured resolution tuple (width, height, target_fps) for Cam0. (Unchanged)"""
        try:
            return config.CAM0_RESOLUTIONS[self.current_resolution_index0]
        except IndexError:
            logging.error(f"Invalid resolution index {self.current_resolution_index0} for Cam0. Using default.")
            safe_default_index = max(0, min(len(config.CAM0_RESOLUTIONS) - 1, config.CAM0_DEFAULT_RESOLUTION_INDEX))
            self.current_resolution_index0 = safe_default_index
            return config.CAM0_RESOLUTIONS[safe_default_index]

    def apply_camera_controls(self, controls_dict):
        """Applies a dictionary of common controls to running cameras. (Unchanged)"""
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

                # Apply to Cam1
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
                if any(k in controls_dict for k in ['AnalogueGain', 'AeExposureMode', 'AeMeteringMode', 'Brightness', 'ExposureTime']):
                    time.sleep(0.5)
                else:
                    time.sleep(0.1)

                if temp_last_error is None and success_count >= required_successes:
                     self.last_error = None

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
        """Returns a dictionary containing the current camera state and common control values. (Unchanged)"""
        with self.config_lock:
            res_w0, res_h0, target_fps0 = self.get_cam0_resolution_config()

            output_w = res_w0
            output_h = res_h0
            if config.ENABLE_CAM1 and self.is_initialized1:
                res_w1, res_h1 = config.CAM1_RESOLUTION
                display_w1 = min(res_w1, output_w)
                display_h1 = res_h1 if display_w1 == res_w1 else int(res_h1 * (display_w1 / res_w1))
                output_h = res_h0 + display_h1 + config.STREAM_BORDER_SIZE

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
                'output_frame_wh': (output_w, output_h),
                'target_cam0_fps': target_fps0,
                'actual_cam0_fps': self.actual_cam0_fps, # Actual reported by driver (for info)
                'measured_cam0_fps': self.measured_cam0_fps_avg, # Measured capture rate (for info)
                'recording_target_fps': self.recording_target_fps, # The FPS being enforced for recording
                'is_recording': self.is_recording,
                'recording_paths': list(self.recording_paths), # Paths being written to
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

    # ===========================================================
    # === Recording Methods (Overhauled) ===
    # ===========================================================

    def start_recording(self):
        """Starts recording video FROM CAM0 (using dedicated thread) and audio (if enabled)."""
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

            # --- Get dimensions and TARGET FPS for VideoWriter ---
            try:
                width, height, target_fps = self.get_cam0_resolution_config()

                # --- CRITICAL: Use TARGET FPS from config ---
                if target_fps <= 0:
                     logging.error(f"Invalid TARGET FPS ({target_fps}) in config for resolution index {self.current_resolution_index0}. Cannot record.")
                     self.last_error = "Rec Param Error: Invalid Target FPS in config"
                     return False

                # Store the target FPS (used for VideoWriter init and fallback)
                self.recording_target_fps = target_fps
                logging.info(f"Using TARGET FPS for VideoWriter initialization: {self.recording_target_fps:.2f} fps")

                if width <= 0 or height <= 0:
                    raise ValueError(f"Invalid Cam0 dimensions: {width}x{height}")

                fourcc = cv2.VideoWriter_fourcc(*config.CAM0_RECORDING_FORMAT)
                timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                base_filename = f"recording_{timestamp}_{width}x{height}"

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
                    video_only_filename = f"{base_filename}_video{config.CAM0_RECORDING_EXTENSION}"
                    full_path = os.path.join(drive_path, video_only_filename)

                    # Initialize VideoWriter with the TARGET FPS
                    # This sets the container metadata but doesn't force frame timing
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
                # Queue size can be smaller now as we don't buffer for duplication
                queue_size = max(10, int(self.recording_target_fps)) # e.g., 1 sec buffer or 10 frames min
                self.recording_frame_queue = queue.Queue(maxsize=queue_size)
                self.stop_recording_event.clear()

                self.recording_thread = threading.Thread(
                    target=self._recording_thread_loop,
                    name="VideoWriteThread"
                    # No need to pass width/height for black frame anymore
                )
                self.recording_thread.daemon = True # Allow exit if main thread exits

                # --- Reset Frame Timestamp Tracking ---
                self.recording_frame_timestamps = []
                self.recording_actual_frame_count = 0

                self.is_recording = True # Set flag before starting thread
                self.recording_start_time_monotonic = time.monotonic()
                self.recording_thread.start()
                logging.info(f"Video recording thread started for {success_count} drive(s).")

                # --- Start Audio ---
                if config.AUDIO_ENABLED:
                    if not self._start_audio_recording(base_filename):
                        logging.error("Failed to start audio recording component.")
                        self.audio_last_error = self.audio_last_error or "Audio start failed"
                        # Continue video recording even if audio fails
                    else:
                        logging.info("Audio recording component started.")
                        self.audio_last_error = None

                # Handle partial success
                if start_error and success_count < len(usb_drives):
                    self.last_error = f"Partial Rec Start: {start_error}"
                elif not start_error:
                     # Clear previous recording-related errors on full success
                    if self.last_error and ("Recording" in self.last_error or "writer" in self.last_error or "USB" in self.last_error or "sync" in self.last_error or "Mux" in self.last_error):
                        logging.info(f"Clearing previous video error on successful start: '{self.last_error}'")
                        self.last_error = None
                return True
            else:
                # Failed to create any writers
                self.is_recording = False
                self.recording_target_fps = None
                logging.error("Failed to start video recording on ANY drive.")
                self.last_error = f"Rec Start Failed: {start_error or 'No writers opened'}"
                # Ensure any partially opened writers are released (though unlikely)
                for writer in temp_writers:
                    try: writer.release()
                    except: pass
                return False

    def _recording_thread_loop(self):
        """Dedicated thread to write actual captured video frames as they arrive."""
        logging.info("Video recording thread loop starting (writing actual frames).")

        frames_written = 0
        last_log_time = time.monotonic()
        queue_timeouts = 0
        max_timeouts_before_warn = 10 # Log a warning if queue is empty for ~5 seconds

        while not self.stop_recording_event.is_set():
            frame_to_write = None
            timestamp = None

            try:
                # --- Get Frame from Queue (Blocking with Timeout) ---
                try:
                    # Wait up to 0.5 seconds for a frame
                    frame_data, timestamp = self.recording_frame_queue.get(block=True, timeout=0.5)
                    if frame_data is not None: # Check if it's the sentinel potentially
                         frame_to_write = frame_data
                         queue_timeouts = 0 # Reset timeout counter on success
                    elif frame_data is None: # Explicit sentinel check (if we implement one)
                         logging.info("Recording thread: Received stop sentinel.")
                         break
                except queue.Empty:
                    # Timeout waiting for frame - capture thread might be slow or stopped
                    queue_timeouts += 1
                    if queue_timeouts >= max_timeouts_before_warn:
                         logging.warning(f"Recording thread: Waited {queue_timeouts * 0.5:.1f}s for frame, queue empty. Capture thread lagging?")
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
                        current_writers = list(self.video_writers) # Copy list

                    if not current_writers:
                        logging.warning("Recording thread: No video writers available, but received frame. Stopping?")
                        # This state shouldn't normally happen if stop_recording cleans up properly
                        # self.stop_recording_event.set() # Consider stopping if writers disappear
                        continue

                    for i, writer in enumerate(current_writers):
                        try:
                            writer.write(frame_to_write)
                        except Exception as e:
                            path_str = self.recording_paths[i] if i < len(self.recording_paths) else f"Writer {i}"
                            logging.error(f"!!! Recording thread: Failed write frame {frames_written+1} to {path_str}: {e}")
                            write_errors += 1
                            if "write error" not in (self.last_error or ""):
                                 self.last_error = f"Frame write error: {os.path.basename(path_str)}"

                    if write_errors == 0:
                        frames_written += 1
                    elif write_errors == len(current_writers):
                        logging.error("!!! Recording thread: All video writers failed to write frame. Stopping recording.")
                        self.last_error = "Rec stopped: All writers failed."
                        self.stop_recording_event.set()
                        break

                # --- Periodic Logging ---
                current_time = time.monotonic()
                if current_time - last_log_time > 10.0: # Log stats every 10 seconds
                    qsize = self.recording_frame_queue.qsize() if self.recording_frame_queue else -1
                    logging.info(f"Rec Thread Stats: Written={frames_written}, QSize={qsize}")
                    last_log_time = current_time

            except Exception as loop_err:
                logging.exception(f"!!! Unexpected error in recording thread loop: {loop_err}")
                time.sleep(0.5) # Pause after error

        # --- Cleanup ---
        logging.info(f"Video recording thread loop finished. Total actual frames written: {frames_written}.")
        # Note: Releasing writers is handled in stop_recording after thread join

    def stop_recording(self):
        """Stops video (signals thread) / audio recording, releases writers, muxes files, syncs."""
        audio_file_to_mux = None
        final_output_paths = []
        released_count = 0
        # We don't calculate average FPS here anymore for muxing hint
        # target_fps_used_for_recording = self.recording_target_fps # Keep for logging maybe?

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
                            try: self.recording_frame_queue.put_nowait((None, None))
                            except queue.Full: pass
                        self.recording_thread.join(timeout=2.0) # Wait briefly
                        if self.recording_thread.is_alive(): logging.error("Orphaned recording thread did not exit!")
                        self.recording_thread = None
                    # Clear lists even if thread join failed
                    self.video_writers.clear()
                    self.recording_paths.clear()
                if config.AUDIO_ENABLED and (self.audio_thread and self.audio_thread.is_alive()):
                    logging.warning("Stopping orphaned audio recording during stop_recording call...")
                    self._stop_audio_recording() # Stop audio separately
                self.recording_target_fps = None
                self.recording_frame_queue = None
                self.recording_frame_timestamps = [] # Clear timestamps too
                self.recording_actual_frame_count = 0
                return

            logging.info("Stopping recording (Video Thread and Audio)...")
            self.is_recording = False # Set flag early
            recording_stop_time_monotonic = time.monotonic()
            recording_duration = (recording_stop_time_monotonic - self.recording_start_time_monotonic) if self.recording_start_time_monotonic else None
            start_time_rec = self.recording_start_time_monotonic
            self.recording_start_time_monotonic = None

            # --- Log Actual Frame Count and Duration (for debugging) ---
            if recording_duration and self.recording_actual_frame_count > 0:
                 actual_avg_fps = self.recording_actual_frame_count / recording_duration
                 logging.info(f"Recording duration: {recording_duration:.2f}s. Actual frames captured: {self.recording_actual_frame_count} (Avg: {actual_avg_fps:.2f} fps)")
            elif self.recording_actual_frame_count > 0:
                 logging.info(f"Actual frames captured: {self.recording_actual_frame_count} (duration unknown).")

            # --- Signal and Wait for Recording Thread ---
            if self.recording_thread and self.recording_thread.is_alive():
                logging.info("Signaling video recording thread to stop...")
                self.stop_recording_event.set()
                # Put sentinel in queue to unblock thread if waiting on get()
                if self.recording_frame_queue:
                    try:
                        self.recording_frame_queue.put_nowait((None, None)) # Send sentinel
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

            self.recording_thread = None
            self.recording_frame_queue = None # Clear queue reference
            self.recording_frame_timestamps = [] # Clear timestamps
            self.recording_actual_frame_count = 0 # Reset frame count

            # --- Get Lists for Cleanup (Still under lock) ---
            writers_to_release = list(self.video_writers)
            video_paths_recorded = list(self.recording_paths)

            # Clear manager's lists
            self.video_writers.clear()
            self.recording_paths.clear()
            self.recording_target_fps = None # Reset target FPS

        # --- Stop Audio (Outside recording lock) ---
        if config.AUDIO_ENABLED:
            audio_file_to_mux = self._stop_audio_recording()
            if audio_file_to_mux:
                logging.info(f"Audio recording stopped. Temp file ready for muxing: {audio_file_to_mux}")
            else:
                logging.warning("Audio recording stop failed or was not running.")

        # --- Release Video Writers and Mux (Outside recording lock) ---
        logging.info(f"Releasing {len(writers_to_release)} video writer(s)...")
        for i, writer in enumerate(writers_to_release):
            video_path = video_paths_recorded[i] if i < len(video_paths_recorded) else f"Unknown_Path_{i}"
            try:
                writer.release()
                logging.info(f"Released video writer for: {video_path}")
                released_count += 1

                # --- Muxing ---
                if audio_file_to_mux and os.path.exists(video_path):
                    # Mux by re-encoding video
                    final_path = self._mux_audio_video(video_path, audio_file_to_mux)
                    if final_path:
                        final_output_paths.append(final_path)
                        if final_path != video_path:
                            try:
                                os.remove(video_path)
                                logging.info(f"Removed temporary video-only file: {video_path}")
                            except OSError as rm_err:
                                logging.warning(f"Could not remove temporary video file {video_path}: {rm_err}")
                    else:
                        logging.error(f"Muxing failed for {video_path}. Keeping video-only file.")
                        final_output_paths.append(video_path)
                        self.last_error = self.last_error or "Muxing Failed"
                elif os.path.exists(video_path):
                    # Keep video-only file if audio failed or wasn't enabled
                    final_output_paths.append(video_path)
                else:
                     logging.warning(f"Video file {video_path} not found after writer release. Cannot mux or keep.")

            except Exception as e:
                logging.error(f"Error releasing VideoWriter for {video_path}: {e}", exc_info=True)
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
        if released_count > 0 or final_output_paths:
            logging.info("Syncing filesystem to ensure data is written to USB drives...")
            try:
                sync_start_time = time.monotonic()
                subprocess.run(['sync'], check=True, timeout=15)
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
        for fpath in final_output_paths: logging.debug(f" - Final file: {fpath}")


    def capture_and_combine_frames(self):
        """
        Captures frames from enabled cameras, combines if necessary,
        updates the stream frame, and puts Cam0 frame into the recording queue
        along with its timestamp.
        """
        frame0 = None
        frame1 = None
        output_frame_for_stream = None
        capture_successful = False # Specifically for Cam0 capture
        frame0_timestamp = None # Timestamp for the captured frame0

        # --- Capture Cam0 ---
        if self.is_initialized0 and self.picam0 and self.picam0.started:
            try:
                capture_start_time = time.monotonic()
                frame0 = self.picam0.capture_array("main")
                # Get timestamp immediately after capture
                frame0_timestamp = time.monotonic()
                capture_successful = True # Mark Cam0 capture as successful

                # --- Calculate Measured FPS (Informational - uses frame0_timestamp now) ---
                if self.last_cam0_capture_time is not None:
                    time_diff = frame0_timestamp - self.last_cam0_capture_time
                    if time_diff > 0.0001:
                        instant_fps = 1.0 / time_diff
                        if self.measured_cam0_fps_avg is None:
                            self.measured_cam0_fps_avg = instant_fps
                        else:
                            alpha = 0.1 # Smoothing factor
                            self.measured_cam0_fps_avg = alpha * instant_fps + (1 - alpha) * self.measured_cam0_fps_avg
                    # else: logging.warning(f"Cam0 capture time difference too small: {time_diff:.5f}s")
                self.last_cam0_capture_time = frame0_timestamp # Store the timestamp of this capture

            except Exception as e0:
                logging.error(f"!!! Error during Cam0 capture: {e0}")
                if self.last_error != f"Cam0 Capture Error: {e0}":
                     self.last_error = f"Cam0 Capture Error: {e0}"
                self.last_cam0_capture_time = None
                self.measured_cam0_fps_avg = None
                frame0 = None # Ensure frame0 is None on error
                frame0_timestamp = None
        # else: pass # Cam0 not ready

        # --- Capture Cam1 (only if enabled) ---
        # (Unchanged)
        if config.ENABLE_CAM1 and self.is_initialized1 and self.picam1 and self.picam1.started:
            try:
                frame1 = self.picam1.capture_array("main")
            except Exception as e1:
                logging.error(f"!!! Error during Cam1 capture: {e1}")
                err_msg = f"Cam1 Capture Error: {e1}"
                self.last_error = (self.last_error + " / " + err_msg) if (self.last_error and "Cam0" in self.last_error) else err_msg
        # elif config.ENABLE_CAM1: pass # Cam1 enabled but not ready

        # --- Prepare Output Frame (Combine or Single) ---
        # (Unchanged)
        if frame0 is not None and frame1 is not None and config.ENABLE_CAM1:
            try:
                h0, w0, _ = frame0.shape
                h1, w1, _ = frame1.shape
                target_w1, target_h1 = config.CAM1_RESOLUTION

                if w1 != target_w1 or h1 != target_h1 or w1 > w0:
                    frame1_resized = cv2.resize(frame1, (target_w1, target_h1), interpolation=cv2.INTER_AREA)
                    h1, w1, _ = frame1_resized.shape
                else:
                    frame1_resized = frame1

                final_w = w0
                final_h = h0 + h1 + config.STREAM_BORDER_SIZE
                output_frame_for_stream = np.zeros((final_h, final_w, 3), dtype=np.uint8)
                output_frame_for_stream[:, :] = config.STREAM_BORDER_COLOR
                output_frame_for_stream[0:h0, 0:w0] = frame0
                y_start1 = h0 + config.STREAM_BORDER_SIZE
                x_start1 = (final_w - w1) // 2
                output_frame_for_stream[y_start1:y_start1 + h1, x_start1:x_start1 + w1] = frame1_resized

            except Exception as e_comb:
                logging.error(f"!!! Error combining frames: {e_comb}", exc_info=True)
                self.last_error = f"Frame Combine Error: {e_comb}"
                output_frame_for_stream = frame0 # Fallback to frame0 if combine fails
        elif frame0 is not None:
            output_frame_for_stream = frame0
        elif frame1 is not None and config.ENABLE_CAM1:
            # Fallback if only frame1 is available
            try:
                target_w1, target_h1 = config.CAM1_RESOLUTION
                output_frame_for_stream = cv2.resize(frame1, (target_w1, target_h1), interpolation=cv2.INTER_AREA)
            except Exception as e_resize1:
                logging.error(f"Could not resize fallback frame1: {e_resize1}")
                output_frame_for_stream = None
        else:
            # No frames captured
            output_frame_for_stream = None

        # --- Update Shared Frame for Streaming ---
        with self.frame_lock:
            self.output_frame = output_frame_for_stream.copy() if output_frame_for_stream is not None else None

        # --- Put Frame into Recording Queue (if recording and Cam0 captured) ---
        # Check is_recording flag *outside* recording_lock for speed, verify inside if needed
        if self.is_recording and capture_successful and frame0 is not None and frame0_timestamp is not None:
            # --- Store Timestamp for Average FPS Calculation ---
            self.recording_frame_timestamps.append(frame0_timestamp)
            self.recording_actual_frame_count += 1

            if self.recording_frame_queue:
                try:
                    # Put the raw frame0 and its timestamp into the queue (non-blocking)
                    self.recording_frame_queue.put_nowait((frame0, frame0_timestamp))
                except queue.Full:
                    # This means the recording thread is falling behind writing frames
                    logging.warning("Recording frame queue is full. Dropping captured frame.")
                    # We are no longer duplicating, so dropping is the only option here.
                    # This indicates a potential performance issue in the writing thread or disk I/O.
                except Exception as e:
                    logging.error(f"Error putting frame into recording queue: {e}")
            # else: # This case should ideally not happen if is_recording is true
            #    logging.error("Inconsistent state: Recording active but queue not initialized.")


        return output_frame_for_stream # Return the frame for the web stream


    def get_latest_frame(self): # Renamed for clarity
        """Returns the latest output frame for streaming (thread-safe)."""
        with self.frame_lock:
            if self.output_frame is not None:
                return self.output_frame.copy()
            else:
                return None

    # --- Audio Methods ---
    # (Audio methods remain unchanged)
    def _find_audio_device(self):
        if not config.AUDIO_ENABLED: return False
        self.audio_device_index = None
        logging.info(f"Searching for audio input device containing hint: '{config.AUDIO_DEVICE_HINT}'")
        try:
            devices = sd.query_devices()
            candidate_indices = []
            for i, device in enumerate(devices):
                if device['max_input_channels'] > 0 and \
                   config.AUDIO_DEVICE_HINT.lower() in device['name'].lower() and \
                   'output' not in device['name'].lower():
                    logging.debug(f"Found potential audio device: Index {i}, Name: {device['name']}, Inputs: {device['max_input_channels']}, Rate: {device['default_samplerate']}")
                    candidate_indices.append(i)
            if not candidate_indices:
                 logging.warning(f"No audio input device found matching hint '{config.AUDIO_DEVICE_HINT}'.")
                 self.audio_last_error = "Audio Device: Not Found"; return False
            for index in candidate_indices:
                 try:
                     if not self.audio_dtype:
                          logging.error("Cannot check audio settings: Invalid audio format in config.")
                          self.audio_last_error = "Audio Device: Invalid config format"; return False
                     sd.check_input_settings(device=index, samplerate=config.AUDIO_SAMPLE_RATE, channels=config.AUDIO_CHANNELS, dtype=self.audio_dtype)
                     logging.info(f"Device {index} ('{devices[index]['name']}') supports required settings. Selecting.")
                     self.audio_device_index = index
                     if "Audio Device" in (self.audio_last_error or ""): self.audio_last_error = None
                     return True
                 except Exception as check_err:
                     logging.warning(f"Device {index} ('{devices[index]['name']}') does not support required settings ({config.AUDIO_SAMPLE_RATE}Hz, {config.AUDIO_CHANNELS}ch, {config.AUDIO_FORMAT}): {check_err}")
            logging.error(f"Found devices matching hint, but none support required audio settings.")
            self.audio_last_error = "Audio Device: Found but incompatible settings"
            self.audio_device_index = None; return False
        except Exception as e:
            logging.error(f"Error querying audio devices: {e}", exc_info=True)
            self.audio_last_error = f"Audio Device Query Error: {e}"; return False

    def _audio_recording_thread(self):
        stream_started_successfully = False
        try:
            logging.info(f"Audio capture thread started for device {self.audio_device_index}.")
            self.stop_audio_event.clear()
            def audio_callback(indata, frames, time_info, status):
                if status: logging.warning(f"Audio callback status: {status}")
                if self.audio_queue:
                    try: self.audio_queue.put(indata.copy(), block=True, timeout=0.1)
                    except queue.Full: logging.warning("Audio queue is full. Discarding audio data.")
                    except Exception as q_err: logging.error(f"Error putting audio data into queue: {q_err}")
            self.audio_stream = sd.InputStream(
                samplerate=config.AUDIO_SAMPLE_RATE, device=self.audio_device_index,
                channels=config.AUDIO_CHANNELS, dtype=self.audio_dtype,
                blocksize=config.AUDIO_BLOCK_SIZE, callback=audio_callback)
            self.audio_stream.start()
            stream_started_successfully = True
            logging.info("Audio stream started.")
            while not self.stop_audio_event.is_set():
                self.stop_audio_event.wait(timeout=0.2)
                if not self.audio_stream.active:
                     logging.warning("Audio stream became inactive unexpectedly.")
                     if not self.audio_last_error: self.audio_last_error = "Audio stream stopped unexpectedly"
                     break
        except sd.PortAudioError as pae:
             logging.error(f"!!! PortAudioError in audio recording thread: {pae}", exc_info=True)
             self.audio_last_error = f"Audio PortAudioError: {pae}"
        except Exception as e:
            logging.error(f"!!! Error in audio recording thread: {e}", exc_info=True)
            if not self.audio_last_error: self.audio_last_error = f"Audio Thread Error: {e}"
        finally:
            if self.audio_stream and stream_started_successfully:
                try:
                    if not self.audio_stream.closed:
                         logging.info("Aborting and closing audio stream...")
                         self.audio_stream.abort(ignore_errors=True)
                         self.audio_stream.close(ignore_errors=True)
                         logging.info("Audio stream aborted and closed.")
                except Exception as e_close: logging.error(f"Error closing audio stream during cleanup: {e_close}")
            elif self.audio_stream: logging.warning("Audio stream object exists but was not started successfully.")
            logging.info("Audio capture thread finished.")
            if self.audio_queue:
                 try: self.audio_queue.put(None, block=False) # Sentinel for write thread
                 except queue.Full: logging.warning("Could not add sentinel to full audio queue during cleanup.")
                 except Exception as q_err: logging.error(f"Error adding sentinel to queue during cleanup: {q_err}")

    def _audio_write_file_thread(self):
        sound_file = None; data_written = False; items_processed = 0
        try:
            if not self.temp_audio_file_path: logging.error("Audio write thread: Temp audio file path not set."); return
            if not self.audio_dtype: logging.error("Audio write thread: Audio dtype not set."); return
            subtype = None
            if self.audio_dtype == np.int16: subtype = 'PCM_16'
            elif self.audio_dtype == np.int32: subtype = 'PCM_32'
            elif self.audio_dtype == np.float32: subtype = 'FLOAT'
            elif self.audio_dtype == np.int8: subtype = 'PCM_S8'
            elif self.audio_dtype == np.uint8: subtype = 'PCM_U8'
            if subtype is None:
                 logging.error(f"Audio write thread: Unsupported audio format '{config.AUDIO_FORMAT}' ({self.audio_dtype}) for soundfile.")
                 self.audio_last_error = f"Audio Write Error: Unsupported format {config.AUDIO_FORMAT}"; return
            logging.info(f"Audio write thread: Opening {self.temp_audio_file_path} (subtype: {subtype}) for writing.")
            sound_file = sf.SoundFile(self.temp_audio_file_path, mode='w', samplerate=config.AUDIO_SAMPLE_RATE, channels=config.AUDIO_CHANNELS, subtype=subtype)
            logging.info(f"Audio write thread: File opened successfully.")
            while True:
                try:
                    audio_data = self.audio_queue.get(block=True, timeout=0.5)
                    if audio_data is None: logging.info(f"Audio write thread: Received stop sentinel after processing {items_processed} blocks."); break
                    if isinstance(audio_data, np.ndarray):
                        sound_file.write(audio_data); data_written = True; items_processed += 1
                    else: logging.warning(f"Audio write thread: Received non-numpy data from queue: {type(audio_data)}")
                except queue.Empty:
                    if self.stop_audio_event.is_set(): logging.info(f"Audio write thread: Stop event detected during queue wait after processing {items_processed} blocks."); break
                    continue
                except sf.SoundFileError as sf_err:
                     logging.error(f"!!! SoundFileError writing audio data: {sf_err}", exc_info=True)
                     self.audio_last_error = f"Audio File Write Error: {sf_err}"; break
                except Exception as write_err:
                     logging.error(f"!!! Unexpected error writing audio data: {write_err}", exc_info=True)
                     self.audio_last_error = f"Audio File Write Error: {write_err}"; break
        except sf.SoundFileError as sf_open_err:
             logging.error(f"!!! SoundFileError opening {self.temp_audio_file_path}: {sf_open_err}", exc_info=True)
             self.audio_last_error = f"Audio File Open Error: {sf_open_err}"
        except Exception as e:
            logging.error(f"!!! Error in audio write thread setup/loop: {e}", exc_info=True)
            self.audio_last_error = f"Audio Write Thread Error: {e}"
        finally:
            if sound_file:
                try: logging.info(f"Audio write thread: Closing {self.temp_audio_file_path}..."); sound_file.close(); logging.info(f"Audio write thread: Closed sound file.")
                except Exception as e_close: logging.error(f"Error closing sound file: {e_close}")
            file_exists = self.temp_audio_file_path and os.path.exists(self.temp_audio_file_path)
            if file_exists and not data_written: logging.warning(f"Audio write thread finished, but no data seems written to {self.temp_audio_file_path}.")
            elif not file_exists and data_written: logging.error("Audio write thread: Data processed, but output file does not exist!"); self.audio_last_error = self.audio_last_error or "Audio file missing after write"
            logging.info("Audio write thread finished.")

    def _start_audio_recording(self, base_filename):
        if not config.AUDIO_ENABLED: return False
        if not self.audio_dtype: self.audio_last_error = "Audio Start Failed: Invalid config format"; return False
        if self.audio_thread and self.audio_thread.is_alive(): logging.warning("Audio start called while capture thread alive. Stopping first."); self._stop_audio_recording()
        elif self.audio_write_thread and self.audio_write_thread.is_alive(): logging.warning("Audio start called while write thread alive. Stopping first."); self._stop_audio_recording()
        while not self.audio_queue.empty():
            try: self.audio_queue.get_nowait()
            except queue.Empty: break
            except Exception: pass
        if not self._find_audio_device() or self.audio_device_index is None: logging.error(f"Audio Start Failed: Could not find suitable audio device. Last error: {self.audio_last_error}"); return False
        try:
            temp_dir = tempfile.gettempdir()
            self.temp_audio_file_path = os.path.join(temp_dir, f"{base_filename}_audio{config.AUDIO_TEMP_EXTENSION}")
            if os.path.exists(self.temp_audio_file_path): logging.warning(f"Removing existing temp audio file: {self.temp_audio_file_path}"); os.remove(self.temp_audio_file_path)
            logging.info(f"Starting audio recording. Temp file: {self.temp_audio_file_path}")
            self.stop_audio_event.clear()
            self.audio_thread = threading.Thread(target=self._audio_recording_thread, name="AudioCaptureThread"); self.audio_thread.daemon = True; self.audio_thread.start()
            time.sleep(0.2)
            self.audio_write_thread = threading.Thread(target=self._audio_write_file_thread, name="AudioWriteThread"); self.audio_write_thread.daemon = True; self.audio_write_thread.start()
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
                 self.temp_audio_file_path = None; self.audio_thread = None; self.audio_write_thread = None; return False
            self.audio_last_error = None; logging.info("Audio recording threads started successfully."); return True
        except Exception as e:
            logging.error(f"!!! Failed to start audio recording setup: {e}", exc_info=True)
            self.audio_last_error = f"Audio Start Error: {e}"; self.stop_audio_event.set()
            if self.audio_thread and self.audio_thread.is_alive(): self.audio_thread.join(timeout=1.0)
            if self.audio_write_thread and self.audio_write_thread.is_alive(): self.audio_write_thread.join(timeout=1.0)
            if self.temp_audio_file_path and os.path.exists(self.temp_audio_file_path):
                try: os.remove(self.temp_audio_file_path)
                except OSError: pass
            self.temp_audio_file_path = None; self.audio_thread = None; self.audio_write_thread = None; return False

    def _stop_audio_recording(self):
        if not config.AUDIO_ENABLED: return None
        capture_thread_running = self.audio_thread and self.audio_thread.is_alive()
        write_thread_running = self.audio_write_thread and self.audio_write_thread.is_alive()
        temp_file_path_at_start = self.temp_audio_file_path
        if not capture_thread_running and not write_thread_running:
            logging.info("Audio recording threads were not running.")
            if temp_file_path_at_start and os.path.exists(temp_file_path_at_start) and os.path.getsize(temp_file_path_at_start) > 44:
                 logging.info(f"Found potentially valid existing temp audio file: {temp_file_path_at_start}")
                 self.temp_audio_file_path = None; return temp_file_path_at_start
            else:
                 if temp_file_path_at_start and os.path.exists(temp_file_path_at_start):
                      try: logging.info(f"Cleaning up empty/invalid temp audio file: {temp_file_path_at_start}"); os.remove(temp_file_path_at_start)
                      except OSError as e: logging.warning(f"Could not remove temp audio file {temp_file_path_at_start}: {e}")
                 self.temp_audio_file_path = None; return None
        logging.info("Stopping audio recording threads..."); self.stop_audio_event.set()
        if capture_thread_running:
            logging.debug("Waiting for audio capture thread to join..."); self.audio_thread.join(timeout=2.0)
            if self.audio_thread.is_alive(): logging.warning("Audio capture thread did not stop cleanly within timeout.")
            else: logging.debug("Audio capture thread joined.")
        if write_thread_running:
            logging.debug("Waiting for audio write thread to join..."); self.audio_write_thread.join(timeout=5.0) # Give write thread longer
            if self.audio_write_thread.is_alive(): logging.warning("Audio write thread did not stop cleanly within timeout.")
            else: logging.debug("Audio write thread joined.")
        self.audio_thread = None; self.audio_write_thread = None; self.audio_stream = None
        if temp_file_path_at_start and os.path.exists(temp_file_path_at_start):
             try:
                 file_size = os.path.getsize(temp_file_path_at_start)
                 if file_size > 44: # Basic WAV header check
                     logging.info(f"Audio stop successful. Temp file ready: {temp_file_path_at_start} (Size: {file_size} bytes)")
                     self.temp_audio_file_path = None; return temp_file_path_at_start
                 else:
                     logging.warning(f"Temporary audio file {temp_file_path_at_start} seems empty/invalid after stop (Size: {file_size}).")
                     try: os.remove(temp_file_path_at_start); logging.info("Cleaned up empty/invalid temp audio file.")
                     except OSError as e: logging.warning(f"Could not remove empty temp audio file: {e}")
                     self.temp_audio_file_path = None; return None
             except OSError as e: logging.error(f"Error checking temp audio file size {temp_file_path_at_start}: {e}"); self.temp_audio_file_path = None; return None
        else: logging.error("Temporary audio file path not set or file does not exist after stop sequence."); self.temp_audio_file_path = None; return None

    def _mux_audio_video(self, video_path, audio_path):
        """
        Merges audio and video files using ffmpeg. Re-encodes video using libx264.
        Lets ffmpeg infer the video frame rate and sync to audio.
        """
        if not config.AUDIO_ENABLED: return None
        if not os.path.exists(config.FFMPEG_PATH): logging.error(f"ffmpeg not found at '{config.FFMPEG_PATH}'. Cannot mux audio."); self.audio_last_error = "Mux Error: ffmpeg not found"; return None
        if not os.path.exists(video_path): logging.error(f"Video file not found for muxing: {video_path}"); self.audio_last_error = "Mux Error: Video file missing"; return None
        if not os.path.exists(audio_path): logging.error(f"Audio file not found for muxing: {audio_path}"); self.audio_last_error = "Mux Error: Audio file missing"; return None
        if os.path.getsize(audio_path) <= 44: logging.error(f"Audio file {audio_path} is too small, likely invalid. Skipping mux."); self.audio_last_error = "Mux Error: Audio file invalid/empty"; return None

        output_path = video_path.replace("_video" + config.CAM0_RECORDING_EXTENSION, config.CAM0_RECORDING_EXTENSION)
        if output_path == video_path: output_path = video_path.replace(config.CAM0_RECORDING_EXTENSION, "_muxed" + config.CAM0_RECORDING_EXTENSION)
        logging.info(f"Muxing (re-encode video) '{os.path.basename(video_path)}' and audio '{os.path.basename(audio_path)}' into '{os.path.basename(output_path)}'...")

        # --- Command for re-encoding video ---
        command = [config.FFMPEG_PATH, "-y",           # Base command, -y to overwrite output
                   "-i", video_path,                   # Input video file
                   "-i", audio_path,                   # Input audio file
                   "-c:v", "libx264",                  # Re-encode video using H.264
                   "-preset", "ultrafast",             # Fastest encoding preset (lower quality)
                   "-crf", "23",                       # Constant Rate Factor (quality, 18-28 is typical)
                   "-c:a", "aac",                      # Encode audio stream to AAC
                   "-map", "0:v:0",                    # Map video stream from first input
                   "-map", "1:a:0",                    # Map audio stream from second input
                   # Removed "-shortest" to let audio dictate length primarily
                   "-loglevel", config.FFMPEG_LOG_LEVEL, # Set logging level
                   output_path                         # Output file path
                  ]
        logging.debug(f"Executing ffmpeg command: {' '.join(command)}")

        mux_start_time = time.monotonic()
        # Increase timeout slightly as re-encoding takes longer than copy
        timeout = config.AUDIO_MUX_TIMEOUT * config.AUDIO_MUX_RECODE_TIMEOUT_MULTIPLIER
        # Ensure multiplier is at least 1, maybe 2 or 3? Let's set it in config, default 1.
        # If config.AUDIO_MUX_RECODE_TIMEOUT_MULTIPLIER is not set, default to 2
        timeout_multiplier = getattr(config, 'AUDIO_MUX_RECODE_TIMEOUT_MULTIPLIER', 2)
        timeout = config.AUDIO_MUX_TIMEOUT * timeout_multiplier
        logging.info(f"Using mux timeout: {timeout}s (Multiplier: {timeout_multiplier})")


        try:
            process = subprocess.run(command, capture_output=True, text=True, check=True, timeout=timeout)
            mux_duration = time.monotonic() - mux_start_time
            logging.info(f"ffmpeg muxing (re-encode) successful for {output_path} in {mux_duration:.2f}s.")
            if logging.getLogger().isEnabledFor(logging.DEBUG): logging.debug(f"ffmpeg output:\n--- stdout ---\n{process.stdout}\n--- stderr ---\n{process.stderr}\n---")
            if "Mux Error" in (self.audio_last_error or ""): self.audio_last_error = None
            return output_path
        except subprocess.TimeoutExpired:
            mux_duration = time.monotonic() - mux_start_time; logging.error(f"!!! ffmpeg muxing (re-encode) timed out ({timeout}s) for {output_path} after {mux_duration:.1f}s."); self.audio_last_error = "Mux Error: ffmpeg re-encode timed out"
        except subprocess.CalledProcessError as e:
            mux_duration = time.monotonic() - mux_start_time; logging.error(f"!!! ffmpeg muxing (re-encode) failed for {output_path} after {mux_duration:.1f}s. Return Code: {e.returncode}"); logging.error(f"ffmpeg stderr:\n{e.stderr}"); logging.error(f"ffmpeg stdout:\n{e.stdout}"); self.audio_last_error = f"Mux Error: ffmpeg re-encode failed (code {e.returncode})"
        except Exception as e:
            mux_duration = time.monotonic() - mux_start_time; logging.error(f"!!! Unexpected error during ffmpeg re-encode execution after {mux_duration:.1f}s: {e}", exc_info=True); self.audio_last_error = f"Mux Error: {e}"
        if os.path.exists(output_path):
             try: logging.warning(f"Attempting to remove failed mux output file: {output_path}"); os.remove(output_path)
             except OSError as rm_err: logging.error(f"Could not remove failed mux output {output_path}: {rm_err}")
        return None


    def shutdown(self):
        """Stops recording, stops and closes enabled cameras cleanly."""
        logging.info("--- CameraManager Shutting Down ---")

        # 1. Stop Recording (handles video thread stop, audio stop, muxing)
        if self.is_recording:
            logging.info("Shutdown: Stopping active recording...")
            self.stop_recording()
        else:
            # Cleanup potentially orphaned threads/files if stop wasn't called properly
            if self.recording_thread and self.recording_thread.is_alive():
                 logging.warning("Shutdown: Stopping orphaned video recording thread...")
                 self.stop_recording_event.set()
                 # Try putting sentinel
                 if self.recording_frame_queue:
                     try: self.recording_frame_queue.put_nowait((None, None))
                     except queue.Full: pass
                 self.recording_thread.join(timeout=2.0)
                 self.recording_thread = None
                 # Release any writers that might be held
                 with self.recording_lock:
                     for writer in self.video_writers:
                        try: writer.release()
                        except: pass
                     self.video_writers.clear(); self.recording_paths.clear()
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
                      except OSError as e: logging.error(f"Could not remove orphaned temp audio file: {e}")

        # 2. Stop and Close Cameras
        # (Unchanged)
        cameras_to_shutdown = [(config.CAM0_ID, 'picam0', 'is_initialized0', "Cam0")]
        if config.ENABLE_CAM1:
            cameras_to_shutdown.append((config.CAM1_ID, 'picam1', 'is_initialized1', "Cam1"))
        for cam_id, picam_instance_attr, init_flag_attr, name in cameras_to_shutdown:
            picam_instance = getattr(self, picam_instance_attr, None)
            if picam_instance:
                logging.info(f"Shutdown: Stopping and closing {name}...")
                try:
                    if picam_instance.started: picam_instance.stop(); logging.info(f"{name} stopped.")
                    picam_instance.close(); logging.info(f"{name} closed.")
                except Exception as e: logging.error(f"Error stopping/closing {name} during shutdown: {e}")
                finally: setattr(self, picam_instance_attr, None); setattr(self, init_flag_attr, False)
            else:
                 if getattr(self, init_flag_attr, False): logging.warning(f"Shutdown: {name} instance was None, but expected to be initialized.")
                 else: logging.debug(f"Shutdown: {name} instance already None or was disabled.")

        # Reset other state variables
        self.actual_cam0_fps = None
        self.last_cam0_capture_time = None
        self.measured_cam0_fps_avg = None
        self.recording_target_fps = None # Reset target FPS
        self.recording_frame_timestamps = [] # Clear timestamps
        self.recording_actual_frame_count = 0
        with self.frame_lock: self.output_frame = None
        logging.info("--- CameraManager Shutdown Complete ---")

# Example Usage (Main testing block - Needs update for new recording logic if used)
# if __name__ == "__main__":
#    # ... (Testing code would need adjustments) ...
