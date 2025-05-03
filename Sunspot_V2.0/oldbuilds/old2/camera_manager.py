# -*- coding: utf-8 -*-
"""
camera_manager.py

Manages the Picamera2 camera, including initialization, configuration,
frame capture, video recording, and audio recording/muxing.
Refactored for single-camera operation.
Includes fix attempt for audio/video sync using ffmpeg -ss offset.
"""

import os
import time
import datetime
import logging
import threading
import cv2
import numpy as np
from picamera2 import Picamera2
from libcamera import controls, Transform # Keep Transform for potential future use

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
    """Handles camera operations and state, including audio."""

    def __init__(self):
        """Initializes the CameraManager based on config."""
        self.picam = None # Camera instance
        self.is_initialized = False
        self.last_error = None # General/last error message

        # State variables
        self.current_resolution_index = config.DEFAULT_RESOLUTION_INDEX
        self.actual_fps = None # Store the ACTUAL FPS reported by the driver
        self.last_capture_time = None # For calculating instantaneous FPS
        self.measured_fps_avg = None # Simple moving average for measured FPS
        # Minimum reasonable measured FPS to use for VideoWriter (as fallback)
        self.MIN_MEASURED_FPS_THRESHOLD = 5.0

        # State variable for output frame
        self.output_frame = None # Stores the latest frame for streaming

        # Recording state
        self.is_recording = False
        self.video_writers = [] # List of OpenCV VideoWriter objects
        self.recording_paths = [] # List of corresponding file paths (video-only initially)
        self.recording_start_time = None # Track recording start time
        self.recording_fps = None # Store the FPS used for the current recording session

        # Locks for thread safety
        self.frame_lock = threading.Lock() # Protects access to output_frame
        self.config_lock = threading.Lock() # Protects access to camera state/config changes
        self.recording_lock = threading.Lock() # Protects access to recording state/writers

        # --- Initialize Camera Controls to Defaults from Config ---
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

        logging.info("CameraManager initialized for single camera operation.")
        logging.debug(f"Initial Controls: ISO={self.current_iso_name}({self.current_analogue_gain:.2f}), AE={self.current_ae_mode.name}, Metering={self.current_metering_mode.name}, NR={self.current_noise_reduction_mode.name}, Bright={self.current_brightness}, Contr={self.current_contrast}, Sat={self.current_saturation}, Sharp={self.current_sharpness}")


    def _initialize_camera_instance(self, resolution_index=None):
        """Internal helper to initialize or re-initialize the camera instance."""
        picam_instance = None
        is_initialized_flag = False
        actual_fps_reported = None # Store the FPS reported by driver
        cam_name = f"Cam{config.CAMERA_ID}" # For logging

        # Determine target settings
        if resolution_index is not None:
            if 0 <= resolution_index < len(config.CAMERA_RESOLUTIONS):
                self.current_resolution_index = resolution_index
            else:
                logging.error(f"{cam_name}: Invalid res index {resolution_index}. Using current {self.current_resolution_index}.")
        res_list = config.CAMERA_RESOLUTIONS
        current_res_index = self.current_resolution_index
        tuning_data = config.TUNING # Use loaded tuning data
        try:
            target_width, target_height, target_fps = res_list[current_res_index]
        except IndexError:
            logging.error(f"{cam_name}: Res index {current_res_index} out of bounds for {len(res_list)} resolutions. Using default index {config.DEFAULT_RESOLUTION_INDEX}.")
            current_res_index = config.DEFAULT_RESOLUTION_INDEX
            self.current_resolution_index = current_res_index
            target_width, target_height, target_fps = res_list[current_res_index]

        logging.info(f"Attempting init {cam_name} (ID: {config.CAMERA_ID}) at index {current_res_index} ({target_width}x{target_height} @ Target {target_fps:.1f}fps)...")

        # --- Stop and Close Existing Instance ---
        if self.picam is not None:
            try:
                if self.picam.started:
                    logging.info(f"Stopping existing {cam_name}...")
                    self.picam.stop()
                logging.info(f"Closing existing {cam_name}...")
                self.picam.close()
            except Exception as e:
                logging.warning(f"Error stopping/closing previous {cam_name}: {e}")
            finally:
                # Ensure the instance variable is cleared
                self.picam = None
                self.is_initialized = False
                self.actual_fps = None # Reset actual FPS
            time.sleep(0.5) # Allow time for resources to release

        # --- Create and Configure New Instance ---
        try:
            # Pass tuning data directly to constructor if available
            picam_instance = Picamera2(camera_num=config.CAMERA_ID, tuning=tuning_data)
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

            # Define transform (e.g., for flipping - not currently used but kept)
            transform = Transform()
            # Example: if config.CAMERA_VFLIP: transform.vflip = True

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
            self.picam = picam_instance
            self.is_initialized = is_initialized_flag
            self.actual_fps = actual_fps_reported # Store the actual FPS
            self.last_capture_time = None # Reset capture timing
            self.measured_fps_avg = None # Reset measured FPS


    def initialize_camera(self, resolution_index=None):
        """Initializes or re-initializes the camera."""
        with self.config_lock:
            logging.info("--- Initializing Camera ---")
            success = self._initialize_camera_instance(resolution_index)

            if success:
                logging.info("--- Camera Initialized Successfully ---")
                return True
            else:
                logging.error("!!! Failed to initialize camera. Check logs. !!!")
                # Ensure last_error reflects the failure
                if not self.last_error: self.last_error = "Camera Initialization Failed"
                return False

    def get_resolution_config(self):
        """Returns the configured resolution tuple (width, height, target_fps) for the camera."""
        try:
            # Return the tuple directly from the config list based on the current index
            return config.CAMERA_RESOLUTIONS[self.current_resolution_index]
        except IndexError:
            logging.error(f"Invalid resolution index {self.current_resolution_index}. Using default.")
            # Ensure default index is valid before using it
            safe_default_index = max(0, min(len(config.CAMERA_RESOLUTIONS) - 1, config.DEFAULT_RESOLUTION_INDEX))
            self.current_resolution_index = safe_default_index # Correct the index
            return config.CAMERA_RESOLUTIONS[safe_default_index]

    def apply_camera_controls(self, controls_dict):
        """Applies a dictionary of controls to the running camera."""
        if not self.is_initialized or not self.picam or not self.picam.started:
            logging.error("Cannot apply controls: Camera not initialized or running.")
            self.last_error = "Control Apply Error: Camera not ready."
            return False

        logging.info(f"Applying controls to camera: {controls_dict}")
        success = False
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

                # Apply to Camera
                logging.debug(f"Applying to Cam{config.CAMERA_ID}...")
                try:
                    self.picam.set_controls(controls_dict)
                    logging.debug("Camera controls applied.")
                    success = True
                except Exception as e0:
                    logging.error(f"!!! Error applying controls to camera: {e0}")
                    temp_last_error = f"Camera Control Error: {e0}"

            # --- Post-Application ---
            if success:
                # Allow some time for controls like exposure/gain to take effect
                if any(k in controls_dict for k in ['AnalogueGain', 'AeExposureMode', 'AeMeteringMode', 'Brightness', 'ExposureTime']):
                    time.sleep(0.5)
                else:
                    time.sleep(0.1)

                # Clear the main error if this call was successful
                self.last_error = None
                logging.info("Controls applied successfully.")
                return True
            else:
                # If failed, update the main error status
                self.last_error = temp_last_error if temp_last_error else "Control Apply Error: Failed."
                logging.error(f"Failed to apply controls. Last Error: {self.last_error}")
                return False

        except Exception as e:
            logging.error(f"!!! Unexpected Error during control application: {e}", exc_info=True)
            self.last_error = f"Control Apply Unexpected Error: {e}"
            return False


    def get_camera_state(self):
        """Returns a dictionary containing the current camera state and control values."""
        with self.config_lock:
            # Get configured resolution details
            res_w, res_h, target_fps_config = self.get_resolution_config()

            # Safely get enum names
            ae_mode_name = getattr(self.current_ae_mode, 'name', str(self.current_ae_mode))
            metering_mode_name = getattr(self.current_metering_mode, 'name', str(self.current_metering_mode))
            nr_mode_name = getattr(self.current_noise_reduction_mode, 'name', str(self.current_noise_reduction_mode))

            state = {
                'is_initialized': self.is_initialized,
                'resolution_index': self.current_resolution_index,
                'resolution_wh': (res_w, res_h),
                'output_frame_wh': (res_w, res_h), # Output is same as camera resolution
                'target_fps': target_fps_config,
                'actual_fps': self.actual_fps,
                'measured_fps': self.measured_fps_avg,
                'is_recording': self.is_recording,
                'recording_paths': list(self.recording_paths),
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
        """Starts recording video and audio (if enabled)."""
        with self.recording_lock:
            if self.is_recording:
                logging.warning("Start recording called, but already recording.")
                return True

            if not self.is_initialized or not self.picam or not self.picam.started:
                logging.error("Cannot start recording, Camera not available.")
                self.last_error = "Camera not available for recording."
                return False

            logging.info("Attempting to start recording...")
            usb_drives = get_usb_mounts()
            if not usb_drives:
                logging.error(f"Cannot start recording: No writable USB drives found in {config.USB_BASE_PATH}.")
                self.last_error = f"Cannot start recording: No writable USB drives found"
                return False

            # --- Get dimensions and FPS for VideoWriter ---
            try:
                width, height, target_fps_config = self.get_resolution_config()

                fps_for_writer = None
                fps_source = "Unknown"

                # Prioritize configured target FPS for stability and sync
                if target_fps_config > 0:
                    fps_for_writer = target_fps_config
                    fps_source = "Config Target"
                elif self.actual_fps is not None and self.actual_fps > 0:
                    fps_for_writer = self.actual_fps
                    fps_source = "Driver Reported"
                    logging.warning(f"Using driver-reported FPS ({fps_for_writer:.2f}) for VideoWriter as config target was invalid ({target_fps_config}).")
                elif self.measured_fps_avg is not None and self.measured_fps_avg >= self.MIN_MEASURED_FPS_THRESHOLD:
                    fps_for_writer = self.measured_fps_avg
                    fps_source = "Measured Average"
                    logging.warning(f"Using measured average FPS ({fps_for_writer:.2f}) for VideoWriter as config/driver FPS were unavailable.")
                else:
                    fps_for_writer = 30.0 # Absolute fallback
                    fps_source = "Hardcoded Fallback"
                    logging.error(f"All FPS sources invalid/unavailable. Using hardcoded {fps_for_writer:.1f}fps for VideoWriter.")

                if width <= 0 or height <= 0:
                    raise ValueError(f"Invalid camera dimensions: {width}x{height}")

                self.recording_fps = fps_for_writer # Store the chosen FPS

                logging.info(f"Selected FPS for VideoWriter: {fps_for_writer:.2f} (Source: {fps_source})")
                logging.info(f"Starting recording: {width}x{height} @ {fps_for_writer:.2f} fps")

                fourcc = cv2.VideoWriter_fourcc(*config.RECORDING_FORMAT)
                timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                base_filename = f"recording_{timestamp}_{width}x{height}"

            except Exception as setup_err:
                logging.error(f"!!! Error getting recording parameters: {setup_err}", exc_info=True)
                self.last_error = f"Rec Param Error: {setup_err}"
                return False

            # --- Create Video Writers ---
            self.video_writers.clear()
            self.recording_paths.clear()
            success_count = 0
            start_error = None

            for drive_path in usb_drives:
                try:
                    video_only_filename = f"{base_filename}_video{config.RECORDING_EXTENSION}"
                    full_path = os.path.join(drive_path, video_only_filename)

                    writer = cv2.VideoWriter(full_path, fourcc, fps_for_writer, (width, height))
                    if not writer.isOpened():
                        raise IOError(f"Failed to open VideoWriter for path: {full_path}")

                    self.video_writers.append(writer)
                    self.recording_paths.append(full_path)
                    logging.info(f"Started video recording component to: {full_path}")
                    success_count += 1
                except Exception as e:
                    logging.error(f"!!! Failed create VideoWriter for drive {drive_path}: {e}", exc_info=True)
                    if start_error is None:
                         start_error = f"Failed writer {os.path.basename(drive_path)}: {e}"

            # --- Handle Writer Creation Results ---
            if success_count > 0:
                self.is_recording = True
                self.recording_start_time = time.monotonic()
                logging.info(f"Video recording started on {success_count} drive(s).")

                if config.AUDIO_ENABLED:
                    if not self._start_audio_recording(base_filename):
                        logging.error("Failed to start audio recording component.")
                        self.audio_last_error = self.audio_last_error or "Audio start failed"
                    else:
                        logging.info("Audio recording component started.")
                        self.audio_last_error = None # Clear audio error on successful start

                if start_error and success_count < len(usb_drives):
                    self.last_error = f"Partial Rec Start: {start_error}"
                elif not start_error:
                    # Clear previous recording errors if start was fully successful
                    if self.last_error and ("Recording" in self.last_error or "writers" in self.last_error or "USB" in self.last_error or "sync" in self.last_error):
                        logging.info(f"Clearing previous video error on successful start: '{self.last_error}'")
                        self.last_error = None
                return True
            else:
                self.is_recording = False
                self.recording_fps = None
                logging.error("Failed to start video recording on ANY drive.")
                self.last_error = f"Rec Start Failed: {start_error or 'No writers opened'}"
                # Ensure any partially opened writers are released
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
        fps_used_for_recording = self.recording_fps # Get FPS used for this session

        with self.recording_lock:
            if not self.is_recording:
                # Cleanup potentially stale state if stop called erroneously
                if self.video_writers or self.recording_paths:
                    logging.warning("stop_recording called when not recording, but writers/paths exist. Clearing stale state.")
                    self.video_writers.clear()
                    self.recording_paths.clear()
                if config.AUDIO_ENABLED and (self.audio_thread and self.audio_thread.is_alive()):
                    logging.warning("Stopping orphaned audio recording during stop_recording call...")
                    self._stop_audio_recording() # Attempt cleanup
                self.recording_fps = None # Reset stored FPS
                return

            logging.info("Stopping recording (Video and Audio)...")
            self.is_recording = False
            recording_duration = (time.monotonic() - self.recording_start_time) if self.recording_start_time else None
            self.recording_start_time = None
            self.recording_fps = None # Reset stored FPS after use

            writers_to_release = list(self.video_writers)
            video_paths_recorded = list(self.recording_paths)

            self.video_writers.clear()
            self.recording_paths.clear()
        # End recording lock

        # Stop audio outside the lock
        if config.AUDIO_ENABLED:
            audio_file_to_mux = self._stop_audio_recording()
            if audio_file_to_mux:
                logging.info(f"Audio recording stopped. Temp file ready for muxing: {audio_file_to_mux}")
            else:
                logging.warning("Audio recording stop failed or was not running.")

        logging.info(f"Releasing {len(writers_to_release)} video writer(s)...")
        for i, writer in enumerate(writers_to_release):
            video_path = video_paths_recorded[i] if i < len(video_paths_recorded) else f"Unknown_Path_{i}"
            try:
                writer.release()
                logging.info(f"Released video writer for: {video_path}")
                released_count += 1

                # Muxing logic (pass the FPS used for this recording)
                if audio_file_to_mux and os.path.exists(video_path):
                    final_path = self._mux_audio_video(video_path, audio_file_to_mux, fps_used_for_recording)
                    if final_path:
                        final_output_paths.append(final_path)
                        # Remove original video-only file if muxing created a new file
                        if final_path != video_path:
                            try:
                                os.remove(video_path)
                                logging.info(f"Removed temporary video-only file: {video_path}")
                            except OSError as rm_err:
                                logging.warning(f"Could not remove temporary video file {video_path}: {rm_err}")
                    else:
                        logging.error(f"Muxing failed for {video_path}. Keeping video-only file.")
                        final_output_paths.append(video_path) # Keep the video file if mux fails
                        if "Mux Error" not in (self.last_error or ""): # Avoid overwriting more specific errors
                            self.last_error = "Muxing Failed"
                elif os.path.exists(video_path):
                    # Keep video file if audio wasn't enabled or failed to produce a file
                    final_output_paths.append(video_path)
                else:
                     logging.warning(f"Video file {video_path} not found after writer release. Cannot mux or keep.")

            except Exception as e:
                logging.error(f"Error releasing VideoWriter for {video_path}: {e}", exc_info=True)
                # Try to keep the video file even if release fails
                if os.path.exists(video_path):
                     final_output_paths.append(video_path)
                     logging.warning(f"Keeping video file {video_path} despite release error.")

        # Clean up temporary audio file after muxing attempts
        if audio_file_to_mux and os.path.exists(audio_file_to_mux):
            try:
                os.remove(audio_file_to_mux)
                logging.info(f"Removed temporary audio file: {audio_file_to_mux}")
            except OSError as e:
                logging.warning(f"Could not remove temporary audio file {audio_file_to_mux}: {e}")

        # Sync filesystem
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


    def capture_frame(self):
        """
        Captures a frame from the camera and writes it if recording.
        Returns the frame to be streamed.
        """
        frame = None
        output_frame_for_stream = None
        capture_successful = False

        # --- Capture Frame ---
        if self.is_initialized and self.picam and self.picam.started:
            try:
                capture_start_time = time.monotonic()
                frame = self.picam.capture_array("main")
                capture_end_time = time.monotonic()
                capture_successful = True

                # --- Calculate Measured FPS ---
                if self.last_capture_time is not None:
                    time_diff = capture_end_time - self.last_capture_time
                    if time_diff > 0.0001: # Avoid division by zero or nonsensical values
                        instant_fps = 1.0 / time_diff
                        # Update moving average
                        if self.measured_fps_avg is None:
                            self.measured_fps_avg = instant_fps
                        else:
                            # Simple exponential moving average
                            alpha = 0.1 # Smoothing factor (adjust as needed)
                            self.measured_fps_avg = alpha * instant_fps + (1 - alpha) * self.measured_fps_avg

                        # Log FPS periodically for debugging
                        if capture_start_time - getattr(self, '_last_fps_log_time', 0) > 5.0: # Log every 5s
                             avg_fps_str = f", Avg: {self.measured_fps_avg:.2f}" if self.measured_fps_avg is not None else ""
                             logging.info(f"Capture Time Diff: {time_diff:.4f}s (Instant FPS: {instant_fps:.2f}{avg_fps_str})")
                             self._last_fps_log_time = capture_start_time
                    else:
                         logging.warning(f"Capture time difference too small or negative: {time_diff:.5f}s")

                self.last_capture_time = capture_end_time # Update last capture time

            except Exception as e0:
                logging.error(f"!!! Error during camera capture: {e0}")
                # Update last error only if it changed to avoid spamming logs
                if self.last_error != f"Camera Capture Error: {e0}":
                     self.last_error = f"Camera Capture Error: {e0}"
                self.last_capture_time = None # Reset timing on error
                self.measured_fps_avg = None # Reset measured FPS on error
        else:
             # Camera not ready, do nothing
             pass

        # --- Prepare Output Frame ---
        output_frame_for_stream = frame # In single camera mode, output is just the captured frame

        # --- Update Shared Frame ---
        with self.frame_lock:
            # Store a copy for the streaming thread
            self.output_frame = output_frame_for_stream.copy() if output_frame_for_stream is not None else None

        # --- Write Frame to Video Files (if recording) ---
        # This part needs to happen *after* the frame is captured and potentially updated
        if self.is_recording and capture_successful and frame is not None:
            force_stop_recording = False
            with self.recording_lock:
                # Check again if recording is active inside the lock
                if not self.is_recording:
                    return output_frame_for_stream # Exit if recording was stopped concurrently

                if not self.video_writers:
                    logging.warning("Recording is True, but VideoWriter list is empty. Forcing stop.")
                    self.last_error = "Rec stopped: writer list empty."
                    force_stop_recording = True
                else:
                    write_errors = 0
                    for i, writer in enumerate(self.video_writers):
                        try:
                            writer.write(frame) # Write the captured frame
                        except Exception as e:
                            path_str = self.recording_paths[i] if i < len(self.recording_paths) else f"Writer {i}"
                            logging.error(f"!!! Failed write frame to {path_str}: {e}")
                            write_errors += 1
                            # Update last error only if it changed
                            if "write error" not in (self.last_error or ""):
                                 self.last_error = f"Frame write error: {os.path.basename(path_str)}"

                    # If all writers failed, stop recording
                    if write_errors > 0 and write_errors == len(self.video_writers):
                        logging.error("All video writers failed to write frame. Stopping recording.")
                        self.last_error = "Rec stopped: All writers failed."
                        force_stop_recording = True
            # End recording lock

            # Schedule stop outside the lock if needed
            if force_stop_recording:
                logging.info("Scheduling recording stop due to writer failure.")
                # Use a timer to avoid potential deadlocks if stop_recording needs the recording_lock
                threading.Timer(0.1, self.stop_recording).start()

        return output_frame_for_stream


    def get_latest_frame(self):
        """Returns the latest captured frame for streaming (thread-safe)."""
        with self.frame_lock:
            if self.output_frame is not None:
                return self.output_frame.copy() # Return a copy
            else:
                return None

    # --- Audio Methods ---
    # (Audio methods remain largely unchanged, ensure they use correct config)
    def _find_audio_device(self):
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
                # Check for input channels, hint, and exclude likely output devices
                if device['max_input_channels'] > 0 and \
                   config.AUDIO_DEVICE_HINT.lower() in device['name'].lower() and \
                   'output' not in device['name'].lower(): # Basic check to exclude pure output devices
                    logging.info(f"Found potential audio device: Index {i}, Name: {device['name']}, Inputs: {device['max_input_channels']}, Rate: {device['default_samplerate']}")
                    candidate_indices.append(i)

            if not candidate_indices:
                 logging.warning(f"No audio input device found matching hint '{config.AUDIO_DEVICE_HINT}'.")
                 self.audio_last_error = "Audio Device: Not Found"
                 return False

            # Check if any candidate supports the required settings
            for index in candidate_indices:
                 try:
                     # Ensure audio format from config is valid before checking
                     if not self.audio_dtype:
                          logging.error("Cannot check audio settings: Invalid audio format in config.")
                          self.audio_last_error = "Audio Device: Invalid config format"
                          return False
                     # Check if the device supports the exact settings
                     sd.check_input_settings(device=index, samplerate=config.AUDIO_SAMPLE_RATE, channels=config.AUDIO_CHANNELS, dtype=self.audio_dtype)
                     logging.info(f"Device {index} ('{devices[index]['name']}') supports required settings. Selecting.")
                     self.audio_device_index = index
                     # Clear previous audio device errors if found successfully
                     if "Audio Device" in (self.audio_last_error or ""):
                          self.audio_last_error = None
                     return True
                 except Exception as check_err:
                     logging.warning(f"Device {index} ('{devices[index]['name']}') does not support required settings ({config.AUDIO_SAMPLE_RATE}Hz, {config.AUDIO_CHANNELS}ch, {config.AUDIO_FORMAT}): {check_err}")

            # If loop completes without finding a suitable device
            logging.error(f"Found devices matching hint, but none support required audio settings.")
            self.audio_last_error = "Audio Device: Found but incompatible settings"
            self.audio_device_index = None
            return False

        except Exception as e:
            logging.error(f"Error querying audio devices: {e}", exc_info=True)
            self.audio_last_error = f"Audio Device Query Error: {e}"
            return False

    def _audio_recording_thread(self):
        stream_started_successfully = False
        try:
            logging.info(f"Audio capture thread started for device {self.audio_device_index}.")
            self.stop_audio_event.clear() # Ensure event is clear at start

            # --- Define Audio Callback ---
            def audio_callback(indata, frames, time_info, status):
                """Callback function for the audio stream."""
                if status:
                    logging.warning(f"Audio callback status: {status}")
                # Use non-blocking put with timeout to avoid blocking the audio thread indefinitely
                if self.audio_queue:
                    try:
                         self.audio_queue.put(indata.copy(), block=True, timeout=0.1) # Short timeout
                    except queue.Full:
                         # Log less frequently if queue remains full?
                         if not hasattr(self, '_last_queue_full_log_time') or time.monotonic() - self._last_queue_full_log_time > 5.0:
                              logging.warning("Audio queue is full. Discarding audio data.")
                              self._last_queue_full_log_time = time.monotonic()
                    except Exception as q_err:
                         logging.error(f"Error putting audio data into queue: {q_err}")
            # --- End Callback ---

            # Create and start the input stream
            self.audio_stream = sd.InputStream(
                samplerate=config.AUDIO_SAMPLE_RATE,
                device=self.audio_device_index,
                channels=config.AUDIO_CHANNELS,
                dtype=self.audio_dtype, # Use validated dtype
                blocksize=config.AUDIO_BLOCK_SIZE,
                callback=audio_callback
            )
            self.audio_stream.start()
            stream_started_successfully = True
            logging.info("Audio stream started.")

            # Keep thread alive while stream is active and stop not requested
            while not self.stop_audio_event.is_set():
                # Wait for stop event with timeout
                self.stop_audio_event.wait(timeout=0.2) # Check periodically
                # Check if stream became inactive unexpectedly
                if not self.audio_stream.active:
                     logging.warning("Audio stream became inactive unexpectedly.")
                     if not self.audio_last_error: # Set error only if not already set
                          self.audio_last_error = "Audio stream stopped unexpectedly"
                     break # Exit loop if stream stops

        except sd.PortAudioError as pae:
             # Specific PortAudio errors
             logging.error(f"!!! PortAudioError in audio recording thread: {pae}", exc_info=True)
             self.audio_last_error = f"Audio PortAudioError: {pae}"
        except Exception as e:
            # Catch other potential errors
            logging.error(f"!!! Error in audio recording thread: {e}", exc_info=True)
            if not self.audio_last_error: # Avoid overwriting more specific errors
                 self.audio_last_error = f"Audio Thread Error: {e}"
        finally:
            # Cleanup: Stop and close the stream if it was started
            if self.audio_stream and stream_started_successfully:
                try:
                    if not self.audio_stream.closed:
                         logging.info("Aborting and closing audio stream...")
                         # Use ignore_errors=True for robustness during shutdown
                         self.audio_stream.abort(ignore_errors=True)
                         self.audio_stream.close(ignore_errors=True)
                         logging.info("Audio stream aborted and closed.")
                except Exception as e_close:
                    # Log errors during close but don't crash
                    logging.error(f"Error closing audio stream during cleanup: {e_close}")
            elif self.audio_stream:
                 # This case indicates the stream object was created but start() failed or wasn't reached
                 logging.warning("Audio stream object exists but was not started successfully.")

            logging.info("Audio capture thread finished.")
            # Signal the write thread that capture is done by putting None in the queue
            if self.audio_queue:
                 try:
                      # Use non-blocking put as write thread might be blocked or finished
                      self.audio_queue.put(None, block=False)
                 except queue.Full:
                      logging.warning("Could not add sentinel to full audio queue during cleanup.")
                 except Exception as q_err:
                      logging.error(f"Error adding sentinel to queue during cleanup: {q_err}")

    def _audio_write_file_thread(self):
        sound_file = None
        data_written = False
        items_processed = 0
        try:
            # Pre-checks
            if not self.temp_audio_file_path:
                logging.error("Audio write thread: Temp audio file path not set."); return
            if not self.audio_dtype:
                 logging.error("Audio write thread: Audio dtype not set."); return

            # Determine soundfile subtype based on numpy dtype
            subtype = None
            if self.audio_dtype == np.int16: subtype = 'PCM_16'
            elif self.audio_dtype == np.int32: subtype = 'PCM_32'
            elif self.audio_dtype == np.float32: subtype = 'FLOAT'
            elif self.audio_dtype == np.int8: subtype = 'PCM_S8' # Check if soundfile supports this
            elif self.audio_dtype == np.uint8: subtype = 'PCM_U8' # Check if soundfile supports this
            # Add other mappings if needed

            if subtype is None:
                 logging.error(f"Audio write thread: Unsupported audio format '{config.AUDIO_FORMAT}' ({self.audio_dtype}) for soundfile. Cannot write WAV.")
                 self.audio_last_error = f"Audio Write Error: Unsupported format {config.AUDIO_FORMAT}"
                 return

            logging.info(f"Audio write thread: Opening {self.temp_audio_file_path} (subtype: {subtype}) for writing.")
            # Open the sound file for writing
            sound_file = sf.SoundFile(
                self.temp_audio_file_path, mode='w',
                samplerate=config.AUDIO_SAMPLE_RATE,
                channels=config.AUDIO_CHANNELS,
                subtype=subtype
            )
            logging.info(f"Audio write thread: File opened successfully.")

            # Process data from the queue
            while True:
                try:
                    # Block with a timeout to allow checking the stop event
                    audio_data = self.audio_queue.get(block=True, timeout=0.5) # Wait up to 0.5s

                    # Check for stop sentinel
                    if audio_data is None:
                        logging.info(f"Audio write thread: Received stop sentinel after processing {items_processed} blocks.");
                        break # Exit the loop gracefully

                    # Write data if it's valid
                    if isinstance(audio_data, np.ndarray):
                        sound_file.write(audio_data)
                        data_written = True
                        items_processed += 1
                    else:
                         # Log unexpected data types
                         logging.warning(f"Audio write thread: Received non-numpy data from queue: {type(audio_data)}")

                except queue.Empty:
                    # Timeout occurred, check if stop was requested
                    if self.stop_audio_event.is_set():
                        logging.info(f"Audio write thread: Stop event detected during queue wait after processing {items_processed} blocks.");
                        break # Exit loop if stop requested
                    # Otherwise, continue waiting for data
                    continue
                except sf.SoundFileError as sf_err:
                     # Handle soundfile specific errors during write
                     logging.error(f"!!! SoundFileError writing audio data: {sf_err}", exc_info=True)
                     self.audio_last_error = f"Audio File Write Error: {sf_err}"
                     break # Exit loop on write error
                except Exception as write_err:
                     # Handle other unexpected errors during write
                     logging.error(f"!!! Unexpected error writing audio data: {write_err}", exc_info=True)
                     self.audio_last_error = f"Audio File Write Error: {write_err}"
                     break # Exit loop on write error

        except sf.SoundFileError as sf_open_err:
             # Handle errors opening the file
             logging.error(f"!!! SoundFileError opening {self.temp_audio_file_path}: {sf_open_err}", exc_info=True)
             self.audio_last_error = f"Audio File Open Error: {sf_open_err}"
        except Exception as e:
            # Handle errors during setup
            logging.error(f"!!! Error in audio write thread setup/loop: {e}", exc_info=True)
            self.audio_last_error = f"Audio Write Thread Error: {e}"
        finally:
            # Ensure the file is closed
            if sound_file:
                try:
                    logging.info(f"Audio write thread: Closing {self.temp_audio_file_path}...")
                    sound_file.close()
                    logging.info(f"Audio write thread: Closed sound file.")
                except Exception as e_close:
                    logging.error(f"Error closing sound file: {e_close}")

            # Post-write checks
            file_exists = self.temp_audio_file_path and os.path.exists(self.temp_audio_file_path)
            if file_exists and not data_written:
                 # File exists but nothing was written (potentially header only)
                 logging.warning(f"Audio write thread finished, but no data seems to have been written to {self.temp_audio_file_path}.")
            elif not file_exists and data_written:
                 # Data was processed, but file is missing - indicates a problem
                 logging.error("Audio write thread: Data was processed, but the output file does not exist!")
                 if not self.audio_last_error: # Set error if not already set
                     self.audio_last_error = "Audio file missing after write"

            logging.info("Audio write thread finished.")

    def _start_audio_recording(self, base_filename):
        # Check if enabled
        if not config.AUDIO_ENABLED:
            logging.info("Audio recording disabled in config.")
            return False
        # Check if format is valid
        if not self.audio_dtype:
            logging.error("Cannot start audio recording: Invalid audio format in config.")
            self.audio_last_error = "Audio Start Failed: Invalid config format"
            return False

        # --- Cleanup previous state ---
        # Stop existing threads if any are running
        if self.audio_thread and self.audio_thread.is_alive():
             logging.warning("Audio start called while previous capture thread alive. Attempting stop first.")
             self._stop_audio_recording() # Ensure clean stop before starting new
        elif self.audio_write_thread and self.audio_write_thread.is_alive():
             logging.warning("Audio start called while previous write thread alive. Attempting stop first.")
             self._stop_audio_recording()
        # Clear the queue
        while not self.audio_queue.empty():
            try: self.audio_queue.get_nowait()
            except queue.Empty: break
            except Exception: pass # Ignore errors clearing queue

        # Find a suitable audio device
        if not self._find_audio_device() or self.audio_device_index is None:
            logging.error(f"Audio Start Failed: Could not find suitable audio device. Last error: {self.audio_last_error}")
            # self.audio_last_error should be set by _find_audio_device
            return False

        # --- Setup and Start Threads ---
        try:
            # Define temporary file path
            temp_dir = tempfile.gettempdir()
            self.temp_audio_file_path = os.path.join(temp_dir, f"{base_filename}_audio{config.AUDIO_TEMP_EXTENSION}")
            # Remove existing temp file if it exists
            if os.path.exists(self.temp_audio_file_path):
                 logging.warning(f"Removing existing temp audio file: {self.temp_audio_file_path}")
                 os.remove(self.temp_audio_file_path)

            logging.info(f"Starting audio recording. Temp file: {self.temp_audio_file_path}")

            self.stop_audio_event.clear() # Ensure stop event is clear

            # Start capture thread
            self.audio_thread = threading.Thread(target=self._audio_recording_thread, name="AudioCaptureThread")
            self.audio_thread.daemon = True # Allow exit even if thread hangs
            self.audio_thread.start()

            time.sleep(0.2) # Allow capture thread to initialize

            # Start write thread
            self.audio_write_thread = threading.Thread(target=self._audio_write_file_thread, name="AudioWriteThread")
            self.audio_write_thread.daemon = True
            self.audio_write_thread.start()

            time.sleep(0.5) # Allow threads to fully start and potentially fail early

            # Check if threads started successfully
            if not self.audio_thread.is_alive() or not self.audio_write_thread.is_alive():
                 logging.error("Audio threads did not remain alive shortly after start.")
                 if not self.audio_last_error: # Set error if not already set
                     self.audio_last_error = "Audio threads failed to start/stay alive"
                 # Ensure threads are stopped and resources cleaned up
                 self.stop_audio_event.set()
                 if self.audio_thread.is_alive(): self.audio_thread.join(timeout=1.0)
                 if self.audio_write_thread.is_alive(): self.audio_write_thread.join(timeout=1.0)
                 # Clean up temp file if created
                 if self.temp_audio_file_path and os.path.exists(self.temp_audio_file_path):
                     try: os.remove(self.temp_audio_file_path)
                     except OSError: pass
                 # Reset state variables
                 self.temp_audio_file_path = None; self.audio_thread = None; self.audio_write_thread = None
                 return False

            # If threads are alive, start was successful
            self.audio_last_error = None # Clear previous audio errors on success
            logging.info("Audio recording threads started successfully.")
            return True

        except Exception as e:
            # Catch errors during setup phase
            logging.error(f"!!! Failed to start audio recording setup: {e}", exc_info=True)
            self.audio_last_error = f"Audio Start Error: {e}"
            # Ensure cleanup on error
            self.stop_audio_event.set()
            if self.audio_thread and self.audio_thread.is_alive(): self.audio_thread.join(timeout=1.0)
            if self.audio_write_thread and self.audio_write_thread.is_alive(): self.audio_write_thread.join(timeout=1.0)
            if self.temp_audio_file_path and os.path.exists(self.temp_audio_file_path):
                 try: os.remove(self.temp_audio_file_path)
                 except OSError: pass
            self.temp_audio_file_path = None; self.audio_thread = None; self.audio_write_thread = None
            return False

    def _stop_audio_recording(self):
        # Check if enabled
        if not config.AUDIO_ENABLED: return None

        capture_thread_running = self.audio_thread and self.audio_thread.is_alive()
        write_thread_running = self.audio_write_thread and self.audio_write_thread.is_alive()
        temp_file_path_at_start = self.temp_audio_file_path # Store path before resetting

        # If threads aren't running, nothing to stop
        if not capture_thread_running and not write_thread_running:
            logging.info("Audio recording threads were not running.")
            # Check if a potentially valid temp file exists from a previous run
            if temp_file_path_at_start and os.path.exists(temp_file_path_at_start):
                 try:
                     # Check if file has more than just a header (simple size check)
                     if os.path.getsize(temp_file_path_at_start) > 44: # WAV header is typically 44 bytes
                          logging.info(f"Found potentially valid existing temp audio file: {temp_file_path_at_start}")
                          self.temp_audio_file_path = None # Clear path after returning it
                          return temp_file_path_at_start
                     else:
                          # Clean up empty/invalid file
                          logging.info(f"Cleaning up empty/invalid temp audio file: {temp_file_path_at_start}")
                          os.remove(temp_file_path_at_start)
                 except OSError as e:
                      logging.warning(f"Could not check/remove temp audio file {temp_file_path_at_start}: {e}")
            # Reset path and return None if no valid file found
            self.temp_audio_file_path = None
            return None

        # --- Stop Running Threads ---
        logging.info("Stopping audio recording threads...")
        self.stop_audio_event.set() # Signal threads to stop

        # Wait for threads to join with timeouts
        if capture_thread_running:
            logging.debug("Waiting for audio capture thread to join...")
            self.audio_thread.join(timeout=2.0) # Adjust timeout as needed
            if self.audio_thread.is_alive():
                logging.warning("Audio capture thread did not stop cleanly within timeout.")
            else:
                 logging.debug("Audio capture thread joined.")

        if write_thread_running:
            logging.debug("Waiting for audio write thread to join...")
            # Give write thread more time to finish writing buffered data
            self.audio_write_thread.join(timeout=5.0)
            if self.audio_write_thread.is_alive():
                logging.warning("Audio write thread did not stop cleanly within timeout.")
            else:
                 logging.debug("Audio write thread joined.")

        # Reset thread and stream variables
        self.audio_thread = None
        self.audio_write_thread = None
        self.audio_stream = None # Ensure stream object is cleared

        # --- Check and Return Temp File ---
        if temp_file_path_at_start and os.path.exists(temp_file_path_at_start):
             try:
                 file_size = os.path.getsize(temp_file_path_at_start)
                 # Check if file has more than just a header
                 if file_size > 44:
                     logging.info(f"Audio stop successful. Temp file ready: {temp_file_path_at_start} (Size: {file_size} bytes)")
                     self.temp_audio_file_path = None # Clear path after returning
                     return temp_file_path_at_start
                 else:
                     # File is likely empty or invalid, clean it up
                     logging.warning(f"Temporary audio file {temp_file_path_at_start} seems empty/invalid after stop (Size: {file_size}).")
                     try:
                          os.remove(temp_file_path_at_start)
                          logging.info("Cleaned up empty/invalid temp audio file.")
                     except OSError as e:
                          logging.warning(f"Could not remove empty temp audio file: {e}")
                     self.temp_audio_file_path = None # Clear path
                     return None
             except OSError as e:
                  # Error accessing the file
                  logging.error(f"Error checking temp audio file size {temp_file_path_at_start}: {e}")
                  self.temp_audio_file_path = None
                  return None
        else:
            # Temp file path wasn't set or file doesn't exist
            logging.error("Temporary audio file path not set or file does not exist after stop sequence.")
            self.temp_audio_file_path = None
            return None

    def _mux_audio_video(self, video_path, audio_path, recording_fps=None):
        """
        Merges audio and video files using ffmpeg. Uses '-c:v copy'.
        Uses the provided recording_fps to hint the input video rate for sync.
        Adds a small -ss offset to the audio input to compensate for start delay.
        """
        if not config.AUDIO_ENABLED: return None # Should not be called if disabled

        # --- Pre-checks ---
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
        try:
            if os.path.getsize(audio_path) <= 44: # Check for minimal WAV header size
                 logging.error(f"Audio file {audio_path} is too small, likely invalid. Skipping mux.")
                 self.audio_last_error = "Mux Error: Audio file invalid/empty"
                 return None
        except OSError as e:
             logging.error(f"Error checking audio file size {audio_path}: {e}. Skipping mux.")
             self.audio_last_error = f"Mux Error: Cannot check audio file {e}"
             return None

        # --- Define Output Path ---
        # Try to replace '_video.ext' with '.ext', otherwise add '_muxed.ext'
        output_path = video_path.replace("_video" + config.RECORDING_EXTENSION, config.RECORDING_EXTENSION)
        if output_path == video_path: # If replacement didn't happen (e.g., filename mismatch)
            output_path = video_path.replace(config.RECORDING_EXTENSION, "_muxed" + config.RECORDING_EXTENSION)

        logging.info(f"Muxing (copy video) '{os.path.basename(video_path)}' and audio '{os.path.basename(audio_path)}' into '{os.path.basename(output_path)}'...")

        # --- Construct ffmpeg Command ---
        command = [
            config.FFMPEG_PATH,
            "-y", # Overwrite output file without asking
        ]

        # --- Add Input Frame Rate Hint (BEFORE video input) ---
        if recording_fps is not None and recording_fps > 0:
            logging.info(f"Hinting input video frame rate to ffmpeg: {recording_fps:.4f} fps")
            command.extend(["-r", f"{recording_fps:.4f}"])
        else:
            logging.warning("No valid recording FPS available to hint ffmpeg. Sync might be affected.")

        # --- Add Video Input ---
        command.extend(["-i", video_path])

        # --- Add Audio Input Offset (BEFORE audio input) ---
        # Introduce a small offset to compensate for potential audio start delay.
        # Adjust this value (in seconds) if sync is still off.
        audio_start_offset = 1 # Start reading audio 0.1 seconds in
        logging.info(f"Applying {audio_start_offset}s start offset to audio input.")
        command.extend(["-ss", str(audio_start_offset)])

        # --- Add Audio Input ---
        command.extend(["-i", audio_path])

        # --- Add Output Options ---
        command.extend([
            "-map", "0:v:0",    # Map video stream from first input (index 0)
            "-map", "1:a:0",    # Map audio stream from second input (index 1)
            "-c:v", "copy",     # Copy video stream without re-encoding
            "-c:a", "aac",      # Encode audio to AAC (common, adjust if needed)
            "-b:a", "128k",     # Audio bitrate (optional, adjust as needed)
            "-shortest",        # Finish encoding when the shortest input stream ends
            "-loglevel", config.FFMPEG_LOG_LEVEL, # Control ffmpeg verbosity
            output_path         # Output file
        ])

        logging.debug(f"Executing ffmpeg command: {' '.join(command)}")

        # --- Execute ffmpeg ---
        mux_start_time = time.monotonic()
        # Use timeout from config (original multiplier was 1)
        timeout = config.AUDIO_MUX_TIMEOUT * config.AUDIO_MUX_RECODE_TIMEOUT_MULTIPLIER
        try:
            # Run ffmpeg process
            process = subprocess.run(command, capture_output=True, text=True, check=True, timeout=timeout)
            mux_duration = time.monotonic() - mux_start_time
            logging.info(f"ffmpeg muxing (copy) successful for {output_path} in {mux_duration:.2f}s.")
            # Log ffmpeg output if debugging is enabled
            if logging.getLogger().isEnabledFor(logging.DEBUG):
                 logging.debug(f"ffmpeg output:\n--- stdout ---\n{process.stdout}\n--- stderr ---\n{process.stderr}\n---")
            # Clear previous muxing errors on success
            if "Mux Error" in (self.audio_last_error or ""):
                 self.audio_last_error = None
            return output_path # Return the path of the successfully muxed file

        except subprocess.TimeoutExpired:
            mux_duration = time.monotonic() - mux_start_time
            logging.error(f"!!! ffmpeg muxing (copy) timed out ({timeout}s) for {output_path} after {mux_duration:.1f}s.")
            self.audio_last_error = "Mux Error: ffmpeg copy timed out"
        except subprocess.CalledProcessError as e:
            # Handle errors reported by ffmpeg
            mux_duration = time.monotonic() - mux_start_time
            logging.error(f"!!! ffmpeg muxing (copy) failed for {output_path} after {mux_duration:.1f}s. Return Code: {e.returncode}")
            # Log ffmpeg's stderr for debugging
            logging.error(f"ffmpeg stderr:\n{e.stderr}")
            logging.error(f"ffmpeg stdout:\n{e.stdout}") # Also log stdout
            self.audio_last_error = f"Mux Error: ffmpeg copy failed (code {e.returncode})"
        except Exception as e:
            # Handle other unexpected errors during execution
            mux_duration = time.monotonic() - mux_start_time
            logging.error(f"!!! Unexpected error during ffmpeg copy execution after {mux_duration:.1f}s: {e}", exc_info=True)
            self.audio_last_error = f"Mux Error: {e}"

        # --- Cleanup Failed Output ---
        # If muxing failed, try to remove the potentially incomplete output file
        if os.path.exists(output_path):
             try:
                 logging.warning(f"Attempting to remove failed mux output file: {output_path}")
                 os.remove(output_path)
             except OSError as rm_err:
                  logging.error(f"Could not remove failed mux output {output_path}: {rm_err}")

        return None # Return None if muxing failed


    def shutdown(self):
        """Stops recording, stops and closes the camera cleanly."""
        logging.info("--- CameraManager Shutting Down ---")

        # 1. Stop Recording (handles audio stop internally)
        if self.is_recording:
            logging.info("Shutdown: Stopping active recording...")
            self.stop_recording()
        else:
            # Ensure any orphaned audio resources are cleaned up
            if config.AUDIO_ENABLED:
                 # Check if threads are alive even if is_recording is False
                 if (self.audio_thread and self.audio_thread.is_alive()) or \
                    (self.audio_write_thread and self.audio_write_thread.is_alive()):
                     logging.warning("Shutdown: Stopping orphaned audio recording...")
                     self._stop_audio_recording() # Attempt cleanup
                 # Check for orphaned temp file
                 elif self.temp_audio_file_path and os.path.exists(self.temp_audio_file_path):
                      try:
                           logging.warning(f"Shutdown: Removing orphaned temp audio file: {self.temp_audio_file_path}")
                           os.remove(self.temp_audio_file_path)
                           self.temp_audio_file_path = None
                      except OSError as e:
                           logging.error(f"Could not remove orphaned temp audio file: {e}")


        # 2. Stop and Close Camera
        cam_name = f"Cam{config.CAMERA_ID}"
        if self.picam:
            logging.info(f"Shutdown: Stopping and closing {cam_name}...")
            try:
                if self.picam.started:
                    self.picam.stop()
                    logging.info(f"{cam_name} stopped.")
                self.picam.close()
                logging.info(f"{cam_name} closed.")
            except Exception as e:
                logging.error(f"Error stopping/closing {cam_name} during shutdown: {e}")
            finally:
                self.picam = None
                self.is_initialized = False
        else:
             if self.is_initialized: # Should not happen if instance is None
                 logging.warning(f"Shutdown: {cam_name} instance was None, but expected to be initialized.")
             else:
                 logging.debug(f"Shutdown: {cam_name} instance already None.")


        # Reset other state variables
        self.actual_fps = None
        self.last_capture_time = None
        self.measured_fps_avg = None
        self.recording_fps = None
        with self.frame_lock: self.output_frame = None # Clear shared frame
        logging.info("--- CameraManager Shutdown Complete ---")

# Example Usage (Main testing block - adapted for single camera)
if __name__ == "__main__":
    # Basic logging setup for testing
    log_level_str = os.environ.get("LOG_LEVEL", "DEBUG")
    log_level = getattr(logging, log_level_str.upper(), logging.INFO)
    logging.basicConfig(level=log_level, format=config.LOG_FORMAT, datefmt=config.LOG_DATE_FORMAT)

    logging.info("--- Camera Manager Single Cam Test (Sync Fix) ---")
    cam_manager = CameraManager()
    test_failed = False

    try:
        # --- Initialization Test ---
        print("\n--- Testing Camera Initialization ---")
        if not cam_manager.initialize_camera():
            print(f"!!! FATAL: Camera initialization failed: {cam_manager.last_error}")
            test_failed = True; raise SystemExit("Exiting due to init failure.")
        print("Camera initialized successfully.")
        print(f"Initial State: {cam_manager.get_camera_state()}")
        print("------------------------------------")

        # --- Frame Capture Test ---
        print("\n--- Testing Frame Capture & Display (5s) ---")
        start_capture_test = time.monotonic()
        frames_captured = 0
        while time.monotonic() - start_capture_test < 5.0:
            frame = cam_manager.capture_frame() # Use updated method name
            if frame is not None:
                 frames_captured += 1
                 cv2.imshow("Stream Test", frame)
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
                     frame = cam_manager.capture_frame() # Use updated method name
                     if frame is None:
                         print("!!! Capture failed during recording test.")
                     else:
                          rec_frames += 1
                          #cv2.imshow("Recording Test", frame) # Optional: display during recording
                     if cv2.waitKey(1) & 0xFF == ord('q'):
                         print("Quit requested during recording.")
                         break
                 print(f"Stopping recording after ~5s ({rec_frames} frames processed)...")
                 cam_manager.stop_recording()
                 print(f"Recording stopped and muxing finished (if audio enabled). Check USB drive(s) in {config.USB_BASE_PATH}")
                 print(f"Final state after recording: {cam_manager.get_camera_state()}")
             else:
                 print(f"!!! Failed to start recording. Error: {cam_manager.last_error} / Audio: {cam_manager.audio_last_error}")
                 test_failed = True
        else:
             print("!!! No writable USB drives found, skipping recording test.")
             test_failed = True
        cv2.destroyAllWindows()
        print("------------------------------------")

        # --- Control Change Test (Example: Brightness) ---
        print("\n--- Testing Control Change (Brightness) ---")
        print("Setting brightness to 0.5...")
        if cam_manager.apply_camera_controls({"Brightness": 0.5}):
             print("Brightness set. Capturing frame...")
             time.sleep(1) # Allow control to apply
             frame = cam_manager.capture_frame()
             if frame is not None: cv2.imshow("Brightness 0.5", frame); cv2.waitKey(2000)
             else: print("Capture failed after brightness change.")

             print("Setting brightness back to default...")
             if cam_manager.apply_camera_controls({"Brightness": config.DEFAULT_BRIGHTNESS}):
                 print("Brightness reset. Capturing frame...")
                 time.sleep(1)
                 frame = cam_manager.capture_frame()
                 if frame is not None: cv2.imshow("Brightness Default", frame); cv2.waitKey(2000)
                 else: print("Capture failed after brightness reset.")
             else: print(f"!!! Failed to reset brightness: {cam_manager.last_error}"); test_failed = True
        else: print(f"!!! Failed to set brightness: {cam_manager.last_error}"); test_failed = True
        cv2.destroyAllWindows()
        print("------------------------------------")

        # --- Resolution Change Test ---
        print("\n--- Testing Resolution Change ---")
        initial_index = cam_manager.current_resolution_index
        target_index = 0 if initial_index != 0 else 1 # Choose a different index
        if target_index >= len(config.CAMERA_RESOLUTIONS): target_index = 0 # Ensure valid index

        print(f"Changing resolution from index {initial_index} to {target_index}...")
        if cam_manager.initialize_camera(target_index):
             print("Resolution change successful.")
             print(f"New State: {cam_manager.get_camera_state()}")
             print("Capturing frame at new resolution...")
             time.sleep(1)
             frame = cam_manager.capture_frame()
             if frame is not None: cv2.imshow(f"Resolution Index {target_index}", frame); cv2.waitKey(2000)
             else: print("Capture failed after resolution change.")

             print(f"Changing resolution back to index {initial_index}...")
             if cam_manager.initialize_camera(initial_index):
                 print("Resolution reset successful.")
                 print(f"Restored State: {cam_manager.get_camera_state()}")
             else: print(f"!!! Failed to reset resolution: {cam_manager.last_error}"); test_failed = True
        else: print(f"!!! Failed to change resolution: {cam_manager.last_error}"); test_failed = True
        cv2.destroyAllWindows()
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
        cv2.destroyAllWindows() # Ensure all OpenCV windows are closed
        print("--- Test Complete ---")
        if test_failed:
             print("!!! One or more tests failed or were skipped due to errors. !!!")
        else:
             print("+++ All tests passed (or skipped gracefully). +++")

