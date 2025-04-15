# -*- coding: utf-8 -*-
"""
camera_manager.py

Manages multiple Picamera2 cameras, including initialization, configuration,
frame capture (combined stream), video recording (from primary camera),
and audio recording/muxing.

Includes fix to use actual reported camera FPS for VideoWriter to prevent
speed mismatch issues in recordings.
"""

import os
import time
import datetime
import logging
import threading
import cv2
import numpy as np
from picamera2 import Picamera2
from libcamera import controls, Transform # Keep commented unless needed

# Audio related imports
import sounddevice as sd
import soundfile as sf
import subprocess
import queue
import tempfile
import numpy # Explicit import for audio dtype check

# Import configuration constants
import config

# Helper function to find writable USB mounts (Unchanged)
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
                test_file = os.path.join(path, f".write_test_{os.getpid()}")
                try:
                    with open(test_file, 'w') as f: f.write('test')
                    os.remove(test_file)
                    mounts.append(path)
                    logging.debug(f"Found writable mount: {path}")
                except Exception as write_err:
                    logging.warning(f"Directory {path} appears mounted but test write failed: {write_err}")
            elif os.path.isdir(path):
                logging.debug(f"Directory {path} found but is not writable.")
        if not mounts: logging.debug("No writable USB mounts found.")
    except Exception as e:
        logging.error(f"Error finding USB mounts in {base_path}: {e}")
    return mounts


class CameraManager:
    """Handles multiple camera operations and state, including audio."""

    def __init__(self):
        """Initializes the CameraManager for two cameras and audio."""
        self.picam0 = None # Primary camera (HQ)
        self.picam1 = None # Secondary camera (IMX219)
        self.is_initialized0 = False
        self.is_initialized1 = False
        self.last_error = None # General/last error message

        # State variables for Cam0 (HQ)
        self.current_resolution_index0 = config.CAM0_DEFAULT_RESOLUTION_INDEX
        self.actual_cam0_fps = None # <<< ADDED: Stores the actual FPS reported by Cam0 driver

        # State variable for combined frame
        self.combined_frame = None # Stores the latest combined frame for streaming
        self.output_frame = None   # Stores the frame ready for retrieval

        # Recording state (only for Cam0)
        self.is_recording = False
        self.video_writers = [] # List of OpenCV VideoWriter objects for Cam0
        self.recording_paths = [] # List of corresponding file paths for Cam0 (video-only initially)

        # Locks for thread safety
        self.frame_lock = threading.Lock() # Protects access to combined_frame/output_frame
        self.config_lock = threading.Lock() # Protects access to camera state/config changes
        self.recording_lock = threading.Lock() # Protects access to recording state/writers

        # --- Initialize Common Camera Controls to Defaults from Config ---
        try:
             self.current_analogue_gain = config.DEFAULT_ANALOGUE_GAIN # Float value
             self.current_iso_name = config.DEFAULT_ISO_NAME
             for name, gain in config.AVAILABLE_ISO_SETTINGS.items():
                 if abs(gain - self.current_analogue_gain) < 0.01: self.current_iso_name = name; break
        except Exception as e: logging.error(f"Error setting default ISO: {e}. Falling back."); self.current_analogue_gain = 1.0; self.current_iso_name = "100"
        try: self.current_ae_mode = controls.AeExposureModeEnum.__members__[config.DEFAULT_AE_MODE_NAME]
        except KeyError: logging.error(f"Default AE mode '{config.DEFAULT_AE_MODE_NAME}' not found! Falling back."); self.current_ae_mode = controls.AeExposureModeEnum.Normal
        try: self.current_metering_mode = controls.AeMeteringModeEnum.__members__[config.DEFAULT_METERING_MODE_NAME]
        except KeyError: logging.error(f"Default Metering mode '{config.DEFAULT_METERING_MODE_NAME}' not found! Falling back."); self.current_metering_mode = controls.AeMeteringModeEnum.CentreWeighted
        try: self.current_noise_reduction_mode = controls.draft.NoiseReductionModeEnum.__members__[config.DEFAULT_NOISE_REDUCTION_MODE_NAME]
        except (AttributeError, KeyError): logging.warning(f"Default NR mode '{config.DEFAULT_NOISE_REDUCTION_MODE_NAME}' not found/unavailable. Setting Off."); self.current_noise_reduction_mode = controls.draft.NoiseReductionModeEnum.Off
        self.current_brightness = config.DEFAULT_BRIGHTNESS; self.current_contrast = config.DEFAULT_CONTRAST
        self.current_saturation = config.DEFAULT_SATURATION; self.current_sharpness = config.DEFAULT_SHARPNESS

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
                self.audio_dtype = np.dtype(config.AUDIO_FORMAT).type
            except TypeError:
                 logging.error(f"Unsupported audio format '{config.AUDIO_FORMAT}' in config. Disabling audio.")
                 config.AUDIO_ENABLED = False

        logging.info("CameraManager initialized with default control settings for dual cameras.")

    def _initialize_camera(self, cam_id, resolution_index=None):
        """Internal helper to initialize or re-initialize a specific camera instance."""
        picam_instance = None
        is_initialized_flag = False
        cam_name = f"Cam{cam_id}"

        # Determine target settings based on camera ID
        if cam_id == config.CAM0_ID:
            if resolution_index is not None:
                if 0 <= resolution_index < len(config.CAM0_RESOLUTIONS):
                    self.current_resolution_index0 = resolution_index
                else:
                    logging.error(f"{cam_name}: Invalid resolution index {resolution_index}. Using current index {self.current_resolution_index0}.")
            res_list = config.CAM0_RESOLUTIONS
            current_res_index = self.current_resolution_index0
            tuning_data = config.CAM0_TUNING_FILE_PATH # Usually None for HQ
            try:
                target_width, target_height, target_fps = res_list[current_res_index]
            except IndexError:
                logging.error(f"{cam_name}: Resolution index {current_res_index} out of bounds for {len(res_list)} resolutions. Falling back to default.")
                current_res_index = config.CAM0_DEFAULT_RESOLUTION_INDEX
                self.current_resolution_index0 = current_res_index
                target_width, target_height, target_fps = res_list[current_res_index]
            # Reset actual FPS when re-initializing Cam0
            self.actual_cam0_fps = None
        elif cam_id == config.CAM1_ID:
            target_width, target_height = config.CAM1_RESOLUTION
            target_fps = config.CAM1_FRAME_RATE
            tuning_data = config.CAM1_TUNING # Might be loaded NoIR tuning
            current_res_index = 0 # Cam1 has fixed resolution
        else:
            logging.error(f"Invalid camera ID {cam_id}"); return False

        logging.info(f"Attempting initialization {cam_name} (ID: {cam_id}) at index {current_res_index} ({target_width}x{target_height} @ {target_fps:.1f}fps)...")

        # --- Close existing instance if present ---
        existing_picam = self.picam0 if cam_id == config.CAM0_ID else self.picam1
        if existing_picam is not None:
            try:
                if existing_picam.started: logging.info(f"Stopping existing {cam_name}..."); existing_picam.stop()
                logging.info(f"Closing existing {cam_name}..."); existing_picam.close()
            except Exception as e: logging.warning(f"Error stopping/closing previous {cam_name}: {e}")
            finally:
                if cam_id == config.CAM0_ID: self.picam0 = None; self.is_initialized0 = False
                else: self.picam1 = None; self.is_initialized1 = False
            time.sleep(0.5) # Give time for resources to release

        # --- Initialize New Instance ---
        try:
            picam_instance = Picamera2(camera_num=cam_id, tuning=tuning_data)
            logging.info(f"{cam_name}: Picamera2 object created.")

            # Prepare controls dictionary
            controls_to_set = {
                "FrameRate": target_fps,
                "NoiseReductionMode": self.current_noise_reduction_mode,
                "AeEnable": True, # Default to True, might be overridden by Auto ISO check
                "AeExposureMode": self.current_ae_mode,
                "AeMeteringMode": self.current_metering_mode,
                "AnalogueGain": self.current_analogue_gain,
                "Brightness": self.current_brightness,
                "Contrast": self.current_contrast,
                "Saturation": self.current_saturation,
                "Sharpness": self.current_sharpness,
            }
            # If Auto ISO is selected (gain 0.0), ensure AE is enabled
            if self.current_analogue_gain == 0.0:
                controls_to_set["AeEnable"] = True
                logging.info(f"{cam_name}: Auto ISO selected (AnalogueGain=0.0). Ensuring AE is enabled.")
            # Remove None values (e.g., if NR mode was unavailable)
            controls_to_set = {k: v for k, v in controls_to_set.items() if v is not None}

            # Setup transform (e.g., for flipping camera 1 if needed)
            transform = Transform() # Add logic here if cam1 needs flipping etc.

            # Create video configuration
            cam_config = picam_instance.create_video_configuration(
                main={"size": (target_width, target_height), "format": "RGB888"},
                controls=controls_to_set,
                transform=transform
            )
            logging.info(f"{cam_name}: Configuring with: main={cam_config['main']}, controls={cam_config['controls']}")

            # Configure the camera
            picam_instance.configure(cam_config)
            time.sleep(0.5) # Allow config to apply

            # Verify applied configuration (optional but good practice)
            new_config = picam_instance.camera_configuration()
            if new_config:
                applied_controls = new_config.get('controls', {})
                applied_main = new_config.get('main', {})
                logging.info(f"{cam_name}: Verified Applied Config: main={applied_main}, controls={applied_controls}")
                actual_fps_v = applied_controls.get('FrameRate')
                if actual_fps_v and abs(actual_fps_v - target_fps) > 0.1:
                    logging.warning(f"{cam_name}: Driver adjusted FrameRate from {target_fps:.1f} to {actual_fps_v:.1f}")
            else:
                logging.warning(f"{cam_name}: Could not get camera configuration after applying.")

            logging.info(f"{cam_name}: Configuration seems successful!")

            # Start the camera stream
            picam_instance.start()
            logging.info(f"{cam_name}: Camera started")
            time.sleep(2.0) # Allow camera to stabilize

            # Final check on actual running configuration
            actual_config = picam_instance.camera_configuration()
            if not actual_config:
                raise RuntimeError(f"{cam_name}: Failed to get camera configuration after start.")

            actual_format = actual_config.get('main', {})
            actual_w = actual_format.get('size', (0,0))[0]
            actual_h = actual_format.get('size', (0,0))[1]
            actual_fmt_str = actual_format.get('format', 'Unknown')
            actual_fps_final_val = actual_config.get('controls', {}).get('FrameRate', None) # Get the actual FPS value
            actual_gain = actual_config.get('controls', {}).get('AnalogueGain', 'N/A')

            logging.info(f"{cam_name}: Initialized. Actual stream: {actual_w}x{actual_h} {actual_fmt_str} @ {actual_fps_final_val} fps. Gain: {actual_gain}")

            # <<< MODIFIED BLOCK: Store actual FPS for Cam0 >>>
            if cam_id == config.CAM0_ID:
                if actual_fps_final_val is not None:
                    try:
                        self.actual_cam0_fps = float(actual_fps_final_val)
                        logging.info(f"{cam_name}: Stored actual FPS: {self.actual_cam0_fps:.2f}")
                    except (ValueError, TypeError):
                         logging.error(f"{cam_name}: Could not convert actual FPS '{actual_fps_final_val}' to float. Falling back.")
                         _, _, configured_fps = self.get_cam0_resolution()
                         self.actual_cam0_fps = configured_fps # Fallback
                else:
                    # Fallback to configured FPS if actual is not reported (less ideal)
                    logging.warning(f"{cam_name}: Could not read actual FPS from controls after start. Falling back to configured FPS for writer.")
                    _, _, configured_fps = self.get_cam0_resolution()
                    self.actual_cam0_fps = configured_fps
            # <<< END MODIFIED BLOCK >>>

            is_initialized_flag = True
            self.last_error = None # Clear previous errors on successful init
            return True

        except Exception as e:
            logging.error(f"!!! Failed to initialize {cam_name} at {target_width}x{target_height}: {e}", exc_info=True)
            self.last_error = f"{cam_name} Init Error: {e}"
            if picam_instance is not None:
                try:
                    if picam_instance.started: picam_instance.stop()
                    picam_instance.close()
                except Exception as close_e: logging.error(f"Error closing {cam_name} after init failure: {close_e}")
            picam_instance = None
            is_initialized_flag = False
            if cam_id == config.CAM0_ID: self.actual_cam0_fps = None # Ensure actual FPS is reset on failure
            return False
        finally:
            # Update the correct instance variable and flag
            if cam_id == config.CAM0_ID:
                self.picam0 = picam_instance
                self.is_initialized0 = is_initialized_flag
            else:
                self.picam1 = picam_instance
                self.is_initialized1 = is_initialized_flag

    def initialize_cameras(self, resolution_index=None):
        """Initializes or re-initializes both cameras."""
        with self.config_lock:
            logging.info("--- Initializing Both Cameras ---")
            success0 = self._initialize_camera(config.CAM0_ID, resolution_index)
            success1 = self._initialize_camera(config.CAM1_ID) # Cam1 uses fixed settings

            if success0 and success1:
                logging.info("--- Both Cameras Initialized Successfully ---")
                return True
            else:
                logging.error("!!! Failed to initialize one or both cameras. Check logs. !!!")
                # self.last_error should contain details from _initialize_camera
                return False

    def get_cam0_resolution(self):
        """Returns the current configured resolution tuple (width, height, fps) for Cam0."""
        try:
            return config.CAM0_RESOLUTIONS[self.current_resolution_index0]
        except IndexError:
            logging.error(f"Invalid resolution index {self.current_resolution_index0} for Cam0.")
            # Return a safe default if index is out of bounds
            safe_default_index = max(0, min(len(config.CAM0_RESOLUTIONS) - 1, config.CAM0_DEFAULT_RESOLUTION_INDEX))
            return config.CAM0_RESOLUTIONS[safe_default_index]

    def apply_camera_controls(self, controls_dict):
        """Applies a dictionary of common controls to BOTH running cameras."""
        if not self.is_initialized0 and not self.is_initialized1:
            logging.error("Cannot apply controls: No cameras initialized.")
            self.last_error = "Control Apply Error: No cameras ready."
            return False

        logging.info(f"Applying common controls to cameras: {controls_dict}")
        success_count = 0
        applied_to_cam0 = False
        applied_to_cam1 = False
        try:
            with self.config_lock:
                # Update internal state first
                iso_name_updated = False
                for key, value in controls_dict.items():
                    if key == 'AnalogueGain':
                        self.current_analogue_gain = value
                        # Update ISO name based on applied gain
                        self.current_iso_name = "Unknown" # Reset name
                        for name, gain in config.AVAILABLE_ISO_SETTINGS.items():
                             if abs(gain - value) < 0.01: # Compare floats carefully
                                 self.current_iso_name = name
                                 iso_name_updated = True
                                 break
                        if not iso_name_updated:
                            logging.warning(f"Applied AnalogueGain {value} did not match any known ISO name.")
                    elif key == 'AeExposureMode': self.current_ae_mode = value
                    elif key == 'AeMeteringMode': self.current_metering_mode = value
                    elif key == 'NoiseReductionMode': self.current_noise_reduction_mode = value
                    elif key == 'Brightness': self.current_brightness = value
                    elif key == 'Contrast': self.current_contrast = value
                    elif key == 'Saturation': self.current_saturation = value
                    elif key == 'Sharpness': self.current_sharpness = value
                    # Add other controls if needed

                # Apply to Cam0 if ready
                logging.debug(f"Applying to Cam0...")
                if self.is_initialized0 and self.picam0 and self.picam0.started:
                    try:
                        self.picam0.set_controls(controls_dict)
                        logging.debug("Cam0 controls applied.")
                        applied_to_cam0 = True
                        success_count += 1
                    except Exception as e0:
                        logging.error(f"!!! Error applying controls to Cam0: {e0}")
                        self.last_error = f"Cam0 Control Error: {e0}"
                else:
                    logging.warning("Skipping control application for Cam0 (not ready).")

                # Apply to Cam1 if ready
                logging.debug(f"Applying to Cam1...")
                if self.is_initialized1 and self.picam1 and self.picam1.started:
                    try:
                        self.picam1.set_controls(controls_dict)
                        logging.debug("Cam1 controls applied.")
                        applied_to_cam1 = True
                        success_count += 1
                    except Exception as e1:
                        logging.error(f"!!! Error applying controls to Cam1: {e1}")
                        # Append error message if one already exists
                        self.last_error = (self.last_error or "") + (self.last_error and " / " or "") + f"Cam1 Control Error: {e1}"
                else:
                    logging.warning("Skipping control application for Cam1 (not ready).")

            # Allow time for controls to take effect (especially exposure/gain)
            if success_count > 0:
                if any(k in controls_dict for k in ['AnalogueGain', 'AeExposureMode', 'AeMeteringMode', 'Brightness']):
                    time.sleep(0.5) # Longer delay for exposure-related changes
                else:
                    time.sleep(0.1) # Shorter delay for others
                # Clear error if controls applied successfully to at least one camera
                if (applied_to_cam0 and self.is_initialized0) or (applied_to_cam1 and self.is_initialized1):
                    self.last_error = None
                return True
            else:
                # If no cameras were ready or application failed on all ready cameras
                if (not self.last_error): # Set a generic error if none was set during application
                       self.last_error = "Control Apply Error: Failed on all ready cameras."
                return False

        except Exception as e:
            logging.error(f"!!! Unexpected Error during control application: {e}", exc_info=True)
            self.last_error = f"Control Apply Unexpected Error: {e}"
            return False
        # return False # Should not be reached ideally

    def get_camera_state(self):
        """Returns a dictionary containing the current camera state and common control values."""
        with self.config_lock:
            res_w, res_h, res_fps_config = self.get_cam0_resolution() # Configured FPS
            state = {
                'is_initialized': self.is_initialized0 and self.is_initialized1,
                'is_initialized0': self.is_initialized0,
                'is_initialized1': self.is_initialized1,
                'resolution_index': self.current_resolution_index0,
                'resolution_wh': (res_w, res_h),
                'resolution_fps': self.actual_cam0_fps if self.actual_cam0_fps else res_fps_config, # Report actual FPS if available
                'is_recording': self.is_recording,
                'recording_paths': list(self.recording_paths), # Return a copy
                'last_error': self.last_error,
                'audio_last_error': self.audio_last_error,
                # --- Control Values ---
                'iso_mode': self.current_iso_name, # Report the friendly name
                'analogue_gain': self.current_analogue_gain, # Report the float value too
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
        """Starts recording video FROM CAM0 and audio (if enabled). Uses ACTUAL Cam0 FPS."""
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
                logging.warning(f"Cannot start recording: No writable USB drives in {config.USB_BASE_PATH}.")
                self.last_error = f"Cannot start recording: No writable USB drives found"
                return False

            # Clear previous state just in case
            self.video_writers.clear()
            self.recording_paths.clear()
            success_count = 0
            start_error = None

            try:
                # Get dimensions from config, but FPS from actual stored value
                width, height, _ = self.get_cam0_resolution() # Get dimensions

                # <<< MODIFIED: Use actual FPS >>>
                fps_for_writer = self.actual_cam0_fps
                if fps_for_writer is None or fps_for_writer <= 0:
                    logging.error("Cannot start recording, actual Cam0 FPS is invalid or not set. Trying configured FPS as fallback.")
                    _, _, configured_fps = self.get_cam0_resolution() # Fallback to configured
                    fps_for_writer = configured_fps
                    if fps_for_writer is None or fps_for_writer <= 0:
                         # If both actual and configured are invalid, fail
                         raise ValueError("Invalid Cam0 FPS (actual and configured). Cannot determine recording FPS.")

                if width <= 0 or height <= 0:
                    raise ValueError(f"Invalid Cam0 dimensions: {width}x{height}")

                logging.info(f"Starting Cam0 recording: {width}x{height} @ {fps_for_writer:.2f} fps (Using {'actual' if self.actual_cam0_fps else 'configured fallback'} FPS)")
                fourcc = cv2.VideoWriter_fourcc(*config.CAM0_RECORDING_FORMAT)
                timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

                # Try creating writer for each USB drive
                for drive_path in usb_drives:
                    try:
                        base_filename = f"recording_{timestamp}_{width}x{height}"
                        # Create video-only filename initially, muxing will rename later
                        video_only_filename = f"{base_filename}_video{config.CAM0_RECORDING_EXTENSION}"
                        full_path = os.path.join(drive_path, video_only_filename)

                        # <<< USE fps_for_writer HERE >>>
                        writer = cv2.VideoWriter(full_path, fourcc, fps_for_writer, (width, height))

                        if not writer.isOpened():
                            raise IOError(f"Failed to open VideoWriter for path: {full_path}")

                        self.video_writers.append(writer)
                        self.recording_paths.append(full_path) # Store the video-only path for now
                        logging.info(f"Started video recording to: {full_path}")
                        success_count += 1
                    except Exception as e:
                        logging.error(f"!!! Failed create VideoWriter for {drive_path}: {e}", exc_info=True)
                        # Store first error encountered during writer creation
                        start_error = start_error or f"Failed writer {os.path.basename(drive_path)}: {e}"

                # Check if at least one writer was successful
                if success_count > 0:
                    self.is_recording = True
                    logging.info(f"Video recording started on {success_count} drive(s).")

                    # Start audio recording if enabled
                    if config.AUDIO_ENABLED:
                        if not self._start_audio_recording(base_filename): # Pass base filename for temp audio file
                            logging.error("Failed start audio recording.")
                            self.audio_last_error = self.audio_last_error or "Audio start failed"
                        else:
                            logging.info("Audio recording started.")
                            self.audio_last_error = None # Clear previous audio error on success

                    # Report partial failure if some writers failed
                    if start_error and success_count < len(usb_drives):
                        self.last_error = f"Partial Rec Start Fail: {start_error}"
                    elif not start_error:
                         # Clear previous video-related errors if start was fully successful
                         if self.last_error and ("Recording" in self.last_error or "writers" in self.last_error or "USB" in self.last_error):
                             logging.info(f"Clearing previous video error: '{self.last_error}'")
                             self.last_error = None
                    return True
                else:
                    # No writers succeeded
                    self.is_recording = False
                    logging.error("Failed start video recording on ANY drive.")
                    self.last_error = f"Rec Start Failed: {start_error or 'No writers opened'}"
                    # Ensure any potentially opened (but failed) writers are released
                    for writer in self.video_writers:
                        try: writer.release()
                        except: pass
                    self.video_writers.clear()
                    self.recording_paths.clear()
                    return False

            except Exception as e:
                logging.error(f"!!! Critical error during recording setup: {e}", exc_info=True)
                self.last_error = f"Recording Setup Error: {e}"
                # Attempt cleanup if setup fails critically
                self.stop_recording() # stop_recording handles writer cleanup
                return False

    def stop_recording(self):
        """Stops video/audio recording, releases writers, muxes files, syncs."""
        # Use separate lock for stopping to avoid deadlocks if called from capture thread
        with self.recording_lock:
            if not self.is_recording:
                # Handle case where stop is called but not recording (e.g., cleanup)
                if self.video_writers or self.recording_paths:
                    logging.warning("stop_recording called when not recording, but writer lists exist. Clearing.")
                    self.video_writers.clear()
                    self.recording_paths.clear()
                # Check if audio thread is orphaned
                if config.AUDIO_ENABLED and (self.audio_thread and self.audio_thread.is_alive()):
                    logging.warning("Stopping orphaned audio recording during stop_recording call...")
                    self._stop_audio_recording() # Attempt cleanup
                return

            logging.info("Stopping recording (Video and Audio)...")
            self.is_recording = False # Set flag immediately

            # Copy lists to avoid modification issues if capture thread writes again briefly
            writers_to_release = list(self.video_writers)
            video_paths_recorded = list(self.recording_paths) # These are the _video paths

            # Clear instance lists
            self.video_writers.clear()
            self.recording_paths.clear() # Clear paths list here

        # --- Stop Audio First ---
        audio_file_to_mux = None
        if config.AUDIO_ENABLED:
            audio_file_to_mux = self._stop_audio_recording() # Gets the temp WAV path
            if audio_file_to_mux:
                logging.info(f"Audio recording stopped. Temporary audio file: {audio_file_to_mux}")
            else:
                logging.error("Audio recording stop failed or was not running.")
                self.audio_last_error = self.audio_last_error or "Audio stop failed"

        # --- Release Video Writers and Mux ---
        logging.info(f"Releasing {len(writers_to_release)} video writer(s)...")
        released_count = 0
        final_output_paths = [] # Store paths of successfully saved/muxed files

        for i, writer in enumerate(writers_to_release):
            # Get corresponding video path
            video_path = video_paths_recorded[i] if i < len(video_paths_recorded) else f"Unknown Path {i}"
            try:
                writer.release()
                logging.info(f"Released video writer for: {video_path}")
                released_count += 1

                # --- Muxing Logic ---
                # Check if audio file exists and the corresponding video file exists
                if audio_file_to_mux and os.path.exists(video_path):
                     final_path = self._mux_audio_video(video_path, audio_file_to_mux)
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
                         # Muxing failed, keep the video-only file
                         logging.error(f"Muxing failed for {video_path}. Keeping video-only file.")
                         final_output_paths.append(video_path) # Keep video path
                         self.last_error = self.last_error or "Muxing Failed" # Set general error
                elif os.path.exists(video_path):
                    # No audio or audio failed, keep video-only path if it exists
                    final_output_paths.append(video_path)

            except Exception as e:
                logging.error(f"Error releasing VideoWriter for {video_path}: {e}", exc_info=True)

        # --- Cleanup Temporary Audio File ---
        if audio_file_to_mux and os.path.exists(audio_file_to_mux):
             try:
                 os.remove(audio_file_to_mux)
                 logging.info(f"Removed temporary audio file: {audio_file_to_mux}")
             except OSError as e:
                 logging.warning(f"Could not remove temporary audio file {audio_file_to_mux}: {e}")

        # --- Sync Filesystem ---
        if released_count > 0 or final_output_paths:
            logging.info("Syncing filesystem to ensure data is written...")
            try:
                sync_start_time = time.monotonic()
                # Using os.system('sync') is common, but consider subprocess for better control if needed
                os.system('sync')
                sync_duration = time.monotonic() - sync_start_time
                logging.info(f"Sync command completed in {sync_duration:.2f}s.")
            except Exception as e:
                logging.error(f"!!! Failed execute 'sync' command: {e}", exc_info=True)
                self.last_error = "Sync failed after recording."
        else:
            logging.warning("No writers were released or final files created, skipping sync.")

        logging.info(f"Recording stopped. Released {released_count} writer(s). Final files created: {len(final_output_paths)}")
        # Note: self.recording_paths is already cleared

    def capture_and_combine_frames(self):
        """Captures frames from both cameras, combines them, writes Cam0 frame if recording."""
        frame0 = None
        frame1 = None
        combined = None
        capture_error = False

        # Capture from Cam0 (Primary)
        if self.is_initialized0 and self.picam0 and self.picam0.started:
            try:
                frame0 = self.picam0.capture_array("main")
                # Reset capture error on success
                if self.last_error and "Cam0 Capture Error" in self.last_error: self.last_error = None
            except Exception as e0:
                logging.error(f"!!! Error Cam0 capture: {e0}")
                self.last_error = f"Cam0 Capture Error: {e0}"
                capture_error = True
        else:
            # Log if expected to be initialized but isn't
            # if self.is_initialized0: logging.warning("Cam0 initialized but instance/started is false during capture.")
            pass # Keep frame0 as None

        # Capture from Cam1 (Secondary)
        if self.is_initialized1 and self.picam1 and self.picam1.started:
            try:
                frame1 = self.picam1.capture_array("main")
                # Reset capture error on success
                if self.last_error and "Cam1 Capture Error" in self.last_error: self.last_error = None
            except Exception as e1:
                logging.error(f"!!! Error Cam1 capture: {e1}")
                # Append error message if one already exists
                self.last_error = (self.last_error or "") + (self.last_error and " / " or "") + f"Cam1 Capture Error: {e1}"
                capture_error = True
        else:
            # if self.is_initialized1: logging.warning("Cam1 initialized but instance/started is false during capture.")
            pass # Keep frame1 as None

        # Combine frames if both are available
        if frame0 is not None and frame1 is not None:
            try:
                h0, w0, _ = frame0.shape
                h1, w1, _ = frame1.shape
                target_w1, target_h1 = config.CAM1_RESOLUTION

                # Resize frame1 if necessary (e.g., if capture size != target or aspect ratio needs fixing)
                # Or if frame1 is larger than frame0 width (unlikely with config)
                if w1 != target_w1 or h1 != target_h1 or w1 > w0:
                    frame1_resized = cv2.resize(frame1, (target_w1, target_h1), interpolation=cv2.INTER_AREA)
                    h1, w1, _ = frame1_resized.shape # Update dimensions after resize
                else:
                    frame1_resized = frame1

                # Create combined frame based on layout
                if config.STREAM_LAYOUT == "VERTICAL":
                    final_w = w0 # Combined width is Cam0 width
                    final_h = h0 + h1 + config.STREAM_BORDER_SIZE
                    combined = np.zeros((final_h, final_w, 3), dtype=np.uint8)
                    combined[:, :] = config.STREAM_BORDER_COLOR # Fill with border color
                    # Place Cam0 at top
                    combined[0:h0, 0:w0] = frame0
                    # Place Cam1 at bottom, centered horizontally
                    y_start1 = h0 + config.STREAM_BORDER_SIZE
                    x_start1 = (final_w - w1) // 2 # Center horizontally
                    combined[y_start1:y_start1 + h1, x_start1:x_start1 + w1] = frame1_resized
                elif config.STREAM_LAYOUT == "HORIZONTAL":
                     # Ensure Cam1 fits vertically if resizing needed (should match target_h1)
                     if h1 > h0:
                         scale = h0 / h1
                         new_w1 = int(w1 * scale)
                         frame1_resized = cv2.resize(frame1_resized, (new_w1, h0), interpolation=cv2.INTER_AREA)
                         w1, h1 = new_w1, h0 # Update dimensions

                     final_h = h0 # Combined height is Cam0 height
                     final_w = w0 + w1 + config.STREAM_BORDER_SIZE
                     combined = np.zeros((final_h, final_w, 3), dtype=np.uint8)
                     combined[:, :] = config.STREAM_BORDER_COLOR
                     # Place Cam0 on left
                     combined[0:h0, 0:w0] = frame0
                     # Place Cam1 on right, centered vertically (already resized to h0)
                     x_start1 = w0 + config.STREAM_BORDER_SIZE
                     combined[0:h0, x_start1:x_start1 + w1] = frame1_resized
                else:
                    logging.warning(f"Unknown STREAM_LAYOUT '{config.STREAM_LAYOUT}'. Defaulting to Cam0 only.")
                    combined = frame0 # Fallback to just frame0

            except Exception as e_comb:
                logging.error(f"!!! Error combining frames: {e_comb}", exc_info=True)
                self.last_error = f"Frame Combine Error: {e_comb}"
                combined = frame0 # Fallback to frame0 if combination fails
        elif frame0 is not None:
            # Only frame0 available
            combined = frame0
        elif frame1 is not None:
            # Only frame1 available, resize it to its target size for consistency
            try:
                target_w1, target_h1 = config.CAM1_RESOLUTION
                combined = cv2.resize(frame1, (target_w1, target_h1), interpolation=cv2.INTER_AREA)
            except Exception as e_resize1:
                logging.error(f"Could not resize fallback frame1: {e_resize1}")
                combined = None # Failed to even provide frame1
        else:
            # No frames captured
            combined = None

        # Update the output frame under lock
        with self.frame_lock:
            if combined is not None:
                self.output_frame = combined.copy() # Store a copy for retrieval
            else:
                self.output_frame = None

        # Write frame0 to video file(s) if recording
        # Check is_recording *outside* the lock to avoid holding it while writing
        if self.is_recording and frame0 is not None:
            force_stop = False # Flag to force stop if all writers fail
            # Acquire recording lock only for writing
            with self.recording_lock:
                # Double-check recording status after acquiring lock
                if not self.is_recording:
                    return combined # Recording was stopped between outer check and acquiring lock

                if not self.video_writers:
                    logging.warning("Recording flag is True, but no video writers available. Forcing stop.")
                    self.last_error = "Recording stopped: writer list empty."
                    force_stop = True
                else:
                    write_errors = 0
                    for i, writer in enumerate(self.video_writers):
                        try:
                            writer.write(frame0)
                        except Exception as e:
                            # Try to get path, handle index error gracefully
                            path_str = self.recording_paths[i] if i < len(self.recording_paths) else f"Writer {i}"
                            logging.error(f"!!! Failed write frame to {path_str}: {e}")
                            write_errors += 1
                            self.last_error = f"Frame write error: {os.path.basename(path_str)}" # Report error for specific path

                    # If all writers failed, stop recording
                    if write_errors > 0 and write_errors == len(self.video_writers):
                        logging.error("All video writers failed to write frame. Stopping recording.")
                        self.last_error = "Recording stopped: All writers failed."
                        force_stop = True

            # Force stop recording outside the lock if needed
            if force_stop:
                # Use a non-blocking call or separate thread if stop_recording can block significantly
                # For simplicity here, call directly, but be aware of potential delays
                self.stop_recording()

        return combined # Return the combined frame (or None)

    def get_latest_combined_frame(self):
        """Returns the latest combined frame in a thread-safe manner."""
        with self.frame_lock:
            if self.output_frame is not None:
                return self.output_frame.copy() # Return a copy
            else:
                return None

    # --- Audio Methods ---

    def _find_audio_device(self):
        """Finds the index of the USB audio input device based on hint."""
        if not config.AUDIO_ENABLED:
            logging.info("Audio is disabled in config.")
            return False

        self.audio_device_index = None # Reset
        logging.info(f"Searching for audio input device containing hint: '{config.AUDIO_DEVICE_HINT}'")
        try:
            devices = sd.query_devices()
            logging.debug(f"Available audio devices:\n{devices}")
            candidate_indices = []
            for i, device in enumerate(devices):
                # Check for input channels and hint in name (case-insensitive)
                if device['max_input_channels'] > 0 and config.AUDIO_DEVICE_HINT.lower() in device['name'].lower():
                    logging.info(f"Found potential audio device: Index {i}, Name: {device['name']}, Inputs: {device['max_input_channels']}, Rate: {device['default_samplerate']}")
                    candidate_indices.append(i)

            if not candidate_indices:
                 logging.warning(f"No audio input device found matching hint '{config.AUDIO_DEVICE_HINT}'.")
                 self.audio_last_error = "Audio Device: Not Found"
                 return False

            # Check if candidates support the required settings
            for index in candidate_indices:
                 try:
                     # Use the pre-checked numpy dtype here
                     sd.check_input_settings(device=index, samplerate=config.AUDIO_SAMPLE_RATE, channels=config.AUDIO_CHANNELS, dtype=self.audio_dtype)
                     logging.info(f"Device {index} ('{devices[index]['name']}') supports required settings. Selecting.")
                     self.audio_device_index = index
                     self.audio_last_error = None # Clear previous device errors
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
        try:
            logging.info(f"Audio capture thread started for device {self.audio_device_index}.")
            self.stop_audio_event.clear()

            def audio_callback(indata, frames, time, status):
                """This runs in a separate thread managed by sounddevice."""
                if status:
                    logging.warning(f"Audio callback status: {status}")
                    # Potential place to set self.audio_last_error, but needs thread safety (e.g., use queue)
                # Put a copy of the numpy array into the queue to avoid modification issues
                self.audio_queue.put(indata.copy())

            # Create and start the input stream
            self.audio_stream = sd.InputStream(
                samplerate=config.AUDIO_SAMPLE_RATE,
                device=self.audio_device_index,
                channels=config.AUDIO_CHANNELS,
                dtype=self.audio_dtype, # Use pre-checked numpy dtype
                blocksize=config.AUDIO_BLOCK_SIZE, # Buffer size
                callback=audio_callback
            )
            self.audio_stream.start()
            logging.info("Audio stream started.")

            # Keep thread alive while stream is active and stop not requested
            while not self.stop_audio_event.is_set():
                # Let sounddevice manage the callback thread; sleep here to check event
                sd.sleep(200) # Sleep for 200ms intervals to check stop event periodically
                if not self.audio_stream.active:
                     logging.warning("Audio stream became inactive unexpectedly.")
                     self.audio_last_error = self.audio_last_error or "Audio stream stopped unexpectedly"
                     break # Exit loop if stream stops on its own

        except sd.PortAudioError as pae:
             logging.error(f"!!! PortAudioError in audio recording thread: {pae}", exc_info=True)
             self.audio_last_error = f"Audio PortAudioError: {pae}"
        except Exception as e:
            logging.error(f"!!! Error in audio recording thread: {e}", exc_info=True)
            self.audio_last_error = f"Audio Thread Error: {e}"
        finally:
            # Cleanup stream if it exists and is active/open
            if self.audio_stream:
                try:
                    if not self.audio_stream.closed:
                         # Abort might be safer than stop/close if errors occurred
                         logging.info("Aborting audio stream...")
                         self.audio_stream.abort(ignore_errors=True)
                         logging.info("Closing audio stream...")
                         self.audio_stream.close(ignore_errors=True)
                    logging.info("Audio stream stopped and closed.")
                except Exception as e_close:
                    # Log error but don't overwrite primary error if one exists
                    logging.error(f"Error closing audio stream: {e_close}")
            logging.info("Audio capture thread finished.")
            # Signal the writer thread that capture is done by putting None in queue
            self.audio_queue.put(None) # Sentinel value

    def _audio_write_file_thread(self):
        """Thread function to write audio data from queue to temporary WAV file."""
        sound_file = None
        data_written = False
        try:
            if not self.temp_audio_file_path:
                logging.error("Audio write thread: Temporary audio file path not set."); return
            if not self.audio_dtype:
                 logging.error("Audio write thread: Audio dtype not set."); return

            # Determine soundfile subtype based on numpy dtype
            subtype = None
            if self.audio_dtype == np.int16: subtype = 'PCM_16'
            elif self.audio_dtype == np.int32: subtype = 'PCM_32'
            elif self.audio_dtype == np.float32: subtype = 'FLOAT'
            # Add other mappings if needed (e.g., 'PCM_24')
            if subtype is None:
                 logging.error(f"Audio write thread: Unsupported audio format '{config.AUDIO_FORMAT}' for soundfile. Cannot write WAV.")
                 self.audio_last_error = f"Audio Write Error: Unsupported format {config.AUDIO_FORMAT}"
                 return

            # Open the sound file for writing
            sound_file = sf.SoundFile(
                self.temp_audio_file_path, mode='w',
                samplerate=config.AUDIO_SAMPLE_RATE, channels=config.AUDIO_CHANNELS, subtype=subtype
            )
            logging.info(f"Audio write thread: Opened {self.temp_audio_file_path} (subtype: {subtype}) for writing.")

            while True:
                try:
                    # Block with a timeout to allow checking stop event
                    audio_data = self.audio_queue.get(block=True, timeout=1.0)

                    if audio_data is None: # Sentinel value received from capture thread
                        logging.info("Audio write thread: Received stop signal (sentinel).")
                        break

                    # Write the data
                    sound_file.write(audio_data)
                    data_written = True # Mark that we've written something

                except queue.Empty:
                    # Timeout occurred, check if main thread requested stop
                    if self.stop_audio_event.is_set():
                        logging.info("Audio write thread: Stop event detected during queue wait.")
                        break
                    # Otherwise, continue waiting for data
                    continue
                except Exception as write_err:
                     logging.error(f"!!! Error writing audio data to file: {write_err}", exc_info=True)
                     self.audio_last_error = f"Audio File Write Error: {write_err}"
                     # Decide whether to break or continue trying to write remaining data
                     # Let's continue for now to capture as much as possible
                     # break # Uncomment to stop writing on first error

        except Exception as e:
            logging.error(f"!!! Error in audio write thread setup/loop: {e}", exc_info=True)
            self.audio_last_error = f"Audio Write Thread Error: {e}"
        finally:
            if sound_file:
                try:
                    sound_file.close()
                    logging.info(f"Audio write thread: Closed {self.temp_audio_file_path}.")
                except Exception as e_close:
                    logging.error(f"Error closing sound file: {e_close}")

            # Check if file exists and has data before declaring success
            if not data_written and self.temp_audio_file_path and os.path.exists(self.temp_audio_file_path):
                 logging.warning(f"Audio write thread finished, but no data seems to have been written to {self.temp_audio_file_path}.")
                 # Optionally delete empty file here? Depends on requirements.
            elif data_written:
                 logging.info(f"Audio write thread finished. Data written to {self.temp_audio_file_path}.")

            logging.info("Audio write thread finished.")


    def _start_audio_recording(self, base_filename):
        """Starts the audio recording threads and sets up the temporary file."""
        if not config.AUDIO_ENABLED or not self.audio_dtype:
            logging.info("Audio recording start skipped (disabled or format unsupported).")
            return False

        # Ensure previous threads/files are cleaned up (should be handled by stop, but double-check)
        if self.audio_thread and self.audio_thread.is_alive():
             logging.warning("Audio start called while previous capture thread alive. Attempting stop first.")
             self._stop_audio_recording()
        if self.audio_write_thread and self.audio_write_thread.is_alive():
             logging.warning("Audio start called while previous write thread alive. Attempting stop first.")
             self._stop_audio_recording() # Stop should handle both

        # Find the audio device
        if not self._find_audio_device() or self.audio_device_index is None:
            # Error message already set by _find_audio_device
            logging.error("Audio Start Failed: No suitable audio device found or configured.")
            return False

        try:
            # Create temporary file path
            temp_dir = tempfile.gettempdir()
            self.temp_audio_file_path = os.path.join(temp_dir, f"{base_filename}_audio{config.AUDIO_TEMP_EXTENSION}")
            logging.info(f"Starting audio recording to temporary file: {self.temp_audio_file_path}")

            # Clear the queue of any stale data
            while not self.audio_queue.empty():
                 try: self.audio_queue.get_nowait()
                 except queue.Empty: break

            self.stop_audio_event.clear() # Reset stop event

            # Start capture thread first
            self.audio_thread = threading.Thread(target=self._audio_recording_thread, name="AudioCaptureThread")
            self.audio_thread.daemon = True # Allow main program to exit even if this thread hangs
            self.audio_thread.start()

            # Give capture thread a moment to start the stream before starting writer
            time.sleep(0.2)

            # Start writer thread
            self.audio_write_thread = threading.Thread(target=self._audio_write_file_thread, name="AudioWriteThread")
            self.audio_write_thread.daemon = True
            self.audio_write_thread.start()

            self.audio_last_error = None # Clear error on successful start attempt
            return True

        except Exception as e:
            logging.error(f"!!! Failed to start audio recording setup: {e}", exc_info=True)
            self.audio_last_error = f"Audio Start Error: {e}"
            # --- Cleanup partial start ---
            self.stop_audio_event.set() # Signal threads to stop
            if self.audio_thread and self.audio_thread.is_alive():
                self.audio_thread.join(timeout=1.0) # Wait briefly
            if self.audio_write_thread and self.audio_write_thread.is_alive():
                self.audio_write_thread.join(timeout=1.0) # Wait briefly
            # Remove temp file if created
            if self.temp_audio_file_path and os.path.exists(self.temp_audio_file_path):
                 try: os.remove(self.temp_audio_file_path)
                 except OSError: pass
            # Reset state variables
            self.temp_audio_file_path = None
            self.audio_thread = None
            self.audio_write_thread = None
            return False

    def _stop_audio_recording(self):
        """Stops the audio recording threads and returns the temp file path if valid."""
        if not config.AUDIO_ENABLED: return None

        # Check if threads exist and are alive
        capture_thread_running = self.audio_thread and self.audio_thread.is_alive()
        write_thread_running = self.audio_write_thread and self.audio_write_thread.is_alive()

        if not capture_thread_running and not write_thread_running:
            logging.info("Audio stop called, but recording threads are not running.")
            # Still return path if it exists and seems valid (e.g., from a previous run)
            # Check size > 44 bytes (basic WAV header check)
            if self.temp_audio_file_path and os.path.exists(self.temp_audio_file_path):
                 try:
                     if os.path.getsize(self.temp_audio_file_path) > 44:
                          final_path = self.temp_audio_file_path
                          self.temp_audio_file_path = None # Clear path after retrieving
                          return final_path
                     else:
                          logging.warning(f"Cleaning up empty/invalid temp audio file: {self.temp_audio_file_path}")
                          os.remove(self.temp_audio_file_path)
                 except OSError as e:
                      logging.warning(f"Error checking/removing temp audio file {self.temp_audio_file_path}: {e}")
            self.temp_audio_file_path = None # Ensure path is cleared
            return None

        logging.info("Stopping audio recording threads...")
        self.stop_audio_event.set() # Signal threads to stop via event

        # Wait for capture thread (which manages the stream)
        if capture_thread_running:
            logging.debug("Waiting for audio capture thread to join...")
            self.audio_thread.join(timeout=2.0) # Timeout in case it hangs
            if self.audio_thread.is_alive():
                logging.warning("Audio capture thread did not stop cleanly within timeout.")
                # Stream closure happens in the thread's finally block

        # Wait for write thread (which waits on queue/event)
        if write_thread_running:
            logging.debug("Waiting for audio write thread to join...")
            # Give writer thread longer timeout as it might be writing final blocks
            self.audio_write_thread.join(timeout=5.0)
            if self.audio_write_thread.is_alive():
                logging.warning("Audio write thread did not stop cleanly within timeout.")

        # Clear thread and stream variables
        self.audio_thread = None
        self.audio_write_thread = None
        self.audio_stream = None # Stream should be closed by capture thread

        final_path = self.temp_audio_file_path # Store path before clearing variable
        self.temp_audio_file_path = None # Clear path variable

        # Return the path if the file exists and seems valid (basic size check)
        if final_path and os.path.exists(final_path):
             try:
                 if os.path.getsize(final_path) > 44: # Basic WAV header check
                     logging.info(f"Audio stop successful. Returning temp file: {final_path}")
                     return final_path
                 else:
                     logging.warning(f"Temporary audio file {final_path} seems empty/invalid after stop.")
                     os.remove(final_path) # Clean up empty file
                     logging.info("Cleaned up empty/invalid temp audio file.")
                     return None
             except OSError as e:
                  logging.warning(f"Error checking/removing final audio file {final_path}: {e}")
                  return None # Don't return path if we can't verify it
        else:
            logging.error("Temporary audio file path not set or file does not exist after stop.")
            return None

    def _mux_audio_video(self, video_path, audio_path):
        """Merges the temporary video and audio files using ffmpeg."""
        if not config.AUDIO_ENABLED: return None # Should not be called if disabled
        # --- Pre-checks ---
        if not os.path.exists(config.FFMPEG_PATH):
            logging.error(f"ffmpeg not found at {config.FFMPEG_PATH}. Cannot mux audio/video.")
            self.audio_last_error = "Mux Error: ffmpeg not found"
            return None # Return None to indicate failure, keep video-only
        if not os.path.exists(video_path):
            logging.error(f"Video file not found for muxing: {video_path}.")
            self.audio_last_error = "Mux Error: Video file missing"
            return None
        if not os.path.exists(audio_path):
            logging.error(f"Audio file not found for muxing: {audio_path}.")
            self.audio_last_error = "Mux Error: Audio file missing"
            return None # Keep video-only

        # --- Determine Output Path ---
        # Replace "_video.ext" with ".ext" for the final filename
        output_path = video_path.replace("_video" + config.CAM0_RECORDING_EXTENSION, config.CAM0_RECORDING_EXTENSION)
        # If replacement didn't happen (e.g., unexpected filename), create a fallback name
        if output_path == video_path:
            output_path = video_path.replace(config.CAM0_RECORDING_EXTENSION, "_muxed" + config.CAM0_RECORDING_EXTENSION)
            logging.warning(f"Could not generate standard muxed name. Using fallback: {output_path}")

        logging.info(f"Muxing video '{os.path.basename(video_path)}' and audio '{os.path.basename(audio_path)}' into '{os.path.basename(output_path)}'...")

        # --- Construct ffmpeg Command ---
        # -y: Overwrite output without asking
        # -i video_path: Input video file
        # -i audio_path: Input audio file
        # -c copy: Copy streams without re-encoding (fastest, preserves quality)
        # -map 0:v:0: Map video stream from first input (0)
        # -map 1:a:0: Map audio stream from second input (1)
        # -shortest: Finish encoding when the shortest input stream ends (usually audio)
        # -loglevel error: Show only errors from ffmpeg
        command = [config.FFMPEG_PATH, "-y", "-i", video_path, "-i", audio_path, "-c:v", "copy", "-c:a", "aac", "-map", "0:v:0", "-map", "1:a:0", "-shortest", "-loglevel", config.FFMPEG_LOG_LEVEL, output_path]

        # --- Execute ffmpeg ---
        try:
            process = subprocess.run(command, capture_output=True, text=True, check=True, timeout=config.AUDIO_MUX_TIMEOUT)
            logging.info(f"ffmpeg muxing successful for {output_path}.")
            # Log ffmpeg output only if debug level is high enough or needed
            logging.debug(f"ffmpeg output:\nSTDOUT:\n{process.stdout}\nSTDERR:\n{process.stderr}")
            self.audio_last_error = None # Clear audio error on successful mux
            return output_path # Return the path of the successfully muxed file

        except subprocess.TimeoutExpired:
            logging.error(f"!!! ffmpeg muxing timed out ({config.AUDIO_MUX_TIMEOUT}s) for {output_path}.")
            self.audio_last_error = "Mux Error: ffmpeg timed out"
        except subprocess.CalledProcessError as e:
            logging.error(f"!!! ffmpeg muxing failed for {output_path}. Return Code: {e.returncode}")
            logging.error(f"ffmpeg stderr:\n{e.stderr}") # Log stderr on error
            logging.error(f"ffmpeg stdout:\n{e.stdout}")
            self.audio_last_error = f"Mux Error: ffmpeg failed (code {e.returncode})"
        except Exception as e:
            logging.error(f"!!! Unexpected error during ffmpeg execution: {e}", exc_info=True)
            self.audio_last_error = f"Mux Error: {e}"

        # --- Cleanup Failed Output ---
        # If muxing failed, try to remove the potentially incomplete output file
        if os.path.exists(output_path):
             try:
                 os.remove(output_path)
                 logging.info(f"Removed incomplete muxed file: {output_path}")
             except OSError as rm_err:
                 logging.warning(f"Could not remove failed mux output file {output_path}: {rm_err}")

        return None # Return None to indicate muxing failure


    def shutdown(self):
        """Stops recording, stops and closes both cameras gracefully."""
        logging.info("CameraManager shutting down...")

        # Stop recording if active (handles audio stop as well)
        if self.is_recording:
            self.stop_recording()
        else:
            # If not recording, ensure any orphaned audio threads are stopped
            if config.AUDIO_ENABLED and ((self.audio_thread and self.audio_thread.is_alive()) or \
                                         (self.audio_write_thread and self.audio_write_thread.is_alive())):
                logging.warning("Shutting down orphaned audio recording threads...")
                self._stop_audio_recording()

        # Stop and close camera instances
        for cam_id, picam_instance, name in [(config.CAM0_ID, self.picam0, "Cam0"), (config.CAM1_ID, self.picam1, "Cam1")]:
            if picam_instance:
                try:
                    if picam_instance.started:
                        logging.info(f"Stopping {name}...")
                        picam_instance.stop()
                    logging.info(f"Closing {name}...")
                    picam_instance.close()
                except Exception as e:
                    logging.error(f"Error stopping/closing {name}: {e}")
                finally:
                    # Clear instance variables regardless of errors during close
                    if cam_id == config.CAM0_ID:
                        self.picam0 = None
                        self.is_initialized0 = False
                        self.actual_cam0_fps = None # Reset actual FPS
                    else:
                        self.picam1 = None
                        self.is_initialized1 = False

        logging.info("CameraManager shutdown complete.")
