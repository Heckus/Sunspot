# -*- coding: utf-8 -*-
"""
camera_manager.py

Manages multiple Picamera2 cameras, including initialization, configuration,
frame capture (combined stream), video recording (from primary camera),
and audio recording/muxing.
"""

import os
import time
import datetime
import logging
import threading
import cv2
import numpy as np
from picamera2 import Picamera2,controls, Transform

# from libcamera import controls, Transform

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
    # (Code is identical to previous version - omitted for brevity)
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

        # State variable for combined frame
        self.combined_frame = None # Stores the latest combined frame for streaming

        # Recording state (only for Cam0)
        self.is_recording = False
        self.video_writers = [] # List of OpenCV VideoWriter objects for Cam0
        self.recording_paths = [] # List of corresponding file paths for Cam0 (video-only initially)

        # Locks for thread safety
        self.frame_lock = threading.Lock() # Protects access to combined_frame
        self.config_lock = threading.Lock() # Protects access to camera state/config changes
        self.recording_lock = threading.Lock() # Protects access to recording state/writers

        # --- Initialize Common Camera Controls to Defaults from Config ---
        # (Control initialization code remains unchanged - omitted for brevity)
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
        # (Code is identical to previous version - omitted for brevity)
        picam_instance = None; is_initialized_flag = False; cam_name = f"Cam{cam_id}"
        if cam_id == config.CAM0_ID:
            if resolution_index is not None:
                if 0 <= resolution_index < len(config.CAM0_RESOLUTIONS): self.current_resolution_index0 = resolution_index
                else: logging.error(f"{cam_name}: Invalid res index {resolution_index}. Using {self.current_resolution_index0}.")
            res_list = config.CAM0_RESOLUTIONS; current_res_index = self.current_resolution_index0
            tuning_data = config.CAM0_TUNING_FILE_PATH
            try: target_width, target_height, target_fps = res_list[current_res_index]
            except IndexError: logging.error(f"{cam_name}: Res index {current_res_index} out bounds."); current_res_index = config.CAM0_DEFAULT_RESOLUTION_INDEX; self.current_resolution_index0 = current_res_index; target_width, target_height, target_fps = res_list[current_res_index]
        elif cam_id == config.CAM1_ID: target_width, target_height = config.CAM1_RESOLUTION; target_fps = config.CAM1_FRAME_RATE; tuning_data = config.CAM1_TUNING; current_res_index = 0
        else: logging.error(f"Invalid camera ID {cam_id}"); return False
        logging.info(f"Attempting init {cam_name} (ID: {cam_id}) at index {current_res_index} ({target_width}x{target_height} @ {target_fps:.1f}fps)...")
        existing_picam = self.picam0 if cam_id == config.CAM0_ID else self.picam1
        if existing_picam is not None:
            try:
                if existing_picam.started: logging.info(f"Stopping existing {cam_name}..."); existing_picam.stop()
                logging.info(f"Closing existing {cam_name}..."); existing_picam.close()
            except Exception as e: logging.warning(f"Error stopping/closing previous {cam_name}: {e}")
            finally:
                if cam_id == config.CAM0_ID: self.picam0 = None; self.is_initialized0 = False
                else: self.picam1 = None; self.is_initialized1 = False
            time.sleep(0.5)
        try:
            picam_instance = Picamera2(camera_num=cam_id, tuning=tuning_data); logging.info(f"{cam_name}: Picamera2 object created.")
            controls_to_set = {"FrameRate": target_fps, "NoiseReductionMode": self.current_noise_reduction_mode, "AeEnable": True, "AeExposureMode": self.current_ae_mode, "AeMeteringMode": self.current_metering_mode, "AnalogueGain": self.current_analogue_gain, "Brightness": self.current_brightness, "Contrast": self.current_contrast, "Saturation": self.current_saturation, "Sharpness": self.current_sharpness,}
            if self.current_analogue_gain == 0.0: controls_to_set["AeEnable"] = True; logging.info(f"{cam_name}: Auto ISO selected. Ensuring AE enabled.")
            controls_to_set = {k: v for k, v in controls_to_set.items() if v is not None}
            transform = Transform() # Add logic here if cam1 needs flipping etc.
            cam_config = picam_instance.create_video_configuration(main={"size": (target_width, target_height), "format": "RGB888"}, controls=controls_to_set, transform=transform)
            logging.info(f"{cam_name}: Configuring with: main={cam_config['main']}, controls={cam_config['controls']}")
            picam_instance.configure(cam_config); time.sleep(0.5)
            new_config = picam_instance.camera_configuration()
            if new_config:
                applied_controls = new_config.get('controls', {}); applied_main = new_config.get('main', {})
                logging.info(f"{cam_name}: Verified Applied Config: main={applied_main}, controls={applied_controls}")
                actual_fps_v = applied_controls.get('FrameRate');
                if actual_fps_v and abs(actual_fps_v - target_fps) > 0.1: logging.warning(f"{cam_name}: Driver adjusted FrameRate from {target_fps:.1f} to {actual_fps_v:.1f}")
            else: logging.warning(f"{cam_name}: Could not get camera configuration after applying.")
            logging.info(f"{cam_name}: Configuration successful!")
            picam_instance.start(); logging.info(f"{cam_name}: Camera started"); time.sleep(2.0)
            actual_config = picam_instance.camera_configuration()
            if not actual_config: raise RuntimeError(f"{cam_name}: Failed get config after start.")
            actual_format = actual_config.get('main', {}); actual_w = actual_format.get('size', (0,0))[0]; actual_h = actual_format.get('size', (0,0))[1]; actual_fmt_str = actual_format.get('format', 'Unknown'); actual_fps_final = actual_config.get('controls', {}).get('FrameRate', 'N/A'); actual_gain = actual_config.get('controls', {}).get('AnalogueGain', 'N/A')
            logging.info(f"{cam_name}: Initialized. Actual stream: {actual_w}x{actual_h} {actual_fmt_str} @ {actual_fps_final} fps. Gain: {actual_gain}")
            is_initialized_flag = True; self.last_error = None
            return True
        except Exception as e:
            logging.error(f"!!! Failed to initialize {cam_name} at {target_width}x{target_height}: {e}", exc_info=True); self.last_error = f"{cam_name} Init Error: {e}"
            if picam_instance is not None:
                try:
                    if picam_instance.started: picam_instance.stop()
                    picam_instance.close()
                except Exception as close_e: logging.error(f"Error closing {cam_name} after init failure: {close_e}")
            picam_instance = None; is_initialized_flag = False
            return False
        finally:
            if cam_id == config.CAM0_ID: self.picam0 = picam_instance; self.is_initialized0 = is_initialized_flag
            else: self.picam1 = picam_instance; self.is_initialized1 = is_initialized_flag

    def initialize_cameras(self, resolution_index=None):
        """Initializes or re-initializes both cameras."""
        # (Code is identical to previous version - omitted for brevity)
        with self.config_lock:
            logging.info("--- Initializing Both Cameras ---")
            success0 = self._initialize_camera(config.CAM0_ID, resolution_index)
            success1 = self._initialize_camera(config.CAM1_ID)
            if success0 and success1: logging.info("--- Both Cameras Initialized Successfully ---"); return True
            else: logging.error("!!! Failed to initialize one or both cameras. Check logs. !!!"); return False

    def get_cam0_resolution(self):
        """Returns the current resolution tuple (width, height, fps) for Cam0."""
        # (Code is identical to previous version - omitted for brevity)
        try: return config.CAM0_RESOLUTIONS[self.current_resolution_index0]
        except IndexError: logging.error(f"Invalid index {self.current_resolution_index0} for Cam0."); safe_default_index = max(0, min(len(config.CAM0_RESOLUTIONS) - 1, config.CAM0_DEFAULT_RESOLUTION_INDEX)); return config.CAM0_RESOLUTIONS[safe_default_index]

    def apply_camera_controls(self, controls_dict):
        """Applies a dictionary of common controls to BOTH running cameras."""
        # (Code is identical to previous version - omitted for brevity)
        if not self.is_initialized0 and not self.is_initialized1: logging.error("Cannot apply controls: No cameras initialized."); self.last_error = "Control Apply Error: No cameras ready."; return False
        logging.info(f"Applying common controls to cameras: {controls_dict}"); success_count = 0; applied_to_cam0 = False; applied_to_cam1 = False
        try:
            with self.config_lock:
                iso_name_updated = False
                for key, value in controls_dict.items():
                    if key == 'AnalogueGain':
                        self.current_analogue_gain = value; self.current_iso_name = "Unknown"
                        for name, gain in config.AVAILABLE_ISO_SETTINGS.items():
                             if abs(gain - value) < 0.01: self.current_iso_name = name; iso_name_updated = True; break
                        if not iso_name_updated: logging.warning(f"Applied AnalogueGain {value} no match known ISO name.")
                    elif key == 'AeExposureMode': self.current_ae_mode = value
                    elif key == 'AeMeteringMode': self.current_metering_mode = value
                    elif key == 'NoiseReductionMode': self.current_noise_reduction_mode = value
                    elif key == 'Brightness': self.current_brightness = value
                    elif key == 'Contrast': self.current_contrast = value
                    elif key == 'Saturation': self.current_saturation = value
                    elif key == 'Sharpness': self.current_sharpness = value
                logging.debug(f"Applying to Cam0...");
                if self.is_initialized0 and self.picam0 and self.picam0.started:
                    try: self.picam0.set_controls(controls_dict); logging.debug("Cam0 controls applied."); applied_to_cam0 = True; success_count += 1
                    except Exception as e0: logging.error(f"!!! Error applying controls to Cam0: {e0}"); self.last_error = f"Cam0 Control Error: {e0}"
                else: logging.warning("Skipping control application for Cam0 (not ready).")
                logging.debug(f"Applying to Cam1...");
                if self.is_initialized1 and self.picam1 and self.picam1.started:
                    try: self.picam1.set_controls(controls_dict); logging.debug("Cam1 controls applied."); applied_to_cam1 = True; success_count += 1
                    except Exception as e1: logging.error(f"!!! Error applying controls to Cam1: {e1}"); self.last_error = (self.last_error or "") + (self.last_error and " / " or "") + f"Cam1 Control Error: {e1}"
                else: logging.warning("Skipping control application for Cam1 (not ready).")
            if success_count > 0:
                if any(k in controls_dict for k in ['AnalogueGain', 'AeExposureMode', 'AeMeteringMode', 'Brightness']): time.sleep(0.5)
                else: time.sleep(0.1)
                if (applied_to_cam0 and self.is_initialized0) or (applied_to_cam1 and self.is_initialized1): self.last_error = None
                return True
            else: 
                if (not self.last_error):
                       self.last_error = "Control Apply Error: Failed on all ready cameras."; return False
        except Exception as e: logging.error(f"!!! Unexpected Error during control application: {e}", exc_info=True); self.last_error = f"Control Apply Unexpected Error: {e}"; return False
        return False # Should not be reached

    def get_camera_state(self):
        """Returns a dictionary containing the current camera state and common control values."""
        # (Code is identical to previous version - omitted for brevity)
        with self.config_lock:
            res_w, res_h, res_fps = self.get_cam0_resolution()
            state = {'is_initialized': self.is_initialized0 and self.is_initialized1, 'is_initialized0': self.is_initialized0, 'is_initialized1': self.is_initialized1, 'resolution_index': self.current_resolution_index0, 'resolution_wh': (res_w, res_h), 'resolution_fps': res_fps, 'is_recording': self.is_recording, 'recording_paths': list(self.recording_paths), 'last_error': self.last_error, 'audio_last_error': self.audio_last_error, 'iso_mode': self.current_iso_name, 'analogue_gain': self.current_analogue_gain, 'ae_mode': self.current_ae_mode.name if hasattr(self.current_ae_mode, 'name') else str(self.current_ae_mode), 'metering_mode': self.current_metering_mode.name if hasattr(self.current_metering_mode, 'name') else str(self.current_metering_mode), 'noise_reduction_mode': self.current_noise_reduction_mode.name if hasattr(self.current_noise_reduction_mode, 'name') else str(self.current_noise_reduction_mode), 'brightness': self.current_brightness, 'contrast': self.current_contrast, 'saturation': self.current_saturation, 'sharpness': self.current_sharpness,}
        return state

    def start_recording(self):
        """Starts recording video FROM CAM0 and audio (if enabled)."""
        # (Code is identical to previous version - omitted for brevity, includes audio start call)
        with self.recording_lock:
            if self.is_recording: logging.warning("Start recording called, but already recording."); return True
            if not self.is_initialized0 or not self.picam0 or not self.picam0.started: logging.error("Cannot start recording, Cam0 not available."); self.last_error = "Cam0 not available for recording."; return False
            logging.info("Attempting to start recording (Video: Cam0)..."); usb_drives = get_usb_mounts()
            if not usb_drives: logging.warning(f"Cannot start recording: No writable USB drives in {config.USB_BASE_PATH}."); self.last_error = f"Cannot start recording: No writable USB drives found"; return False
            self.video_writers.clear(); self.recording_paths.clear(); success_count = 0; start_error = None
            try:
                width, height, fps = self.get_cam0_resolution()
                if width <= 0 or height <= 0 or fps <= 0: raise ValueError(f"Invalid Cam0 dims/FPS: {width}x{height} @ {fps}fps")
                logging.info(f"Starting Cam0 recording: {width}x{height} @ {fps:.2f} fps"); fourcc = cv2.VideoWriter_fourcc(*config.CAM0_RECORDING_FORMAT); timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                for drive_path in usb_drives:
                    try:
                        base_filename = f"recording_{timestamp}_{width}x{height}"; video_only_filename = f"{base_filename}_video{config.CAM0_RECORDING_EXTENSION}"; full_path = os.path.join(drive_path, video_only_filename)
                        writer = cv2.VideoWriter(full_path, fourcc, fps, (width, height))
                        if not writer.isOpened(): raise IOError(f"Failed to open VideoWriter: {full_path}")
                        self.video_writers.append(writer); self.recording_paths.append(full_path); logging.info(f"Started video recording to: {full_path}"); success_count += 1
                    except Exception as e: logging.error(f"!!! Failed create VideoWriter for {drive_path}: {e}", exc_info=True); start_error = start_error or f"Failed writer {os.path.basename(drive_path)}: {e}"
                if success_count > 0:
                    self.is_recording = True; logging.info(f"Video recording started on {success_count} drive(s).")
                    if config.AUDIO_ENABLED:
                        if not self._start_audio_recording(base_filename): logging.error("Failed start audio recording."); self.audio_last_error = self.audio_last_error or "Audio start failed"
                        else: logging.info("Audio recording started."); self.audio_last_error = None
                    if start_error and success_count < len(usb_drives): self.last_error = f"Partial Rec Start Fail: {start_error}"
                    elif not start_error:
                         if self.last_error and ("Recording" in self.last_error or "writers" in self.last_error or "USB" in self.last_error): logging.info(f"Clearing previous video error: '{self.last_error}'"); self.last_error = None
                    return True
                else: self.is_recording = False; logging.error("Failed start video recording ANY drive."); self.last_error = f"Rec Start Failed: {start_error or 'No writers opened'}";
                for writer in self.video_writers:
                    try: writer.release()
                    except: pass
                self.video_writers.clear(); self.recording_paths.clear(); return False
            except Exception as e: logging.error(f"!!! Critical error during recording setup: {e}", exc_info=True); self.last_error = f"Recording Setup Error: {e}"; self.stop_recording(); return False

    def stop_recording(self):
        """Stops video/audio recording, releases writers, muxes files, syncs."""
        # (Code is identical to previous version - omitted for brevity, includes audio stop/mux calls)
        with self.recording_lock:
            if not self.is_recording:
                if self.video_writers or self.recording_paths: logging.warning("stop_recording: clearing stale writer lists."); self.video_writers.clear(); self.recording_paths.clear()
                if config.AUDIO_ENABLED and (self.audio_thread and self.audio_thread.is_alive()): logging.warning("Stopping orphaned audio recording..."); self._stop_audio_recording()
                return
            logging.info("Stopping recording (Video and Audio)..."); self.is_recording = False; released_count = 0
            writers_to_release = list(self.video_writers); video_paths_recorded = list(self.recording_paths)
            self.video_writers.clear(); self.recording_paths.clear()
        audio_file_to_mux = None
        if config.AUDIO_ENABLED:
            audio_file_to_mux = self._stop_audio_recording()
            if audio_file_to_mux: logging.info(f"Audio recording stopped. Temp file: {audio_file_to_mux}")
            else: logging.error("Audio recording stop failed/not running."); self.audio_last_error = self.audio_last_error or "Audio stop failed"
        logging.info(f"Releasing {len(writers_to_release)} video writer(s)..."); final_output_paths = []
        for i, writer in enumerate(writers_to_release):
            video_path = video_paths_recorded[i] if i < len(video_paths_recorded) else f"Unknown Path {i}"
            try:
                writer.release(); logging.info(f"Released video writer for: {video_path}"); released_count += 1
                if audio_file_to_mux and os.path.exists(video_path):
                     final_path = self._mux_audio_video(video_path, audio_file_to_mux)
                     if final_path:
                         final_output_paths.append(final_path)
                         if final_path != video_path:
                              try: os.remove(video_path); logging.info(f"Removed temp video file: {video_path}")
                              except OSError as rm_err: logging.warning(f"Could not remove temp video {video_path}: {rm_err}")
                     else: logging.error(f"Muxing failed for {video_path}. Keeping video-only."); final_output_paths.append(video_path); self.last_error = self.last_error or "Muxing Failed"
                elif os.path.exists(video_path): final_output_paths.append(video_path)
            except Exception as e: logging.error(f"Error releasing VideoWriter for {video_path}: {e}", exc_info=True)
        if audio_file_to_mux and os.path.exists(audio_file_to_mux):
             try: os.remove(audio_file_to_mux); logging.info(f"Removed temp audio file: {audio_file_to_mux}")
             except OSError as e: logging.warning(f"Could not remove temp audio {audio_file_to_mux}: {e}")
        if released_count > 0 or final_output_paths:
            logging.info("Syncing filesystem...");
            try: sync_start_time = time.monotonic(); os.system('sync'); sync_duration = time.monotonic() - sync_start_time; logging.info(f"Sync completed in {sync_duration:.2f}s.")
            except Exception as e: logging.error(f"!!! Failed execute 'sync': {e}", exc_info=True); self.last_error = "Sync failed after recording."
        else: logging.warning("No writers released/files created, skipping sync.")
        logging.info(f"Recording stopped. Released {released_count} writer(s). Final files: {len(final_output_paths)}")

    def capture_and_combine_frames(self):
        """Captures frames from both cameras, combines them, writes Cam0 frame if recording."""
        # (Code is identical to previous version - omitted for brevity)
        frame0 = None; frame1 = None; combined = None
        if self.is_initialized0 and self.picam0 and self.picam0.started:
            try: frame0 = self.picam0.capture_array("main");
            except Exception as e0: logging.error(f"!!! Error Cam0 capture: {e0}"); self.last_error = f"Cam0 Capture Error: {e0}"
        if self.is_initialized1 and self.picam1 and self.picam1.started:
            try: frame1 = self.picam1.capture_array("main");
            except Exception as e1: logging.error(f"!!! Error Cam1 capture: {e1}"); self.last_error = (self.last_error or "") + (self.last_error and " / " or "") + f"Cam1 Capture Error: {e1}"
        if frame0 is not None and frame1 is not None:
            try:
                h0, w0, _ = frame0.shape; h1, w1, _ = frame1.shape; target_w1, target_h1 = config.CAM1_RESOLUTION
                if w1 != target_w1 or h1 != target_h1 or w1 > w0: frame1_resized = cv2.resize(frame1, (target_w1, target_h1), interpolation=cv2.INTER_AREA); h1, w1, _ = frame1_resized.shape
                else: frame1_resized = frame1
                final_w = w0; final_h = h0 + h1 + config.STREAM_BORDER_SIZE; combined = np.zeros((final_h, final_w, 3), dtype=np.uint8); combined[:, :] = config.STREAM_BORDER_COLOR
                combined[0:h0, 0:w0] = frame0; y_start1 = h0 + config.STREAM_BORDER_SIZE; x_start1 = (final_w - w1) // 2; combined[y_start1:y_start1 + h1, x_start1:x_start1 + w1] = frame1_resized
            except Exception as e_comb: logging.error(f"!!! Error combining frames: {e_comb}", exc_info=True); self.last_error = f"Frame Combine Error: {e_comb}"; combined = frame0
        elif frame0 is not None: combined = frame0
        elif frame1 is not None:
            try: target_w1, target_h1 = config.CAM1_RESOLUTION; combined = cv2.resize(frame1, (target_w1, target_h1), interpolation=cv2.INTER_AREA)
            except Exception as e_resize1: logging.error(f"Could not resize fallback frame1: {e_resize1}"); combined = None
        else: combined = None
        with self.frame_lock:
            if combined is not None: self.output_frame = combined.copy()
            else: self.output_frame = None
        if self.is_recording and frame0 is not None:
            with self.recording_lock:
                if not self.is_recording: return combined
                if not self.video_writers: logging.warning("Rec True, but no writers. Forcing stop."); self.last_error = "Rec stopped: writer list empty."; force_stop = True
                else:
                    force_stop = False; write_errors = 0
                    for i, writer in enumerate(self.video_writers):
                        try: writer.write(frame0)
                        except Exception as e: path_str = self.recording_paths[i] if i < len(self.recording_paths) else f"Writer {i}"; logging.error(f"!!! Failed write frame {path_str}: {e}"); write_errors += 1; self.last_error = f"Frame write error: {os.path.basename(path_str)}"
                    if write_errors > 0 and write_errors == len(self.video_writers): logging.error("All writers failed. Stopping rec."); self.last_error = "Rec stopped: All writers failed."; force_stop = True
            if force_stop: self.stop_recording()
        return combined

    def get_latest_combined_frame(self):
        """Returns the latest combined frame in a thread-safe manner."""
        # (Code is identical to previous version - omitted for brevity)
        with self.frame_lock:
            if self.output_frame is not None: return self.output_frame.copy()
            else: return None

    # --- Audio Methods ---

    def _find_audio_device(self):
        """Finds the index of the USB audio input device based on hint."""
        if not config.AUDIO_ENABLED: return False
        self.audio_device_index = None
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
                     sd.check_input_settings(device=index, samplerate=config.AUDIO_SAMPLE_RATE, channels=config.AUDIO_CHANNELS, dtype=config.AUDIO_FORMAT)
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
                    # Could set self.audio_last_error here, but be careful with thread safety
                # Put a copy of the numpy array into the queue
                self.audio_queue.put(indata.copy())

            # Create and start the input stream
            self.audio_stream = sd.InputStream(
                samplerate=config.AUDIO_SAMPLE_RATE,
                device=self.audio_device_index,
                channels=config.AUDIO_CHANNELS,
                dtype=self.audio_dtype, # Use pre-checked numpy dtype
                blocksize=config.AUDIO_BLOCK_SIZE,
                callback=audio_callback
            )
            self.audio_stream.start()
            logging.info("Audio stream started.")

            # Keep thread alive while stream is active and stop not requested
            while not self.stop_audio_event.is_set():
                # Let sounddevice manage the callback thread; sleep here
                sd.sleep(200) # Sleep for 200ms intervals to check stop event
                if not self.audio_stream.active:
                     logging.warning("Audio stream became inactive unexpectedly.")
                     self.audio_last_error = self.audio_last_error or "Audio stream stopped unexpectedly"
                     break # Exit loop if stream stops

        except Exception as e:
            logging.error(f"!!! Error in audio recording thread: {e}", exc_info=True)
            self.audio_last_error = f"Audio Thread Error: {e}"
        finally:
            # Cleanup stream if it exists and is active/open
            if self.audio_stream:
                try:
                    if not self.audio_stream.closed:
                         # Abort might be safer than stop/close if errors occurred
                         self.audio_stream.abort(ignore_errors=True)
                         self.audio_stream.close(ignore_errors=True)
                    logging.info("Audio stream stopped and closed.")
                except Exception as e_close:
                    # Log error but don't overwrite primary error
                    logging.error(f"Error closing audio stream: {e_close}")
            logging.info("Audio capture thread finished.")
            # Signal the writer thread that capture is done
            self.audio_queue.put(None) # Sentinel value

    def _audio_write_file_thread(self):
        """Thread function to write audio data from queue to temporary WAV file."""
        sound_file = None
        data_written = False
        try:
            if not self.temp_audio_file_path:
                logging.error("Audio write thread: Temp audio file path not set."); return
            if not self.audio_dtype:
                 logging.error("Audio write thread: Audio dtype not set."); return

            # Determine soundfile subtype based on numpy dtype
            subtype = None
            if self.audio_dtype == np.int16: subtype = 'PCM_16'
            elif self.audio_dtype == np.int32: subtype = 'PCM_32'
            elif self.audio_dtype == np.float32: subtype = 'FLOAT'
            # Add other mappings if needed
            if subtype is None:
                 logging.error(f"Audio write thread: Unsupported audio format for soundfile: {config.AUDIO_FORMAT}"); return

            # Open the sound file for writing
            sound_file = sf.SoundFile(
                self.temp_audio_file_path, mode='w',
                samplerate=config.AUDIO_SAMPLE_RATE, channels=config.AUDIO_CHANNELS, subtype=subtype
            )
            logging.info(f"Audio write thread: Opened {self.temp_audio_file_path} (subtype: {subtype}) for writing.")

            while True:
                try:
                    audio_data = self.audio_queue.get(block=True, timeout=1.0)
                    if audio_data is None: # Sentinel value
                        logging.info("Audio write thread: Received stop signal."); break
                    sound_file.write(audio_data); data_written = True
                except queue.Empty:
                    if self.stop_audio_event.is_set(): logging.info("Audio write thread: Stop event detected."); break
                    continue # Continue waiting
                except Exception as write_err:
                     logging.error(f"!!! Error writing audio data to file: {write_err}", exc_info=True)
                     self.audio_last_error = f"Audio File Write Error: {write_err}"
                     # Should we break here? Let's continue to try writing remaining data.

        except Exception as e:
            logging.error(f"!!! Error in audio write thread setup/loop: {e}", exc_info=True)
            self.audio_last_error = f"Audio Write Thread Error: {e}"
        finally:
            if sound_file:
                try: sound_file.close(); logging.info(f"Audio write thread: Closed {self.temp_audio_file_path}.")
                except Exception as e_close: logging.error(f"Error closing sound file: {e_close}")
            # Check if file exists and has data before declaring success
            if not data_written and self.temp_audio_file_path and os.path.exists(self.temp_audio_file_path):
                 logging.warning(f"Audio write thread finished, but no data was written to {self.temp_audio_file_path}.")
                 # Optionally delete empty file here?
            logging.info("Audio write thread finished.")

    def _start_audio_recording(self, base_filename):
        """Starts the audio recording threads and sets up the temporary file."""
        if not config.AUDIO_ENABLED or not self.audio_dtype: return False
        # Ensure previous threads/files are cleaned up (should be handled by stop)
        if self.audio_thread and self.audio_thread.is_alive():
             logging.warning("Audio start called while previous thread alive. Attempting stop first.")
             self._stop_audio_recording()

        if not self._find_audio_device() or self.audio_device_index is None:
            self.audio_last_error = "Audio Start Failed: No suitable device found."
            return False

        try:
            temp_dir = tempfile.gettempdir()
            self.temp_audio_file_path = os.path.join(temp_dir, f"{base_filename}_audio{config.AUDIO_TEMP_EXTENSION}")
            logging.info(f"Starting audio recording to temp file: {self.temp_audio_file_path}")

            # Clear queue
            while not self.audio_queue.empty():
                 try: self.audio_queue.get_nowait()
                 except queue.Empty: break

            self.stop_audio_event.clear()
            # Start capture thread first
            self.audio_thread = threading.Thread(target=self._audio_recording_thread, name="AudioCaptureThread")
            self.audio_thread.daemon = True
            self.audio_thread.start()
            # Give capture thread a moment to start stream before starting writer
            time.sleep(0.2)
            # Start writer thread
            self.audio_write_thread = threading.Thread(target=self._audio_write_file_thread, name="AudioWriteThread")
            self.audio_write_thread.daemon = True
            self.audio_write_thread.start()

            self.audio_last_error = None # Clear error on successful start
            return True

        except Exception as e:
            logging.error(f"!!! Failed to start audio recording setup: {e}", exc_info=True)
            self.audio_last_error = f"Audio Start Error: {e}"
            # Cleanup partial start
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
        # Check if threads exist and are alive
        capture_thread_running = self.audio_thread and self.audio_thread.is_alive()
        write_thread_running = self.audio_write_thread and self.audio_write_thread.is_alive()

        if not capture_thread_running and not write_thread_running:
            logging.info("Audio recording threads not running.")
            # Still return path if it exists and seems valid
            if self.temp_audio_file_path and os.path.exists(self.temp_audio_file_path) and os.path.getsize(self.temp_audio_file_path) > 44:
                 return self.temp_audio_file_path
            else:
                 # Clean up potentially empty/invalid file
                 if self.temp_audio_file_path and os.path.exists(self.temp_audio_file_path):
                      try: os.remove(self.temp_audio_file_path); logging.info("Cleaned up empty/invalid temp audio file.")
                      except OSError: pass
                 self.temp_audio_file_path = None
                 return None

        logging.info("Stopping audio recording threads...")
        self.stop_audio_event.set() # Signal threads

        # Wait for capture thread
        if capture_thread_running:
            self.audio_thread.join(timeout=2.0)
            if self.audio_thread.is_alive(): logging.warning("Audio capture thread did not stop cleanly.")

        # Wait for write thread
        if write_thread_running:
            self.audio_write_thread.join(timeout=5.0) # Longer timeout for writing
            if self.audio_write_thread.is_alive(): logging.warning("Audio write thread did not stop cleanly.")

        self.audio_thread = None; self.audio_write_thread = None; self.audio_stream = None
        final_path = self.temp_audio_file_path # Store path before clearing
        self.temp_audio_file_path = None # Clear path variable

        # Return the path if file exists and seems valid
        if final_path and os.path.exists(final_path):
             if os.path.getsize(final_path) > 44: # Basic WAV header check
                 return final_path
             else:
                 logging.warning(f"Temporary audio file {final_path} seems empty/invalid after stop.")
                 try: os.remove(final_path); logging.info("Cleaned up empty/invalid temp audio file.")
                 except OSError: pass
                 return None
        else:
            logging.error("Temporary audio file path not set or file does not exist after stop.")
            return None

    def _mux_audio_video(self, video_path, audio_path):
        """Merges audio and video files using ffmpeg."""
        # (Code is identical to previous version - omitted for brevity)
        if not config.AUDIO_ENABLED: return None
        if not os.path.exists(config.FFMPEG_PATH): logging.error(f"ffmpeg not found at {config.FFMPEG_PATH}. Cannot mux."); self.audio_last_error = "Mux Error: ffmpeg not found"; return None
        if not os.path.exists(video_path): logging.error(f"Video file not found: {video_path}. Cannot mux."); self.audio_last_error = "Mux Error: Video file missing"; return None
        if not os.path.exists(audio_path): logging.error(f"Audio file not found: {audio_path}. Cannot mux."); self.audio_last_error = "Mux Error: Audio file missing"; return None
        output_path = video_path.replace("_video" + config.CAM0_RECORDING_EXTENSION, config.CAM0_RECORDING_EXTENSION)
        if output_path == video_path: output_path = video_path.replace(config.CAM0_RECORDING_EXTENSION, "_muxed" + config.CAM0_RECORDING_EXTENSION)
        logging.info(f"Muxing video '{os.path.basename(video_path)}' and audio '{os.path.basename(audio_path)}' into '{os.path.basename(output_path)}'...")
        command = [config.FFMPEG_PATH, "-y", "-i", video_path, "-i", audio_path, "-c", "copy", "-map", "0:v:0", "-map", "1:a:0", "-shortest", "-loglevel", config.FFMPEG_LOG_LEVEL, output_path]
        try:
            process = subprocess.run(command, capture_output=True, text=True, check=True, timeout=config.AUDIO_MUX_TIMEOUT)
            logging.info(f"ffmpeg muxing successful for {output_path}."); logging.debug(f"ffmpeg output:\n{process.stdout}\n{process.stderr}"); self.audio_last_error = None
            return output_path
        except subprocess.TimeoutExpired: logging.error(f"!!! ffmpeg muxing timed out ({config.AUDIO_MUX_TIMEOUT}s) for {output_path}."); self.audio_last_error = "Mux Error: ffmpeg timed out";
        except subprocess.CalledProcessError as e: logging.error(f"!!! ffmpeg muxing failed for {output_path}. Code: {e.returncode}"); logging.error(f"ffmpeg stderr:\n{e.stderr}"); logging.error(f"ffmpeg stdout:\n{e.stdout}"); self.audio_last_error = f"Mux Error: ffmpeg failed (code {e.returncode})"
        except Exception as e: logging.error(f"!!! Unexpected error during ffmpeg execution: {e}", exc_info=True); self.audio_last_error = f"Mux Error: {e}"
        # Cleanup failed output file
        if os.path.exists(output_path):
             try: os.remove(output_path)
             except OSError: pass
        return None


    def shutdown(self):
        """Stops recording, stops and closes both cameras."""
        # (Code is identical to previous version - omitted for brevity, includes audio stop)
        logging.info("CameraManager shutting down...")
        if self.is_recording: self.stop_recording()
        else:
            if config.AUDIO_ENABLED and (self.audio_thread and self.audio_thread.is_alive()): logging.warning("Shutting down orphaned audio recording..."); self._stop_audio_recording()
        for cam_id, picam_instance, name in [(config.CAM0_ID, self.picam0, "Cam0"), (config.CAM1_ID, self.picam1, "Cam1")]:
            if picam_instance:
                try:
                    if picam_instance.started: logging.info(f"Stopping {name}..."); picam_instance.stop()
                    logging.info(f"Closing {name}..."); picam_instance.close()
                except Exception as e: logging.error(f"Error stopping/closing {name}: {e}")
                finally:
                    if cam_id == config.CAM0_ID: self.picam0 = None; self.is_initialized0 = False
                    else: self.picam1 = None; self.is_initialized1 = False
        logging.info("CameraManager shutdown complete.")

# Example Usage (Main testing block - unchanged, but should now test audio if enabled)
if __name__ == "__main__":
    # (Code is identical to previous version - omitted for brevity)
    logging.basicConfig(level=config.LOG_LEVEL, format=config.LOG_FORMAT, datefmt=config.LOG_DATE_FORMAT)
    logging.info("--- Camera Manager Dual Cam Test ---")
    cam_manager = CameraManager()
    try:
        if not cam_manager.initialize_cameras(): print(f"Camera init failed: {cam_manager.last_error}"); exit()
        print("Cameras initialized."); print(f"Initial State: {cam_manager.get_camera_state()}")
        print("\nCapturing combined frames...");
        for i in range(10):
            frame = cam_manager.capture_and_combine_frames()
            if frame is not None: print(f"Frame {i+1} captured, shape: {frame.shape}"); cv2.imshow("Combined Stream", frame);
            else: print(f"Frame {i+1} capture failed. Error: {cam_manager.last_error}")
            if cv2.waitKey(100) & 0xFF == ord('q'): break
        cv2.destroyAllWindows()
        print("\nTesting recording...");
        if get_usb_mounts():
             if cam_manager.start_recording():
                 print("Recording started. Recording for 5s..."); start_time = time.time()
                 while time.time() - start_time < 5:
                     frame = cam_manager.capture_and_combine_frames();
                     if frame is None: print("Capture failed during rec test.")
                     cv2.imshow("Combined Stream", frame);
                     if cv2.waitKey(1) & 0xFF == ord('q'): break
                 print("Stopping recording..."); cam_manager.stop_recording()
                 print(f"Recording stopped. Check USB drive(s) in {config.USB_BASE_PATH}"); print(f"Final state: {cam_manager.get_camera_state()}")
             else: print(f"Failed start recording. Error: {cam_manager.last_error} / Audio: {cam_manager.audio_last_error}")
        else: print("No writable USB drives, skipping recording test.")
        cv2.destroyAllWindows()
        print("\nTesting control change (ISO)..."); initial_state = cam_manager.get_camera_state(); print(f"Initial ISO: {initial_state.get('iso_mode')} (Gain: {initial_state.get('analogue_gain')})")
        current_iso_name = initial_state.get('iso_mode', config.DEFAULT_ISO_NAME); iso_names = list(config.AVAILABLE_ISO_SETTINGS.keys());
        try: current_idx = iso_names.index(current_iso_name); next_idx = (current_idx + 1) % len(iso_names)
        except ValueError: next_idx = 0
        next_iso_name = iso_names[next_idx]; next_analogue_gain = config.AVAILABLE_ISO_SETTINGS[next_iso_name]
        print(f"Changing ISO to {next_iso_name} (Gain: {next_analogue_gain:.2f})")
        if cam_manager.apply_camera_controls({'AnalogueGain': next_analogue_gain}):
             print(f"ISO change requested. Capturing frame..."); time.sleep(1.5); frame = cam_manager.capture_and_combine_frames()
             if frame is not None: print("Frame captured after ISO change."); cv2.imshow("ISO Change Test", frame); cv2.waitKey(1000); cv2.destroyWindow("ISO Change Test")
             final_state = cam_manager.get_camera_state(); print(f"Final ISO state: {final_state.get('iso_mode')} (Gain: {final_state.get('analogue_gain')})")
        else: print(f"Failed set ISO. Error: {cam_manager.last_error}")
        print("\nTesting Cam0 resolution change..."); current_index0 = cam_manager.get_camera_state()['resolution_index']; next_index0 = (current_index0 + 1) % len(config.CAM0_RESOLUTIONS)
        if next_index0 == current_index0 and len(config.CAM0_RESOLUTIONS) > 1: next_index0 = (current_index0 - 1 + len(config.CAM0_RESOLUTIONS)) % len(config.CAM0_RESOLUTIONS)
        if next_index0 != current_index0:
            print(f"Changing Cam0 res index {current_index0} -> {next_index0}...")
            if cam_manager.initialize_cameras(next_index0):
                print("Resolution change successful."); print(f"New State: {cam_manager.get_camera_state()}"); print("Capturing frame..."); frame = cam_manager.capture_and_combine_frames()
                if frame is not None: print(f"Frame captured, shape: {frame.shape}"); cv2.imshow("Res Change Test", frame); cv2.waitKey(1000); cv2.destroyWindow("Res Change Test")
            else: print(f"Resolution change failed. Error: {cam_manager.last_error}")
        else: print("Only one res available/change failed, skipping reconfig test.")
    except KeyboardInterrupt: print("\nExiting test.")
    except Exception as e: print(f"\nError during test: {e}"); logging.exception("Test failed")
    finally: print("Shutting down camera manager..."); cam_manager.shutdown(); cv2.destroyAllWindows(); print("--- Test Complete ---")

