# -*- coding: utf-8 -*-
"""
camera_manager.py

Manages multiple Picamera2 cameras, including initialization, configuration,
frame capture (combined stream), and video recording (from primary camera).

**Modification 15:** Reverted to recording thread model, now piping raw frames
                     to an external ffmpeg process for encoding and saving to
                     address file compatibility issues.
"""

import os
import time
import datetime
import logging
import threading
import numpy as np
from picamera2 import Picamera2
# Removed Picamera2 encoder/output imports
import queue # Re-added queue for ffmpeg thread
import shutil
from libcamera import controls, Transform
import subprocess # Re-added for Popen

# Import configuration constants
import config

# Helper function to find writable USB mounts (unchanged)
def get_usb_mounts():
    """Finds writable directories under the configured USB base path."""
    # ... (function unchanged) ...
    mounts = []; base_path = config.USB_BASE_PATH
    logging.debug(f"USB Check: Base path = {base_path}")
    if not os.path.isdir(base_path): logging.warning(f"USB base path '{base_path}' missing."); return mounts
    try:
        items = os.listdir(base_path); logging.debug(f"USB Check: Items found = {items}")
        for item in items:
            path = os.path.join(base_path, item); logging.debug(f"USB Check: Checking '{item}' at '{path}'")
            if os.path.isdir(path):
                logging.debug(f"USB Check: '{path}' is dir.")
                if os.access(path, os.W_OK):
                    logging.debug(f"USB Check: '{path}' has W_OK.")
                    test_file = os.path.join(path, f".write_test_{os.getpid()}_{time.time_ns()}")
                    try:
                        with open(test_file, 'w') as f: f.write('test'); os.remove(test_file)
                        mounts.append(path); logging.debug(f"USB Check: Write test OK for '{path}'. Added.")
                    except Exception as write_err: logging.warning(f"USB Check: Write test failed for '{path}': {write_err}")
                else: logging.debug(f"USB Check: '{path}' lacks W_OK.")
            else: logging.debug(f"USB Check: '{path}' not dir.")
        if not mounts: logging.debug("USB Check: No writable mounts found.")
        else: logging.info(f"Found {len(mounts)} writable USB mount(s): {mounts}")
    except Exception as e: logging.error(f"Error finding USB mounts: {e}")
    return mounts


class CameraManager:
    """Handles camera operations, now using ffmpeg pipe for recording."""

    def __init__(self):
        """Initializes the CameraManager (ffmpeg pipe recording)."""
        self.picam0 = None; self.picam1 = None
        self.is_initialized0 = False; self.is_initialized1 = False
        self.last_error = None
        self.current_resolution_index0 = config.CAM0_DEFAULT_RESOLUTION_INDEX
        self.actual_cam0_fps = None; self.last_cam0_capture_time = None; self.measured_cam0_fps_avg = None
        self.output_frame = None; self.latest_raw_frame0 = None

        # --- Recording State (Using ffmpeg via Thread/Queue) ---
        self.is_recording = False
        self.recording_target_fps = None
        self.primary_recording_path = None # Path ffmpeg writes to
        self.target_recording_paths = []   # All desired final paths
        self.recording_start_time_monotonic = None
        self.recording_frame_queue = None # Re-added queue
        self.recording_thread = None # Re-added thread
        self.stop_recording_event = threading.Event() # Re-added event

        # Locks
        self.frame_lock = threading.Lock(); self.raw_frame_lock = threading.Lock()
        self.config_lock = threading.Lock(); self.recording_lock = threading.Lock()

        # --- Camera Controls ---
        # ... (control initializations unchanged) ...
        self.current_analogue_gain = config.DEFAULT_ANALOGUE_GAIN; self.current_iso_name = "Unknown"
        self.current_ae_mode = controls.AeExposureModeEnum.Normal; self.current_metering_mode = controls.AeMeteringModeEnum.CentreWeighted
        self.current_noise_reduction_mode = controls.draft.NoiseReductionModeEnum.Off
        self.current_brightness = config.DEFAULT_BRIGHTNESS; self.current_contrast = config.DEFAULT_CONTRAST
        self.current_saturation = config.DEFAULT_SATURATION; self.current_sharpness = config.DEFAULT_SHARPNESS
        try:
            for name, gain in config.AVAILABLE_ISO_SETTINGS.items():
                if abs(gain - self.current_analogue_gain) < 0.01: self.current_iso_name = name; break
        except Exception as e: logging.error(f"Error setting default ISO name: {e}.")
        try: self.current_ae_mode = controls.AeExposureModeEnum.__members__[config.DEFAULT_AE_MODE_NAME]
        except: logging.error(f"Default AE mode '{config.DEFAULT_AE_MODE_NAME}' invalid.")
        try: self.current_metering_mode = controls.AeMeteringModeEnum.__members__[config.DEFAULT_METERING_MODE_NAME]
        except: logging.error(f"Default Metering mode '{config.DEFAULT_METERING_MODE_NAME}' invalid.")
        try: self.current_noise_reduction_mode = controls.draft.NoiseReductionModeEnum.__members__[config.DEFAULT_NOISE_REDUCTION_MODE_NAME]
        except: logging.warning(f"Default NR mode '{config.DEFAULT_NOISE_REDUCTION_MODE_NAME}' invalid/unavailable.")

        logging.info("CameraManager initialized (Using FFMPEG Pipe Recording, Audio Disabled).")
        if not config.ENABLE_CAM1: logging.warning("Cam1 is DISABLED.")
        # ... (debug log unchanged) ...
        logging.debug(f"Initial Controls: ISO={self.current_iso_name}({self.current_analogue_gain:.2f}), AE={self.current_ae_mode.name}, Metering={self.current_metering_mode.name}, NR={self.current_noise_reduction_mode.name}, Bright={self.current_brightness}, Contr={self.current_contrast}, Sat={self.current_saturation}, Sharp={self.current_sharpness}")


    def _initialize_camera(self, cam_id, resolution_index=None):
        """Initializes camera. Stops ffmpeg recording if active."""
        # ... (Initialization logic largely unchanged, but ensures recording thread is stopped) ...
        picam_instance = None; is_initialized_flag = False; cam_name = f"Cam{cam_id}"; actual_fps_reported = None
        if cam_id == config.CAM1_ID and not config.ENABLE_CAM1: logging.info(f"Skipping initialization of {cam_name} (disabled)."); self.picam1 = None; self.is_initialized1 = False; return True
        # ... (Get target W, H, FPS logic unchanged) ...
        if cam_id == config.CAM0_ID:
             if resolution_index is not None:
                if 0 <= resolution_index < len(config.CAM0_RESOLUTIONS): self.current_resolution_index0 = resolution_index
                else: logging.error(f"{cam_name}: Invalid res index {resolution_index}. Using current {self.current_resolution_index0}.")
             res_list = config.CAM0_RESOLUTIONS; current_res_index = self.current_resolution_index0; tuning_data = config.CAM0_TUNING
             try: target_width, target_height, target_fps = res_list[current_res_index]
             except IndexError: logging.error(f"{cam_name}: Res index {current_res_index} out of bounds. Using default."); current_res_index = config.CAM0_DEFAULT_RESOLUTION_INDEX; self.current_resolution_index0 = current_res_index; target_width, target_height, target_fps = res_list[current_res_index]
        elif cam_id == config.CAM1_ID: target_width, target_height = config.CAM1_RESOLUTION; target_fps = config.CAM1_FRAME_RATE; tuning_data = config.CAM1_TUNING; current_res_index = 0
        else: logging.error(f"Invalid camera ID {cam_id}"); return False
        logging.info(f"Attempting init {cam_name} (ID: {cam_id}) at index {current_res_index} ({target_width}x{target_height} @ Target {target_fps:.1f}fps)...")
        existing_picam = self.picam0 if cam_id == config.CAM0_ID else self.picam1
        if existing_picam is not None:
            try:
                # *** Stop FFMPEG RECORDING THREAD before stopping camera ***
                if cam_id == config.CAM0_ID and self.is_recording:
                    logging.warning(f"Stopping active ffmpeg recording on {cam_name} due to re-initialization.")
                    self.stop_recording() # Stops thread, waits for ffmpeg

                if existing_picam.started: logging.info(f"Stopping existing {cam_name}..."); existing_picam.stop()
                logging.info(f"Closing existing {cam_name}..."); existing_picam.close()
            except Exception as e: logging.warning(f"Error stopping/closing previous {cam_name}: {e}")
            finally:
                if cam_id == config.CAM0_ID: self.picam0 = None; self.is_initialized0 = False; self.actual_cam0_fps = None; # No encoder/output refs
                elif cam_id == config.CAM1_ID: self.picam1 = None; self.is_initialized1 = False
            time.sleep(0.5)
        # ... (Camera creation, configuration, start, verification mostly unchanged) ...
        try:
            picam_instance = Picamera2(camera_num=cam_id, tuning=tuning_data); logging.info(f"{cam_name}: Picamera2 object created.")
            controls_to_set = {"FrameRate": target_fps, "NoiseReductionMode": self.current_noise_reduction_mode,"AeEnable": True,"AeExposureMode": self.current_ae_mode,"AeMeteringMode": self.current_metering_mode,"AnalogueGain": self.current_analogue_gain,"Brightness": self.current_brightness,"Contrast": self.current_contrast,"Saturation": self.current_saturation,"Sharpness": self.current_sharpness,}
            if self.current_analogue_gain == 0.0: controls_to_set["AeEnable"] = True
            controls_to_set = {k: v for k, v in controls_to_set.items() if v is not None}
            transform = Transform();
            if cam_id == config.CAM1_ID:
                if config.CAM1_VFLIP: transform.vflip = True;
                if config.CAM1_HFLIP: transform.hflip = True
            cam_config = picam_instance.create_video_configuration(main={"size": (target_width, target_height), "format": "RGB888"}, controls=controls_to_set, transform=transform)
            logging.info(f"{cam_name}: Configuring with: main={cam_config['main']}, controls={cam_config['controls']}")
            picam_instance.configure(cam_config); time.sleep(0.5)
            new_config = picam_instance.camera_configuration()
            if new_config:
                applied_controls = new_config.get('controls', {}); applied_main = new_config.get('main', {})
                logging.info(f"{cam_name}: Verified Config: main={applied_main}, controls={applied_controls}")
                actual_fps_reported = applied_controls.get('FrameRate')
                if actual_fps_reported is not None:
                    logging.info(f"{cam_name}: Driver reported FrameRate: {actual_fps_reported:.2f} fps")
                    if abs(actual_fps_reported - target_fps) > 1.0: logging.warning(f"{cam_name}: Driver adjusted FPS from target {target_fps:.1f} to {actual_fps_reported:.2f}")
                else: logging.warning(f"{cam_name}: Could not read back FrameRate."); actual_fps_reported = target_fps
            else: logging.warning(f"{cam_name}: Could not get config after applying."); actual_fps_reported = target_fps
            logging.info(f"{cam_name}: Starting camera..."); picam_instance.start(); logging.info(f"{cam_name}: Camera started"); time.sleep(1.0)
            actual_config_after_start = picam_instance.camera_configuration()
            if not actual_config_after_start: raise RuntimeError(f"{cam_name}: Failed get config after start.")
            actual_format=actual_config_after_start.get('main',{}); actual_w=actual_format.get('size',(0,0))[0]; actual_h=actual_format.get('size',(0,0))[1]; actual_fmt_str=actual_format.get('format','Unknown')
            actual_fps_final = actual_config_after_start.get('controls', {}).get('FrameRate', actual_fps_reported)
            actual_gain = actual_config_after_start.get('controls', {}).get('AnalogueGain', 'N/A')
            logging.info(f"{cam_name}: Initialized. Stream: {actual_w}x{actual_h} {actual_fmt_str} @ {actual_fps_final:.2f} fps. Gain: {actual_gain}")
            is_initialized_flag = True; self.last_error = None; return True
        except Exception as e:
            logging.error(f"!!! Failed initialize {cam_name} at {target_width}x{target_height}: {e}", exc_info=True); self.last_error = f"{cam_name} Init Error: {e}"
            if picam_instance is not None:
                try:
                    if picam_instance.started: picam_instance.stop()
                    picam_instance.close()
                except Exception as close_e: logging.error(f"Error closing {cam_name} after failure: {close_e}")
            picam_instance = None; is_initialized_flag = False; actual_fps_reported = None; return False
        finally:
            if cam_id == config.CAM0_ID: self.picam0 = picam_instance; self.is_initialized0 = is_initialized_flag; self.actual_cam0_fps = actual_fps_reported; self.last_cam0_capture_time = None; self.measured_cam0_fps_avg = None
            elif cam_id == config.CAM1_ID: self.picam1 = picam_instance; self.is_initialized1 = is_initialized_flag

    def initialize_cameras(self, resolution_index=None):
        """Initializes or re-initializes camera(s) based on config."""
        # ... (logic unchanged) ...
        with self.config_lock:
            logging.info("--- Initializing Camera(s) ---"); success0 = self._initialize_camera(config.CAM0_ID, resolution_index); success1 = True
            if config.ENABLE_CAM1: success1 = self._initialize_camera(config.CAM1_ID)
            else: logging.info("Cam1 initialization skipped (disabled)."); self.picam1 = None; self.is_initialized1 = False
            if success0 and success1: cam_count = "One" if not config.ENABLE_CAM1 else "Both"; logging.info(f"--- {cam_count} Camera(s) Initialized Successfully ---"); return True
            else:
                logging.error("!!! Failed initialize cameras. Check logs. !!!")
                if not success0 and not self.last_error: self.last_error = "Cam0 Initialization Failed"
                if not success1 and config.ENABLE_CAM1 and not self.last_error: self.last_error = "Cam1 Initialization Failed"
                elif not success1 and config.ENABLE_CAM1: self.last_error += " / Cam1 Initialization Failed"
                return False

    def get_cam0_resolution_config(self):
        """Returns the configured resolution tuple (width, height, target_fps) for Cam0."""
        # ... (logic unchanged) ...
        try: return config.CAM0_RESOLUTIONS[self.current_resolution_index0]
        except IndexError: logging.error(f"Invalid resolution index {self.current_resolution_index0}. Using default."); safe_default_index = max(0, min(len(config.CAM0_RESOLUTIONS) - 1, config.CAM0_DEFAULT_RESOLUTION_INDEX)); self.current_resolution_index0 = safe_default_index; return config.CAM0_RESOLUTIONS[safe_default_index]

    def apply_camera_controls(self, controls_dict):
        """Applies a dictionary of common controls to running cameras."""
        # ... (logic unchanged) ...
        if not self.is_initialized0 and (config.ENABLE_CAM1 and not self.is_initialized1): logging.error("Cannot apply controls: No cameras initialized."); self.last_error = "Control Apply Error: No cameras ready."; return False
        if not self.is_initialized0: logging.error("Cannot apply controls: Cam0 not initialized."); self.last_error = "Control Apply Error: Cam0 not ready."; return False
        logging.info(f"Applying common controls: {controls_dict}"); success_count = 0; temp_last_error = None
        try:
            with self.config_lock:
                iso_name_updated = False
                for key, value in controls_dict.items():
                    if key == 'AnalogueGain': 
                        self.current_analogue_gain = value; self.current_iso_name = "Unknown"; 
                        try:
                            for name, gain in config.AVAILABLE_ISO_SETTINGS.items():
                                if abs(gain - value) < 0.01: self.current_iso_name = name; iso_name_updated = True; break
                            if not iso_name_updated: logging.warning(f"Applied AnalogueGain {value} not known ISO.")
                        except Exception as e: logging.error(f"Error updating ISO name for gain {value}: {e}")
                    elif key == 'AeExposureMode': self.current_ae_mode = value; 
                    elif key == 'AeMeteringMode': self.current_metering_mode = value
                    elif key == 'NoiseReductionMode': self.current_noise_reduction_mode = value; 
                    elif key == 'Brightness': self.current_brightness = value
                    elif key == 'Contrast': self.current_contrast = value; 
                    elif key == 'Saturation': self.current_saturation = value; 
                    elif key == 'Sharpness': self.current_sharpness = value
                logging.debug(f"Applying to Cam0...");
                if self.is_initialized0 and self.picam0 and self.picam0.started:
                    try: self.picam0.set_controls(controls_dict); success_count += 1
                    except Exception as e0: logging.error(f"!!! Error applying controls to Cam0: {e0}"); temp_last_error = f"Cam0 Control Error: {e0}"
                else: logging.warning("Skipping control application for Cam0 (not ready).")
                if config.ENABLE_CAM1:
                    logging.debug(f"Applying to Cam1...");
                    if self.is_initialized1 and self.picam1 and self.picam1.started:
                        try: self.picam1.set_controls(controls_dict); success_count += 1
                        except Exception as e1: logging.error(f"!!! Error applying controls to Cam1: {e1}"); err_msg = f"Cam1 Control Error: {e1}"; temp_last_error = (temp_last_error + " / " + err_msg) if temp_last_error else err_msg
                    else: logging.warning("Skipping control application for Cam1 (not ready).")
                else: logging.debug("Skipping control application for Cam1 (disabled).")
            required_successes = 1 if not config.ENABLE_CAM1 else 2
            if success_count > 0:
                if any(k in controls_dict for k in ['AnalogueGain', 'AeExposureMode', 'AeMeteringMode', 'Brightness', 'ExposureTime']): time.sleep(0.5)
                else: time.sleep(0.1)
                if temp_last_error is None and success_count >= required_successes: self.last_error = None
                logging.info(f"Controls applied successfully to {success_count} camera(s)."); return True
            else: self.last_error = temp_last_error if temp_last_error else "Control Apply Error: Failed"; logging.error(f"Failed apply controls. Last Error: {self.last_error}"); return False
        except Exception as e: logging.error(f"!!! Unexpected Error during control application: {e}", exc_info=True); self.last_error = f"Control Apply Unexpected Error: {e}"; return False


    def get_camera_state(self):
        """Returns a dictionary containing the current camera state."""
        # ... (logic unchanged) ...
        with self.config_lock:
            res_w0, res_h0, target_fps0 = self.get_cam0_resolution_config(); output_w = res_w0; output_h = res_h0
            if config.ENABLE_CAM1 and self.is_initialized1: res_w1, res_h1 = config.CAM1_RESOLUTION; display_w1 = min(res_w1, output_w); display_h1 = res_h1 if display_w1 == res_w1 else int(res_h1 * (display_w1 / res_w1)); output_h = res_h0 + display_h1 + config.STREAM_BORDER_SIZE
            ae_mode_name = getattr(self.current_ae_mode, 'name', str(self.current_ae_mode)); metering_mode_name = getattr(self.current_metering_mode, 'name', str(self.current_metering_mode)); nr_mode_name = getattr(self.current_noise_reduction_mode, 'name', str(self.current_noise_reduction_mode))
            state = {'is_initialized': self.is_initialized0 and (not config.ENABLE_CAM1 or self.is_initialized1),'is_initialized0': self.is_initialized0,'is_initialized1': self.is_initialized1,'is_cam1_enabled': config.ENABLE_CAM1,'resolution_index': self.current_resolution_index0,'resolution_wh': (res_w0, res_h0),'output_frame_wh': (output_w, output_h),'target_cam0_fps': target_fps0,'actual_cam0_fps': self.actual_cam0_fps,'measured_cam0_fps': self.measured_cam0_fps_avg,'recording_target_fps': self.recording_target_fps,'is_recording': self.is_recording,'recording_paths': list(self.target_recording_paths),'last_error': self.last_error,'iso_mode': self.current_iso_name,'analogue_gain': self.current_analogue_gain,'ae_mode': ae_mode_name,'metering_mode': metering_mode_name,'noise_reduction_mode': nr_mode_name,'brightness': self.current_brightness,'contrast': self.current_contrast,'saturation': self.current_saturation,'sharpness': self.current_sharpness,}
        return state

    # ===========================================================
    # === Recording Methods (Using FFMPEG Pipe) ===
    # ===========================================================

    def start_recording(self):
        """Starts recording video FROM CAM0 by piping frames to ffmpeg."""
        with self.recording_lock:
            if self.is_recording: logging.warning("Start recording called, but already recording."); return True
            if not self.is_initialized0 or not self.picam0 or not self.picam0.started: logging.error("Cannot start recording, Cam0 not available."); self.last_error = "Cam0 not available"; return False
            logging.info("Attempting to start video recording (using ffmpeg pipe)..."); usb_drives = get_usb_mounts()
            if not usb_drives: logging.error(f"Cannot start recording: No writable USB drives found."); self.last_error = f"No writable USB drives"; return False

            try: # Get params and set up paths
                width, height, target_fps = self.get_cam0_resolution_config()
                if target_fps <= 0: logging.error(f"Invalid TARGET FPS ({target_fps})."); self.last_error = "Invalid Target FPS"; return False
                if width <= 0 or height <= 0: raise ValueError(f"Invalid dimensions: {width}x{height}")
                self.recording_target_fps = target_fps; logging.info(f"Recording parameters: {width}x{height} @ Target {target_fps:.2f} fps")
                timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S"); base_filename = f"recording_{timestamp}_{width}x{height}.mp4" # Assume mp4 output
                self.primary_recording_path = os.path.join(usb_drives[0], base_filename); self.target_recording_paths = [os.path.join(drive, base_filename) for drive in usb_drives]
                logging.info(f"Primary recording path (ffmpeg target): {self.primary_recording_path}")
                if len(self.target_recording_paths) > 1: logging.info(f"Target paths for duplication: {self.target_recording_paths[1:]}")
            except Exception as setup_err:
                logging.error(f"!!! Error setting up recording parameters/paths: {setup_err}", exc_info=True); self.last_error = f"Rec Setup Error: {setup_err}"
                self.recording_target_fps = None; self.primary_recording_path = None; self.target_recording_paths = []; return False

            # --- Construct ffmpeg command ---
            ffmpeg_cmd = [
                config.FFMPEG_PATH,
                '-loglevel', config.FFMPEG_LOG_LEVEL,
                '-f', 'rawvideo',                       # Input format: raw video
                '-pix_fmt', config.FFMPEG_INPUT_PIX_FMT,# Input pixel format (rgb24 for RGB888)
                '-s', f'{width}x{height}',              # Input size
                '-r', str(target_fps),                  # Input framerate *** IMPORTANT ***
                '-i', '-',                              # Input source: stdin pipe
                '-c:v', 'libx264',                      # Output codec: H.264
                '-preset', config.FFMPEG_H264_PRESET,   # Encoding speed/compression preset
                '-b:v', config.FFMPEG_VIDEO_BITRATE,    # Output video bitrate
                '-pix_fmt', config.FFMPEG_OUTPUT_PIX_FMT,# Output pixel format (yuv420p for compatibility)
                '-an',                                  # No audio input/output
                '-y',                                   # Overwrite output file if exists
                self.primary_recording_path             # Output file path
            ]
            logging.info(f"FFMPEG command: {' '.join(ffmpeg_cmd)}")

            # --- Setup and Start Recording Thread ---
            try:
                queue_size = max(10, int(target_fps)) # Buffer ~1 sec or 10 frames
                self.recording_frame_queue = queue.Queue(maxsize=queue_size)
                self.stop_recording_event.clear()
                self.recording_thread = threading.Thread(
                    target=self._recording_thread_loop,
                    args=(ffmpeg_cmd, self.primary_recording_path), # Pass command and path
                    name="FFMPEGWriteThread"
                )
                self.recording_thread.daemon = True
                self.recording_thread.start()
                self.is_recording = True # Set flag only after thread setup
                self.recording_start_time_monotonic = time.monotonic()
                logging.info(f"FFMPEG recording thread started, writing to {self.primary_recording_path}.")
                if self.last_error and ("Recording" in self.last_error or "USB" in self.last_error or "sync" in self.last_error): logging.info(f"Clearing previous recording error: '{self.last_error}'"); self.last_error = None
                return True
            except Exception as thread_err:
                logging.error(f"!!! Failed to start ffmpeg recording thread: {thread_err}", exc_info=True); self.last_error = f"Rec Thread Start Error: {thread_err}"
                self.is_recording = False; self.recording_thread = None; self.recording_frame_queue = None; self.recording_target_fps = None; self.primary_recording_path = None; self.target_recording_paths = []; return False


    def _recording_thread_loop(self, ffmpeg_cmd, primary_path):
        """Dedicated thread to run ffmpeg and pipe frames to it."""
        ffmpeg_proc = None
        frames_written = 0
        last_log_time = time.monotonic()
        logging.info(f"FFMPEG recording thread loop started for: {primary_path}")

        try:
            # Start the ffmpeg process
            ffmpeg_proc = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)
            logging.info(f"Launched ffmpeg process (PID: {ffmpeg_proc.pid}). Waiting for frames...")

            while not self.stop_recording_event.is_set():
                frame_to_write = None
                try:
                    # Get frame from queue (blocking with timeout)
                    # Timestamp is captured but not used by ffmpeg pipe directly
                    frame_data, _ = self.recording_frame_queue.get(block=True, timeout=0.5)
                    if frame_data is None: # Check for sentinel
                         logging.info("FFMPEG thread: Received stop sentinel.")
                         break # Exit loop gracefully
                    else:
                         frame_to_write = frame_data
                except queue.Empty:
                    # No frame available within timeout, check stop event and continue
                    continue
                except Exception as q_err:
                     logging.error(f"FFMPEG thread: Error getting frame from queue: {q_err}", exc_info=True)
                     time.sleep(0.1); continue # Avoid busy-looping

                # --- Pipe Frame Bytes to ffmpeg ---
                if frame_to_write is not None and ffmpeg_proc and ffmpeg_proc.stdin:
                    try:
                        ffmpeg_proc.stdin.write(frame_to_write.tobytes())
                        frames_written += 1
                    except BrokenPipeError:
                        logging.error("!!! FFMPEG thread: BrokenPipeError. ffmpeg process likely terminated unexpectedly.")
                        self.last_error = "FFMPEG process terminated"
                        break # Exit loop
                    except Exception as write_err:
                        logging.error(f"!!! FFMPEG thread: Error writing frame to ffmpeg stdin: {write_err}", exc_info=True)
                        self.last_error = f"FFMPEG stdin write error: {write_err}"
                        # Should we stop recording on write error? Maybe log and continue for a bit?
                        # For now, break the loop to stop recording on error.
                        break

                # --- Periodic Logging ---
                current_time = time.monotonic()
                if current_time - last_log_time > 10.0:
                    qsize = self.recording_frame_queue.qsize() if self.recording_frame_queue else -1
                    logging.info(f"FFMPEG Thread Stats: Written={frames_written}, QSize={qsize}")
                    last_log_time = current_time

        except Exception as proc_err:
            logging.exception(f"!!! Error starting/running ffmpeg process: {proc_err}")
            self.last_error = f"FFMPEG Popen/Run Error: {proc_err}"
            # Ensure stop event is set if process fails to start
            if not self.stop_recording_event.is_set(): self.stop_recording_event.set()
        finally:
            logging.info(f"FFMPEG recording thread loop finishing. Frames piped: {frames_written}.")
            # --- Cleanup ffmpeg process ---
            if ffmpeg_proc:
                if ffmpeg_proc.stdin:
                    logging.debug("Closing ffmpeg stdin pipe...")
                    try: ffmpeg_proc.stdin.close()
                    except Exception as e: logging.warning(f"Error closing ffmpeg stdin: {e}")
                if ffmpeg_proc.poll() is None: # Check if process is still running
                    logging.info("Waiting for ffmpeg process to terminate...")
                    try:
                        return_code = ffmpeg_proc.wait(timeout=5.0) # Wait up to 5 seconds
                        logging.info(f"ffmpeg process terminated with return code: {return_code}")
                        if return_code != 0 and not self.last_error:
                             self.last_error = f"FFMPEG exited with code {return_code}"
                    except subprocess.TimeoutExpired:
                        logging.error("!!! ffmpeg process did not terminate within timeout. Killing...")
                        try: ffmpeg_proc.kill(); ffmpeg_proc.wait(timeout=1.0)
                        except: logging.error("Failed to kill ffmpeg process.")
                        self.last_error = "FFMPEG process killed (timeout)"
                    except Exception as e:
                         logging.error(f"Error waiting for ffmpeg: {e}")
                         if not self.last_error: self.last_error = f"FFMPEG wait error: {e}"
                else:
                     logging.info(f"ffmpeg process already terminated with code: {ffmpeg_proc.returncode}")
                     if ffmpeg_proc.returncode != 0 and not self.last_error:
                          self.last_error = f"FFMPEG exited code {ffmpeg_proc.returncode}"

            logging.info(f"FFMPEG recording thread finished for: {primary_path}")


    def stop_recording(self):
        """Stops ffmpeg recording thread, waits for completion, copies file, syncs."""
        primary_path = None; target_paths = []; start_time_rec = None; thread_to_join = None

        with self.recording_lock:
            if not self.is_recording: logging.debug("stop_recording called when not recording."); return
            logging.info("Stopping ffmpeg recording thread..."); self.is_recording = False
            recording_stop_time_monotonic = time.monotonic()
            primary_path = self.primary_recording_path; target_paths = list(self.target_recording_paths); start_time_rec = self.recording_start_time_monotonic
            thread_to_join = self.recording_thread # Get thread ref before clearing
            self.primary_recording_path = None; self.target_recording_paths = []; self.recording_target_fps = None; self.recording_thread = None;

            # Signal the thread to stop
            self.stop_recording_event.set()
            # Put sentinel in queue to unblock thread if waiting
            if self.recording_frame_queue:
                try: self.recording_frame_queue.put_nowait((None, None)) # Frame=None is sentinel
                except queue.Full: logging.warning("Could not put stop sentinel in full recording queue.")
                except Exception as e: logging.error(f"Error putting sentinel in recording queue: {e}")
            # Clear queue ref after signaling
            self.recording_frame_queue = None

        # --- Wait for Recording Thread (which waits for ffmpeg) ---
        if thread_to_join and thread_to_join.is_alive():
            logging.info("Waiting for ffmpeg recording thread to finish...")
            thread_to_join.join(timeout=10.0) # Allow reasonable time for ffmpeg close/wait
            if thread_to_join.is_alive(): logging.error("!!! FFMPEG recording thread did not exit cleanly within timeout!")
            else: logging.info("FFMPEG recording thread joined successfully.")
        elif thread_to_join: logging.warning("FFMPEG thread object existed but was not alive during stop.")
        else: logging.warning("FFMPEG thread object did not exist during stop sequence.")

        recording_duration = (recording_stop_time_monotonic - start_time_rec) if start_time_rec else None
        log_duration = f" ~{recording_duration:.1f}s" if recording_duration else ""; logging.info(f"Recording duration:{log_duration}")

        # --- Duplicate File to Other USB Drives ---
        # ... (duplication logic unchanged) ...
        copied_count = 0; copy_errors = 0
        if primary_path and os.path.exists(primary_path) and len(target_paths) > 1:
             # Check file size sanity check? > 0 bytes?
             if os.path.getsize(primary_path) > 100: # Basic check for non-empty file
                logging.info(f"Duplicating recorded file from {primary_path} to other drives...")
                for target_path in target_paths:
                    if target_path == primary_path: continue
                    try: dest_dir = os.path.dirname(target_path); os.makedirs(dest_dir, exist_ok=True); shutil.copy2(primary_path, target_path); logging.info(f"Successfully copied to: {target_path}"); copied_count += 1
                    except Exception as copy_err: logging.error(f"!!! Failed copy to {target_path}: {copy_err}"); copy_errors += 1
                if copy_errors > 0: self.last_error = self.last_error or f"Rec Copy Error ({copy_errors} failed)"
             else:
                  logging.warning(f"Skipping duplication: Primary file {primary_path} is suspiciously small.")
                  self.last_error = self.last_error or "Rec Error: Primary file empty/small"
        elif len(target_paths) <= 1: logging.debug("Only one target path, skipping duplication.")
        elif not primary_path or not os.path.exists(primary_path): logging.error(f"Skipping duplication: Primary file missing! Path: {primary_path}"); self.last_error = self.last_error or "Rec Error: Primary file missing"


        # --- Filesystem Sync ---
        # ... (sync logic unchanged) ...
        final_files_exist = any(os.path.exists(p) for p in target_paths if p)
        if final_files_exist:
            logging.info("Syncing filesystem..."); 
            try: sync_start_time = time.monotonic(); subprocess.run(['sync'], check=True, timeout=15); sync_duration = time.monotonic() - sync_start_time; logging.info(f"Sync completed in {sync_duration:.2f}s.")
            except subprocess.TimeoutExpired: logging.error("!!! Filesystem sync timed out!"); self.last_error = "Sync timed out"
            except Exception as e: logging.error(f"!!! Failed execute 'sync': {e}"); self.last_error = "Sync failed"
        else: logging.warning("No final recording files found, skipping sync.")
        final_file_count = sum(1 for p in target_paths if p and os.path.exists(p)); logging.info(f"Recording stopped. Final files: {final_file_count}/{len(target_paths)}.")
        if final_file_count < len(target_paths): logging.warning(f"Failed save/copy to {len(target_paths) - final_file_count} locations.")


    def capture_and_combine_frames(self):
        """Captures frames for streaming/CV, puts raw frame in queue for ffmpeg."""
        frame0 = None; frame1 = None; output_frame_for_stream = None; frame0_timestamp = None; capture_successful_cam0 = False
        if self.is_initialized0 and self.picam0 and self.picam0.started:
            try:
                frame0 = self.picam0.capture_array("main"); frame0_timestamp = time.monotonic(); capture_successful_cam0 = True
                with self.raw_frame_lock: self.latest_raw_frame0 = frame0.copy()
                if self.last_cam0_capture_time is not None:
                    time_diff = frame0_timestamp - self.last_cam0_capture_time
                    if time_diff > 0.0001: instant_fps = 1.0 / time_diff; alpha = 0.1; self.measured_cam0_fps_avg = (alpha * instant_fps + (1 - alpha) * self.measured_cam0_fps_avg) if self.measured_cam0_fps_avg else instant_fps
                self.last_cam0_capture_time = frame0_timestamp
            except Exception as e0: logging.error(f"!!! Error Cam0 capture: {e0}"); error_msg = f"Cam0 Capture Error: {e0}"; self.last_error = error_msg if self.last_error != error_msg else self.last_error; self.last_cam0_capture_time = None; self.measured_cam0_fps_avg = None; frame0 = None; frame0_timestamp = None; 
            with self.raw_frame_lock: self.latest_raw_frame0 = None
        # ... (Cam1 capture logic unchanged) ...
        if config.ENABLE_CAM1 and self.is_initialized1 and self.picam1 and self.picam1.started:
            try: frame1 = self.picam1.capture_array("main")
            except Exception as e1: logging.error(f"!!! Error Cam1 capture: {e1}"); err_msg = f"Cam1 Capture Error: {e1}"; self.last_error = (self.last_error + " / " + err_msg) if (self.last_error and "Cam0" in self.last_error) else err_msg; frame1 = None

        # ... (Frame combination logic unchanged) ...
        if frame0 is not None and frame1 is not None and config.ENABLE_CAM1:
            try: # Combine frame0 and frame1 ...
                 h0, w0, _ = frame0.shape; h1, w1, _ = frame1.shape; target_w1, target_h1 = config.CAM1_RESOLUTION
                 # ... resize logic ...
                 if w1 != target_w1 or h1 != target_h1 or w1 > w0:
                    if w1 > w0: scale = w0 / w1; target_w1_scaled = w0; target_h1_scaled = int(h1 * scale); frame1_resized = cv2.resize(frame1, (target_w1_scaled, target_h1_scaled), interpolation=cv2.INTER_AREA)
                    else: frame1_resized = cv2.resize(frame1, (target_w1, target_h1), interpolation=cv2.INTER_AREA)
                    h1, w1, _ = frame1_resized.shape
                 else: frame1_resized = frame1
                 # ... combine logic ...
                 final_w = w0; final_h = h0 + h1 + config.STREAM_BORDER_SIZE; output_frame_for_stream = np.zeros((final_h, final_w, 3), dtype=np.uint8); output_frame_for_stream[:, :] = config.STREAM_BORDER_COLOR; output_frame_for_stream[0:h0, 0:w0] = frame0; y_start1 = h0 + config.STREAM_BORDER_SIZE; x_start1 = (final_w - w1) // 2; output_frame_for_stream[y_start1:y_start1 + h1, x_start1:x_start1 + w1] = frame1_resized
            except Exception as e_comb: logging.error(f"!!! Error combining frames: {e_comb}"); self.last_error = f"Frame Combine Error: {e_comb}"; output_frame_for_stream = frame0
        elif frame0 is not None: output_frame_for_stream = frame0
        elif frame1 is not None and config.ENABLE_CAM1:
            try: target_w1, target_h1 = config.CAM1_RESOLUTION; output_frame_for_stream = cv2.resize(frame1, (target_w1, target_h1), interpolation=cv2.INTER_AREA)
            except Exception as e_resize1: logging.error(f"Could not resize fallback frame1: {e_resize1}"); output_frame_for_stream = None
        else: output_frame_for_stream = None

        with self.frame_lock: self.output_frame = output_frame_for_stream.copy() if output_frame_for_stream is not None else None

        # --- Put Frame into Recording Queue for ffmpeg ---
        if self.is_recording and capture_successful_cam0 and frame0 is not None and frame0_timestamp is not None:
            if self.recording_frame_queue:
                try:
                    # Put raw frame and timestamp (though timestamp not used by pipe)
                    self.recording_frame_queue.put_nowait((frame0, frame0_timestamp))
                except queue.Full: logging.warning("FFMPEG recording frame queue FULL. Dropping frame.")
                except Exception as e: logging.error(f"Error putting frame into recording queue: {e}")

        return output_frame_for_stream


    def get_latest_frame(self):
        """Returns the latest COMBINED output frame for streaming (thread-safe)."""
        with self.frame_lock: return self.output_frame.copy() if self.output_frame is not None else None

    def get_latest_raw_frame0(self):
        """Returns the latest RAW frame captured from Cam0 for CV processing (thread-safe)."""
        with self.raw_frame_lock: return self.latest_raw_frame0.copy() if self.latest_raw_frame0 is not None else None

    def shutdown(self):
        """Stops ffmpeg recording thread (if active) and closes cameras cleanly."""
        logging.info("--- CameraManager Shutting Down (FFMPEG Pipe Recording) ---")
        if self.is_recording: logging.info("Shutdown: Stopping active FFMPEG recording thread..."); self.stop_recording()
        else: # Check if thread exists but is_recording is false (e.g., error state)
            with self.recording_lock: thread_ref = self.recording_thread # Get ref under lock
            if thread_ref and thread_ref.is_alive():
                logging.warning("Shutdown: FFMPEG recording thread alive but manager flag false. Attempting stop.")
                self.stop_recording_event.set()
                if self.recording_frame_queue: # Check if queue still exists
                     try: self.recording_frame_queue.put_nowait((None, None))
                     except: pass # Ignore errors putting sentinel
                thread_ref.join(timeout=5.0) # Wait briefly
                if thread_ref.is_alive(): logging.error("Orphaned FFMPEG thread did not exit on shutdown!")

        cameras_to_shutdown = [];
        if self.picam0: cameras_to_shutdown.append((self.picam0, "Cam0"))
        if self.picam1 and config.ENABLE_CAM1: cameras_to_shutdown.append((self.picam1, "Cam1"))
        for picam_instance, name in cameras_to_shutdown:
            logging.info(f"Shutdown: Stopping and closing {name}...")
            try:
                if picam_instance.started: picam_instance.stop(); logging.info(f"{name} stopped.")
                picam_instance.close(); logging.info(f"{name} closed.")
            except Exception as e: logging.error(f"Error stopping/closing {name} during shutdown: {e}")

        self.picam0 = None; self.is_initialized0 = False; self.picam1 = None; self.is_initialized1 = False;
        self.actual_cam0_fps = None; self.last_cam0_capture_time = None; self.measured_cam0_fps_avg = None; self.recording_target_fps = None
        with self.frame_lock: self.output_frame = None; 
        with self.raw_frame_lock: self.latest_raw_frame0 = None
        self.primary_recording_path = None; self.target_recording_paths = []
        self.recording_thread = None; self.recording_frame_queue = None # Ensure these are cleared
        logging.info("--- CameraManager Shutdown Complete ---")