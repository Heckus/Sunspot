# -*- coding: utf-8 -*-
import cv2
import time
import datetime
import os                     # <--- Ensure os is imported
import threading
import signal
from flask import Flask, Response, render_template_string, jsonify, redirect, url_for
from gpiozero import Button, Device
from gpiozero.pins.native import NativeFactory
import logging
from picamera2 import Picamera2 # Import picamera2
from picamera2.tuning import Tuning # <--- IMPORT Tuning
# Import controls for AWB, Denoise, etc.:
from libcamera import controls
import smbus # For battery monitor
import math # For battery monitor clamping

# --- Configuration ---
SUPPORTED_RESOLUTIONS = [
    (640, 480),      # VGA
    (1280, 720),     # 720p HD
    (1640, 1232),    # Custom Pi Cam V2 mode
    (1920, 1080),    # 1080p FHD
    (3280, 2464)     # Max Native Resolution IMX219
]
DEFAULT_RESOLUTION_INDEX = 1
FRAME_RATE = 30 # Note: Max resolution might not achieve this FPS
SWITCH_GPIO_PIN = 17
SWITCH_BOUNCE_TIME = 0.1
USB_BASE_PATH = "/media/hecke/"
RECORDING_FORMAT = "mp4v"
RECORDING_EXTENSION = ".mp4"
WEB_PORT = 8000

# --- Tuning File Configuration --- # <--- NEW SECTION
# Path to the standard NoIR tuning file for the IMX219 sensor
# Use this if your camera behaves like a NoIR (lacks IR filter)
USE_NOIR_TUNING = True # Set to False to use default tuning
NOIR_TUNING_FILE_PATH = "/usr/share/libcamera/ipa/rpi/vc4/imx219_noir.json"
# --- End Tuning File Configuration ---

# --- Battery Monitor Configuration ---
INA219_I2C_ADDRESS = 0x41 # <<< CHANGE THIS if your sensor address is different
BATTERY_READ_INTERVAL = 30.0 # Seconds between battery reads
# !!! IMPORTANT: Set these voltages accurately for your specific battery pack !!!
BATTERY_MAX_VOLTAGE = 12.6 # Voltage when fully charged (e.g., 3S LiPo)
BATTERY_MIN_VOLTAGE = 9.0  # Voltage when empty (e.g., 3S LiPo cut-off)
# --- End Battery Monitor Configuration ---

# --- Global Variables ---
app = Flask(__name__)
picam2 = None
output_frame = None
frame_lock = threading.Lock()
config_lock = threading.Lock()
capture_thread = None
flask_thread = None
shutdown_event = threading.Event()
switch = None
current_resolution_index = DEFAULT_RESOLUTION_INDEX
is_recording = False
video_writers = []
recording_paths = []
reconfigure_resolution_index = None
last_error = None
digital_recording_active = False
ina219_sensor = None
battery_percentage = None
last_battery_read_time = 0.0

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')

# ===========================================================
# === BATTERY MONITOR (INA219) CODE START ===
# ===========================================================
# Config Register (R/W)
_REG_CONFIG                      = 0x00
_REG_SHUNTVOLTAGE                = 0x01
_REG_BUSVOLTAGE                  = 0x02
_REG_POWER                       = 0x03
_REG_CURRENT                     = 0x04
_REG_CALIBRATION                 = 0x05

class BusVoltageRange:
    RANGE_16V                      = 0x00
    RANGE_32V                      = 0x01

class Gain:
    DIV_1_40MV                     = 0x00
    DIV_2_80MV                     = 0x01
    DIV_4_160MV                    = 0x02
    DIV_8_320MV                    = 0x03

class ADCResolution:
    ADCRES_9BIT_1S                 = 0x00
    ADCRES_10BIT_1S                = 0x01
    ADCRES_11BIT_1S                = 0x02
    ADCRES_12BIT_1S                = 0x03
    ADCRES_12BIT_2S                = 0x09
    ADCRES_12BIT_4S                = 0x0A
    ADCRES_12BIT_8S                = 0x0B
    ADCRES_12BIT_16S               = 0x0C
    ADCRES_12BIT_32S               = 0x0D
    ADCRES_12BIT_64S               = 0x0E
    ADCRES_12BIT_128S              = 0x0F

class Mode:
    POWERDOW                       = 0x00
    SVOLT_TRIGGERED                = 0x01
    BVOLT_TRIGGERED                = 0x02
    SANDBVOLT_TRIGGERED            = 0x03
    ADCOFF                         = 0x04
    SVOLT_CONTINUOUS               = 0x05
    BVOLT_CONTINUOUS               = 0x06
    SANDBVOLT_CONTINUOUS           = 0x07

class INA219:
    def __init__(self, i2c_bus=1, addr=0x40):
        try:
            self.bus = smbus.SMBus(i2c_bus)
            self.addr = addr
            self._cal_value = 0
            self._current_lsb = 0
            self._power_lsb = 0
            # Use 16V setting by default
            self.set_calibration_16V_5A()
            logging.info(f"INA219 sensor initialized at address 0x{addr:X} on bus {i2c_bus}")
        except FileNotFoundError:
            logging.error(f"!!! I2C bus {i2c_bus} not found. Check raspi-config.")
            raise
        except Exception as e:
            logging.error(f"!!! Failed to initialize INA219 sensor at 0x{addr:X}: {e}", exc_info=False) # Keep log less verbose on expected errors
            raise

    def read(self,address):
        data = self.bus.read_i2c_block_data(self.addr, address, 2)
        return ((data[0] * 256 ) + data[1])

    def write(self,address,data):
        temp = [0,0]
        temp[1] = data & 0xFF
        temp[0] =(data & 0xFF00) >> 8
        self.bus.write_i2c_block_data(self.addr,address,temp)

    def set_calibration_16V_5A(self):
        self._current_lsb = 0.0001524
        self._cal_value = 26868
        self._power_lsb = 0.003048
        self.write(_REG_CALIBRATION, self._cal_value)
        self.bus_voltage_range = BusVoltageRange.RANGE_16V
        self.gain = Gain.DIV_2_80MV
        self.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.mode = Mode.SANDBVOLT_CONTINUOUS
        config = self.bus_voltage_range << 13 | self.gain << 11 | self.bus_adc_resolution << 7 | self.shunt_adc_resolution << 3 | self.mode
        self.write(_REG_CONFIG, config)

    # Keep 32V method for reference if needed
    def set_calibration_32V_2A(self):
        self._current_lsb = 0.0001
        self._cal_value = 4096
        self._power_lsb = 0.002
        self.write(_REG_CALIBRATION, self._cal_value)
        self.bus_voltage_range = BusVoltageRange.RANGE_32V
        self.gain = Gain.DIV_8_320MV
        self.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.mode = Mode.SANDBVOLT_CONTINUOUS
        config = self.bus_voltage_range << 13 | self.gain << 11 | self.bus_adc_resolution << 7 | self.shunt_adc_resolution << 3 | self.mode
        self.write(_REG_CONFIG, config)

    def getShuntVoltage_mV(self):
        value = self.read(_REG_SHUNTVOLTAGE)
        if value > 32767: value -= 65536
        return value * 0.01

    def getBusVoltage_V(self):
        self.read(_REG_BUSVOLTAGE) # Dummy read?
        raw_val = self.read(_REG_BUSVOLTAGE)
        shifted_val = raw_val >> 3
        return shifted_val * 0.004

    def getCurrent_mA(self):
        value = self.read(_REG_CURRENT)
        if value > 32767: value -= 65536
        return value * self._current_lsb * 1000

    def getPower_W(self):
        value = self.read(_REG_POWER)
        if value > 32767: value -= 65536
        return value * self._power_lsb
# ===========================================================
# === BATTERY MONITOR (INA219) CODE END ===
# ===========================================================


# --- Setup GPIO ---
def setup_gpio():
    global switch, last_error
    logging.info(f"Setting up GPIO pin {SWITCH_GPIO_PIN} using gpiozero Button")
    try:
        try:
            Device.pin_factory = NativeFactory()
            logging.info("Using gpiozero NativeFactory")
        except Exception as e:
            logging.warning(f"Could not set NativeFactory for gpiozero, using default: {e}")
        switch = Button(SWITCH_GPIO_PIN, pull_up=True, bounce_time=SWITCH_BOUNCE_TIME)
        logging.info(f"gpiozero Button on pin {SWITCH_GPIO_PIN} setup complete.")
        return True
    except Exception as e:
        logging.error(f"!!! Failed to setup gpiozero Button: {e}", exc_info=True)
        last_error = f"GPIO Setup Error: {e}"
        switch = None
        return False

# --- Cleanup GPIO ---
def cleanup_gpio():
    global switch
    logging.info("Cleaning up GPIO (gpiozero).")
    try:
        if switch:
            switch.close()
            switch = None
            logging.info("gpiozero Button closed.")
    except Exception as e:
        logging.warning(f"Error during gpiozero cleanup: {e}")

# --- Setup Battery Monitor ---
def setup_battery_monitor():
    global ina219_sensor, last_error
    logging.info("Setting up Battery Monitor (INA219)...")
    try:
        ina219_sensor = INA219(addr=INA219_I2C_ADDRESS)
        voltage = ina219_sensor.getBusVoltage_V() # Initial read test
        logging.info(f"INA219 Sensor setup complete. Initial voltage reading: {voltage:.2f}V")
        read_battery_level() # Try initial percentage read
        return True
    except Exception as e:
        logging.error(f"!!! Failed to setup INA219 Battery Monitor: {e}", exc_info=False) # Keep log less verbose
        last_error = f"Battery Monitor Setup Error: {e}"
        ina219_sensor = None
        return False

# --- Read Battery Level ---
def read_battery_level():
    global battery_percentage, ina219_sensor, last_error, config_lock
    if ina219_sensor is None: return

    try:
        bus_voltage = ina219_sensor.getBusVoltage_V()
        voltage_range = BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE
        if voltage_range <= 0:
             logging.warning("Battery min/max voltages invalid. Cannot calculate percentage.")
             percent = None
        else:
            percent = ((bus_voltage - BATTERY_MIN_VOLTAGE) / voltage_range) * 100.0
            percent = max(0.0, min(100.0, percent)) # Clamp 0-100

        with config_lock:
            battery_percentage = percent

    except OSError as e: # Catch I2C communication errors
        # Log less frequently? Only if state changes?
        # current_batt_state = None
        # with config_lock: current_batt_state = battery_percentage
        # if current_batt_state is not None: # Only log error if we previously had a reading
        logging.error(f"!!! I2C Error reading INA219: {e}")
        with config_lock:
            battery_percentage = None # Set to None on error
    except Exception as e:
        logging.error(f"!!! Error reading battery level: {e}", exc_info=True)
        with config_lock:
            battery_percentage = None

# --- Get Current Resolution ---
def get_current_resolution():
    global current_resolution_index, config_lock
    with config_lock:
        if 0 <= current_resolution_index < len(SUPPORTED_RESOLUTIONS):
            return SUPPORTED_RESOLUTIONS[current_resolution_index]
        else:
            logging.warning(f"Invalid resolution index {current_resolution_index}, falling back to default.")
            safe_default_index = max(0, min(len(SUPPORTED_RESOLUTIONS) - 1, DEFAULT_RESOLUTION_INDEX))
            return SUPPORTED_RESOLUTIONS[safe_default_index]

# --- Modified initialize_camera for Picamera2 with Tuning ---
def initialize_camera(target_width, target_height):
    global picam2, last_error
    logging.info(f"Attempting to initialize camera with Picamera2 at {target_width}x{target_height}...")

    # --- Stop/Close existing instance ---
    if picam2 is not None:
        try:
            if picam2.started: picam2.stop()
            picam2.close()
            logging.info("Previous Picamera2 instance stopped and closed.")
        except Exception as e:
            logging.warning(f"Error stopping/closing previous Picamera2 instance: {e}")
        picam2 = None
        time.sleep(0.5)

    try:
        # --- Create Picamera2 instance with NoIR Tuning (if enabled) ---  # <--- MODIFIED SECTION
        tuning = None
        if USE_NOIR_TUNING:
            if os.path.exists(NOIR_TUNING_FILE_PATH):
                try:
                    tuning = Tuning.load_file(NOIR_TUNING_FILE_PATH)
                    logging.info(f"Loading NoIR tuning from: {NOIR_TUNING_FILE_PATH}")
                except Exception as e:
                    logging.error(f"!!! Failed to load tuning file '{NOIR_TUNING_FILE_PATH}': {e}. Using default tuning.", exc_info=True)
                    tuning = None # Ensure tuning is None if loading failed
            else:
                logging.warning(f"NoIR tuning file not found at {NOIR_TUNING_FILE_PATH}. Using default tuning.")
        else:
             logging.info("Using default tuning (NoIR tuning disabled).")

        # Pass the loaded tuning object (or None if failed/not found/disabled) to the constructor
        picam2 = Picamera2(tuning=tuning)
        # --- End Tuning Modification ---

        # --- Configure Camera ---
        # Set AWB back to Auto initially when using specific tuning,
        # as the tuning file heavily influences colour/monochrome output.
        config = picam2.create_video_configuration(
            main={"size": (target_width, target_height), "format": "BGR888"},
            controls={
                "FrameRate": float(FRAME_RATE),
                "AwbEnable": True,
                "AwbMode": controls.AwbModeEnum.Auto, # Let tuning file dominate first
                # Keep NoiseReductionMode commented out if it caused errors before
                # You can try uncommenting this later if needed:
                # "NoiseReductionMode": controls.NoiseReductionMode.Fast,
                "Brightness": 0.0, # Can be adjusted later if needed
                "Contrast": 1.0,   # Can be adjusted later if needed
                "Saturation": 1.0, # Usually 0 or 1 for monochrome from NoIR tuning
                }
        )
        # If using NoIR tuning, saturation might be forced low, explicitly set if you want color potential
        # if USE_NOIR_TUNING:
        #    config['controls']['Saturation'] = 0.0 # Force monochrome if desired with NoIR tune

        logging.info(f"Configuring Picamera2 with: {config}")
        picam2.configure(config)
        logging.info("Picamera2 configuration successful!")

        picam2.start()
        logging.info("Camera started")
        time.sleep(2.0) # Allow camera time to settle

        actual_config = picam2.camera_configuration()
        if not actual_config:
             raise RuntimeError("Failed to get camera configuration after start.")
        actual_format = actual_config['main']
        actual_w = actual_format['size'][0]
        actual_h = actual_format['size'][1]
        logging.info(f"Picamera2 initialized. Actual config: {actual_w}x{actual_h}, Format: {actual_format['format']}")
        last_error = None
        return True

    except Exception as e:
        logging.error(f"!!! Failed to initialize Picamera2 (possibly tuning related) at {target_width}x{target_height}: {e}", exc_info=True)
        last_error = f"Picamera2 Init Error ({target_width}x{target_height}): {e}"
        if picam2 is not None:
            try:
                if picam2.started: picam2.stop()
                picam2.close()
            except Exception as close_e:
                logging.error(f"Error closing picam2 after init failure: {close_e}")
        picam2 = None
        return False

# --- USB Mounts and Recording ---
def get_usb_mounts():
    mounts = []
    logging.debug(f"Checking for USB mounts under: {USB_BASE_PATH}")
    if not os.path.isdir(USB_BASE_PATH):
        logging.warning(f"USB base path '{USB_BASE_PATH}' does not exist or is not a directory.")
        return mounts
    try:
        for item in os.listdir(USB_BASE_PATH):
            path = os.path.join(USB_BASE_PATH, item)
            if os.path.isdir(path) and os.access(path, os.W_OK):
                test_file = os.path.join(path, f".write_test_{os.getpid()}")
                try:
                    with open(test_file, 'w') as f: f.write('test')
                    os.remove(test_file)
                    mounts.append(path)
                    logging.debug(f"Found writable mount: {path}")
                except Exception as write_err:
                     logging.warning(f"Directory {path} appears mounted but not truly writable: {write_err}")
            elif os.path.isdir(path):
                 logging.debug(f"Directory {path} found but not writable.")
        if not mounts: logging.debug("No writable USB mounts found.")
    except Exception as e:
        logging.error(f"Error finding USB mounts in {USB_BASE_PATH}: {e}")
    return mounts

def start_recording():
    global is_recording, video_writers, recording_paths, last_error, picam2
    if is_recording: return True

    logging.info("Attempting to start recording...")
    usb_drives = get_usb_mounts()
    if not usb_drives:
        logging.warning(f"Cannot start recording: No writable USB drives found in {USB_BASE_PATH}.")
        last_error = f"Cannot start recording: No writable USB drives found"
        return False

    video_writers.clear(); recording_paths.clear()
    success_count = 0; start_error = None

    if picam2 is None or not picam2.started:
        logging.error("Cannot start recording, camera is not available.")
        last_error = "Camera not available for recording."
        return False

    try:
        cam_config = picam2.camera_configuration()['main']
        width, height = cam_config['size']
        fps = float(FRAME_RATE)
        if width >= 3280 and fps > 15: # Cap high res FPS
            logging.warning(f"High resolution ({width}x{height}) detected. Capping recording FPS to 15.")
            fps = 15.0

        if width <= 0 or height <= 0: raise ValueError(f"Invalid camera dimensions {width}x{height}")

        logging.info(f"Starting recording with dimensions: {width}x{height} @ {fps:.1f}fps")
        fourcc = cv2.VideoWriter_fourcc(*RECORDING_FORMAT)
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        for drive_path in usb_drives:
            try:
                filename = f"recording_{timestamp}_{width}x{height}{RECORDING_EXTENSION}"
                full_path = os.path.join(drive_path, filename)
                writer = cv2.VideoWriter(full_path, fourcc, fps, (width, height))
                if not writer.isOpened():
                    raise IOError(f"Failed to open VideoWriter for {full_path}")
                video_writers.append(writer)
                recording_paths.append(full_path)
                logging.info(f"Successfully started recording to: {full_path}")
                success_count += 1
            except Exception as e:
                logging.error(f"!!! Failed to create VideoWriter for {drive_path}: {e}", exc_info=True)
                start_error = f"Failed writer {os.path.basename(drive_path)}: {e}" # Keep error brief

        if success_count > 0:
            is_recording = True
            logging.info(f"Recording started on {success_count} drive(s).")
            if start_error and success_count < len(usb_drives):
                last_error = f"Partial Rec Start Fail: {start_error}"
            else: last_error = None # Clear errors if fully successful or only partially failed
            return True
        else: # No writers succeeded
            is_recording = False
            logging.error("Failed to start recording on ANY USB drive.")
            last_error = f"Recording Start Failed: {start_error or 'No writers opened'}"
            video_writers.clear(); recording_paths.clear()
            return False
    except Exception as e:
        logging.error(f"!!! Critical error during recording setup: {e}", exc_info=True)
        last_error = f"Recording Setup Error: {e}"
        stop_recording()
        return False

def stop_recording():
    global is_recording, video_writers, recording_paths
    if not is_recording:
        if video_writers or recording_paths: # Clear lists if state is inconsistent
             video_writers.clear(); recording_paths.clear()
        return

    logging.info("Stopping recording...")
    is_recording = False # Set state first
    released_count = 0
    writers_to_release = list(video_writers)
    paths_recorded = list(recording_paths)
    video_writers.clear(); recording_paths.clear()

    for i, writer in enumerate(writers_to_release):
        path = paths_recorded[i] if i < len(paths_recorded) else "Unknown Path"
        try:
            writer.release()
            logging.info(f"Stopped recording and saved: {path}")
            released_count += 1
        except Exception as e:
            logging.error(f"Error releasing VideoWriter for {path}: {e}")
    logging.info(f"Recording stopped. Released {released_count} writer(s).")

# --- Capture Loop ---
def capture_and_process_loop():
    global output_frame, is_recording, last_error, picam2, switch, digital_recording_active
    global current_resolution_index, reconfigure_resolution_index, last_battery_read_time

    logging.info("Starting frame capture loop...")
    consecutive_error_count = 0
    max_consecutive_errors = 15

    # Initial camera setup happens here
    width, height = get_current_resolution()
    if not initialize_camera(width, height):
        logging.error("Initial camera setup failed. Capture thread cannot start.")
        last_error = last_error or "Initial camera setup failed"
        return # Exit thread

    while not shutdown_event.is_set():
        loop_start_time = time.monotonic()

        try:
            # --- Reconfiguration Check ---
            target_index = -1
            if reconfigure_resolution_index is not None:
                with config_lock:
                    if reconfigure_resolution_index is not None:
                        target_index = reconfigure_resolution_index
                        reconfigure_resolution_index = None # Consume request

            if target_index != -1:
                logging.info(f"--- Reconfiguring resolution to index {target_index} ---")
                physical_switch_on_before = (switch is not None and switch.is_pressed)
                digital_switch_on_before = digital_recording_active
                should_be_recording_after = physical_switch_on_before or digital_switch_on_before
                was_actually_recording = is_recording

                if was_actually_recording: stop_recording()

                new_width, new_height = SUPPORTED_RESOLUTIONS[target_index]
                if initialize_camera(new_width, new_height):
                    current_resolution_index = target_index # Update index only on success
                    logging.info(f"--- Reconfiguration successful to {new_width}x{new_height} ---")
                    if should_be_recording_after:
                        logging.info("Resuming recording after reconfiguration...")
                        time.sleep(1.0)
                        if not start_recording(): logging.error("Failed to restart recording after reconfig!")
                else: # Reconfig failed, try restoring previous
                    logging.error(f"!!! Failed reconfigure to index {target_index}. Restoring previous... !!!")
                    prev_width, prev_height = get_current_resolution() # Get the unchanged resolution
                    if not initialize_camera(prev_width, prev_height):
                        logging.critical("!!! Failed to restore previous camera resolution. Stopping. !!!")
                        last_error = "Camera failed fatally during reconfig restore."
                        shutdown_event.set(); break # Exit loop
                    else:
                        logging.info("Successfully restored previous camera resolution.")
                        if should_be_recording_after:
                            logging.info("Attempting recording restart with restored resolution...")
                            time.sleep(1.0)
                            if not start_recording(): logging.error("Failed to restart recording after failed reconfig.")
                continue # Skip rest of loop iteration during reconfig


            # --- Camera Status Check ---
            if picam2 is None or not picam2.started:
                if not last_error: last_error = "Picamera2 became unavailable."
                logging.error(f"Camera unavailable: {last_error}. Attempting reinitialization...")
                width, height = get_current_resolution()
                if initialize_camera(width, height):
                     logging.info("Camera re-initialized successfully.")
                     last_error = None; consecutive_error_count = 0
                else:
                    logging.error(f"Camera re-initialization failed: {last_error}. Stopping capture loop.")
                    shutdown_event.set(); break # Exit loop
                continue # Skip rest of loop


            # --- Read Frame ---
            frame_bgr = picam2.capture_array("main")
            if frame_bgr is None:
                logging.warning("Failed capture. Retrying...")
                consecutive_error_count += 1
                if consecutive_error_count > max_consecutive_errors:
                    last_error = f"Failed capture {max_consecutive_errors} consecutive times."
                    logging.error(last_error); shutdown_event.set(); break
                time.sleep(0.1); continue
            if consecutive_error_count > 0: logging.info(f"Recovered frame grab after {consecutive_error_count} errors.")
            consecutive_error_count = 0

            # --- Handle Recording State (Combined Switch + Digital) ---
            physical_switch_on = False
            if switch is not None:
                try: physical_switch_on = switch.is_pressed
                except Exception as e: logging.error(f"Error reading switch: {e}"); last_error = f"Switch Read Error: {e}"

            with config_lock: digital_switch_on = digital_recording_active
            should_be_recording = physical_switch_on or digital_switch_on

            if should_be_recording and not is_recording:
                log_msg = "Physical ON" if physical_switch_on else ""
                if digital_switch_on: log_msg += (" / " if log_msg else "") + "Digital ON"
                logging.info(f"Recording trigger active ({log_msg}) - starting.")
                if not start_recording() and not last_error: last_error = "Attempted recording start failed."
            elif not should_be_recording and is_recording:
                logging.info("Recording trigger(s) OFF - stopping.")
                stop_recording()

            # --- Write Frame if Recording ---
            if is_recording:
                if not video_writers:
                    logging.warning("is_recording=True, but no writers. Stopping state.")
                    is_recording = False; last_error = "Rec stopped: writers missing."
                else:
                    write_errors = 0
                    current_writers = list(video_writers)
                    current_paths = list(recording_paths)
                    for i, writer in enumerate(current_writers):
                        try: writer.write(frame_bgr)
                        except Exception as e:
                            path_str = current_paths[i] if i < len(current_paths) else f"Writer {i}"
                            logging.error(f"!!! Failed write frame to {path_str}: {e}")
                            write_errors += 1
                    if write_errors > 0 and write_errors == len(current_writers):
                        logging.error("All writers failed write. Stopping recording.")
                        last_error = "Rec stopped: All writers failed."
                        stop_recording()

            # --- Update Shared Frame for Streaming ---
            with frame_lock:
                output_frame = frame_bgr.copy()

            # --- Read Battery Level Periodically ---
            if ina219_sensor and (loop_start_time - last_battery_read_time > BATTERY_READ_INTERVAL):
                 read_battery_level()
                 last_battery_read_time = loop_start_time

        except Exception as e:
            logging.exception(f"!!! Unexpected Error in capture loop: {e}")
            last_error = f"Capture Loop Error: {e}"
            consecutive_error_count += 1
            if consecutive_error_count > max_consecutive_errors / 2:
                logging.error(f"Too many errors ({consecutive_error_count}). Signaling shutdown.")
                shutdown_event.set()
            time.sleep(1) # Pause after error

    # --- Cleanup after loop exit ---
    logging.info("Exiting frame capture thread.")
    if is_recording: stop_recording()
    if picam2:
        try:
            if picam2.started: picam2.stop()
            picam2.close()
            logging.info("Picamera2 released by capture thread.")
        except Exception as e:
            logging.error(f"Error stopping/closing Picamera2 in thread cleanup: {e}")
    picam2 = None

# ===========================================================
# === FLASK ROUTES START HERE ===
# ===========================================================
def generate_stream_frames():
    global output_frame, frame_lock, shutdown_event
    frame_counter = 0
    logging.info("MJPEG stream generator started.")
    while not shutdown_event.is_set():
        frame_to_encode = None
        with frame_lock:
            if output_frame is not None: frame_to_encode = output_frame.copy()
        if frame_to_encode is None: time.sleep(0.05); continue

        try:
            (flag, encodedImage) = cv2.imencode(".jpg", frame_to_encode, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not flag: logging.warning("Stream generator: Could not encode frame."); time.sleep(0.1); continue
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage) + b'\r\n')
            frame_counter += 1
            time.sleep(1.0 / (FRAME_RATE + 5)) # Aim slightly faster than target rate
        except GeneratorExit: logging.info(f"Streaming client disconnected after {frame_counter} frames."); break
        except Exception as e: logging.exception(f"!!! Error in streaming generator: {e}"); time.sleep(0.5)
    logging.info("Stream generator thread exiting.")

@app.route("/")
def index():
    global last_error, digital_recording_active, battery_percentage, config_lock
    current_w, current_h = get_current_resolution()
    resolution_text = f"{current_w}x{current_h}"
    err_msg = last_error if last_error else ""
    with config_lock:
        digital_rec_state_initial = digital_recording_active
        batt_perc_initial = battery_percentage
    batt_text_initial = f"{batt_perc_initial:.1f}" if batt_perc_initial is not None else "--"

    # (Keep the existing long HTML string from the previous version)
    return render_template_string("""
    <!DOCTYPE html>
    <html>
      <head>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Pi Camera Stream & Record</title>
        <style>
          body { font-family: sans-serif; line-height: 1.4; margin: 1em; background-color: #f0f0f0;}
          .container { max-width: 960px; margin: auto; background: #fff; padding: 15px; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }
          h1 { text-align: center; color: #333; }
          .status-grid { display: grid; grid-template-columns: auto 1fr; gap: 5px 10px; margin-bottom: 15px; align-items: center;}
          .status-grid span { font-weight: bold; }
          #status, #rec-status, #resolution, #battery-level { color: #0056b3; }
          .controls { text-align: center; margin-bottom: 15px; }
          .controls button { padding: 10px 15px; margin: 5px; font-size: 1em; cursor: pointer; border-radius: 5px; border: 1px solid #ccc; background-color: #e9e9e9; transition: background-color 0.2s;}
          .controls button:hover:not(:disabled) { background-color: #dcdcdc; }
          #error { color: red; margin-top: 10px; white-space: pre-wrap; font-weight: bold; min-height: 1.2em; text-align: center; background-color: #ffebeb; border: 1px solid red; padding: 5px; border-radius: 4px;}
          img#stream { display: block; margin: 15px auto; border: 1px solid black; max-width: 100%; height: auto; background-color: #eee; }
          button#btn-record.recording-active { background-color: #ff4d4d; color: white; border-color: #ff1a1a; }
          button#btn-record.recording-active:hover:not(:disabled) { background-color: #e60000; }
          button#btn-record.recording-inactive { background-color: #4CAF50; color: white; border-color: #367c39;}
          button#btn-record.recording-inactive:hover:not(:disabled) { background-color: #45a049; }
          button:disabled { background-color: #cccccc !important; cursor: not-allowed !important; border-color: #999 !important; color: #666 !important;}
        </style>
      </head>
      <body>
        <div class="container">
          <h1>Pi Camera Stream & Record</h1>
          <div class="status-grid">
            <span>Status:</span> <span id="status">Initializing...</span>
            <span>Recording:</span> <span id="rec-status">OFF</span>
            <span>Resolution:</span> <span id="resolution">{{ resolution_text }}</span>
            <span>Battery:</span> <span id="battery-level">{{ batt_text_initial }}</span>%
          </div>
          <div class="controls">
            <button onclick="changeResolution('down')" id="btn-down" title="Decrease resolution">&laquo; Lower Res</button>
            <button onclick="toggleRecording()" id="btn-record" class="recording-inactive" title="Toggle recording via web interface">Start Rec (Web)</button>
             <button onclick="changeResolution('up')" id="btn-up" title="Increase resolution">Higher Res &raquo;</button>
          </div>
          <div id="error" {% if not err_msg %}style="display: none;"{% endif %}>{{ err_msg }}</div>
          <img id="stream" src="{{ url_for('video_feed') }}" width="{{ current_w }}" height="{{ current_h }}" alt="Loading stream...">
        </div>
        <script>
          const statusElement = document.getElementById('status');
          const resolutionElement = document.getElementById('resolution');
          const errorElement = document.getElementById('error');
          const streamImage = document.getElementById('stream');
          const btnUp = document.getElementById('btn-up');
          const btnDown = document.getElementById('btn-down');
          const btnRecord = document.getElementById('btn-record');
          const recStatusElement = document.getElementById('rec-status');
          const batteryLevelElement = document.getElementById('battery-level');
          let isChangingResolution = false;
          let isTogglingRecording = false;
          let currentDigitalRecordState = {{ 'true' if digital_rec_state_initial else 'false' }};

          function updateRecordButtonState() { /* ... same as before ... */
              if (currentDigitalRecordState) {
                  btnRecord.textContent = "Stop Rec (Web)";
                  btnRecord.classList.remove('recording-inactive');
                  btnRecord.classList.add('recording-active');
              } else {
                  btnRecord.textContent = "Start Rec (Web)";
                  btnRecord.classList.add('recording-inactive');
                  btnRecord.classList.remove('recording-active');
              }
          }
          function updateStatus() { /* ... same as before, includes battery update ... */
              if (isChangingResolution || isTogglingRecording) return;
              fetch('/status')
                  .then(response => response.ok ? response.json() : Promise.reject(`HTTP error! status: ${response.status}`))
                  .then(data => {
                      statusElement.textContent = data.status_text;
                      recStatusElement.textContent = data.is_recording ? "ACTIVE" : "OFF";
                      if (data.resolution) {
                          resolutionElement.textContent = data.resolution;
                          const [w, h] = data.resolution.split('x');
                           if (streamImage.getAttribute('width') != w || streamImage.getAttribute('height') != h) {
                                console.log(`Updating stream image size to ${w}x${h}`);
                                streamImage.setAttribute('width', w);
                                streamImage.setAttribute('height', h);
                            }
                      }
                      if (data.error) {
                          errorElement.textContent = data.error;
                          errorElement.style.display = 'block';
                      } else {
                          if (errorElement.textContent !== '') errorElement.textContent = '';
                          errorElement.style.display = 'none';
                      }
                      if (typeof data.digital_recording_active === 'boolean') {
                          if (currentDigitalRecordState !== data.digital_recording_active) {
                             currentDigitalRecordState = data.digital_recording_active;
                             updateRecordButtonState();
                          }
                      }
                      if (data.battery_percent !== null && data.battery_percent !== undefined) {
                          batteryLevelElement.textContent = data.battery_percent.toFixed(1);
                      } else {
                          batteryLevelElement.textContent = "--";
                      }
                  })
                  .catch(err => { /* ... same error handling ... */
                      console.error("Error fetching status:", err);
                      statusElement.textContent = "Error fetching status";
                      errorElement.textContent = 'Error fetching status from server.';
                      errorElement.style.display = 'block';
                      recStatusElement.textContent = "Unknown";
                      batteryLevelElement.textContent = "Err";
                  });
          }
          function disableControls() { /* ... same as before ... */
             btnUp.disabled = true; btnDown.disabled = true; btnRecord.disabled = true;
          }
          function enableControls() { /* ... same as before ... */
             btnUp.disabled = false; btnDown.disabled = false; btnRecord.disabled = false;
          }
          function changeResolution(direction) { /* ... same as before ... */
              if (isChangingResolution || isTogglingRecording) return;
              isChangingResolution = true; disableControls();
              statusElement.textContent = 'Changing resolution... Please wait.';
              errorElement.textContent = ''; errorElement.style.display = 'none';
              fetch(`/set_resolution/${direction}`, { method: 'POST' })
                  .then(response => response.json().then(data => ({ status: response.status, body: data })))
                  .then(({ status, body }) => {
                      if (status === 200 && body.success) {
                          statusElement.textContent = 'Resolution change initiated.';
                          resolutionElement.textContent = body.new_resolution;
                          const [w, h] = body.new_resolution.split('x');
                          streamImage.setAttribute('width', w); streamImage.setAttribute('height', h);
                          console.log("Resolution change requested, waiting for effect...");
                      } else {
                          errorElement.textContent = `Error: ${body.message || 'Failed change resolution.'}`;
                          errorElement.style.display = 'block';
                          console.error("Resolution change failed:", body);
                          isChangingResolution = false; enableControls(); updateStatus();
                      }
                  })
                  .catch(err => { /* ... same error handling ... */
                      console.error("Error sending resolution change:", err);
                      errorElement.textContent = 'Network error changing resolution.';
                      errorElement.style.display = 'block';
                      isChangingResolution = false; enableControls(); updateStatus();
                  })
                  .finally(() => {
                      if (isChangingResolution) {
                        setTimeout(() => {
                            console.log("Re-enabling controls after resolution change delay.");
                            isChangingResolution = false; enableControls(); updateStatus();
                        }, 5000);
                      }
                  });
          }
          function toggleRecording() { /* ... same as before ... */
              if (isChangingResolution || isTogglingRecording) return;
              isTogglingRecording = true; disableControls();
              statusElement.textContent = 'Sending record command...';
              fetch('/toggle_recording', { method: 'POST' })
                  .then(response => response.ok ? response.json() : Promise.reject(`HTTP error! Status: ${response.status}`))
                  .then(data => {
                      if (data.success) {
                          currentDigitalRecordState = data.digital_recording_active;
                          updateRecordButtonState();
                          statusElement.textContent = `Digital recording ${currentDigitalRecordState ? 'enabled' : 'disabled'}. State updating...`;
                          setTimeout(updateStatus, 1500);
                      } else {
                          errorElement.textContent = `Error: ${data.message || 'Failed to toggle recording.'}`;
                          errorElement.style.display = 'block'; statusElement.textContent = 'Command failed.';
                           setTimeout(updateStatus, 1000);
                      }
                  })
                  .catch(err => { /* ... same error handling ... */
                      console.error("Error toggling recording:", err);
                      errorElement.textContent = 'Network error toggling recording.';
                      errorElement.style.display = 'block'; statusElement.textContent = 'Command failed (Network).';
                      setTimeout(updateStatus, 1000);
                  })
                  .finally(() => { isTogglingRecording = false; enableControls(); });
          }
          document.addEventListener('DOMContentLoaded', () => { /* ... same as before ... */
              updateRecordButtonState(); updateStatus();
          });
          setInterval(() => { /* ... same as before ... */
                if (!isChangingResolution && !isTogglingRecording) updateStatus();
              }, 5000);
          let errorReloadTimeout = null; /* ... same stream error handling ... */
          streamImage.onerror = function() {
              console.warn("Stream image error detected (onerror).");
              if (errorReloadTimeout) return;
              statusElement.textContent = 'Stream interrupted. Reloading...';
              errorReloadTimeout = setTimeout(() => {
                  console.log("Attempting stream reload...");
                  streamImage.src = "{{ url_for('video_feed') }}?" + new Date().getTime();
                  errorReloadTimeout = null;
                  setTimeout(updateStatus, 1000);
              }, 3000);
          };
          streamImage.onload = function() {
              if (errorReloadTimeout) {
                 console.log("Stream loaded successfully, clearing reload timeout.");
                 clearTimeout(errorReloadTimeout); errorReloadTimeout = null;
              }
          };
        </script>
      </body>
    </html>
    """, resolution_text=resolution_text, current_w=current_w, current_h=current_h,
         err_msg=err_msg, digital_rec_state_initial=digital_rec_state_initial, batt_text_initial=batt_text_initial)

@app.route("/video_feed")
def video_feed():
    logging.info("Client connected to video feed.")
    return Response(generate_stream_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/status")
def status():
    global last_error, digital_recording_active, is_recording, battery_percentage, config_lock
    status_text = "Streaming"
    rec_stat_detail = ""
    current_w, current_h = get_current_resolution()
    batt_perc = None # Default to None
    current_digital_state = False # Default

    with config_lock: # Read shared state safely
        current_digital_state = digital_recording_active
        batt_perc = battery_percentage

    if is_recording:
        rec_stat_detail = f" (Recording to {len(recording_paths)} USBs)" if video_writers else " (ERROR: Rec stopped - writers missing)"
        if not video_writers and not last_error : last_error = "Recording stopped: writers missing"
    status_text += rec_stat_detail

    err_msg = last_error if last_error else ""
    # Auto-clear some non-critical errors if things seem okay now
    if switch is not None and err_msg and "GPIO Setup Error" in err_msg: last_error = None; err_msg = ""
    if output_frame is not None and err_msg and ("Init Error" in err_msg or "unavailable" in err_msg or "capture" in err_msg): last_error = None; err_msg = ""
    if batt_perc is not None and err_msg and ("Battery Monitor" in err_msg or "INA219" in err_msg or "I2C" in err_msg): last_error = None; err_msg = ""

    return jsonify({
        'is_recording': is_recording,
        'digital_recording_active': current_digital_state,
        'resolution': f"{current_w}x{current_h}",
        'status_text': status_text,
        'error': err_msg,
        'active_recordings': recording_paths,
        'battery_percent': batt_perc # Will be null if sensor failed
    })

@app.route("/set_resolution/<direction>", methods=['POST'])
def set_resolution(direction):
    global current_resolution_index, reconfigure_resolution_index, last_error, config_lock
    with config_lock:
        if reconfigure_resolution_index is not None:
            return jsonify({'success': False, 'message': 'Reconfiguration already in progress.'}), 429
        if not (0 <= current_resolution_index < len(SUPPORTED_RESOLUTIONS)):
             logging.error(f"Internal Error: Invalid current resolution index {current_resolution_index}!")
             return jsonify({'success': False, 'message': 'Internal state error: Invalid resolution index.'}), 500

        original_index = current_resolution_index
        new_index = current_resolution_index
        if direction == 'up': new_index += 1
        elif direction == 'down': new_index -= 1
        else: return jsonify({'success': False, 'message': 'Invalid direction specified.'}), 400

        new_index = max(0, min(len(SUPPORTED_RESOLUTIONS) - 1, new_index)) # Clamp index

        if new_index == original_index:
            msg = 'Already at highest resolution.' if direction == 'up' else 'Already at lowest resolution.'
            return jsonify({'success': False, 'message': msg}), 400

        new_w, new_h = SUPPORTED_RESOLUTIONS[new_index]
        logging.info(f"Web request: change resolution index {original_index} -> {new_index} ({new_w}x{new_h})")
        reconfigure_resolution_index = new_index
        last_error = None # Clear errors on user action
        return jsonify({'success': True, 'message': 'Resolution change requested.', 'new_resolution': f"{new_w}x{new_h}"})

@app.route('/toggle_recording', methods=['POST'])
def toggle_recording():
    global digital_recording_active, last_error, config_lock
    new_state = False
    with config_lock:
        digital_recording_active = not digital_recording_active
        new_state = digital_recording_active
        logging.info(f"Digital recording toggled via web UI to: {'ON' if new_state else 'OFF'}")
        if last_error and ("Recording" in last_error or "writers" in last_error or "USB" in last_error): last_error = None
    return jsonify({'success': True, 'digital_recording_active': new_state})

# ===========================================================
# === FLASK ROUTES END HERE ===
# ===========================================================

# --- Main Execution ---
def signal_handler(sig, frame):
    global shutdown_event
    if shutdown_event.is_set(): return # Avoid multiple signals
    logging.warning(f"Received signal {sig}. Initiating graceful shutdown...")
    shutdown_event.set()

def main():
    global last_error, capture_thread, flask_thread, picam2, shutdown_event

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    logging.info(" --- Starting Camera Stream & Record Service --- ")
    logging.info(f"--- Using Picamera2 and gpiozero ---")

    while not shutdown_event.is_set():
        last_error = None; capture_thread = None; flask_thread = None
        if picam2: # Pre-loop cleanup
            try:
                if picam2.started: picam2.stop()
                picam2.close()
            except Exception: pass
            picam2 = None

        try:
            logging.info("Initializing Hardware...")
            if not setup_gpio(): logging.error(f"GPIO setup failed: {last_error}. Switch unavailable.")
            if not setup_battery_monitor(): logging.warning(f"Battery monitor setup failed: {last_error}. Battery level unavailable.")

            logging.info("Starting frame capture thread (includes camera init)...")
            capture_thread = threading.Thread(target=capture_and_process_loop, name="CaptureThread", daemon=True)
            capture_thread.start()
            time.sleep(4.0) # Wait for camera init attempt

            if not capture_thread.is_alive(): raise RuntimeError(f"Capture thread failed during startup: {last_error or 'Thread died'}")
            if last_error and ("Init Error" in last_error or "failed" in last_error): raise RuntimeError(f"Camera init failed in thread: {last_error}")
            logging.info("Capture thread running.")

            logging.info(f"Starting Flask web server on port {WEB_PORT}...")
            flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=WEB_PORT, debug=False, use_reloader=False, threaded=True), name="FlaskThread", daemon=True)
            flask_thread.start()
            time.sleep(1.5)
            if not flask_thread.is_alive(): raise RuntimeError("Flask thread failed to start.")

            logging.info("--- System Running ---")
            logging.info(f"Access web interface at: http://<YOUR_PI_IP>:{WEB_PORT}")

            while not shutdown_event.is_set():
                if not capture_thread.is_alive(): raise RuntimeError(last_error or "Capture thread terminated.")
                if not flask_thread.is_alive(): raise RuntimeError(last_error or "Flask thread terminated.")
                shutdown_event.wait(timeout=5.0)
            break # Exit outer loop if shutdown_event set cleanly

        except RuntimeError as e:
            logging.error(f"!!! Runtime Error in Main Loop: {e}")
            logging.error("Attempting restart after 10s pause...")
            shutdown_event.set() # Signal threads
            if capture_thread and capture_thread.is_alive(): capture_thread.join(timeout=3.0)
            # Flask is daemon, should exit
            shutdown_event.clear() # Reset for next loop
            time.sleep(10.0)
        except Exception as e:
            logging.exception(f"!!! Unhandled Exception in Main: {e}")
            logging.error("Attempting restart after 10s pause...")
            shutdown_event.set();
            if capture_thread and capture_thread.is_alive(): capture_thread.join(timeout=3.0)
            shutdown_event.clear()
            time.sleep(10.0)

    # --- Final Cleanup ---
    logging.info("--- Shutdown initiated ---")
    shutdown_event.set() # Ensure flag is set
    if capture_thread and capture_thread.is_alive():
        logging.info("Waiting for capture thread exit...")
        capture_thread.join(timeout=5.0)
        if capture_thread.is_alive(): logging.warning("Capture thread did not exit cleanly.")
        else: logging.info("Capture thread finished.")

    # Flask thread is daemon

    if is_recording: logging.warning("Force stopping recording during final shutdown."); stop_recording()
    if picam2:
        try:
            logging.info("Ensuring Picamera2 is closed...");
            if picam2.started: picam2.stop()
            picam2.close()
            logging.info("Picamera2 closed final.")
        except Exception as e: logging.warning(f"Error during final Picamera2 close: {e}")
    cleanup_gpio()
    logging.info("--- Program Exit ---")

if __name__ == '__main__':
    main()