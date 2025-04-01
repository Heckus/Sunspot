# -*- coding: utf-8 -*-
import cv2
import time
import datetime
import os
import threading
import signal
from flask import Flask, Response, render_template_string, jsonify, redirect, url_for
from gpiozero import Button, Device
from gpiozero.pins.native import NativeFactory
import logging
from picamera2 import Picamera2 # Import picamera2
# Import controls for AWB setting:
from libcamera import controls
import numpy as np # Needed for camera calibration arrays

# --- Battery Monitoring Imports ---
try:
    import smbus
    SMBUS_AVAILABLE = True
except ImportError:
    smbus = None # Define smbus as None if it's not available
    SMBUS_AVAILABLE = False
    logging.warning("!!! smbus library not found. Battery monitoring will be disabled. Install with 'sudo apt-get install python3-smbus' !!!")
# --- End Battery Monitoring Imports ---

# --- Configuration ---
SUPPORTED_RESOLUTIONS = [
    (640, 480),     # VGA
    (1280, 720),    # 720p HD
    (1640, 1232),   # Custom Pi Cam V2 mode
    (1920, 1080),   # 1080p FHD
    (3280, 2464)    # Max Native Resolution IMX219 (May struggle with FPS/Undistort)
]
DEFAULT_RESOLUTION_INDEX = 1 # Default to 720p

FRAME_RATE = 30 # Note: Max resolution or undistortion might not achieve this FPS
SWITCH_GPIO_PIN = 17
SWITCH_BOUNCE_TIME = 0.1
USB_BASE_PATH = "/media/hecke/"
RECORDING_FORMAT = "mp4v" # Use 'mp4v' for .mp4, check codec availability
RECORDING_EXTENSION = ".mp4"
WEB_PORT = 8000

# --- Image Quality Configuration ---
# Set to True to attempt lens undistortion. Requires K and D matrices below.
# WARNING: This adds significant CPU load and latency.
ENABLE_UNDISTORTION = False # <<< SET TO True IF YOU HAVE CALIBRATION DATA AND WANT TO USE IT
# --- IMPORTANT: CAMERA CALIBRATION ---
# You MUST calibrate your specific camera+lens combo to get these values.
# Use OpenCV calibration tutorials (e.g., with a chessboard).
# Replace None with your actual NumPy arrays if ENABLE_UNDISTORTION is True.
# Example format: camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
# Example format: dist_coeffs = np.array([k1, k2, p1, p2, k3])
CAMERA_MATRIX = None # e.g., np.array([[1000.,    0.,  960.], [   0., 1000.,  540.], [   0.,    0.,    1.]])
DIST_COEFFS = None   # e.g., np.array([-0.2, 0.05, 0., 0., 0.])

# --- Battery Monitor Configuration ---
# Requires INA219 sensor connected via I2C and smbus library installed
ENABLE_BATTERY_MONITOR = SMBUS_AVAILABLE # Set to False to disable even if smbus is installed
INA219_I2C_BUS = 1   # Default I2C bus on Raspberry Pi
INA219_I2C_ADDR = 0x41 # <<< VERIFY Your INA219 I2C Address (often 0x40 or 0x41)
BATTERY_MIN_VOLTAGE = 9.0  # <<< ADJUST: Voltage corresponding to 0%
BATTERY_MAX_VOLTAGE = 12.6 # <<< ADJUST: Voltage corresponding to 100%
BATTERY_READ_INTERVAL = 10 # Seconds between battery reads

# --- Global Variables ---
app = Flask(__name__)
picam2 = None
output_frame = None
frame_lock = threading.Lock()
config_lock = threading.Lock() # Lock for resolution index, digital record state
battery_lock = threading.Lock() # Lock for battery percentage access
capture_thread = None
flask_thread = None
battery_thread = None # Thread for battery monitoring
shutdown_event = threading.Event()
switch = None
current_resolution_index = DEFAULT_RESOLUTION_INDEX
is_recording = False # Actual recording state
video_writers = []
recording_paths = []
reconfigure_resolution_index = None
last_error = None
digital_recording_active = False
battery_percentage = -1 # Initial value (-1 indicates not read yet or error)
ina219_sensor = None # Instance of INA219 class

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')

# --- Camera Calibration Check ---
if ENABLE_UNDISTORTION and (CAMERA_MATRIX is None or DIST_COEFFS is None):
    logging.warning("!!! ENABLE_UNDISTORTION is True, but CAMERA_MATRIX or DIST_COEFFS are not set. Undistortion will be skipped. Perform camera calibration! !!!")
    ENABLE_UNDISTORTION = False # Disable if data is missing

# ===========================================================
# === INA219 Battery Monitor Code START ===
# (Copied from your example)
# ===========================================================
if SMBUS_AVAILABLE: # Only define if smbus is available
    # Config Register (R/W)
    _REG_CONFIG                 = 0x00
    # SHUNT VOLTAGE REGISTER (R)
    _REG_SHUNTVOLTAGE           = 0x01
    # BUS VOLTAGE REGISTER (R)
    _REG_BUSVOLTAGE             = 0x02
    # POWER REGISTER (R)
    _REG_POWER                  = 0x03
    # CURRENT REGISTER (R)
    _REG_CURRENT                = 0x04
    # CALIBRATION REGISTER (R/W)
    _REG_CALIBRATION            = 0x05

    class BusVoltageRange:
        RANGE_16V               = 0x00 # set bus voltage range to 16V
        RANGE_32V               = 0x01 # set bus voltage range to 32V (default)

    class Gain:
        DIV_1_40MV              = 0x00 # shunt prog. gain set to  1, 40 mV range
        DIV_2_80MV              = 0x01 # shunt prog. gain set to /2, 80 mV range
        DIV_4_160MV             = 0x02 # shunt prog. gain set to /4, 160 mV range
        DIV_8_320MV             = 0x03 # shunt prog. gain set to /8, 320 mV range

    class ADCResolution:
        ADCRES_9BIT_1S          = 0x00 #  9bit,   1 sample,     84us
        ADCRES_10BIT_1S         = 0x01 # 10bit,   1 sample,    148us
        ADCRES_11BIT_1S         = 0x02 # 11 bit,  1 sample,    276us
        ADCRES_12BIT_1S         = 0x03 # 12 bit,  1 sample,    532us
        ADCRES_12BIT_2S         = 0x09 # 12 bit,  2 samples,  1.06ms
        ADCRES_12BIT_4S         = 0x0A # 12 bit,  4 samples,  2.13ms
        ADCRES_12BIT_8S         = 0x0B # 12bit,   8 samples,  4.26ms
        ADCRES_12BIT_16S        = 0x0C # 12bit,  16 samples,  8.51ms
        ADCRES_12BIT_32S        = 0x0D # 12bit,  32 samples, 17.02ms
        ADCRES_12BIT_64S        = 0x0E # 12bit,  64 samples, 34.05ms
        ADCRES_12BIT_128S       = 0x0F # 12bit, 128 samples, 68.10ms

    class Mode:
        POWERDOW                = 0x00 # power down
        SVOLT_TRIGGERED         = 0x01 # shunt voltage triggered
        BVOLT_TRIGGERED         = 0x02 # bus voltage triggered
        SANDBVOLT_TRIGGERED     = 0x03 # shunt and bus voltage triggered
        ADCOFF                  = 0x04 # ADC off
        SVOLT_CONTINUOUS        = 0x05 # shunt voltage continuous
        BVOLT_CONTINUOUS        = 0x06 # bus voltage continuous
        SANDBVOLT_CONTINUOUS    = 0x07 # shunt and bus voltage continuous


    class INA219:
        def __init__(self, i2c_bus=1, addr=0x40):
            self.bus = smbus.SMBus(i2c_bus)
            self.addr = addr
            # Set chip to known config values to start
            self._cal_value = 0
            self._current_lsb = 0
            self._power_lsb = 0
            # *** Choose the calibration that matches your SHUNT resistor and expected MAX Voltage/Current ***
            # If your battery setup is closer to 16V/5A and uses a 0.01 Ohm shunt:
            self.set_calibration_16V_5A()
            # If your battery setup is closer to 32V/2A and uses a 0.1 Ohm shunt:
            # self.set_calibration_32V_2A()
            logging.info(f"INA219 Initialized at I2C address 0x{addr:x} on bus {i2c_bus}")

        def read(self,address):
            # Read 2 bytes from the specified register
            data = self.bus.read_i2c_block_data(self.addr, address, 2)
            return ((data[0] * 256 ) + data[1])

        def write(self,address,data):
            # Write 2 bytes to the specified register
            temp = [0,0]
            temp[1] = data & 0xFF
            temp[0] =(data & 0xFF00) >> 8
            self.bus.write_i2c_block_data(self.addr,address,temp)

        # --- Calibration Settings (Choose one that fits your hardware) ---
        def set_calibration_16V_5A(self):
             # VBUS_MAX = 16V             (Assumes 16V, can also be set to 32V)
             # VSHUNT_MAX = 0.08         (Assumes Gain 2, 80mV, can also be 0.32, 0.16, 0.04)
             # RSHUNT = 0.01              (Resistor value in ohms) - CHECK YOUR BOARD
             # Expected MAX current = 5.0A
             # Current LSB = 0.1524 mA per bit -> roughly max ~5A (32767 * 0.1524mA)
             self._current_lsb = 0.1524
             # Calibration = 26868 (0x68ec) -> from 0.04096 / (0.0001524 * 0.01)
             self._cal_value = 26868
             # Power LSB = 20 * Current LSB = 3.048 mW per bit
             self._power_lsb = 3.048 # Used 3.048mW instead of 0.003048W for clarity with current LSB units

             self.write(_REG_CALIBRATION,self._cal_value)
             config = (BusVoltageRange.RANGE_16V << 13 |
                       Gain.DIV_2_80MV << 11 |
                       ADCResolution.ADCRES_12BIT_32S << 7 |
                       ADCResolution.ADCRES_12BIT_32S << 3 |
                       Mode.SANDBVOLT_CONTINUOUS)
             self.write(_REG_CONFIG, config)
             logging.info("INA219 configured for 16V, 5A (Gain=2/80mV, Shunt=0.01 Ohm assumed)")


        def set_calibration_32V_2A(self):
             # VBUS_MAX = 32V             (Assumes 32V, can also be set to 16V)
             # VSHUNT_MAX = 0.32         (Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
             # RSHUNT = 0.1               (Resistor value in ohms) - CHECK YOUR BOARD
             # Expected MAX current = 2.0A
             # Current LSB = 0.1 mA per bit -> roughly max ~3.2A (32767 * 0.1mA)
             self._current_lsb = 0.1
             # Calibration = 4096 (0x1000) -> from 0.04096 / (0.0001 * 0.1)
             self._cal_value = 4096
             # Power LSB = 20 * Current LSB = 2 mW per bit
             self._power_lsb = 2 # Used 2mW instead of 0.002W for clarity with current LSB units

             self.write(_REG_CALIBRATION, self._cal_value)
             config = (BusVoltageRange.RANGE_32V << 13 |
                       Gain.DIV_8_320MV << 11 |
                       ADCResolution.ADCRES_12BIT_32S << 7 |
                       ADCResolution.ADCRES_12BIT_32S << 3 |
                       Mode.SANDBVOLT_CONTINUOUS)
             self.write(_REG_CONFIG, config)
             logging.info("INA219 configured for 32V, 2A (Gain=8/320mV, Shunt=0.1 Ohm assumed)")

        # --- Reading Functions ---
        def getShuntVoltage_mV(self):
            # Sometimes a write happens? Ensure calibration is set. Might not be needed often.
            # self.write(_REG_CALIBRATION,self._cal_value)
            value = self.read(_REG_SHUNTVOLTAGE)
            # Check for negative value (2's complement)
            if value > 32767:
                value -= 65536
            # LSB is 10uV = 0.01mV
            return value * 0.01

        def getBusVoltage_V(self):
            # self.write(_REG_CALIBRATION,self._cal_value) # Maybe not needed?
            self.read(_REG_BUSVOLTAGE) # First read might be old data
            raw_val = self.read(_REG_BUSVOLTAGE)
            # Shift to remove flags (lower 3 bits), LSB is 4mV
            return (raw_val >> 3) * 0.004

        def getCurrent_mA(self):
            # Ensure calibration is correct for current LSB calculation
            # self.write(_REG_CALIBRATION,self._cal_value) # Probably not needed here if set initially
            value = self.read(_REG_CURRENT)
            if value > 32767:
                value -= 65536
            return value * self._current_lsb # self._current_lsb is already in mA/bit

        def getPower_mW(self): # Changed to mW to match Power LSB calculation
            # Ensure calibration is correct for power LSB calculation
            # self.write(_REG_CALIBRATION,self._cal_value) # Probably not needed here if set initially
            value = self.read(_REG_POWER)
            if value > 32767:
                value -= 65536
            return value * self._power_lsb # self._power_lsb is already in mW/bit

# ===========================================================
# === INA219 Battery Monitor Code END ===
# ===========================================================

# --- GPIO Setup ---
def setup_gpio():
    """Sets up the GPIO pin using gpiozero."""
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

def cleanup_gpio():
    """Cleans up GPIO resources using gpiozero."""
    global switch
    logging.info("Cleaning up GPIO (gpiozero).")
    try:
        if switch:
            switch.close()
            switch = None
            logging.info("gpiozero Button closed.")
    except Exception as e:
        logging.warning(f"Error during gpiozero cleanup: {e}")

# --- Battery Sensor Setup ---
def setup_ina219():
    """Initializes the INA219 sensor."""
    global ina219_sensor, last_error
    if not ENABLE_BATTERY_MONITOR:
        logging.info("Battery monitor is disabled in configuration.")
        return False
    if not SMBUS_AVAILABLE:
        logging.error("Cannot initialize INA219: smbus library not available.")
        last_error = "Battery Monitor Error: smbus library missing"
        return False

    logging.info(f"Setting up INA219 sensor at I2C Addr: 0x{INA219_I2C_ADDR:x} on Bus: {INA219_I2C_BUS}")
    try:
        ina219_sensor = INA219(addr=INA219_I2C_ADDR, i2c_bus=INA219_I2C_BUS)
        # Perform a quick read test
        voltage = ina219_sensor.getBusVoltage_V()
        logging.info(f"INA219 sensor initialized successfully. Initial voltage reading: {voltage:.2f}V")
        return True
    except FileNotFoundError:
        logging.error(f"!!! Failed to setup INA219: I2C bus {INA219_I2C_BUS} not found. Check bus number and enable I2C in raspi-config. !!!")
        last_error = f"INA219 Setup Error: I2C bus {INA219_I2C_BUS} not found"
        ina219_sensor = None
        return False
    except Exception as e:
        logging.error(f"!!! Failed to setup INA219 sensor: {e}", exc_info=True)
        last_error = f"INA219 Setup Error: {e}"
        ina219_sensor = None
        return False

# --- Battery Monitoring Thread ---
def battery_monitor_loop():
    """Periodically reads battery voltage and updates the global percentage."""
    global battery_percentage, ina219_sensor, last_error
    logging.info("Starting battery monitoring thread.")

    while not shutdown_event.is_set():
        if ina219_sensor:
            try:
                bus_voltage = ina219_sensor.getBusVoltage_V()
                # Calculate percentage using the configured min/max voltage
                # Clamp the percentage between 0 and 100
                percent = ((bus_voltage - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100
                percent = max(0, min(100, percent)) # Clamp between 0 and 100

                with battery_lock:
                    battery_percentage = round(percent) # Store as integer percentage

                logging.debug(f"Battery Read: {bus_voltage:.2f}V -> {battery_percentage}%")
                # Clear previous battery errors if successful
                if last_error and "INA219 Read Error" in last_error:
                    last_error = None

            except Exception as e:
                logging.warning(f"Could not read from INA219 sensor: {e}")
                # Set percentage to error value (-1) on failure
                with battery_lock:
                    battery_percentage = -1
                if not last_error: # Don't overwrite other errors
                    last_error = f"INA219 Read Error: {e}"
                # Optionally attempt re-init? For now, just log and continue trying.
                time.sleep(2) # Shorter sleep on error?

            # Wait for the configured interval or until shutdown
            shutdown_event.wait(BATTERY_READ_INTERVAL)
        else:
            # Sensor not initialized, wait longer before checking again
            logging.debug("Battery monitor loop: Sensor not available.")
            shutdown_event.wait(BATTERY_READ_INTERVAL * 2) # Check less frequently

    logging.info("Exiting battery monitoring thread.")


# --- Resolution and Camera ---
def get_current_resolution():
    """Gets the resolution tuple based on the current index."""
    with config_lock: # Protect access to current_resolution_index
        if 0 <= current_resolution_index < len(SUPPORTED_RESOLUTIONS):
            return SUPPORTED_RESOLUTIONS[current_resolution_index]
        else:
            logging.warning(f"Invalid resolution index {current_resolution_index}, falling back to default.")
            safe_default_index = max(0, min(len(SUPPORTED_RESOLUTIONS) - 1, DEFAULT_RESOLUTION_INDEX))
            return SUPPORTED_RESOLUTIONS[safe_default_index]

def initialize_camera(target_width, target_height):
    """Initializes or re-initializes the camera capture object with specific resolution."""
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
        picam2 = Picamera2()
        config = picam2.create_video_configuration(
            main={"size": (target_width, target_height), "format": "RGB888"},
            # Try limiting buffer count if memory issues arise, default is often 4
            # buffer_count=3,
            controls={
                "FrameRate": float(FRAME_RATE),
                # --- Auto White Balance ---
                # If 'Auto' gives a color cast, try other modes like:
                # controls.AwbModeEnum.Daylight, controls.AwbModeEnum.Cloudy,
                # controls.AwbModeEnum.Tungsten, controls.AwbModeEnum.Fluorescent
                "AwbEnable": True,
                "AwbMode": controls.AwbModeEnum.Auto
            }
        )
        logging.info(f"Configuring Picamera2 with: {config}")
        picam2.configure(config)
        logging.info("Configuration successful!")

        # Optional: Apply other controls after configure if needed
        # Example: Adjust brightness (0-1) or contrast (0-2)
        # picam2.set_controls({"Brightness": 0.55, "Contrast": 1.1})

        picam2.start()
        logging.info("Camera started")
        time.sleep(1.5) # Allow more time for AWB and exposure to settle

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
        logging.error(f"!!! Failed to initialize Picamera2 at {target_width}x{target_height}: {e}", exc_info=True)
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
    """Finds mounted, writable directories under the USB base path."""
    mounts = []
    logging.debug(f"Checking for USB mounts under: {USB_BASE_PATH}")
    if not os.path.isdir(USB_BASE_PATH):
        logging.warning(f"USB base path '{USB_BASE_PATH}' does not exist or is not a directory.")
        return mounts
    try:
        for item in os.listdir(USB_BASE_PATH):
            path = os.path.join(USB_BASE_PATH, item)
            if os.path.isdir(path) and os.access(path, os.W_OK):
                mounts.append(path)
        if not mounts:
            logging.debug("No writable USB mounts found.")
        else:
            logging.debug(f"Found writable USB mounts: {mounts}")
    except Exception as e:
        logging.error(f"Error finding USB mounts in {USB_BASE_PATH}: {e}")
    return mounts


def start_recording():
    """Starts recording to all detected writable USB drives."""
    global is_recording, video_writers, recording_paths, last_error, picam2
    if is_recording:
        logging.warning("Request to start recording, but already recording.")
        return True

    logging.info("Attempting to start recording...")
    usb_drives = get_usb_mounts()
    if not usb_drives:
        logging.warning(f"Cannot start recording: No writable USB drives found in {USB_BASE_PATH}.")
        last_error = f"Cannot start recording: No writable USB drives found in {USB_BASE_PATH}"
        return False

    video_writers.clear()
    recording_paths.clear()
    success_count = 0
    start_error = None

    if picam2 is None or not picam2.started:
        logging.error("Cannot start recording, camera (Picamera2) is not available.")
        last_error = "Camera (Picamera2) not available for recording."
        return False

    try:
        cam_config = picam2.camera_configuration()['main']
        width = cam_config['size'][0]
        height = cam_config['size'][1]
        fps = float(FRAME_RATE)
        # Cap FPS for recording if it exceeds a threshold? Undistort might impact this.
        # max_rec_fps = 20 if ENABLE_UNDISTORTION and width > 1920 else FRAME_RATE
        # fps = min(float(max_rec_fps), float(FRAME_RATE))

        if width <= 0 or height <= 0:
            logging.error(f"Picamera2 reported invalid dimensions ({width}x{height}). Cannot start recording.")
            last_error = "Invalid camera dimensions reported by Picamera2."
            return False

        logging.info(f"Starting recording with dimensions: {width}x{height} @ {fps}fps") # Log resolution used

        fourcc = cv2.VideoWriter_fourcc(*RECORDING_FORMAT)
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        for drive_path in usb_drives:
            try:
                filename = f"recording_{timestamp}_{width}x{height}{RECORDING_EXTENSION}"
                full_path = os.path.join(drive_path, filename)
                writer = cv2.VideoWriter(full_path, fourcc, fps, (width, height)) # Use determined fps

                if not writer.isOpened():
                    logging.error(f"!!! Failed to open VideoWriter for: {full_path}. Check codec ('{RECORDING_FORMAT}'), permissions, disk space.")
                    start_error = f"Failed writer: {full_path} (Codec: {RECORDING_FORMAT})"
                    continue

                video_writers.append(writer)
                recording_paths.append(full_path)
                logging.info(f"Successfully started recording to: {full_path}")
                success_count += 1

            except Exception as e:
                logging.error(f"!!! Failed to create VideoWriter for {drive_path}: {e}", exc_info=True)
                start_error = f"Error writer: {drive_path}: {e}"

        if success_count > 0:
            is_recording = True
            logging.info(f"Recording started on {success_count} drive(s).")
            if start_error:
                logging.warning(f"Note: Issues starting recording on all drives: {start_error}")
            last_error = None
            return True
        else:
            is_recording = False
            logging.error("Failed to start recording on any USB drive.")
            if start_error: last_error = f"Recording Start Failed: {start_error}"
            else: last_error = "Recording Start Failed: No writers could be opened."
            video_writers.clear()
            recording_paths.clear()
            return False

    except Exception as e:
        logging.error(f"!!! Critical error during recording setup: {e}", exc_info=True)
        last_error = f"Recording Setup Error: {e}"
        # Ensure we attempt to stop/clear if setup fails badly
        video_writers.clear()
        recording_paths.clear()
        is_recording = False
        return False


def stop_recording():
    """Stops recording and releases video writer objects."""
    global is_recording, video_writers, recording_paths
    if not is_recording:
        if video_writers or recording_paths:
             logging.warning("stop_recording called while not 'is_recording', but writers/paths exist. Clearing.")
             video_writers.clear()
             recording_paths.clear()
        return

    logging.info("Stopping recording...")
    is_recording = False # Set state immediately
    released_count = 0
    # Make copies to prevent modification issues if another thread accesses them
    writers_to_release = list(video_writers)
    paths_recorded = list(recording_paths)
    video_writers.clear()
    recording_paths.clear()

    for i, writer in enumerate(writers_to_release):
        try:
            writer.release()
            logging.info(f"Stopped recording and saved: {paths_recorded[i]}")
            released_count += 1
        except Exception as e:
            logging.error(f"Error releasing VideoWriter for {paths_recorded[i]}: {e}")
    logging.info(f"Recording stopped. Released {released_count} writer(s).")


# --- Capture Loop ---
def capture_and_process_loop():
    """Main loop executed in a background thread for capturing frames."""
    global output_frame, is_recording, last_error, picam2, switch, digital_recording_active
    global current_resolution_index, reconfigure_resolution_index, battery_percentage

    logging.info("Starting frame capture loop (using Picamera2)...")
    consecutive_error_count = 0
    max_consecutive_errors = 15

    # --- Undistortion setup (if enabled) ---
    undistort_map1, undistort_map2 = None, None
    current_undistort_res = None

    width, height = get_current_resolution()
    if not initialize_camera(width, height):
        logging.error("Initial camera setup failed (Picamera2). Capture thread cannot start.")
        last_error = last_error or "Initial camera setup failed"
        return # Exit thread if camera fails initially

    while not shutdown_event.is_set():
        try:
            # --- Check for Reconfiguration Request ---
            target_index = -1 # Local variable for reconfiguration check
            with config_lock:
                if reconfigure_resolution_index is not None:
                    target_index = reconfigure_resolution_index
                    reconfigure_resolution_index = None # Consume the request

            if target_index != -1:
                logging.info(f"--- Reconfiguring resolution to index {target_index} ---")
                was_recording = is_recording # Actual recording state
                with config_lock: was_digital_active = digital_recording_active # Digital toggle state
                should_be_recording_after = (switch is not None and switch.is_pressed) or was_digital_active

                if was_recording:
                    logging.info("Pausing recording for reconfiguration...")
                    stop_recording()

                new_width, new_height = SUPPORTED_RESOLUTIONS[target_index]
                if initialize_camera(new_width, new_height):
                    current_resolution_index = target_index
                    # Reset undistortion map as resolution changed
                    undistort_map1, undistort_map2 = None, None
                    current_undistort_res = None
                    logging.info(f"--- Reconfiguration successful to {new_width}x{new_height} ---")

                    if should_be_recording_after:
                        logging.info("Resuming recording after reconfiguration...")
                        time.sleep(1.0) # Give camera time to settle again
                        if not start_recording():
                            logging.error("Failed to restart recording after reconfiguration!")
                            if not last_error: last_error = "Failed auto-restart recording"
                else:
                    logging.error(f"!!! Failed reconfigure to index {target_index}. Restoring previous... !!!")
                    prev_width, prev_height = SUPPORTED_RESOLUTIONS[current_resolution_index] # Use current_resolution_index as it wasn't updated
                    if not initialize_camera(prev_width, prev_height):
                        logging.critical("!!! Failed to restore previous camera resolution. Stopping. !!!")
                        last_error = "Camera failed fatally during reconfiguration."
                        shutdown_event.set()
                        break # Fatal error, exit loop
                    else:
                         logging.info("Successfully restored previous camera resolution.")
                         # Reset undistortion map as resolution changed (back)
                         undistort_map1, undistort_map2 = None, None
                         current_undistort_res = None
                         if should_be_recording_after:
                            logging.info("Attempting recording restart with restored resolution...")
                            time.sleep(1.0)
                            if not start_recording():
                                logging.error("Failed to restart recording after failed reconfig.")
                                if not last_error: last_error = "Failed auto-restart recording (restore)"

            # --- Check Camera Status ---
            if picam2 is None or not picam2.started:
                if not last_error: last_error = "Picamera2 became unavailable unexpectedly."
                logging.error(f"Camera unavailable: {last_error}. Stopping capture loop.")
                if is_recording: stop_recording() # Stop if it died while recording
                shutdown_event.set()
                break

            # --- Read Frame using Picamera2 ---
            frame_array = picam2.capture_array("main") # Returns RGB
            frame_bgr = cv2.cvtColor(frame_array, cv2.COLOR_RGB2BGR)

            if frame_bgr is None:
                logging.warning("Failed capture/convert. Retrying...")
                consecutive_error_count += 1
                if consecutive_error_count > max_consecutive_errors:
                   last_error = f"Failed capture/convert {max_consecutive_errors} consecutive times."
                   logging.error(last_error)
                   shutdown_event.set()
                   break
                time.sleep(0.1)
                continue # Try again
            if consecutive_error_count > 0:
                logging.info(f"Recovered frame grab after {consecutive_error_count} errors.")
            consecutive_error_count = 0

            # --- Apply Undistortion (if enabled and configured) ---
            if ENABLE_UNDISTORTION and CAMERA_MATRIX is not None and DIST_COEFFS is not None:
                h, w = frame_bgr.shape[:2]
                if current_undistort_res != (w, h): # Check if map needs recalculation
                    logging.info(f"Calculating undistortion map for {w}x{h}...")
                    # Calculate new optimal camera matrix to crop black borders
                    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(CAMERA_MATRIX, DIST_COEFFS, (w, h), 1, (w, h))
                    # Calculate the mapping (this is the slow part, but only done once per resolution)
                    undistort_map1, undistort_map2 = cv2.initUndistortRectifyMap(CAMERA_MATRIX, DIST_COEFFS, None, new_camera_matrix, (w, h), 5)
                    current_undistort_res = (w, h)
                    logging.info("Undistortion map calculated.")

                if undistort_map1 is not None:
                    # Apply the mapping (faster than cv2.undistort)
                    frame_bgr = cv2.remap(frame_bgr, undistort_map1, undistort_map2, cv2.INTER_LINEAR)
                    # Optional: Crop the image based on ROI from getOptimalNewCameraMatrix
                    # x, y, w_roi, h_roi = roi
                    # if w_roi > 0 and h_roi > 0: # Check if ROI is valid
                    #    frame_bgr = frame_bgr[y:y+h_roi, x:x+w_roi]
                    # Note: Cropping changes the output dimensions, which might affect recording/streaming if not handled.
                    # Sticking with full frame remap is simpler.

            # --- Add Battery Percentage Overlay ---
            if ENABLE_BATTERY_MONITOR:
                with battery_lock:
                    current_batt_perc = battery_percentage # Read protected value
                if current_batt_perc != -1: # Only draw if valid reading exists
                    batt_text = f"BAT: {current_batt_perc}%"
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 0.7
                    thickness = 2
                    text_size, _ = cv2.getTextSize(batt_text, font, font_scale, thickness)
                    # Position: Top right corner with padding
                    text_x = frame_bgr.shape[1] - text_size[0] - 10
                    text_y = text_size[1] + 10
                    # Add black outline for better visibility
                    cv2.putText(frame_bgr, batt_text, (text_x, text_y), font, font_scale, (0, 0, 0), thickness + 2)
                    # Add white text
                    cv2.putText(frame_bgr, batt_text, (text_x, text_y), font, font_scale, (255, 255, 255), thickness)
                # else: Draw "BAT: N/A" ? Optional.

            # --- Handle Recording State (Combined Switch + Digital) ---
            should_be_recording = False # Default to false
            physical_switch_on = False
            if switch is not None: # Check physical switch if available
                try:
                    physical_switch_on = switch.is_pressed
                except Exception as e:
                    logging.error(f"Error reading gpiozero switch state: {e}")
                    if not last_error: last_error = f"Switch Read Error: {e}"
                    physical_switch_on = False # Assume off on error

            # Check digital state (use config_lock)
            with config_lock:
                digital_switch_on = digital_recording_active

            # OR logic: if either physical OR digital switch is on, we should record
            should_be_recording = physical_switch_on or digital_switch_on

            if should_be_recording:
                if not is_recording: # If we should be recording, but aren't...
                    log_msg = "Physical switch ON" if physical_switch_on else ""
                    if digital_switch_on:
                        if log_msg: log_msg += " / " # Add separator if both are triggers
                        log_msg += "Digital toggle ON"
                    logging.info(f"Recording trigger active ({log_msg}) - attempting start.")
                    if not start_recording() and not last_error:
                        last_error = "Attempted recording start (trigger ON) failed."
            else: # Should NOT be recording
                if is_recording: # If we are recording, but shouldn't be...
                    logging.info("Recording trigger(s) OFF - stopping recording.")
                    stop_recording()

            # --- Write Frame if Recording ---
            if is_recording:
                if not video_writers:
                    logging.warning("is_recording=True, but no video writers. Stopping recording state.")
                    is_recording = False # Correct the state
                    if not last_error: last_error = "Recording stopped: writers missing unexpectedly."
                else:
                    write_errors = 0
                    frame_to_write = frame_bgr # Use the (potentially modified) frame
                    # Make copies in case list is modified elsewhere (unlikely but safer)
                    current_writers = list(video_writers)
                    current_paths = list(recording_paths)
                    for i, writer in enumerate(current_writers):
                        try:
                            writer.write(frame_to_write)
                        except Exception as e:
                            logging.error(f"!!! Failed write frame to {current_paths[i]}: {e}")
                            write_errors += 1
                            # Consider removing failed writer? For now, just log.

                    if write_errors > 0 and write_errors == len(current_writers):
                        logging.error("All writers failed. Stopping recording.")
                        if not last_error: last_error = "Recording stopped: All writers failed."
                        stop_recording() # Stop if all failed

            # --- Update Shared Frame for Streaming ---
            with frame_lock:
                # Make a copy for the streaming thread to avoid conflicts
                output_frame = frame_bgr.copy()

        except Exception as e:
            logging.exception(f"!!! Unexpected Error in capture loop: {e}")
            last_error = f"Capture Loop Error: {e}"
            consecutive_error_count += 1
            if consecutive_error_count > max_consecutive_errors / 2: # Lower threshold for general errors
                logging.error(f"Too many errors ({consecutive_error_count}). Signaling shutdown.")
                shutdown_event.set()
            time.sleep(1) # Pause after unexpected error

    # --- Cleanup after loop exit ---
    logging.info("Exiting frame capture thread.")
    if is_recording:
        logging.warning("Capture loop exiting while recording. Forcing stop.")
        stop_recording()
    if picam2:
        try:
            if picam2.started: picam2.stop()
            picam2.close()
            logging.info("Picamera2 resource released by capture thread.")
        except Exception as e:
            logging.error(f"Error stopping/closing Picamera2 in thread cleanup: {e}")
    picam2 = None


# ===========================================================
# === FLASK ROUTES START HERE ===
# ===========================================================

# --- MJPEG Streaming Frame Generator ---
def generate_stream_frames():
    """Generator function for the MJPEG stream."""
    global output_frame
    frame_counter = 0
    last_log_time = time.monotonic()
    logging.info("MJPEG stream generator started.")

    while not shutdown_event.is_set():
        frame_to_encode = None
        with frame_lock:
            if output_frame is not None:
                # Create a copy to work on, releasing the lock quickly
                frame_to_encode = output_frame.copy()

        if frame_to_encode is None:
            # No frame available yet, wait briefly
            shutdown_event.wait(0.05) # Use event wait for faster shutdown response
            continue

        try:
            # Encode the frame (which already has overlays) as JPEG
            (flag, encodedImage) = cv2.imencode(".jpg", frame_to_encode, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if not flag:
                logging.warning("Stream generator: Could not encode frame.")
                shutdown_event.wait(0.1)
                continue

            # Yield the frame in MJPEG format
            yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
                  bytearray(encodedImage) + b'\r\n')

            frame_counter += 1
            # Optional: Log frame count periodically for debugging
            current_time = time.monotonic()
            if current_time - last_log_time > 60: # Log every 60 seconds
                 logging.debug(f"Stream generator sent {frame_counter} frames.")
                 last_log_time = current_time
                 frame_counter = 0 # Reset counter

            # Simple delay to roughly match target FPS for the stream output
            # This doesn't guarantee capture rate, just stream output rate
            time.sleep(1.0 / FRAME_RATE * 0.8) # Sleep slightly less than interval

        except GeneratorExit:
            # Client disconnected
            logging.info(f"Streaming client disconnected (sent approx {frame_counter} frames this session).")
            break # Exit the loop gracefully
        except Exception as e:
            logging.exception(f"!!! Error in streaming generator: {e}")
            # If encoding or sending fails, wait a bit before retrying
            shutdown_event.wait(0.5)

    logging.info("Stream generator thread exiting.")

# --- Flask Web Routes ---
@app.route("/")
def index():
    """Serves the main HTML page with controls."""
    global last_error, digital_recording_active
    current_w, current_h = get_current_resolution()
    resolution_text = f"{current_w}x{current_h}"
    err_msg = last_error if last_error else ""
    # Read initial digital state under lock
    with config_lock:
        initial_digital_state = digital_recording_active

    # The battery percentage is drawn on the stream itself,
    # so no need to pass it separately to the template anymore.
    return render_template_string("""
    <!DOCTYPE html>
    <html>
      <head>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Pi Camera Stream & Record</title>
        <style>
          body { font-family: sans-serif; line-height: 1.4; margin: 1em; background-color: #f4f4f4; color: #333;}
          h1 { color: #555; }
          #status-box p { margin: 0.3em 0; }
          #status, #rec-status, #resolution { font-weight: bold; padding: 2px 6px; border-radius: 4px; background-color: #ddd; color: #333;}
          #rec-status.active { background-color: #d9534f; color: white; }
          #rec-status.inactive { background-color: #5cb85c; color: white; }
          .controls button { padding: 10px 15px; margin: 5px; font-size: 1em; cursor: pointer; border-radius: 5px; border: 1px solid #ccc; background-color: #e7e7e7;}
          .controls button:hover { background-color: #ddd; }
          #error { color: red; margin-top: 10px; white-space: pre-wrap; font-weight: bold; min-height: 1.2em; background-color: #fdd; padding: 5px; border-radius: 4px; border: 1px solid red;}
          img#stream { display: block; margin: 15px 0; border: 1px solid black; max-width: 95%; height: auto; background-color: #eee; }
          button#btn-record.recording-active { background-color: #ff4d4d; color: white; border-color: #ff1a1a; }
          button#btn-record.recording-inactive { background-color: #4CAF50; color: white; border-color: #367c39;}
          button:disabled { background-color: #cccccc; cursor: not-allowed; border-color: #999; color: #666;}
          #status-box { border: 1px solid #ccc; padding: 10px; background-color: white; border-radius: 5px; margin-bottom: 15px; max-width: 600px;}
        </style>
      </head>
      <body>
        <h1>Pi Camera Stream & Record</h1>

        <div id="status-box">
             <p>Status: <span id="status">Initializing...</span></p>
             <p>Recording: <span id="rec-status" class="inactive">OFF</span></p>
             <p>Current Resolution: <span id="resolution">{{ resolution_text }}</span></p>
        </div>

        <div class="controls">
          <button onclick="changeResolution('down')" id="btn-down">&laquo; Lower Res</button>
          <button onclick="changeResolution('up')" id="btn-up">Higher Res &raquo;</button>
          <button onclick="toggleRecording()" id="btn-record" class="recording-inactive">Start Recording</button>
        </div>

        <div id="error" style="{{ 'display: block;' if err_msg else 'display: none;' }}">{{ err_msg }}</div>
        <img id="stream" src="{{ url_for('video_feed') }}" width="{{ current_w }}" height="{{ current_h }}" alt="Loading stream...">

        <script>
          const statusElement = document.getElementById('status');
          const resolutionElement = document.getElementById('resolution');
          const errorElement = document.getElementById('error');
          const streamImage = document.getElementById('stream');
          const btnUp = document.getElementById('btn-up');
          const btnDown = document.getElementById('btn-down');
          const btnRecord = document.getElementById('btn-record');
          const recStatusElement = document.getElementById('rec-status'); // Get recording status span

          let isChangingResolution = false;
          let isTogglingRecording = false;
          // Get initial state directly from server response now
          let currentDigitalRecordState = false;

          // Function to update button based on digital state
          function updateRecordButtonState() {
               if (currentDigitalRecordState) {
                   btnRecord.textContent = "Stop Recording (Digital)";
                   btnRecord.classList.remove('recording-inactive');
                   btnRecord.classList.add('recording-active');
               } else {
                   btnRecord.textContent = "Start Recording (Digital)";
                   btnRecord.classList.add('recording-inactive');
                   btnRecord.classList.remove('recording-active');
               }
               // Ensure button is enabled unless an action is in progress
               btnRecord.disabled = isTogglingRecording || isChangingResolution;
          }

          // Update status display, including recording status and button state
          function updateStatus() {
               if (isChangingResolution || isTogglingRecording) return;

               fetch('/status')
                   .then(response => response.ok ? response.json() : Promise.reject(`HTTP error! status: ${response.status}`))
                   .then(data => {
                       statusElement.textContent = data.status_text;
                       // Update recording status display style
                       if (data.is_recording) {
                           recStatusElement.textContent = "ACTIVE";
                           recStatusElement.className = 'active';
                       } else {
                            recStatusElement.textContent = "OFF";
                           recStatusElement.className = 'inactive';
                       }

                       if (data.resolution) {
                           resolutionElement.textContent = data.resolution;
                           const [w, h] = data.resolution.split('x');
                           // Only update img size if needed, prevent flicker
                           if (streamImage.width != w || streamImage.height != h) {
                             console.log(`Updating stream image dimensions to <span class="math-inline">\{w\}x</span>{h}`);
                             streamImage.width = w;
                             streamImage.height = h;
                           }
                       }

                       // Update error display
                       if (data.error) {
                           errorElement.textContent = data.error;
                           errorElement.style.display = 'block';
                       } else {
                           errorElement.textContent = '';
                           errorElement.style.display = 'none';
                       }

                       // Update digital button state based on server status
                       if (typeof data.digital_recording_active === 'boolean') {
                           // Only update if state changed from last known
                           if (currentDigitalRecordState !== data.digital_recording_active) {
                               currentDigitalRecordState = data.digital_recording_active;
                               updateRecordButtonState();
                           }
                       }
                       // Ensure buttons enabled if no action active
                       btnUp.disabled = isChangingResolution || isTogglingRecording;
                       btnDown.disabled = isChangingResolution || isTogglingRecording;
                       btnRecord.disabled = isChangingResolution || isTogglingRecording;


                   })
                   .catch(err => {
                       console.error("Error fetching status:", err);
                       statusElement.textContent = "Error fetching status";
                       errorElement.textContent = 'Error fetching status from server.';
                       errorElement.style.display = 'block';
                       recStatusElement.textContent = "Unknown";
                       recStatusElement.className = ''; // Reset class
                       // Disable buttons on comms error? Maybe allow trying again?
                       // btnUp.disabled = true; btnDown.disabled = true; btnRecord.disabled = true;
                   });
          }

          // Function for resolution change
          function changeResolution(direction) {
               if (isChangingResolution || isTogglingRecording) return;
               isChangingResolution = true;
               btnUp.disabled = true; btnDown.disabled = true; btnRecord.disabled = true; // Disable all buttons
               statusElement.textContent = 'Changing resolution... Please wait.';
               errorElement.textContent = ''; errorElement.style.display = 'none';

               fetch(`/set_resolution/${direction}`, { method: 'POST' })
                    .then(response => response.json().then(data => ({ status: response.status, body: data })))
                    .then(({ status, body }) => {
                        if (status === 200 && body.success) {
                            statusElement.textContent = 'Resolution change initiated. Stream may pause.';
                            resolutionElement.textContent = body.new_resolution;
                            const [w, h] = body.new_resolution.split('x');
                            // Update image size immediately
                            streamImage.width = w; streamImage.height = h;
                            // Use timeout to re-enable buttons after expected reconfig time
                            setTimeout(() => {
                                isChangingResolution = false;
                                // Re-enable buttons AFTER status update confirms state
                                updateStatus();
                            }, 5000); // Increased timeout for reconfig + settle
                        } else {
                            errorElement.textContent = `Error: ${body.message || 'Failed change resolution.'}`;
                            errorElement.style.display = 'block';
                            console.error("Resolution change failed:", body);
                            isChangingResolution = false; // Re-enable immediately on failure
                            updateStatus(); // Fetch status to re-enable buttons
                        }
                   })
                   .catch(err => {
                       console.error("Error sending resolution change:", err);
                       errorElement.textContent = 'Network error changing resolution.';
                       errorElement.style.display = 'block';
                       isChangingResolution = false;
                       updateStatus(); // Fetch status to re-enable buttons
                   });
          }

          // Function for toggling recording via web UI
          function toggleRecording() {
               if (isChangingResolution || isTogglingRecording) return; // Prevent overlaps
               isTogglingRecording = true;
               btnRecord.disabled = true; // Disable button during request
               statusElement.textContent = 'Sending record command...';

               fetch('/toggle_recording', { method: 'POST' })
                    .then(response => response.ok ? response.json() : Promise.reject(`HTTP error! Status: ${response.status}`))
                    .then(data => {
                        if (data.success) {
                            currentDigitalRecordState = data.digital_recording_active; // Update local state immediately
                            updateRecordButtonState(); // Update button appearance
                            statusElement.textContent = `Digital recording ${currentDigitalRecordState ? 'enabled' : 'disabled'}.`;
                            // Fetch full status shortly after to confirm actual recording state
                            setTimeout(updateStatus, 1500);
                        } else {
                            errorElement.textContent = `Error: ${data.message || 'Failed to toggle recording.'}`;
                            errorElement.style.display = 'block';
                            statusElement.textContent = 'Command failed.';
                            // Fetch status to see if state changed anyway or error occurred
                             setTimeout(updateStatus, 1000);
                        }
                   })
                   .catch(err => {
                       console.error("Error toggling recording:", err);
                       errorElement.textContent = 'Network error toggling recording.';
                       errorElement.style.display = 'block';
                       statusElement.textContent = 'Command failed (Network).';
                       // Fetch status after error
                       setTimeout(updateStatus, 1000);
                   })
                   .finally(() => {
                       // Re-enable button after request finishes (success or fail)
                       isTogglingRecording = false;
                       // updateStatus will handle re-enabling based on global flags
                       // btnRecord.disabled = false; // Re-enabled by updateStatus() check
                       updateStatus(); // Fetch status to ensure button state is correct
                   });
          }


          // --- Initial setup and intervals ---
          document.addEventListener('DOMContentLoaded', () => {
               updateStatus(); // Initial status fetch includes button state update
          });
          // Use setInterval for periodic status updates
          setInterval(() => {
                // Only update if no actions are currently blocking UI updates
                if (!isChangingResolution && !isTogglingRecording) {
                    updateStatus();
                }
          }, 5000); // Check every 5 seconds

          // --- Stream error handling ---
          let errorReloadTimeout = null;
          streamImage.onerror = function() {
               console.warn("Stream image error detected (onerror).");
               if (errorReloadTimeout) return; // Already scheduled
               statusElement.textContent = 'Stream interrupted. Reloading...';
               errorReloadTimeout = setTimeout(() => {
                   console.log("Attempting stream reload...");
                   // Cache bust to force reload
                   streamImage.src = "{{ url_for('video_feed') }}?" + new Date().getTime();
                   errorReloadTimeout = null;
                   // Update status shortly after attempting reload
                   setTimeout(updateStatus, 1500);
               }, 3000); // Wait 3 seconds before reload attempt
          };
          streamImage.onload = function() { // Clear error timeout on success
               if (errorReloadTimeout) {
                    clearTimeout(errorReloadTimeout);
                    errorReloadTimeout = null;
                    console.log("Stream reloaded successfully.");
                    // Optionally update status immediately on successful load
                    // updateStatus();
               }
          };

        </script>
      </body>
    </html>
    """, resolution_text=resolution_text, current_w=current_w, current_h=current_h, err_msg=err_msg, digital_recording_active=initial_digital_state)


@app.route("/video_feed")
def video_feed():
    """Returns the MJPEG stream."""
    logging.info("Client connected to video feed.")
    return Response(generate_stream_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/status")
def status():
    """Returns the current status as JSON."""
    global last_error, digital_recording_active, is_recording, battery_percentage # Access globals
    status_text = "Streaming"
    rec_stat = ""
    current_w, current_h = get_current_resolution()

    # Check actual recording state (is_recording reflects combined logic outcome)
    current_is_recording = is_recording # Sample current state
    if current_is_recording:
        if not video_writers:
            rec_stat = " (ERROR: Rec stopped - writers missing)"
            if not last_error: last_error = "Recording stopped: writers missing"
            # Correct state if inconsistent
            if is_recording: is_recording = False # Use actual global here
        else:
            rec_stat = f" (Recording to {len(recording_paths)} USBs)"
    status_text += rec_stat

    # Get the digital button's desired state under lock
    with config_lock:
        current_digital_state = digital_recording_active

    # Get battery state under lock
    with battery_lock:
        current_batt_perc = battery_percentage

    err_msg = last_error if last_error else ""

    # Simple logic to clear non-fatal errors if system seems ok now
    if switch is not None and last_error and "GPIO Setup Error" in last_error:
        last_error = None; err_msg = "" # GPIO came back
    if picam2 is not None and picam2.started and last_error and ("Init Error" in last_error or "unavailable" in last_error):
         last_error = None; err_msg = "" # Camera seems ok now
    if ina219_sensor is not None and current_batt_perc != -1 and last_error and ("INA219" in last_error):
        last_error = None; err_msg = "" # Battery reading seems ok now


    return jsonify({
        'is_recording': current_is_recording, # Actual recording state
        'digital_recording_active': current_digital_state, # Digital button's state
        'resolution': f"{current_w}x{current_h}",
        'status_text': status_text,
        'error': err_msg,
        'battery_percent': current_batt_perc, # Include battery % (-1 if error/disabled)
        'active_recordings': recording_paths # List of files being written
    })


@app.route("/set_resolution/<direction>", methods=['POST'])
def set_resolution(direction):
    """Endpoint to request resolution change."""
    global current_resolution_index, reconfigure_resolution_index, last_error
    with config_lock:
        if reconfigure_resolution_index is not None:
            logging.warning("Reconfiguration already in progress.")
            return jsonify({'success': False, 'message': 'Reconfiguration already in progress.'}), 429

        # Validate current_resolution_index before proceeding
        if not (0 <= current_resolution_index < len(SUPPORTED_RESOLUTIONS)):
             logging.error(f"Internal state error: Invalid current_resolution_index ({current_resolution_index}). Resetting to default.")
             current_resolution_index = DEFAULT_RESOLUTION_INDEX # Attempt recovery
             # Still report error to client
             return jsonify({'success': False, 'message': 'Internal error: Invalid state. Try again.'}), 500

        original_index = current_resolution_index
        new_index = current_resolution_index

        if direction == 'up': new_index += 1
        elif direction == 'down': new_index -= 1
        else:
             return jsonify({'success': False, 'message': 'Invalid direction.'}), 400

        # Clamp index within valid range
        new_index = max(0, min(len(SUPPORTED_RESOLUTIONS) - 1, new_index))

        if new_index == original_index:
            msg = 'Already at highest resolution.' if direction == 'up' else 'Already at lowest resolution.'
            return jsonify({'success': False, 'message': msg}), 400

        new_w, new_h = SUPPORTED_RESOLUTIONS[new_index]
        logging.info(f"Web request: change resolution index {original_index} -> {new_index} ({new_w}x{new_h})")
        # Set the request flag for the capture thread
        reconfigure_resolution_index = new_index
        last_error = None # Clear previous errors on user action
        return jsonify({'success': True, 'message': 'Resolution change requested.', 'new_resolution': f"{new_w}x{new_h}"})


@app.route('/toggle_recording', methods=['POST'])
def toggle_recording():
    """Toggles the digital recording state."""
    global digital_recording_active, last_error
    new_state = False
    with config_lock: # Ensure thread-safe modification
        digital_recording_active = not digital_recording_active
        new_state = digital_recording_active
        logging.info(f"Digital recording toggled via web UI to: {'ON' if new_state else 'OFF'}")
        # Clear any previous recording start/stop errors when user interacts
        if last_error and ("Recording" in last_error or "writers" in last_error or "USB" in last_error):
             last_error = None

    # The capture loop will pick up this change and start/stop recording accordingly
    return jsonify({'success': True, 'digital_recording_active': new_state})

# ===========================================================
# === FLASK ROUTES END HERE ===
# ===========================================================


# --- Main Execution ---
def signal_handler(sig, frame):
    """Handles termination signals for graceful shutdown."""
    if shutdown_event.is_set():
        logging.warning("Shutdown already in progress. Please wait.")
        return
    logging.warning(f"Received signal {sig}. Initiating graceful shutdown...")
    shutdown_event.set()

def main():
    """Main function to initialize and manage threads."""
    global last_error, capture_thread, flask_thread, battery_thread, picam2, ina219_sensor

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    logging.info(" --- Starting Camera Stream & Record Service (Picamera2 & gpiozero) --- ")

    # Main loop allows for restarts on critical failures
    while not shutdown_event.is_set():
        last_error = None # Reset errors for this run
        capture_thread = None
        flask_thread = None
        battery_thread = None # Reset battery thread too

        # Pre-loop cleanup of camera resource
        if picam2:
            try:
                if picam2.started: picam2.stop()
                picam2.close()
            except Exception as cleanup_e:
                logging.warning(f"Pre-loop camera cleanup error: {cleanup_e}")
            picam2 = None
        # Reset sensor instance (setup will recreate)
        ina219_sensor = None

        try:
            logging.info("Initializing Hardware...")
            if not setup_gpio():
                 logging.error(f"GPIO setup failed: {last_error}. Switch control unavailable.")
                 # Continue without switch? Or exit? Decide based on importance. For now, continue.

            # Setup Battery monitor (only if enabled)
            if ENABLE_BATTERY_MONITOR:
                if not setup_ina219():
                    logging.error(f"INA219 setup failed: {last_error}. Battery monitoring unavailable.")
                    # Continue without battery monitor

            logging.info("Starting frame capture thread (Picamera2)...")
            capture_thread = threading.Thread(target=capture_and_process_loop, name="CaptureThread", daemon=True)
            capture_thread.start()

            # Start battery monitor thread (only if setup was successful)
            if ina219_sensor:
                logging.info("Starting battery monitoring thread...")
                battery_thread = threading.Thread(target=battery_monitor_loop, name="BatteryThread", daemon=True)
                battery_thread.start()

            # Wait for camera initialization within the capture thread
            time.sleep(4) # Increased wait time for camera init + first frame
            if not capture_thread.is_alive():
                 raise RuntimeError(f"Capture thread failed start/init: {last_error or 'Thread died before streaming'}")
            # Check if init failed specifically
            if last_error and ("Error" in last_error or "failed" in last_error):
                 # If the error is fatal for the camera, raise it to trigger restart
                 if "Picamera2 Init Error" in last_error or "Camera failed fatally" in last_error:
                    raise RuntimeError(f"Camera init failed in thread: {last_error}")
                 else:
                    logging.warning(f"Non-fatal error during init: {last_error}") # Log non-fatal errors

            logging.info("Capture thread seems to be running.")

            logging.info(f"Starting Flask web server on port {WEB_PORT}...")
            # Use Flask's development server (simple, but ok for this)
            # For production, consider a more robust WSGI server like Gunicorn or Waitress
            flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=WEB_PORT, debug=False, use_reloader=False, threaded=True), name="FlaskThread", daemon=True)
            flask_thread.start()

            time.sleep(1.5) # Give Flask a moment to start
            if not flask_thread.is_alive():
                 raise RuntimeError("Flask thread failed to start.")

            logging.info("--- System Running ---")
            logging.info(f"Access stream at: http://<YOUR_PI_IP>:{WEB_PORT}")

            # Keep main thread alive, checking worker threads periodically
            while not shutdown_event.is_set():
                if not capture_thread.is_alive():
                    raise RuntimeError(last_error or "Capture thread terminated unexpectedly.")
                if not flask_thread.is_alive():
                    raise RuntimeError(last_error or "Flask thread terminated unexpectedly.")
                if battery_thread and not battery_thread.is_alive():
                    # Battery thread dying might be less critical? Log it.
                    logging.warning("Battery monitoring thread terminated unexpectedly.")
                    # Clear the thread variable so we don't try to join it later if it died.
                    battery_thread = None
                    # Optionally try to restart it? For now, just log.

                # Wait for shutdown signal or timeout
                shutdown_event.wait(timeout=5.0) # Check threads every 5s

            # If we exited the loop because shutdown_event was set, break the outer loop too
            break

        except RuntimeError as e:
            logging.error(f"!!! Runtime Error in Main loop: {e}")
            logging.error("Attempting restart after 10s pause...")
            # Signal threads to stop (if they aren't already the cause of the error)
            shutdown_event.set()
            # Wait briefly for threads to potentially exit
            if capture_thread and capture_thread.is_alive(): capture_thread.join(timeout=3.0)
            if battery_thread and battery_thread.is_alive(): battery_thread.join(timeout=1.0)
            # Flask is daemon, should exit; joining might hang if it's stuck
            # Reset shutdown event for the next iteration of the loop
            shutdown_event.clear()
            # Ensure resources are clear before next attempt
            if picam2: picam2.close(); picam2 = None
            ina219_sensor = None
            time.sleep(10.0) # Pause before restarting loop
        except Exception as e:
            logging.exception(f"!!! Unhandled Exception in Main: {e}")
            logging.error("Attempting restart after 10s pause...")
            shutdown_event.set()
            if capture_thread and capture_thread.is_alive(): capture_thread.join(timeout=3.0)
            if battery_thread and battery_thread.is_alive(): battery_thread.join(timeout=1.0)
            shutdown_event.clear()
            if picam2: picam2.close(); picam2 = None
            ina219_sensor = None
            time.sleep(10.0)

    # --- Final Cleanup ---
    logging.info("--- Program Exit ---")


if __name__ == '__main__':
    main()