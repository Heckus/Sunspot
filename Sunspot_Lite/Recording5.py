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
# Import controls for AWB, Denoise, etc.:
from libcamera import controls
import smbus # <<< CHANGE: Added for battery monitor
import math # <<< CHANGE: Added for battery monitor clamping

# --- Configuration ---
# ### CHANGE 2 START ###
# Reordered resolutions and added max native resolution for IMX219
SUPPORTED_RESOLUTIONS = [
    (640, 480),      # VGA
    (1280, 720),     # 720p HD
    (1640, 1232),    # Custom Pi Cam V2 mode
    (1920, 1080),    # 1080p FHD
    (3280, 2464)     # Max Native Resolution IMX219
]
# Default to 720p maybe? Index 1
DEFAULT_RESOLUTION_INDEX = 1
# ### CHANGE 2 END ###

FRAME_RATE = 30 # Note: Max resolution might not achieve this FPS
SWITCH_GPIO_PIN = 17
SWITCH_BOUNCE_TIME = 0.1
USB_BASE_PATH = "/media/hecke/"
RECORDING_FORMAT = "mp4v" # Use 'mp4v' for .mp4, check codec availability
RECORDING_EXTENSION = ".mp4"
WEB_PORT = 8000

# --- Battery Monitor Configuration --- # <<< CHANGE: New section
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
config_lock = threading.Lock() # Lock for shared config like resolution index, digital record state, AND battery %
capture_thread = None
flask_thread = None
shutdown_event = threading.Event()
switch = None
current_resolution_index = DEFAULT_RESOLUTION_INDEX
is_recording = False # Actual recording state
video_writers = []
recording_paths = []
reconfigure_resolution_index = None
last_error = None

# ### CHANGE 3 START ###
# State for the digital recording button
digital_recording_active = False
# ### CHANGE 3 END ###

# --- Battery Monitor Globals --- # <<< CHANGE: New section
ina219_sensor = None
battery_percentage = None # Stores the calculated percentage
last_battery_read_time = 0.0
# --- End Battery Monitor Globals ---

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')

# ===========================================================
# === BATTERY MONITOR (INA219) CODE START ===
# From provided example, minimally adapted
# ===========================================================

# Config Register (R/W)
_REG_CONFIG                      = 0x00
# SHUNT VOLTAGE REGISTER (R)
_REG_SHUNTVOLTAGE                = 0x01
# BUS VOLTAGE REGISTER (R)
_REG_BUSVOLTAGE                  = 0x02
# POWER REGISTER (R)
_REG_POWER                       = 0x03
# CURRENT REGISTER (R)
_REG_CURRENT                     = 0x04
# CALIBRATION REGISTER (R/W)
_REG_CALIBRATION                 = 0x05

class BusVoltageRange:
    """Constants for ``bus_voltage_range``"""
    RANGE_16V                      = 0x00      # set bus voltage range to 16V
    RANGE_32V                      = 0x01      # set bus voltage range to 32V (default)

class Gain:
    """Constants for ``gain``"""
    DIV_1_40MV                     = 0x00      # shunt prog. gain set to  1, 40 mV range
    DIV_2_80MV                     = 0x01      # shunt prog. gain set to /2, 80 mV range
    DIV_4_160MV                    = 0x02      # shunt prog. gain set to /4, 160 mV range
    DIV_8_320MV                    = 0x03      # shunt prog. gain set to /8, 320 mV range

class ADCResolution:
    """Constants for ``bus_adc_resolution`` or ``shunt_adc_resolution``"""
    ADCRES_9BIT_1S                 = 0x00      #  9bit,   1 sample,     84us
    ADCRES_10BIT_1S                = 0x01      # 10bit,   1 sample,    148us
    ADCRES_11BIT_1S                = 0x02      # 11 bit,  1 sample,    276us
    ADCRES_12BIT_1S                = 0x03      # 12 bit,  1 sample,    532us
    ADCRES_12BIT_2S                = 0x09      # 12 bit,  2 samples,  1.06ms
    ADCRES_12BIT_4S                = 0x0A      # 12 bit,  4 samples,  2.13ms
    ADCRES_12BIT_8S                = 0x0B      # 12bit,   8 samples,  4.26ms
    ADCRES_12BIT_16S               = 0x0C      # 12bit,  16 samples,  8.51ms
    ADCRES_12BIT_32S               = 0x0D      # 12bit,  32 samples, 17.02ms
    ADCRES_12BIT_64S               = 0x0E      # 12bit,  64 samples, 34.05ms
    ADCRES_12BIT_128S              = 0x0F      # 12bit, 128 samples, 68.10ms

class Mode:
    """Constants for ``mode``"""
    POWERDOW                       = 0x00      # power down
    SVOLT_TRIGGERED                = 0x01      # shunt voltage triggered
    BVOLT_TRIGGERED                = 0x02      # bus voltage triggered
    SANDBVOLT_TRIGGERED            = 0x03      # shunt and bus voltage triggered
    ADCOFF                         = 0x04      # ADC off
    SVOLT_CONTINUOUS               = 0x05      # shunt voltage continuous
    BVOLT_CONTINUOUS               = 0x06      # bus voltage continuous
    SANDBVOLT_CONTINUOUS           = 0x07      # shunt and bus voltage continuous


class INA219:
    def __init__(self, i2c_bus=1, addr=0x40):
        # <<< CHANGE: Added try/except for smbus initialization
        try:
            self.bus = smbus.SMBus(i2c_bus)
            self.addr = addr
            # Set chip to known config values to start
            self._cal_value = 0
            self._current_lsb = 0
            self._power_lsb = 0
            # Use the 16V setting as it's more common for Pi battery packs
            # If you use > 16V packs, uncomment the 32V call instead
            self.set_calibration_16V_5A()
            # self.set_calibration_32V_2A()
            logging.info(f"INA219 sensor initialized at address 0x{addr:X} on bus {i2c_bus}")
        except FileNotFoundError:
            logging.error(f"!!! I2C bus {i2c_bus} not found. Check raspi-config.")
            raise
        except Exception as e:
            logging.error(f"!!! Failed to initialize INA219 sensor at 0x{addr:X}: {e}", exc_info=True)
            raise # Re-raise exception to signal failure

    def read(self,address):
        # Reads unsigned 16-bit value from I2C device
        data = self.bus.read_i2c_block_data(self.addr, address, 2)
        return ((data[0] * 256 ) + data[1])

    def write(self,address,data):
        # Writes 16-bit value to I2C device
        temp = [0,0]
        temp[1] = data & 0xFF
        temp[0] =(data & 0xFF00) >> 8
        self.bus.write_i2c_block_data(self.addr,address,temp)

    def set_calibration_16V_5A(self):
        """Configures to INA219 to be able to measure up to 16V and 5A of current.
           Assumes a 0.01 shunt ohm resistor is present (adjust calculations if different).
           Calculation Details:
           VBUS_MAX = 16V
           VSHUNT_MAX = 0.08 (Gain 2, 80mV)
           RSHUNT = 0.01 ohm
           MaxPossible_I = VSHUNT_MAX / RSHUNT = 8.0A
           MaxExpected_I = 5.0A (adjust if necessary)
           MinLSB = MaxExpected_I / 32767 = 0.0001529 A/bit
           MaxLSB = MaxExpected_I / 4096 = 0.0012207 A/bit
           Chosen LSB (CurrentLSB) = 0.0001524 A/bit (approx 152.4 uA/bit) - slightly adjusted for integer calibration
           Cal = trunc(0.04096 / (CurrentLSB * RSHUNT)) = trunc(0.04096 / (0.0001524 * 0.01)) = 26868 (0x68F4)
           PowerLSB = 20 * CurrentLSB = 0.003048 W/bit (approx 3.05 mW/bit)
           Max Current before overflow = CurrentLSB * 32767 = 5.0A
           Max Shunt Voltage = Max_Current * RSHUNT = 0.05V (well below VSHUNT_MAX 0.08V)
           Max Power = Max_Current * VBUS_MAX = 5.0A * 16V = 80W
        """
        self._current_lsb = 0.0001524  # Current LSB set to ~152.4uA per bit
        self._cal_value = 26868        # Calibration value 0x68F4

        # Compute power LSB = 20 * CurrentLSB
        self._power_lsb = 0.003048     # Power LSB = ~3.05mW per bit

        # Set Calibration register
        self.write(_REG_CALIBRATION, self._cal_value)

        # Set Config register: 16V, Gain/2 (80mV), 12bit/32 sample ADC average, Continuous mode
        self.bus_voltage_range = BusVoltageRange.RANGE_16V
        self.gain = Gain.DIV_2_80MV
        self.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.mode = Mode.SANDBVOLT_CONTINUOUS
        config = self.bus_voltage_range << 13 | \
                 self.gain << 11 | \
                 self.bus_adc_resolution << 7 | \
                 self.shunt_adc_resolution << 3 | \
                 self.mode
        self.write(_REG_CONFIG, config)
        # logging.debug(f"INA219 configured for 16V/5A (Cal: {self._cal_value}, Config: {config:04X})")

    # Kept 32V method for reference, but 16V is used by default
    def set_calibration_32V_2A(self):
        """Configures to INA219 to be able to measure up to 32V and 2A of current.
           Assumes a 0.1 shunt ohm resistor is present.
        """
        # VBUS_MAX = 32V, VSHUNT_MAX = 0.32 (Gain 8, 320mV), RSHUNT = 0.1
        # MaxPossible_I = 3.2A
        # MaxExpected_I = 2.0A
        # MinLSB = 2.0A/32767 = 0.000061 A/bit
        # MaxLSB = 2.0A/4096 = 0.000488 A/bit
        # CurrentLSB = 0.0001 (100uA per bit chosen)
        # Cal = trunc(0.04096 / (0.0001 * 0.1)) = 4096 (0x1000)
        # PowerLSB = 20 * CurrentLSB = 0.002 W/bit
        # Max Current before overflow = 0.0001 * 32767 = 3.2767A
        # Max Shunt Voltage = 3.2767A * 0.1 = 0.327V (~VSHUNT_MAX)
        # Max Power = 3.2767A * 32V = 104.8W
        self._current_lsb = 0.0001  # Current LSB = 100uA per bit
        self._cal_value = 4096      # Calibration value 0x1000
        self._power_lsb = 0.002     # Power LSB = 2mW per bit

        self.write(_REG_CALIBRATION, self._cal_value)

        self.bus_voltage_range = BusVoltageRange.RANGE_32V
        self.gain = Gain.DIV_8_320MV
        self.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.mode = Mode.SANDBVOLT_CONTINUOUS
        config = self.bus_voltage_range << 13 | \
                 self.gain << 11 | \
                 self.bus_adc_resolution << 7 | \
                 self.shunt_adc_resolution << 3 | \
                 self.mode
        self.write(_REG_CONFIG, config)
        # logging.debug(f"INA219 configured for 32V/2A (Cal: {self._cal_value}, Config: {config:04X})")

    def getShuntVoltage_mV(self):
        # Reads the shunt voltage in mV
        # Sometimes a calibration write is needed before reading registers? Seems optional usually.
        # self.write(_REG_CALIBRATION, self._cal_value)
        value = self.read(_REG_SHUNTVOLTAGE)
        # Convert to signed value if necessary (using two's complement)
        if value > 32767:
            value -= 65536
        return value * 0.01 # LSB is 10uV = 0.01mV

    def getBusVoltage_V(self):
        # Reads the bus voltage in V
        # self.write(_REG_CALIBRATION, self._cal_value)
        self.read(_REG_BUSVOLTAGE) # Seems to need a read first? Example had this.
        raw_val = self.read(_REG_BUSVOLTAGE)
        # Shift to remove lower 3 bits (status bits), LSB is 4mV
        shifted_val = raw_val >> 3
        return shifted_val * 0.004

    def getCurrent_mA(self):
        # Reads the current in mA
        # self.write(_REG_CALIBRATION, self._cal_value) # Might be needed
        value = self.read(_REG_CURRENT)
        if value > 32767:
            value -= 65536
        # Multiply by current LSB (which is in A/bit) and convert A to mA
        return value * self._current_lsb * 1000

    def getPower_W(self):
        # Reads the power in W
        # self.write(_REG_CALIBRATION, self._cal_value) # Might be needed
        value = self.read(_REG_POWER)
        if value > 32767:
            value -= 65536
        # Multiply by power LSB (which is W/bit)
        return value * self._power_lsb

# ===========================================================
# === BATTERY MONITOR (INA219) CODE END ===
# ===========================================================


# --- Setup GPIO ---
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

# --- Setup Battery Monitor --- # <<< CHANGE: New function
def setup_battery_monitor():
    """Initializes the INA219 sensor."""
    global ina219_sensor, last_error
    logging.info("Setting up Battery Monitor (INA219)...")
    try:
        ina219_sensor = INA219(addr=INA219_I2C_ADDRESS)
        # Perform an initial read to check connectivity
        voltage = ina219_sensor.getBusVoltage_V()
        logging.info(f"INA219 Sensor setup complete. Initial voltage reading: {voltage:.2f}V")
        # Try an initial battery level read
        read_battery_level()
        return True
    except Exception as e:
        logging.error(f"!!! Failed to setup INA219 Battery Monitor: {e}", exc_info=True)
        last_error = f"Battery Monitor Setup Error: {e}"
        ina219_sensor = None
        return False

# --- Read Battery Level --- # <<< CHANGE: New function
def read_battery_level():
    """Reads voltage from INA219 and calculates battery percentage."""
    global battery_percentage, ina219_sensor, last_error, config_lock
    if ina219_sensor is None:
        # logging.debug("Battery sensor not available for reading.") # Too noisy maybe
        return

    try:
        bus_voltage = ina219_sensor.getBusVoltage_V()
        # Calculate percentage using configured min/max voltages
        # Ensure range denominator is not zero
        voltage_range = BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE
        if voltage_range <= 0:
             logging.warning("Battery min/max voltages invalid. Cannot calculate percentage.")
             percent = None
        else:
            percent = ((bus_voltage - BATTERY_MIN_VOLTAGE) / voltage_range) * 100.0
            # Clamp the percentage between 0 and 100
            percent = max(0.0, min(100.0, percent))

        # Update global state (use lock for thread safety)
        with config_lock:
            battery_percentage = percent

        # logging.debug(f"Read battery: {bus_voltage:.2f}V -> {percent:.1f}%") # Log if needed

    except OSError as e: # Catch I2C communication errors
        logging.error(f"!!! I2C Error reading INA219: {e}")
        # Don't flood last_error, maybe set battery % to None
        with config_lock:
            battery_percentage = None
    except Exception as e:
        logging.error(f"!!! Error reading battery level: {e}", exc_info=True)
        with config_lock:
            battery_percentage = None


def get_current_resolution():
    """Gets the resolution tuple based on the current index."""
    with config_lock: # Protect access to current_resolution_index
        if 0 <= current_resolution_index < len(SUPPORTED_RESOLUTIONS):
            return SUPPORTED_RESOLUTIONS[current_resolution_index]
        else:
            logging.warning(f"Invalid resolution index {current_resolution_index}, falling back to default.")
            safe_default_index = max(0, min(len(SUPPORTED_RESOLUTIONS) - 1, DEFAULT_RESOLUTION_INDEX))
            return SUPPORTED_RESOLUTIONS[safe_default_index]

# --- Modified initialize_camera for Picamera2 ---
def initialize_camera(target_width, target_height):
    """Initializes or re-initializes the camera capture object with specific resolution and quality settings."""
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
        time.sleep(0.5) # Give hardware time to release

    try:
        picam2 = Picamera2()
        # <<< CHANGE: Using BGR888 directly for OpenCV compatibility
        # <<< CHANGE: Added more controls for image quality tuning
        config = picam2.create_video_configuration(
            main={"size": (target_width, target_height), "format": "BGR888"}, # Use BGR for easier OpenCV use
            # Try limiting buffer count if memory issues arise, default is often 4
            # buffer_count=3,
            controls={
                "FrameRate": float(FRAME_RATE),
                # --- White Balance ---
                # Explicitly set Auto White Balance mode
                # Try different modes if 'Auto' gives a tint (e.g., Daylight, Cloudy, Tungsten)
                "AwbEnable": True,
                "AwbMode": controls.AwbModeEnum.Indoor, # Options: Auto, Tungsten, Fluorescent, Indoor, Daylight, Cloudy
                # --- Denoising ---
                # Options: Off, Fast, HighQuality. 'Fast' preferred for lower latency.
                #"NoiseReductionMode": controls.NoiseReductionMode.Fast,
                # --- Other Adjustments (Optional - Experiment) ---
                # Range 0.0 to 1.0 typically. Default 0.0
                "Brightness": 0.0,
                # Range 0.0 to potentially 16.0. Default 1.0 (no change)
                "Contrast": 1.0,
                 # Range 0.0 to potentially 16.0. Default 1.0 (no change)
                "Saturation": 1.0,
                # --- Exposure (usually best left auto unless needed) ---
                # "AeEnable": True, # Auto exposure
                # "ExposureTime": 10000, # Manual exposure time in microseconds (if AeEnable=False)
                # "AnalogueGain": 1.0, # Manual gain (if AeEnable=False)
                }
        )
        logging.info(f"Configuring Picamera2 with: {config}")
        picam2.configure(config)
        logging.info("Picamera2 configuration successful!")

        # Some controls might be better set *after* configure/start
        # Example: picam2.set_controls({"AwbMode": controls.AwbModeEnum.Daylight})

        picam2.start()
        logging.info("Camera started")
        # Allow camera time to settle, especially for AWB
        # Increase if exposure/AWB takes longer with new settings
        time.sleep(2.0)

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
            # Check if it's a directory and writable
            if os.path.isdir(path) and os.access(path, os.W_OK):
                # Basic check: try creating and deleting a temporary file
                test_file = os.path.join(path, f".write_test_{os.getpid()}")
                try:
                    with open(test_file, 'w') as f:
                        f.write('test')
                    os.remove(test_file)
                    mounts.append(path)
                    logging.debug(f"Found writable mount: {path}")
                except Exception as write_err:
                     logging.warning(f"Directory {path} is not truly writable: {write_err}")
            elif os.path.isdir(path):
                 logging.debug(f"Directory {path} found but not writable.")

        if not mounts:
            logging.debug("No writable USB mounts found.")
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
        last_error = f"Cannot start recording: No writable USB drives found"
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
        # Get current camera configuration dynamically
        cam_config = picam2.camera_configuration()['main']
        width = cam_config['size'][0]
        height = cam_config['size'][1]
        # Use configured FRAME_RATE, but high resolutions might struggle
        fps = float(FRAME_RATE)
        if width >= 3280 and fps > 15:
            logging.warning(f"High resolution ({width}x{height}) detected. Capping recording FPS to 15 for stability.")
            fps = 15.0 # Cap FPS for very high resolutions if needed

        if width <= 0 or height <= 0:
            logging.error(f"Picamera2 reported invalid dimensions ({width}x{height}). Cannot start recording.")
            last_error = "Invalid camera dimensions reported by Picamera2."
            return False

        logging.info(f"Starting recording with dimensions: {width}x{height} @ {fps:.1f}fps")

        fourcc = cv2.VideoWriter_fourcc(*RECORDING_FORMAT)
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        for drive_path in usb_drives:
            try:
                filename = f"recording_{timestamp}_{width}x{height}{RECORDING_EXTENSION}"
                full_path = os.path.join(drive_path, filename)
                writer = cv2.VideoWriter(full_path, fourcc, fps, (width, height))

                if not writer.isOpened():
                    logging.error(f"!!! Failed to open VideoWriter for: {full_path}. Check codec ('{RECORDING_FORMAT}'), permissions, disk space.")
                    start_error = f"Failed writer: {os.path.basename(drive_path)}/{filename} (Codec: {RECORDING_FORMAT})"
                    continue # Try next drive

                video_writers.append(writer)
                recording_paths.append(full_path)
                logging.info(f"Successfully started recording to: {full_path}")
                success_count += 1

            except Exception as e:
                logging.error(f"!!! Failed to create VideoWriter for {drive_path}: {e}", exc_info=True)
                start_error = f"Error writer: {os.path.basename(drive_path)}: {e}"

        if success_count > 0:
            is_recording = True
            logging.info(f"Recording started on {success_count} drive(s).")
            if start_error:
                logging.warning(f"Note: Issues starting recording on all drives: {start_error}")
                # Set last error only if *some* drives failed, but not if all failed (handled below)
                if success_count < len(usb_drives):
                    last_error = f"Partial Rec Start Fail: {start_error}"
                else:
                     last_error = None # Succeeded on all available drives
            else:
                last_error = None # Clear previous errors if successful now
            return True
        else:
            is_recording = False
            logging.error("Failed to start recording on ANY USB drive.")
            if start_error: last_error = f"Recording Start Failed: {start_error}"
            else: last_error = "Recording Start Failed: No writers could be opened."
            # Clean up just in case
            video_writers.clear()
            recording_paths.clear()
            return False

    except Exception as e:
        logging.error(f"!!! Critical error during recording setup: {e}", exc_info=True)
        last_error = f"Recording Setup Error: {e}"
        stop_recording() # Attempt cleanup
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
    is_recording = False # Set state first
    released_count = 0
    # Make copies to iterate over while clearing originals
    writers_to_release = list(video_writers)
    paths_recorded = list(recording_paths)
    video_writers.clear()
    recording_paths.clear()

    for i, writer in enumerate(writers_to_release):
        path = paths_recorded[i] if i < len(paths_recorded) else "Unknown Path"
        try:
            writer.release()
            logging.info(f"Stopped recording and saved: {path}")
            released_count += 1
        except Exception as e:
            logging.error(f"Error releasing VideoWriter for {path}: {e}")

    logging.info(f"Recording stopped. Released {released_count} writer(s).")


# --- Modified Capture Loop for Picamera2 ---
def capture_and_process_loop():
    """Main loop for capturing frames, handling recording, and reading battery."""
    global output_frame, is_recording, last_error, picam2, switch, digital_recording_active
    global current_resolution_index, reconfigure_resolution_index
    global last_battery_read_time # <<< CHANGE: Need access to battery timer

    logging.info("Starting frame capture loop (using Picamera2)...")
    consecutive_error_count = 0
    max_consecutive_errors = 15

    width, height = get_current_resolution()
    if not initialize_camera(width, height):
        logging.error("Initial camera setup failed (Picamera2). Capture thread cannot start.")
        last_error = last_error or "Initial camera setup failed"
        return # Exit thread if camera fails initially

    while not shutdown_event.is_set():
        loop_start_time = time.monotonic() # <<< CHANGE: For battery timing

        try:
            # --- Check for Reconfiguration Request ---
            if reconfigure_resolution_index is not None:
                target_index = -1
                with config_lock:
                    if reconfigure_resolution_index is not None:
                        target_index = reconfigure_resolution_index
                        reconfigure_resolution_index = None # Consume the request

                if target_index != -1:
                    logging.info(f"--- Reconfiguring resolution to index {target_index} ---")
                    # Determine if recording SHOULD be active based on combined state *before* stopping
                    physical_switch_on_before = (switch is not None and switch.is_pressed)
                    digital_switch_on_before = digital_recording_active
                    should_be_recording_after = physical_switch_on_before or digital_switch_on_before

                    was_actually_recording = is_recording
                    if was_actually_recording:
                        logging.info("Pausing recording for reconfiguration...")
                        stop_recording()

                    new_width, new_height = SUPPORTED_RESOLUTIONS[target_index]
                    if initialize_camera(new_width, new_height):
                        current_resolution_index = target_index
                        logging.info(f"--- Reconfiguration successful to {new_width}x{new_height} ---")
                        if should_be_recording_after:
                            logging.info("Resuming recording after reconfiguration...")
                            time.sleep(1.0) # Allow camera to stabilize fully
                            if not start_recording():
                                logging.error("Failed to restart recording after reconfiguration!")
                                # last_error already set by start_recording()
                    else:
                        logging.error(f"!!! Failed reconfigure to index {target_index}. Restoring previous... !!!")
                        # last_error already set by initialize_camera()
                        prev_width, prev_height = SUPPORTED_RESOLUTIONS[current_resolution_index] # Use the index that didn't change
                        if not initialize_camera(prev_width, prev_height):
                            logging.critical("!!! Failed to restore previous camera resolution. Stopping. !!!")
                            last_error = "Camera failed fatally during reconfiguration."
                            shutdown_event.set() # Critical failure
                            break # Exit loop
                        else:
                             logging.info("Successfully restored previous camera resolution.")
                             if should_be_recording_after: # Try restarting recording even after failed reconfig
                                 logging.info("Attempting recording restart with restored resolution...")
                                 time.sleep(1.0)
                                 if not start_recording():
                                     logging.error("Failed to restart recording after failed reconfig.")
                                     # last_error set by start_recording()

            # --- Check Camera Status ---
            if picam2 is None or not picam2.started:
                if not last_error: last_error = "Picamera2 became unavailable unexpectedly."
                logging.error(f"Camera unavailable: {last_error}. Attempting reinitialization...")
                # Try to re-initialize
                width, height = get_current_resolution()
                if initialize_camera(width, height):
                     logging.info("Camera re-initialized successfully.")
                     last_error = None # Clear error on success
                     consecutive_error_count = 0
                else:
                    logging.error(f"Camera re-initialization failed: {last_error}. Stopping capture loop.")
                    shutdown_event.set()
                    break # Exit loop on persistent camera failure
                continue # Skip rest of loop iteration after re-init attempt


            # --- Read Frame using Picamera2 ---
            # <<< CHANGE: capture_array("main") now returns BGR directly due to config change
            frame_bgr = picam2.capture_array("main")

            if frame_bgr is None:
                logging.warning("Failed capture. Retrying...")
                consecutive_error_count += 1
                if consecutive_error_count > max_consecutive_errors:
                    last_error = f"Failed capture {max_consecutive_errors} consecutive times."
                    logging.error(last_error)
                    shutdown_event.set() # Treat repeated capture failures as fatal
                    break
                time.sleep(0.1) # Short pause before retry
                continue # Skip rest of loop iteration
            # Reset error count on successful capture
            if consecutive_error_count > 0:
                logging.info(f"Recovered frame grab after {consecutive_error_count} errors.")
            consecutive_error_count = 0


            # --- Handle Recording State (Combined Switch + Digital) ---
            should_be_recording = False # Default to false
            physical_switch_on = False
            if switch is not None: # Check physical switch if available
                try:
                    physical_switch_on = switch.is_pressed
                except Exception as e:
                    logging.error(f"Error reading gpiozero switch state: {e}")
                    last_error = f"Switch Read Error: {e}"
                    physical_switch_on = False # Assume off on error

            # Check digital state (needs lock for read?) Use config_lock
            with config_lock:
                digital_switch_on = digital_recording_active

            # OR logic: if either physical OR digital switch is on, we should record
            should_be_recording = physical_switch_on or digital_switch_on

            # Compare desired state with actual state
            if should_be_recording and not is_recording:
                log_msg = "Physical switch ON" if physical_switch_on else ""
                if digital_switch_on: log_msg += (" / " if log_msg else "") + "Digital toggle ON"
                logging.info(f"Recording trigger active ({log_msg}) - attempting start.")
                if not start_recording() and not last_error: # start_recording sets last_error on failure
                     last_error = "Attempted recording start (trigger ON) failed."
            elif not should_be_recording and is_recording:
                logging.info("Recording trigger(s) OFF - stopping recording.")
                stop_recording()


            # --- Write Frame if Recording ---
            if is_recording:
                if not video_writers:
                    logging.warning("is_recording=True, but no video writers. Stopping recording state.")
                    is_recording = False # Correct the state
                    last_error = "Recording stopped: writers missing unexpectedly."
                else:
                    write_errors = 0
                    # frame_bgr is already available
                    current_writers = list(video_writers) # Copy for safe iteration
                    current_paths = list(recording_paths) # Copy for error messages
                    for i, writer in enumerate(current_writers):
                        try:
                            writer.write(frame_bgr)
                        except Exception as e:
                            path_str = current_paths[i] if i < len(current_paths) else f"Writer {i}"
                            logging.error(f"!!! Failed write frame to {path_str}: {e}")
                            write_errors += 1
                            # Optional: Could try removing the failed writer from the list here
                    # If all active writers failed, stop recording
                    if write_errors > 0 and write_errors == len(current_writers):
                        logging.error("All active writers failed during write. Stopping recording.")
                        last_error = "Recording stopped: All writers failed."
                        stop_recording() # This clears video_writers and sets is_recording=False


            # --- Update Shared Frame for Streaming ---
            with frame_lock:
                output_frame = frame_bgr.copy()

            # --- Read Battery Level Periodically --- # <<< CHANGE: New section
            if ina219_sensor and (loop_start_time - last_battery_read_time > BATTERY_READ_INTERVAL):
                 read_battery_level()
                 last_battery_read_time = loop_start_time # Update time after read attempt

        except Exception as e:
            logging.exception(f"!!! Unexpected Error in capture loop: {e}")
            last_error = f"Capture Loop Error: {e}"
            consecutive_error_count += 1
            # If too many errors occur rapidly, assume a serious problem
            if consecutive_error_count > max_consecutive_errors / 2:
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
    global output_frame, frame_lock, shutdown_event
    frame_counter = 0
    # last_frame_time = time.monotonic() # Using monotonic for intervals
    logging.info("MJPEG stream generator started.")

    while not shutdown_event.is_set():
        frame_to_encode = None
        with frame_lock:
            if output_frame is not None:
                frame_to_encode = output_frame.copy()
            else:
                 # If no frame available yet, wait briefly
                 no_frame = True

        if frame_to_encode is None:
            time.sleep(0.05) # Wait if no frame is ready
            continue

        try:
            # Encode frame as JPEG
            (flag, encodedImage) = cv2.imencode(".jpg", frame_to_encode, [cv2.IMWRITE_JPEG_QUALITY, 85]) # Quality 85-90 is usually good

            if not flag:
                logging.warning("Stream generator: Could not encode frame.")
                time.sleep(0.1) # Wait a bit longer on encoding failure
                continue

            # Yield the frame in MJPEG format
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
                   bytearray(encodedImage) + b'\r\n')

            frame_counter += 1

            # Control streaming rate - Aim for slightly less than camera rate to avoid buffer build-up
            # This simple sleep aims for the target rate, but actual rate depends on processing
            # time. A more sophisticated rate limiter could be used if needed.
            # Adjust sleep based on FRAME_RATE, maybe sleep less than 1/FRAME_RATE
            time.sleep(1.0 / (FRAME_RATE + 5)) # Sleep slightly less than frame interval

            # Optional: Calculate actual FPS for logging
            # current_time = time.monotonic()
            # elapsed = current_time - last_frame_time
            # if elapsed > 0: print(f"Stream FPS: {1.0/elapsed:.1f}")
            # last_frame_time = current_time


        except GeneratorExit:
            # This happens when the client disconnects
            logging.info(f"Streaming client disconnected after {frame_counter} frames.")
            break # Exit the loop cleanly
        except Exception as e:
            # Log other errors but try to continue
            logging.exception(f"!!! Error in streaming generator: {e}")
            time.sleep(0.5) # Pause after an error

    logging.info("Stream generator thread exiting.")


# --- Flask Web Routes ---
@app.route("/")
def index():
    """Serves the main HTML page with controls."""
    global last_error, digital_recording_active, battery_percentage, config_lock # Access state
    current_w, current_h = get_current_resolution()
    resolution_text = f"{current_w}x{current_h}"
    err_msg = last_error if last_error else ""

    # Pass initial digital recording state and battery state to template
    with config_lock: # Read shared state safely
        digital_rec_state_initial = digital_recording_active
        batt_perc_initial = battery_percentage # Could be None

    # Format battery percentage for initial display
    batt_text_initial = f"{batt_perc_initial:.1f}" if batt_perc_initial is not None else "--"

    # ### CHANGE 3 START ### (Updated HTML/JS for Battery)
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
            <span>Battery:</span> <span id="battery-level">{{ batt_text_initial }}</span>% </div>

          <div class="controls">
            <button onclick="changeResolution('down')" id="btn-down" title="Decrease resolution">&laquo; Lower Res</button>
            <button onclick="toggleRecording()" id="btn-record" class="recording-inactive" title="Toggle recording via web interface">Start Recording</button>
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
          const batteryLevelElement = document.getElementById('battery-level'); // <<< CHANGE: Get battery span

          let isChangingResolution = false;
          let isTogglingRecording = false;
          // Get initial state from template rendering
          let currentDigitalRecordState = {{ 'true' if digital_rec_state_initial else 'false' }};

          // Function to update button based on digital state
          function updateRecordButtonState() {
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

          // Update status display, including recording status, battery, and button state
          function updateStatus() {
              // Don't fetch if an action is in progress
              if (isChangingResolution || isTogglingRecording) return;

              fetch('/status')
                  .then(response => response.ok ? response.json() : Promise.reject(`HTTP error! status: ${response.status}`))
                  .then(data => {
                      statusElement.textContent = data.status_text;
                      recStatusElement.textContent = data.is_recording ? "ACTIVE" : "OFF"; // Actual recording state

                      // Update Resolution Display & Image Size
                      if (data.resolution) {
                          resolutionElement.textContent = data.resolution;
                          const [w, h] = data.resolution.split('x');
                           if (streamImage.getAttribute('width') != w || streamImage.getAttribute('height') != h) {
                                console.log(`Updating stream image size to ${w}x${h}`);
                                streamImage.setAttribute('width', w);
                                streamImage.setAttribute('height', h);
                                // Reload image source if size changes drastically? Maybe not needed.
                                // streamImage.src = "{{ url_for('video_feed') }}?" + new Date().getTime();
                            }
                      }

                      // Update Error Display
                      if (data.error) {
                          errorElement.textContent = data.error;
                          errorElement.style.display = 'block';
                      } else {
                          // Clear error only if it was previously shown
                          if (errorElement.textContent !== '') {
                             errorElement.textContent = '';
                          }
                          errorElement.style.display = 'none';
                      }


                      // Update digital button state based on server status
                      if (typeof data.digital_recording_active === 'boolean') {
                          // Only update if the state has actually changed from our local copy
                          if (currentDigitalRecordState !== data.digital_recording_active) {
                             currentDigitalRecordState = data.digital_recording_active;
                             updateRecordButtonState();
                          }
                      }

                      // <<< CHANGE: Update Battery Display ---
                      if (data.battery_percent !== null && data.battery_percent !== undefined) {
                          batteryLevelElement.textContent = data.battery_percent.toFixed(1); // Format to 1 decimal place
                      } else {
                          batteryLevelElement.textContent = "--"; // Show placeholder if no data
                      }
                      // <<< --- End Battery Display Update

                  })
                  .catch(err => {
                      console.error("Error fetching status:", err);
                      statusElement.textContent = "Error fetching status";
                      errorElement.textContent = 'Error fetching status from server.';
                      errorElement.style.display = 'block';
                      recStatusElement.textContent = "Unknown";
                      batteryLevelElement.textContent = "Err"; // <<< CHANGE: Indicate battery error
                  });
          }

          // Disable all control buttons
          function disableControls() {
             btnUp.disabled = true;
             btnDown.disabled = true;
             btnRecord.disabled = true;
          }

          // Enable all control buttons
          function enableControls() {
             btnUp.disabled = false;
             btnDown.disabled = false;
             btnRecord.disabled = false;
          }


          // Function for resolution change
          function changeResolution(direction) {
              if (isChangingResolution || isTogglingRecording) return;
              isChangingResolution = true;
              disableControls();
              statusElement.textContent = 'Changing resolution... Please wait.';
              errorElement.textContent = ''; errorElement.style.display = 'none'; // Hide error during change

              fetch(`/set_resolution/${direction}`, { method: 'POST' })
                  .then(response => response.json().then(data => ({ status: response.status, body: data })))
                  .then(({ status, body }) => {
                      if (status === 200 && body.success) {
                          statusElement.textContent = 'Resolution change initiated.';
                          resolutionElement.textContent = body.new_resolution; // Update immediately
                          const [w, h] = body.new_resolution.split('x');
                          streamImage.setAttribute('width', w);
                          streamImage.setAttribute('height', h);
                          // Reload stream with new size? Might cause flicker.
                          // streamImage.src = "{{ url_for('video_feed') }}?" + new Date().getTime();
                          console.log("Resolution change requested, waiting for effect...");
                      } else {
                          errorElement.textContent = `Error: ${body.message || 'Failed change resolution.'}`;
                          errorElement.style.display = 'block';
                          console.error("Resolution change failed:", body);
                          // Re-enable controls immediately on failure to request change
                          isChangingResolution = false;
                          enableControls();
                          updateStatus(); // Refresh status to show current reality
                      }
                  })
                  .catch(err => {
                      console.error("Error sending resolution change:", err);
                      errorElement.textContent = 'Network error changing resolution.';
                      errorElement.style.display = 'block';
                      isChangingResolution = false;
                      enableControls();
                      updateStatus(); // Refresh status
                  })
                  .finally(() => {
                     // Re-enable buttons after a delay to allow camera to reconfigure
                     // Do this only if the request was successful initially
                      if (isChangingResolution) { // Check flag again in case it was set false by error handling
                        setTimeout(() => {
                            console.log("Re-enabling controls after resolution change delay.");
                            isChangingResolution = false;
                            enableControls();
                            updateStatus(); // Update status after delay
                        }, 5000); // Increased delay to ensure camera is stable
                      }
                  });
          }

          // Function for toggling recording via web UI
          function toggleRecording() {
              if (isChangingResolution || isTogglingRecording) return; // Prevent overlaps
              isTogglingRecording = true;
              disableControls(); // Disable buttons during request
              statusElement.textContent = 'Sending record command...';

              fetch('/toggle_recording', { method: 'POST' })
                  .then(response => response.ok ? response.json() : Promise.reject(`HTTP error! Status: ${response.status}`))
                  .then(data => {
                      if (data.success) {
                          currentDigitalRecordState = data.digital_recording_active; // Update local state immediately
                          updateRecordButtonState(); // Update button appearance
                          statusElement.textContent = `Digital recording ${currentDigitalRecordState ? 'enabled' : 'disabled'}. Actual state may take moments to update.`;
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
                      // Re-enable buttons after request finishes (success or fail)
                       isTogglingRecording = false;
                       enableControls();
                       // Don't call updateStatus immediately here, rely on the timeouts in then/catch
                  });
          }


          // --- Initial setup and intervals ---
          document.addEventListener('DOMContentLoaded', () => {
              updateRecordButtonState(); // Set initial button state based on rendered value
              updateStatus(); // Initial status fetch
          });
          // Periodic status update interval
          setInterval(() => {
                // Only update if no other action is running
                if (!isChangingResolution && !isTogglingRecording) {
                    updateStatus();
                }
              }, 5000); // Update every 5 seconds

          // --- Stream error handling ---
          let errorReloadTimeout = null;
          streamImage.onerror = function() {
              console.warn("Stream image error detected (onerror).");
              if (errorReloadTimeout) return; // Already scheduled
              statusElement.textContent = 'Stream interrupted. Reloading...';
              errorReloadTimeout = setTimeout(() => {
                  console.log("Attempting stream reload...");
                  streamImage.src = "{{ url_for('video_feed') }}?" + new Date().getTime(); // Cache bust
                  errorReloadTimeout = null; // Reset timeout ID
                  setTimeout(updateStatus, 1000); // Update status after trying reload
              }, 3000); // Wait 3 seconds before reload attempt
          };
          streamImage.onload = function() { // Clear error timeout on success
              if (errorReloadTimeout) {
                 console.log("Stream loaded successfully, clearing reload timeout.");
                 clearTimeout(errorReloadTimeout);
                 errorReloadTimeout = null;
                 // Optionally update status immediately on successful load
                 // updateStatus();
              }
          };

        </script>
      </body>
    </html>
    """, resolution_text=resolution_text, current_w=current_w, current_h=current_h,
         err_msg=err_msg, digital_rec_state_initial=digital_rec_state_initial, batt_text_initial=batt_text_initial)
    # ### CHANGE 3 END ###


@app.route("/video_feed")
def video_feed():
    """Returns the MJPEG stream."""
    logging.info("Client connected to video feed.")
    return Response(generate_stream_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/status")
def status():
    """Returns the current status as JSON, including battery level."""
    global last_error, digital_recording_active, is_recording, battery_percentage, config_lock # Access globals
    status_text = "Streaming"
    rec_stat_detail = ""
    current_w, current_h = get_current_resolution()

    # Read shared variables under lock
    with config_lock:
        current_digital_state = digital_recording_active
        batt_perc = battery_percentage # Get current battery percentage

    # Check actual recording state (is_recording reflects combined logic outcome)
    if is_recording:
        if not video_writers:
            rec_stat_detail = " (ERROR: Rec stopped - writers missing)"
            if not last_error: last_error = "Recording stopped: writers missing"
        else:
            rec_stat_detail = f" (Recording to {len(recording_paths)} USBs)"
    status_text += rec_stat_detail

    # Get error message, potentially clearing non-fatal ones
    err_msg = last_error if last_error else ""
    # Simple logic to clear non-fatal, recoverable errors after a while if system is otherwise okay
    # Example: Clear GPIO error if switch object exists now
    if switch is not None and err_msg and "GPIO Setup Error" in err_msg:
         last_error = None; err_msg = ""
    # Example: Clear old init errors if we have frames now
    if output_frame is not None and err_msg and ("Init Error" in err_msg or "unavailable" in err_msg or "capture/convert" in err_msg):
          last_error = None; err_msg = ""
    # Example: Clear battery errors if we have a reading now
    if batt_perc is not None and err_msg and ("Battery Monitor" in err_msg or "INA219" in err_msg):
         last_error = None; err_msg = ""


    return jsonify({
        'is_recording': is_recording,                  # Actual recording state
        'digital_recording_active': current_digital_state, # Digital button's desired state
        'resolution': f"{current_w}x{current_h}",
        'status_text': status_text,
        'error': err_msg,
        'active_recordings': recording_paths,          # List of files being written
        'battery_percent': batt_perc                   # <<< CHANGE: Added battery percentage
    })


@app.route("/set_resolution/<direction>", methods=['POST'])
def set_resolution(direction):
    """Endpoint to request resolution change."""
    global current_resolution_index, reconfigure_resolution_index, last_error, config_lock
    with config_lock:
        if reconfigure_resolution_index is not None:
            logging.warning("Reconfiguration already in progress.")
            return jsonify({'success': False, 'message': 'Reconfiguration already in progress.'}), 429 # Too Many Requests

        # Validate current index state before proceeding
        if not (0 <= current_resolution_index < len(SUPPORTED_RESOLUTIONS)):
             logging.error(f"Internal Error: Invalid current resolution index {current_resolution_index}!")
             # Attempt to reset index safely? Or just report error.
             # current_resolution_index = max(0, min(len(SUPPORTED_RESOLUTIONS) - 1, DEFAULT_RESOLUTION_INDEX))
             return jsonify({'success': False, 'message': 'Internal state error: Invalid resolution index.'}), 500 # Internal Server Error

        original_index = current_resolution_index
        new_index = current_resolution_index

        if direction == 'up':
             new_index += 1
        elif direction == 'down':
             new_index -= 1
        else:
              return jsonify({'success': False, 'message': 'Invalid direction specified.'}), 400 # Bad Request

        # Clamp new index within valid bounds
        new_index = max(0, min(len(SUPPORTED_RESOLUTIONS) - 1, new_index))

        if new_index == original_index:
            msg = 'Already at highest resolution.' if direction == 'up' else 'Already at lowest resolution.'
            return jsonify({'success': False, 'message': msg}), 400 # Bad Request (or maybe 200 OK with message?)

        # If valid change requested, store the target index for the capture loop
        new_w, new_h = SUPPORTED_RESOLUTIONS[new_index]
        logging.info(f"Web request: change resolution index {original_index} -> {new_index} ({new_w}x{new_h})")
        reconfigure_resolution_index = new_index
        last_error = None # Clear previous errors on user action
        return jsonify({'success': True, 'message': 'Resolution change requested.', 'new_resolution': f"{new_w}x{new_h}"})


# ### CHANGE 3 START ###
# New route to handle the digital record button toggle
@app.route('/toggle_recording', methods=['POST'])
def toggle_recording():
    """Toggles the digital recording state."""
    global digital_recording_active, last_error, config_lock
    new_state = False
    with config_lock: # Ensure thread-safe modification
        digital_recording_active = not digital_recording_active
        new_state = digital_recording_active
        logging.info(f"Digital recording toggled via web UI to: {'ON' if new_state else 'OFF'}")
        # Clear any previous recording start/stop errors when user interacts
        if last_error and ("Recording" in last_error or "writers" in last_error or "USB" in last_error):
             last_error = None

    # The capture loop will pick up this change in its next iteration
    # and call start_recording() or stop_recording() accordingly.
    return jsonify({'success': True, 'digital_recording_active': new_state})
# ### CHANGE 3 END ###


# ===========================================================
# === FLASK ROUTES END HERE ===
# ===========================================================


# --- Main Execution ---
def signal_handler(sig, frame):
    """Handles termination signals for graceful shutdown."""
    if shutdown_event.is_set():
        logging.warning("Shutdown already in progress.")
        return
    logging.warning(f"Received signal {sig}. Initiating graceful shutdown...")
    shutdown_event.set()

def main():
    """Main function to initialize and manage threads."""
    global last_error, capture_thread, flask_thread, picam2, shutdown_event

    # Setup signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    logging.info(" --- Starting Camera Stream & Record Service --- ")
    logging.info(f"--- Using Picamera2 and gpiozero ---")

    # Main loop allows for restart attempts on critical failures
    while not shutdown_event.is_set():
        last_error = None
        capture_thread = None
        flask_thread = None
        # Ensure clean state before starting threads
        if picam2:
            try:
                if picam2.started: picam2.stop()
                picam2.close()
            except Exception: pass # Ignore cleanup errors here
            picam2 = None

        try:
            logging.info("Initializing Hardware...")
            # Initialize GPIO first
            if not setup_gpio():
                logging.error(f"GPIO setup failed: {last_error}. Switch control will be unavailable.")
                # Continue without switch if desired, error is logged

            # Initialize Battery Monitor <<< CHANGE: Added battery setup
            if not setup_battery_monitor():
                 logging.warning(f"Battery monitor setup failed: {last_error}. Battery level unavailable.")
                 # Continue without battery monitor if desired

            # Start the camera capture thread (which also handles camera init)
            logging.info("Starting frame capture thread (includes camera init)...")
            capture_thread = threading.Thread(target=capture_and_process_loop, name="CaptureThread", daemon=True)
            capture_thread.start()

            # Wait for the capture thread to initialize the camera (or fail)
            time.sleep(4.0) # Allow time for camera init inside the thread

            if not capture_thread.is_alive():
                 # If thread died quickly, initialization likely failed critically
                 raise RuntimeError(f"Capture thread failed during startup: {last_error or 'Thread died unexpectedly'}")
            if last_error and ("Init Error" in last_error or "failed" in last_error):
                # Check if initialization within the thread reported a failure
                 raise RuntimeError(f"Camera initialization failed within thread: {last_error}")

            logging.info("Capture thread appears to be running.")

            # Start the Flask web server thread
            logging.info(f"Starting Flask web server on port {WEB_PORT}...")
            # Use daemon=True so Flask thread exits when main thread exits
            flask_thread = threading.Thread(target=lambda: app.run(host='0.0.0.0', port=WEB_PORT, debug=False, use_reloader=False, threaded=True), name="FlaskThread", daemon=True)
            flask_thread.start()

            time.sleep(1.5) # Short wait for Flask to start
            if not flask_thread.is_alive():
                raise RuntimeError("Flask web server thread failed to start.")

            logging.info("--- System Running ---")
            logging.info(f"Access web interface at: http://<YOUR_PI_IP>:{WEB_PORT}")

            # Keep main thread alive, checking worker threads periodically
            while not shutdown_event.is_set():
                if not capture_thread.is_alive():
                    raise RuntimeError(last_error or "Capture thread terminated unexpectedly.")
                if not flask_thread.is_alive():
                     # Flask thread dying is less common but possible
                     raise RuntimeError(last_error or "Flask thread terminated unexpectedly.")
                # Wait for shutdown signal or timeout
                shutdown_event.wait(timeout=5.0) # Check thread health every 5s

            # If shutdown_event is set, break outer loop for cleanup
            break

        except RuntimeError as e:
            logging.error(f"!!! Runtime Error in Main Loop: {e}")
            logging.error("This might be due to thread failure. Attempting restart after 10s pause...")
            # Signal existing threads to stop (if any are still alive)
            shutdown_event.set()
            if capture_thread and capture_thread.is_alive(): capture_thread.join(timeout=3.0)
            # Flask thread is daemon, should exit automatically, but good practice to signal
            # No explicit join needed for daemon threads usually
            shutdown_event.clear() # Reset event for the next loop iteration
            time.sleep(10.0) # Pause before restarting loop
        except Exception as e:
            logging.exception(f"!!! Unhandled Exception in Main Loop: {e}")
            logging.error("Attempting restart after 10s pause...")
            shutdown_event.set()
            if capture_thread and capture_thread.is_alive(): capture_thread.join(timeout=3.0)
            shutdown_event.clear()
            time.sleep(10.0)

    # --- Final Cleanup --- (Executed when shutdown_event is set)
    logging.info("--- Shutdown initiated ---")
    # Ensure event is set one last time
    shutdown_event.set()

    # Wait for the capture thread to finish (it handles camera/recording stop)
    if capture_thread and capture_thread.is_alive():
        logging.info("Waiting for capture thread exit...")
        capture_thread.join(timeout=5.0) # Give it time to stop recording/close camera
        if capture_thread.is_alive():
             logging.warning("Capture thread did not exit cleanly.")
        else:
             logging.info("Capture thread finished.")

    # Flask thread is daemon, no explicit join needed/possible after main exits

    # Explicitly stop recording again just in case (if capture thread failed cleanup)
    if is_recording:
         logging.warning("Force stopping recording during final shutdown.")
         stop_recording()

    # Explicitly close camera again just in case
    if picam2:
        try:
            logging.info("Ensuring Picamera2 is closed...")
            if picam2.started: picam2.stop()
            picam2.close()
            logging.info("Picamera2 closed final.")
        except Exception as e:
            logging.warning(f"Error during final Picamera2 close: {e}")

    # Cleanup GPIO
    cleanup_gpio()

    logging.info("--- Program Exit ---")


if __name__ == '__main__':
    main()