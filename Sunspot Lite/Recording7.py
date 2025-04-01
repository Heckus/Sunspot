# -*- coding: utf-8 -*-
import cv2
import time
import datetime
import os                       # <--- Ensure os is imported
import threading
import signal
from flask import Flask, Response, render_template_string, jsonify, redirect, url_for
from gpiozero import Button, Device
from gpiozero.pins.native import NativeFactory
import logging
from picamera2 import Picamera2 # Import picamera2
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
USB_BASE_PATH = "/media/hecke/" # <<< CHANGE this if your mount point base is different
RECORDING_FORMAT = "mp4v"       # H.264 often needs specific GStreamer pipelines, mp4v (MPEG-4) is simpler with OpenCV
RECORDING_EXTENSION = ".mp4"
WEB_PORT = 8000

# --- Tuning File Configuration --- # <--- NEW SECTION
# Path to the standard NoIR tuning file for the IMX219 sensor
# Use this if your camera behaves like a NoIR (lacks IR filter)
USE_NOIR_TUNING = True # Set to False to use default tuning
NOIR_TUNING_FILE_PATH = "/usr/share/libcamera/ipa/rpi/pisp/imx219_noir.json"
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
_REG_CONFIG                     = 0x00
_REG_SHUNTVOLTAGE               = 0x01
_REG_BUSVOLTAGE                 = 0x02
_REG_POWER                      = 0x03
_REG_CURRENT                    = 0x04
_REG_CALIBRATION                = 0x05

class BusVoltageRange:
    RANGE_16V                   = 0x00
    RANGE_32V                   = 0x01

class Gain:
    DIV_1_40MV                  = 0x00
    DIV_2_80MV                  = 0x01
    DIV_4_160MV                 = 0x02
    DIV_8_320MV                 = 0x03

class ADCResolution:
    ADCRES_9BIT_1S              = 0x00
    ADCRES_10BIT_1S             = 0x01
    ADCRES_11BIT_1S             = 0x02
    ADCRES_12BIT_1S             = 0x03
    ADCRES_12BIT_2S             = 0x09
    ADCRES_12BIT_4S             = 0x0A
    ADCRES_12BIT_8S             = 0x0B
    ADCRES_12BIT_16S            = 0x0C
    ADCRES_12BIT_32S            = 0x0D
    ADCRES_12BIT_64S            = 0x0E
    ADCRES_12BIT_128S           = 0x0F

class Mode:
    POWERDOW                    = 0x00
    SVOLT_TRIGGERED             = 0x01
    BVOLT_TRIGGERED             = 0x02
    SANDBVOLT_TRIGGERED         = 0x03
    ADCOFF                      = 0x04
    SVOLT_CONTINUOUS            = 0x05
    BVOLT_CONTINUOUS            = 0x06
    SANDBVOLT_CONTINUOUS        = 0x07

class INA219:
    def __init__(self, i2c_bus=1, addr=0x40):
        try:
            self.bus = smbus.SMBus(i2c_bus)
            self.addr = addr
            self._cal_value = 0
            self._current_lsb = 0
            self._power_lsb = 0
            # Use 16V setting by default - suitable for typical 3S LiPo
            self.set_calibration_16V_5A() # Default calibration
            logging.info(f"INA219 sensor initialized at address 0x{addr:X} on bus {i2c_bus}")
        except FileNotFoundError:
            logging.error(f"!!! I2C bus {i2c_bus} not found. Check raspi-config.")
            raise
        except Exception as e:
            logging.error(f"!!! Failed to initialize INA219 sensor at 0x{addr:X}: {e}", exc_info=False) # Keep log less verbose on expected errors
            raise

    def read(self,address):
        # Read 2 bytes from the specified address
        data = self.bus.read_i2c_block_data(self.addr, address, 2)
        return ((data[0] * 256 ) + data[1]) # Combine bytes

    def write(self,address,data):
        # Write 2 bytes to the specified address
        temp = [0,0]
        temp[1] = data & 0xFF           # Low byte
        temp[0] =(data & 0xFF00) >> 8   # High byte
        self.bus.write_i2c_block_data(self.addr,address,temp)

    # Set calibration for 16V and up to 5A range (good default for Pi/accessories)
    def set_calibration_16V_5A(self):
        # Calibration formula involves expected max current & shunt resistor value.
        # These values are pre-calculated for common INA219 boards (like Adafruit's)
        # assuming a 0.1 ohm shunt resistor. Adjust if your board differs.
        self._current_lsb = 0.0001524 # (Current LSB is 152.4uA per bit) approx 5A / 2^15
        self._cal_value = 26868       # Calibration register value for this config
        self._power_lsb = 0.003048    # (Power LSB is 3.048mW per bit) = 20 * current LSB
        self.write(_REG_CALIBRATION, self._cal_value)

        # Configure the sensor settings
        self.bus_voltage_range = BusVoltageRange.RANGE_16V # Up to 16V bus voltage
        self.gain = Gain.DIV_2_80MV # PGA gain /+-80mV (allows shunt up to 80mV)
        self.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S # 12-bit bus ADC, 32 samples average
        self.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S # 12-bit shunt ADC, 32 samples average
        self.mode = Mode.SANDBVOLT_CONTINUOUS # Continuous bus and shunt voltage measurements

        config = self.bus_voltage_range << 13 | self.gain << 11 | self.bus_adc_resolution << 7 | self.shunt_adc_resolution << 3 | self.mode
        self.write(_REG_CONFIG, config)
        logging.info("INA219 calibrated for 16V 5A range.")


    # Keep 32V method for reference if needed, but default to 16V
    def set_calibration_32V_2A(self):
        # Example for a 32V, 2A setup (adjust _cal_value if needed for your shunt)
        self._current_lsb = 0.0001 # Current LSB set to 100uA
        self._cal_value = 4096     # Calibration value
        self._power_lsb = 0.002    # Power LSB = 2mW
        self.write(_REG_CALIBRATION, self._cal_value)

        self.bus_voltage_range = BusVoltageRange.RANGE_32V
        self.gain = Gain.DIV_8_320MV # Wider range for shunt voltage
        self.bus_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.shunt_adc_resolution = ADCResolution.ADCRES_12BIT_32S
        self.mode = Mode.SANDBVOLT_CONTINUOUS
        config = self.bus_voltage_range << 13 | self.gain << 11 | self.bus_adc_resolution << 7 | self.shunt_adc_resolution << 3 | self.mode
        self.write(_REG_CONFIG, config)
        logging.info("INA219 calibrated for 32V 2A range.")

    def getShuntVoltage_mV(self):
        value = self.read(_REG_SHUNTVOLTAGE)
        # Convert to signed value
        if value > 32767: value -= 65536
        # LSB is 10uV, so multiply by 0.01 to get mV
        return value * 0.01

    def getBusVoltage_V(self):
        self.read(_REG_BUSVOLTAGE) # First read might be stale? Common practice.
        raw_val = self.read(_REG_BUSVOLTAGE)
        # Shift right 3 bits (ignore status bits CNVR, OVF)
        shifted_val = raw_val >> 3
        # LSB is 4mV, multiply by 0.004 to get Volts
        return shifted_val * 0.004

    def getCurrent_mA(self):
        # Sometimes a read prior to reading current causes a delay/update.
        # self.write(_REG_CALIBRATION, self._cal_value) # Optional: force re-calc? Might slow things down.
        value = self.read(_REG_CURRENT)
        # Convert to signed value
        if value > 32767: value -= 65536
        # Multiply by the calculated current LSB (which is in Amps) and convert to mA
        return value * self._current_lsb * 1000

    def getPower_W(self):
        # self.write(_REG_CALIBRATION, self._cal_value) # Optional: force re-calc?
        value = self.read(_REG_POWER)
        # Convert to signed value (though power is usually positive)
        if value > 32767: value -= 65536
        # Multiply by the calculated power LSB (which is in Watts)
        return value * self._power_lsb
# ===========================================================
# === BATTERY MONITOR (INA219) CODE END ===
# ===========================================================


# --- Setup GPIO ---
def setup_gpio():
    global switch, last_error
    logging.info(f"Setting up GPIO pin {SWITCH_GPIO_PIN} using gpiozero Button")
    try:
        # Try using NativeFactory for potentially better performance/lower CPU
        # Fallback to default if it fails
        try:
            Device.pin_factory = NativeFactory()
            logging.info("Using gpiozero NativeFactory")
        except Exception as e:
            logging.warning(f"Could not set NativeFactory for gpiozero, using default: {e}")
            # Let gpiozero use its default factory

        # pull_up=True assumes the switch connects the GPIO pin to GND when pressed
        switch = Button(SWITCH_GPIO_PIN, pull_up=True, bounce_time=SWITCH_BOUNCE_TIME)
        logging.info(f"gpiozero Button on pin {SWITCH_GPIO_PIN} setup complete.")
        return True
    except Exception as e:
        logging.error(f"!!! Failed to setup gpiozero Button: {e}", exc_info=True)
        last_error = f"GPIO Setup Error: {e}"
        switch = None # Ensure switch is None if setup fails
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
        # Reset pin factory if needed (though usually not necessary on script exit)
        # Device.pin_factory = None
    except Exception as e:
        logging.warning(f"Error during gpiozero cleanup: {e}")

# --- Setup Battery Monitor ---
def setup_battery_monitor():
    global ina219_sensor, last_error
    logging.info("Setting up Battery Monitor (INA219)...")
    try:
        ina219_sensor = INA219(addr=INA219_I2C_ADDRESS)
        # Perform an initial read to check communication
        voltage = ina219_sensor.getBusVoltage_V()
        logging.info(f"INA219 Sensor setup complete. Initial voltage reading: {voltage:.2f}V")
        read_battery_level() # Try initial percentage read
        return True
    except Exception as e:
        # Log the error but allow the program to continue without battery monitoring
        logging.error(f"!!! Failed to setup INA219 Battery Monitor: {e}", exc_info=False) # Keep log less verbose
        last_error = f"Battery Monitor Setup Error: {e}"
        ina219_sensor = None
        return False

# --- Read Battery Level ---
def read_battery_level():
    global battery_percentage, ina219_sensor, last_error, config_lock
    if ina219_sensor is None:
        # logging.debug("Battery sensor not available, skipping read.")
        return # Sensor wasn't initialized or failed

    try:
        # Read the bus voltage (the battery voltage)
        bus_voltage = ina219_sensor.getBusVoltage_V()
        # Calculate percentage based on configured min/max
        voltage_range = BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE
        if voltage_range <= 0:
             logging.warning("Battery min/max voltages invalid (max <= min). Cannot calculate percentage.")
             percent = None
        else:
            # Calculate percentage
            percent = ((bus_voltage - BATTERY_MIN_VOLTAGE) / voltage_range) * 100.0
            # Clamp the value between 0 and 100
            percent = max(0.0, min(100.0, percent))

        # Safely update the global variable
        with config_lock:
            battery_percentage = percent
        # logging.debug(f"Battery level read: {bus_voltage:.2f}V -> {percent:.1f}%" if percent is not None else f"Battery level read: {bus_voltage:.2f}V (Invalid Range)")

    except OSError as e: # Catch I2C communication errors specifically
        # Log this error, could indicate a loose wire or sensor issue
        # Consider logging less frequently if this error repeats often?
        # current_batt_state = None
        # with config_lock: current_batt_state = battery_percentage
        # if current_batt_state is not None: # Only log error if we previously had a reading
        logging.error(f"!!! I2C Error reading INA219 sensor: {e}")
        with config_lock:
            battery_percentage = None # Set to None on error
    except Exception as e:
        logging.error(f"!!! Error reading battery level: {e}", exc_info=True)
        with config_lock:
            battery_percentage = None # Set to None on other errors too

# --- Get Current Resolution ---
def get_current_resolution():
    global current_resolution_index, config_lock
    with config_lock:
        # Ensure index is valid, fallback to default if not
        if 0 <= current_resolution_index < len(SUPPORTED_RESOLUTIONS):
            return SUPPORTED_RESOLUTIONS[current_resolution_index]
        else:
            logging.warning(f"Invalid resolution index {current_resolution_index} detected, falling back to default.")
            # Ensure default index itself is valid before using
            safe_default_index = max(0, min(len(SUPPORTED_RESOLUTIONS) - 1, DEFAULT_RESOLUTION_INDEX))
            return SUPPORTED_RESOLUTIONS[safe_default_index]

# --- Modified initialize_camera for Picamera2 with Tuning ---
def initialize_camera(target_width, target_height):
    global picam2, last_error
    logging.info(f"Attempting to initialize camera with Picamera2 at {target_width}x{target_height}...")

    # --- Stop/Close existing instance ---
    if picam2 is not None:
        try:
            if picam2.started:
                logging.info("Stopping existing Picamera2 instance...")
                picam2.stop()
            logging.info("Closing existing Picamera2 instance...")
            picam2.close()
            logging.info("Previous Picamera2 instance stopped and closed.")
        except Exception as e:
            logging.warning(f"Error stopping/closing previous Picamera2 instance: {e}")
        finally:
            picam2 = None # Ensure it's None even if closing failed
        time.sleep(0.5) # Give hardware time to release

    try:
        # --- Create Picamera2 instance with NoIR Tuning (if enabled) --- # <--- MODIFIED SECTION
        tuning = None
        if USE_NOIR_TUNING:
            if os.path.exists(NOIR_TUNING_FILE_PATH):
                try:
                    # Load the tuning file using the Picamera2 helper
                    tuning = Picamera2.load_tuning_file(NOIR_TUNING_FILE_PATH)
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
        # Create a configuration. Use RGB888 for OpenCV compatibility.
        # Controls are applied via this config.
        # Set AWB back to Auto initially when using specific tuning,
        # as the tuning file heavily influences colour/monochrome output.
        config = picam2.create_video_configuration(
            main={"size": (target_width, target_height), "format": "RGB888"}, # Use RGB for OpenCV
            controls={
                "FrameRate": float(FRAME_RATE), # Set desired frame rate
                "AwbEnable": True, # Auto White Balance enabled
                "AwbMode": controls.AwbModeEnum.Auto, # Let AWB run automatically initially
                # Noise reduction - try 'Fast' or 'HighQuality'. 'Off' is also an option.
                # Note: Certain modes might not be available on all sensors or firmwares.
                # If you get errors, try commenting this out.
                # "NoiseReductionMode": controls.NoiseReductionModeEnum.Fast,
                "Brightness": 0.0,   # Default brightness (range typically -1.0 to 1.0)
                "Contrast": 1.0,     # Default contrast (range typically 0.0 to 2.0)
                "Saturation": 1.0,   # Default saturation (0.0 is B&W, 1.0 normal, >1.0 boosted)
                # AE (Auto Exposure) can also be controlled if needed:
                # "AeEnable": True,
                # "AeConstraintMode": controls.AeConstraintModeEnum.Normal,
                # "ExposureTime": 10000, # Example: Set specific exposure time (in microseconds) - disable AE first
            }
        )

        # If using NoIR tuning, saturation might be forced low by the tuning file.
        # If you specifically want color even with the NoIR tuning *applied* (which affects AWB/CCM),
        # you might need to override saturation here AFTER creating the config.
        # However, usually, if you load a NoIR tune, you expect monochrome IR.
        # If you want *color* with a NoIR camera, you typically DON'T load the noir tuning file.
        # if USE_NOIR_TUNING and config.get('controls', {}).get('Saturation', 1.0) == 0.0:
        #    logging.warning("NoIR tuning likely set Saturation to 0. Forcing back to 1.0 if color is desired despite tuning.")
        #    config['controls']['Saturation'] = 1.0 # Example: Force color if needed

        logging.info(f"Configuring Picamera2 with: {config}")
        picam2.configure(config)
        logging.info("Picamera2 configuration successful!")

        # Start the camera stream
        picam2.start()
        logging.info("Camera started")
        # Allow some time for sensor initialisation and AWB/AE to settle
        time.sleep(2.0)

        # Log the actual configuration applied (might differ slightly from requested)
        actual_config = picam2.camera_configuration()
        if not actual_config:
            # This shouldn't happen if start() succeeded, but check anyway
             raise RuntimeError("Failed to get camera configuration after start.")

        # Extract relevant details from the complex config dictionary
        actual_format = actual_config.get('main', {})
        actual_w = actual_format.get('size', (0,0))[0]
        actual_h = actual_format.get('size', (0,0))[1]
        actual_fmt_str = actual_format.get('format', 'Unknown')
        logging.info(f"Picamera2 initialized. Actual main stream config: {actual_w}x{actual_h}, Format: {actual_fmt_str}")

        last_error = None # Clear previous errors on successful init
        return True

    except Exception as e:
        logging.error(f"!!! Failed to initialize Picamera2 (possibly tuning related) at {target_width}x{target_height}: {e}", exc_info=True)
        last_error = f"Picamera2 Init Error ({target_width}x{target_height}): {e}"
        # Ensure camera is cleaned up if init failed mid-way
        if picam2 is not None:
            try:
                if picam2.started: picam2.stop()
                picam2.close()
            except Exception as close_e:
                logging.error(f"Error closing picam2 after init failure: {close_e}")
        picam2 = None # Ensure picam2 is None after failure
        return False


# --- USB Mounts and Recording ---
def get_usb_mounts():
    """Finds writable directories under the USB_BASE_PATH."""
    mounts = []
    logging.debug(f"Checking for USB mounts under: {USB_BASE_PATH}")
    if not os.path.isdir(USB_BASE_PATH):
        logging.warning(f"USB base path '{USB_BASE_PATH}' does not exist or is not a directory.")
        return mounts # Return empty list

    try:
        for item in os.listdir(USB_BASE_PATH):
            path = os.path.join(USB_BASE_PATH, item)
            # Check if it's a directory and if we have write permissions
            if os.path.isdir(path) and os.access(path, os.W_OK):
                # Perform a quick write test to be more certain
                test_file = os.path.join(path, f".write_test_{os.getpid()}") # Unique test file name
                try:
                    with open(test_file, 'w') as f:
                        f.write('test')
                    os.remove(test_file) # Clean up test file
                    mounts.append(path)
                    logging.debug(f"Found writable mount: {path}")
                except Exception as write_err:
                     # Log if the directory appeared writable but the test failed
                     logging.warning(f"Directory {path} appears mounted but test write failed: {write_err}")
            elif os.path.isdir(path):
                # Log directories found but not writable (might be useful for debugging permissions)
                 logging.debug(f"Directory {path} found but is not writable.")

        if not mounts:
            logging.debug("No writable USB mounts found.") # Log if the loop finishes with no mounts
    except Exception as e:
        logging.error(f"Error finding USB mounts in {USB_BASE_PATH}: {e}")

    return mounts

def start_recording():
    global is_recording, video_writers, recording_paths, last_error, picam2
    if is_recording:
        logging.warning("Start recording called, but already recording.")
        return True # Already recording, consider it a success

    logging.info("Attempting to start recording...")
    usb_drives = get_usb_mounts()
    if not usb_drives:
        logging.warning(f"Cannot start recording: No writable USB drives found in {USB_BASE_PATH}.")
        last_error = f"Cannot start recording: No writable USB drives found"
        return False

    # Clear previous writers/paths just in case
    video_writers.clear()
    recording_paths.clear()
    success_count = 0
    start_error = None # Store the first error encountered

    if picam2 is None or not picam2.started:
        logging.error("Cannot start recording, camera is not available.")
        last_error = "Camera not available for recording."
        return False

    try:
        # Get actual resolution and FPS from the running camera config
        cam_config = picam2.camera_configuration().get('main', {})
        width = cam_config.get('size', (0,0))[0]
        height = cam_config.get('size', (0,0))[1]
        # Use the globally configured FRAME_RATE, but cap if resolution is very high
        fps = float(FRAME_RATE)
        # Example: Cap FPS for max resolution recording if it struggles
        if width >= 3280 and fps > 15: # If using max res and target FPS is high
            logging.warning(f"High resolution ({width}x{height}) detected. Capping recording FPS to 15 to reduce load.")
            fps = 15.0

        # Basic validation
        if width <= 0 or height <= 0:
            raise ValueError(f"Invalid camera dimensions obtained: {width}x{height}")

        logging.info(f"Starting recording with dimensions: {width}x{height} @ {fps:.1f}fps")

        # Define the codec (FourCC)
        fourcc = cv2.VideoWriter_fourcc(*RECORDING_FORMAT) # e.g., 'm', 'p', '4', 'v'
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        # Create a VideoWriter for each detected USB drive
        for drive_path in usb_drives:
            try:
                filename = f"recording_{timestamp}_{width}x{height}{RECORDING_EXTENSION}"
                full_path = os.path.join(drive_path, filename)

                # Create the VideoWriter object
                writer = cv2.VideoWriter(full_path, fourcc, fps, (width, height))

                if not writer.isOpened():
                    # Raise an error if the writer couldn't be opened
                    raise IOError(f"Failed to open VideoWriter for path: {full_path}")

                # Add the successful writer and path to our lists
                video_writers.append(writer)
                recording_paths.append(full_path)
                logging.info(f"Successfully started recording to: {full_path}")
                success_count += 1

            except Exception as e:
                logging.error(f"!!! Failed to create VideoWriter for drive {drive_path}: {e}", exc_info=True)
                if start_error is None: # Keep only the first error message for brevity
                    start_error = f"Failed writer {os.path.basename(drive_path)}: {e}"

        # After trying all drives:
        if success_count > 0:
            is_recording = True
            logging.info(f"Recording successfully started on {success_count} drive(s).")
            # If some failed, update last_error, otherwise clear it
            if start_error and success_count < len(usb_drives):
                 last_error = f"Partial Rec Start Fail: {start_error}"
            else:
                 last_error = None # Clear errors if fully successful
            return True
        else:
            # No writers were successfully created
            is_recording = False
            logging.error("Failed to start recording on ANY USB drive.")
            last_error = f"Recording Start Failed: {start_error or 'No writers opened'}"
            # Clean up just in case something was partially added
            video_writers.clear()
            recording_paths.clear()
            return False

    except Exception as e:
        # Catch unexpected errors during setup (e.g., getting config, invalid dimensions)
        logging.error(f"!!! Critical error during recording setup: {e}", exc_info=True)
        last_error = f"Recording Setup Error: {e}"
        stop_recording() # Attempt cleanup
        return False

# --- MODIFIED stop_recording Function ---
def stop_recording():
    """Stops recording to all USB drives and forces filesystem sync."""
    global is_recording, video_writers, recording_paths, last_error
    if not is_recording:
        # Clear lists if state is inconsistent, maybe log a warning
        if video_writers or recording_paths:
            logging.warning("stop_recording called but not recording; clearing potentially stale writer lists.")
            video_writers.clear(); recording_paths.clear()
        return

    logging.info("Stopping recording...")
    is_recording = False # Set state first to prevent new writes
    released_count = 0
    # Make copies to iterate over, as the global lists will be cleared
    writers_to_release = list(video_writers)
    paths_recorded = list(recording_paths)
    video_writers.clear(); recording_paths.clear() # Clear global lists immediately

    logging.info(f"Releasing {len(writers_to_release)} video writer(s)...")
    for i, writer in enumerate(writers_to_release):
        # Get path safely, provide default if index mismatch (shouldn't happen)
        path = paths_recorded[i] if i < len(paths_recorded) else f"Unknown Path (Writer Index {i})"
        try:
            writer.release() # Tell OpenCV to finalize and close the file
            logging.info(f"Successfully released writer for: {path}")
            released_count += 1
        except Exception as e:
            # Log error but continue trying to release others
            logging.error(f"Error releasing VideoWriter for {path}: {e}", exc_info=True)

    # --- CRITICAL ADDITION: Force OS to write buffers to disk ---
    if released_count > 0: # Only sync if we actually released files
        logging.info("Syncing filesystem to ensure data is written to USB drives...")
        try:
            sync_start_time = time.monotonic()
            # Execute the 'sync' command (requires 'os' module)
            os.system('sync')
            sync_duration = time.monotonic() - sync_start_time
            logging.info(f"Filesystem sync completed in {sync_duration:.2f} seconds.")
            # OPTIONAL: Add a very short sleep AFTER sync for extra safety margin on slow drives
            # time.sleep(0.5)
            # logging.info("Short delay after sync complete.")
        except Exception as e:
            # This might happen if os.system isn't allowed, though unlikely for 'sync'
            logging.error(f"!!! Failed to execute 'sync' command: {e}", exc_info=True)
            # Optionally set last_error here if sync failure is considered critical
            last_error = "Filesystem sync failed after recording."
    else:
        # Log if sync was skipped because no writers were released (e.g., start failed partially)
        logging.warning("No video writers were successfully released, skipping filesystem sync.")

    # Final log message
    logging.info(f"Recording fully stopped. Released {released_count} writer(s) out of {len(writers_to_release)} attempted.")
    # No need to clear lists again, already done earlier

# --- Capture Loop ---
def capture_and_process_loop():
    global output_frame, is_recording, last_error, picam2, switch, digital_recording_active
    global current_resolution_index, reconfigure_resolution_index, last_battery_read_time
    global video_writers, recording_paths # Make sure these are accessible if needed

    logging.info("Starting frame capture loop...")
    consecutive_error_count = 0
    max_consecutive_errors = 15 # Threshold to decide if the loop is failing badly

    # --- Initial Camera Setup ---
    # Get the starting resolution based on the default index
    width, height = get_current_resolution()
    if not initialize_camera(width, height):
        logging.error("Initial camera setup failed. Capture thread cannot start.")
        # last_error should have been set by initialize_camera
        return # Exit the thread if camera doesn't start

    while not shutdown_event.is_set():
        loop_start_time = time.monotonic() # For timing and battery check interval

        try:
            # --- Reconfiguration Check ---
            target_index = -1 # Flag value
            # Check if a reconfiguration was requested (thread-safe check)
            if reconfigure_resolution_index is not None:
                with config_lock:
                    if reconfigure_resolution_index is not None: # Double check inside lock
                        target_index = reconfigure_resolution_index
                        reconfigure_resolution_index = None # Consume the request

            # --- Handle Reconfiguration ---
            if target_index != -1:
                logging.info(f"--- Reconfiguring resolution to index {target_index} ---")
                # Check if recording should be active based on current triggers
                physical_switch_on_before = (switch is not None and switch.is_pressed)
                digital_switch_on_before = digital_recording_active # Read directly as we are in the loop managing it
                should_be_recording_after = physical_switch_on_before or digital_switch_on_before
                was_actually_recording = is_recording # State before we stop

                # Stop recording if it was active
                if was_actually_recording:
                    logging.info("Stopping recording for reconfiguration...")
                    stop_recording() # This now includes sync

                # Attempt to initialize with the new resolution
                new_width, new_height = SUPPORTED_RESOLUTIONS[target_index]
                if initialize_camera(new_width, new_height):
                    # Update the current index only on success
                    with config_lock:
                        current_resolution_index = target_index
                    logging.info(f"--- Reconfiguration successful to {new_width}x{new_height} ---")
                    # Restart recording if it was supposed to be active
                    if should_be_recording_after:
                        logging.info("Resuming recording after successful reconfiguration...")
                        time.sleep(1.0) # Short pause before restarting
                        if not start_recording():
                            logging.error("Failed to restart recording after reconfiguration!")
                            # last_error should be set by start_recording
                else: # Reconfiguration failed
                    logging.error(f"!!! Failed reconfigure to index {target_index}. Attempting to restore previous resolution... !!!")
                    # last_error set by initialize_camera failure
                    # Try to go back to the resolution we *know* worked (the one before this attempt)
                    prev_width, prev_height = get_current_resolution() # Get the *unchanged* resolution index
                    if not initialize_camera(prev_width, prev_height):
                        # This is critical - failed to reconfig AND failed to restore
                        logging.critical("!!! Failed to restore previous camera resolution after failed reconfig. Stopping service. !!!")
                        last_error = "Camera failed fatally during reconfig restore."
                        shutdown_event.set(); break # Exit loop and trigger main shutdown
                    else:
                        # Restored previous resolution successfully
                        logging.info("Successfully restored previous camera resolution.")
                        if should_be_recording_after:
                            logging.info("Attempting recording restart with restored resolution...")
                            time.sleep(1.0)
                            if not start_recording():
                                logging.error("Failed to restart recording after failed reconfig and restore.")
                                # last_error set by start_recording

                # Skip the rest of this loop iteration after handling reconfiguration
                continue


            # --- Camera Status Check ---
            if picam2 is None or not picam2.started:
                # Camera has stopped unexpectedly
                if not last_error: last_error = "Picamera2 became unavailable."
                logging.error(f"Camera unavailable: {last_error}. Attempting reinitialization...")
                width, height = get_current_resolution() # Use current known good resolution
                if initialize_camera(width, height):
                     # Re-initialized successfully
                     logging.info("Camera re-initialized successfully after unexpected stop.")
                     last_error = None; consecutive_error_count = 0 # Reset error state
                else:
                    # Re-initialization failed, this might be fatal
                    logging.error(f"Camera re-initialization failed: {last_error}. Stopping capture loop.")
                    shutdown_event.set(); break # Exit loop

                # Skip the rest of this loop iteration after attempting recovery
                continue


            # --- Read Frame ---
            # capture_array should return a NumPy array in the configured format (RGB888)
            frame_bgr = picam2.capture_array("main")

            if frame_bgr is None:
                # This indicates a problem with capturing
                logging.warning("Failed to capture frame from Picamera2. Retrying...")
                consecutive_error_count += 1
                if consecutive_error_count > max_consecutive_errors:
                    last_error = f"Failed capture {max_consecutive_errors} consecutive times. Assuming camera failure."
                    logging.error(last_error); shutdown_event.set(); break # Exit loop
                time.sleep(0.1); continue # Short pause and try again
            # Reset error count if capture succeeds after failures
            if consecutive_error_count > 0:
                logging.info(f"Recovered frame grab after {consecutive_error_count} errors.")
            consecutive_error_count = 0 # Reset error counter on success


            # --- Handle Recording State (Combined Switch + Digital) ---
            physical_switch_on = False
            if switch is not None:
                try:
                    # is_pressed is True if the button is pressed (connected to GND if pull_up=True)
                    physical_switch_on = switch.is_pressed
                except Exception as e:
                    # Log error reading switch but don't stop everything
                    logging.error(f"Error reading switch state: {e}")
                    last_error = f"Switch Read Error: {e}"
                    # Decide on fallback behavior - assume off? Or keep last state? Assume off for safety.
                    physical_switch_on = False

            # Read digital state (set by Flask) safely
            with config_lock:
                digital_switch_on = digital_recording_active

            # Determine desired recording state
            should_be_recording = physical_switch_on or digital_switch_on

            # Compare desired state with actual state and act
            if should_be_recording and not is_recording:
                # --- Start Recording Trigger ---
                log_msg = "Physical switch ON" if physical_switch_on else ""
                if digital_switch_on: log_msg += (" / " if log_msg else "") + "Digital trigger ON"
                logging.info(f"Recording trigger active ({log_msg}) - initiating start.")
                if not start_recording():
                    # start_recording() logs errors and sets last_error
                    logging.error("Attempt to start recording failed.")
                    # Optional: If start fails repeatedly, maybe disable trigger?
            elif not should_be_recording and is_recording:
                # --- Stop Recording Trigger ---
                logging.info("Recording trigger(s) OFF - initiating stop.")
                stop_recording() # This handles release and sync


            # --- Write Frame if Recording ---
            if is_recording:
                # Check if we have active writers (robustness check)
                if not video_writers:
                    logging.warning("Inconsistent state: is_recording=True, but no video writers. Forcing stop.")
                    is_recording = False # Correct the state flag
                    last_error = "Rec stopped: writer list was empty."
                else:
                    write_errors = 0
                    # Use a copy of the list in case it's modified elsewhere (though unlikely here)
                    current_writers = list(video_writers)
                    current_paths = list(recording_paths) # For error reporting
                    for i, writer in enumerate(current_writers):
                        try:
                            writer.write(frame_bgr) # Write the frame
                        except Exception as e:
                             # Get the path corresponding to the failing writer
                             path_str = current_paths[i] if i < len(current_paths) else f"Writer {i}"
                             logging.error(f"!!! Failed to write frame to {path_str}: {e}")
                             # Consider removing the failed writer? Or just log? For now, just log.
                             write_errors += 1
                             # Set last_error for UI feedback
                             last_error = f"Frame write error: {os.path.basename(path_str)}"

                    # If *all* writers failed, stop recording to prevent spamming errors
                    if write_errors > 0 and write_errors == len(current_writers):
                        logging.error("All active video writers failed to write frame. Stopping recording.")
                        last_error = "Rec stopped: All writers failed write."
                        stop_recording() # Stop recording


            # --- Update Shared Frame for Streaming ---
            # Make a copy for the streaming thread to avoid race conditions
            with frame_lock:
                output_frame = frame_bgr.copy()


            # --- Read Battery Level Periodically ---
            # Check if sensor exists and enough time has passed
            if ina219_sensor and (loop_start_time - last_battery_read_time > BATTERY_READ_INTERVAL):
                 # logging.debug("Reading battery level...")
                 read_battery_level() # Reads sensor and updates global battery_percentage
                 last_battery_read_time = loop_start_time # Update last read time


        except Exception as e:
            # Catch-all for unexpected errors within the loop
            logging.exception(f"!!! Unexpected Error in capture loop: {e}") # Use exception to log traceback
            last_error = f"Capture Loop Error: {e}"
            consecutive_error_count += 1
            # If errors persist, maybe the whole system is unstable
            if consecutive_error_count > max_consecutive_errors / 2: # Lower threshold for general errors
                logging.error(f"Too many consecutive errors ({consecutive_error_count}). Signaling shutdown.")
                shutdown_event.set() # Trigger main shutdown
            time.sleep(1) # Pause briefly after an unexpected error

    # --- Cleanup after loop exit (due to shutdown_event) ---
    logging.info("Exiting frame capture thread.")
    # Ensure recording is stopped cleanly
    if is_recording:
        logging.info("Capture loop exiting: Stopping active recording...")
        stop_recording()
    # Release camera resources
    if picam2:
        try:
            logging.info("Capture loop exiting: Stopping and closing Picamera2...")
            if picam2.started: picam2.stop()
            picam2.close()
            logging.info("Picamera2 released by capture thread.")
        except Exception as e:
            logging.error(f"Error stopping/closing Picamera2 in thread cleanup: {e}")
    picam2 = None # Ensure it's marked as None


# ===========================================================
# === FLASK ROUTES START HERE ===
# ===========================================================
def generate_stream_frames():
    """Generates JPEG frames for the MJPEG stream."""
    global output_frame, frame_lock, shutdown_event
    frame_counter = 0
    last_frame_time = time.monotonic()
    logging.info("MJPEG stream client connected. Starting frame generation.")

    while not shutdown_event.is_set():
        frame_to_encode = None
        # Safely get the latest frame
        with frame_lock:
            if output_frame is not None:
                frame_to_encode = output_frame.copy() # Copy to avoid holding lock during encoding

        if frame_to_encode is None:
            # No frame available yet, wait briefly
            time.sleep(0.05) # 50ms
            continue

        try:
            # Encode the frame as JPEG
            # Quality 85 is a reasonable balance between quality and size
            (flag, encodedImage) = cv2.imencode(".jpg", frame_to_encode, [cv2.IMWRITE_JPEG_QUALITY, 85])

            if not flag:
                logging.warning("Stream generator: Could not encode frame to JPEG.")
                time.sleep(0.1) # Wait a bit longer if encoding fails
                continue

            # Yield the frame in MJPEG format
            yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
                   bytearray(encodedImage) + b'\r\n')

            frame_counter += 1
            # --- Frame Rate Control for Stream ---
            # Try to maintain a reasonable streaming rate, but don't sleep excessively
            current_time = time.monotonic()
            elapsed = current_time - last_frame_time
            # Aim slightly faster than target rate to buffer a bit, but have a minimum sleep
            target_delay = 1.0 / (FRAME_RATE + 5) # Target delay based on config + margin
            sleep_time = max(0.01, target_delay - elapsed) # Min sleep 10ms
            time.sleep(sleep_time)
            last_frame_time = time.monotonic() # Update last frame time after sleep

        except GeneratorExit:
            # This happens when the client disconnects
            logging.info(f"Streaming client disconnected after {frame_counter} frames.")
            break # Exit the loop cleanly
        except Exception as e:
            # Log other errors (e.g., network issues, encoding problems)
            logging.exception(f"!!! Error in MJPEG streaming generator: {e}")
            time.sleep(0.5) # Pause after an error before retrying

    logging.info("Stream generator thread exiting.")

@app.route("/")
def index():
    """Serves the main HTML page."""
    global last_error, digital_recording_active, battery_percentage, config_lock
    current_w, current_h = get_current_resolution()
    resolution_text = f"{current_w}x{current_h}"
    # Get initial state safely
    err_msg = last_error if last_error else ""
    with config_lock:
        digital_rec_state_initial = digital_recording_active
        batt_perc_initial = battery_percentage
    # Format battery percentage for display, handle None case
    batt_text_initial = f"{batt_perc_initial:.1f}" if batt_perc_initial is not None else "--"

    # Render the HTML template string
    # Using f-string or .format() could make this slightly cleaner, but render_template_string works
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
          h1 { text-align: center; color: #333; margin-bottom: 20px; }
          .status-grid { display: grid; grid-template-columns: auto 1fr; gap: 5px 15px; margin-bottom: 15px; align-items: center; background-color: #eef; padding: 10px; border-radius: 5px;}
          .status-grid span:first-child { font-weight: bold; color: #555; text-align: right;}
          #status, #rec-status, #resolution, #battery-level { color: #0056b3; font-weight: normal;}
          #rec-status.active { color: #D83B01; font-weight: bold;}
          .controls { text-align: center; margin-bottom: 15px; }
          .controls button { padding: 10px 20px; margin: 5px; font-size: 1em; cursor: pointer; border-radius: 5px; border: 1px solid #ccc; background-color: #e9e9e9; transition: background-color 0.2s, border-color 0.2s;}
          .controls button:hover:not(:disabled) { background-color: #dcdcdc; border-color: #bbb;}
          #error { color: red; margin-top: 10px; white-space: pre-wrap; font-weight: bold; min-height: 1.2em; text-align: center; background-color: #ffebeb; border: 1px solid red; padding: 8px; border-radius: 4px; display: none; /* Initially hidden */}
          img#stream { display: block; margin: 15px auto; border: 1px solid black; max-width: 100%; height: auto; background-color: #ddd; } /* Placeholder color */
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
            <span>Battery:</span> <span id="battery-level">{{ batt_text_initial }}%</span>
          </div>
          <div class="controls">
            <button onclick="changeResolution('down')" id="btn-down" title="Decrease resolution">&laquo; Lower Res</button>
            <button onclick="toggleRecording()" id="btn-record" class="recording-inactive" title="Toggle recording via web interface">Start Rec (Web)</button>
            <button onclick="changeResolution('up')" id="btn-up" title="Increase resolution">Higher Res &raquo;</button>
          </div>
          <div id="error" {% if err_msg %}style="display: block;"{% endif %}>{{ err_msg }}</div> <img id="stream" src="{{ url_for('video_feed') }}" width="{{ current_w }}" height="{{ current_h }}" alt="Loading stream..."
               onerror="handleStreamError()" onload="handleStreamLoad()"> </div>

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
          // Initialize state from template variables passed by Flask
          let currentDigitalRecordState = {{ 'true' if digital_rec_state_initial else 'false' }};
          let statusUpdateInterval;
          let streamErrorTimeout = null;

          // Function to update the Record button text and style
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

          // Function to fetch and update status information
          function updateStatus() {
              // Avoid fetching status if an action is in progress
              if (isChangingResolution || isTogglingRecording) return;

              fetch('/status')
                  .then(response => {
                      if (!response.ok) {
                          throw new Error(`HTTP error! Status: ${response.status}`);
                      }
                      return response.json();
                  })
                  .then(data => {
                      statusElement.textContent = data.status_text || 'Unknown';
                      recStatusElement.textContent = data.is_recording ? "ACTIVE" : "OFF";
                      recStatusElement.classList.toggle('active', data.is_recording); // Add/remove class

                      // Update resolution display and image size if changed
                      if (data.resolution && resolutionElement.textContent !== data.resolution) {
                          resolutionElement.textContent = data.resolution;
                          const [w, h] = data.resolution.split('x');
                           if (streamImage.getAttribute('width') != w || streamImage.getAttribute('height') != h) {
                               console.log(`Updating stream image size attribute to ${w}x${h}`);
                               streamImage.setAttribute('width', w);
                               streamImage.setAttribute('height', h);
                           }
                      }

                      // Update error message display
                      if (data.error) {
                          errorElement.textContent = data.error;
                          errorElement.style.display = 'block';
                      } else {
                          // Clear error only if it was previously shown
                          if (errorElement.style.display !== 'none') {
                              errorElement.textContent = '';
                              errorElement.style.display = 'none';
                          }
                      }

                      // Update digital recording button state if changed
                      if (typeof data.digital_recording_active === 'boolean' && currentDigitalRecordState !== data.digital_recording_active) {
                           currentDigitalRecordState = data.digital_recording_active;
                           updateRecordButtonState();
                      }

                      // Update battery level display
                      if (data.battery_percent !== null && data.battery_percent !== undefined) {
                          batteryLevelElement.textContent = data.battery_percent.toFixed(1);
                      } else {
                          batteryLevelElement.textContent = "--"; // Placeholder if no reading
                      }
                  })
                  .catch(err => {
                      console.error("Error fetching status:", err);
                      statusElement.textContent = "Error fetching status";
                      errorElement.textContent = `Failed to fetch status: ${err.message}. Check server connection.`;
                      errorElement.style.display = 'block';
                      recStatusElement.textContent = "Unknown";
                      batteryLevelElement.textContent = "Err";
                      // Consider stopping status updates if fetch consistently fails?
                  });
          }

          // Functions to disable/enable control buttons during actions
          function disableControls() {
              btnUp.disabled = true;
              btnDown.disabled = true;
              btnRecord.disabled = true;
          }
          function enableControls() {
              btnUp.disabled = false;
              btnDown.disabled = false;
              btnRecord.disabled = false;
          }

          // Function to handle resolution change requests
          function changeResolution(direction) {
              if (isChangingResolution || isTogglingRecording) return; // Prevent overlapping actions
              isChangingResolution = true;
              disableControls();
              statusElement.textContent = 'Changing resolution... Please wait.';
              errorElement.textContent = ''; errorElement.style.display = 'none'; // Clear old errors

              fetch(`/set_resolution/${direction}`, { method: 'POST' })
                  .then(response => response.json().then(data => ({ status: response.status, body: data }))) // Parse JSON regardless of status
                  .then(({ status, body }) => {
                      if (status === 200 && body.success) {
                          statusElement.textContent = 'Resolution change initiated. Stream will update.';
                          // Update displayed resolution immediately for feedback
                          resolutionElement.textContent = body.new_resolution;
                          // Update image size attributes (browser might reload anyway)
                          const [w, h] = body.new_resolution.split('x');
                          streamImage.setAttribute('width', w);
                          streamImage.setAttribute('height', h);
                          console.log("Resolution change request sent, backend is processing...");
                          // Don't re-enable controls immediately, wait for status update or timeout
                      } else {
                          // Handle API errors or non-success responses
                          errorElement.textContent = `Error changing resolution: ${body.message || 'Unknown error from server.'}`;
                          errorElement.style.display = 'block';
                          statusElement.textContent = 'Resolution change failed.';
                          console.error("Resolution change failed:", body);
                          // Re-enable controls only on failure here
                          isChangingResolution = false;
                          enableControls();
                          updateStatus(); // Refresh status after failure
                      }
                  })
                  .catch(err => {
                      // Handle network errors
                      console.error("Network error sending resolution change:", err);
                      errorElement.textContent = `Network error changing resolution: ${err.message}`;
                      errorElement.style.display = 'block';
                      statusElement.textContent = 'Resolution change failed (Network).';
                      // Re-enable controls on network failure
                      isChangingResolution = false;
                      enableControls();
                      updateStatus(); // Refresh status
                  })
                  .finally(() => {
                      // Use a timeout to re-enable controls if the process takes too long or hangs
                      // This assumes the backend takes ~5-7 seconds max to reconfigure
                      if (isChangingResolution) {
                          setTimeout(() => {
                              if (isChangingResolution) { // Check again in case it finished quickly
                                  console.log("Re-enabling controls after resolution change timeout.");
                                  isChangingResolution = false;
                                  enableControls();
                                  updateStatus(); // Fetch latest status
                              }
                          }, 7000); // 7 second timeout
                      }
                  });
          }

          // Function to handle toggling digital recording
          function toggleRecording() {
              if (isChangingResolution || isTogglingRecording) return; // Prevent overlapping actions
              isTogglingRecording = true;
              disableControls();
              statusElement.textContent = 'Sending record command...';
              errorElement.textContent = ''; errorElement.style.display = 'none'; // Clear old errors

              fetch('/toggle_recording', { method: 'POST' })
                  .then(response => {
                      if (!response.ok) { throw new Error(`HTTP error! Status: ${response.status}`); }
                      return response.json();
                  })
                  .then(data => {
                      if (data.success) {
                          currentDigitalRecordState = data.digital_recording_active;
                          updateRecordButtonState(); // Update button immediately
                          statusElement.textContent = `Digital recording ${currentDigitalRecordState ? 'enabled' : 'disabled'}. State updating...`;
                          // Trigger a status update slightly later to catch backend state change
                          setTimeout(updateStatus, 1500);
                      } else {
                          // Handle API errors
                          errorElement.textContent = `Error toggling recording: ${data.message || 'Unknown server error.'}`;
                          errorElement.style.display = 'block';
                          statusElement.textContent = 'Record command failed.';
                          setTimeout(updateStatus, 1000); // Update status even on failure
                      }
                  })
                  .catch(err => {
                      // Handle network errors
                      console.error("Error toggling recording:", err);
                      errorElement.textContent = `Network error toggling recording: ${err.message}`;
                      errorElement.style.display = 'block';
                      statusElement.textContent = 'Record command failed (Network).';
                      setTimeout(updateStatus, 1000);
                  })
                  .finally(() => {
                      // Re-enable controls relatively quickly for recording toggle
                      isTogglingRecording = false;
                      enableControls();
                  });
          }

          // --- Stream Error Handling ---
          function handleStreamError() {
              console.warn("Stream image 'onerror' event triggered.");
              if (streamErrorTimeout) return; // Don't stack timeouts
              statusElement.textContent = 'Stream interrupted. Attempting reload...';
              // Set a timeout to try reloading the stream source
              streamErrorTimeout = setTimeout(() => {
                  console.log("Attempting stream reload by changing src...");
                  // Append timestamp to force browser to reload
                  streamImage.src = "{{ url_for('video_feed') }}?" + new Date().getTime();
                  streamErrorTimeout = null; // Clear timeout ID
                  // Schedule a status update shortly after attempting reload
                  setTimeout(updateStatus, 1000);
              }, 3000); // Wait 3 seconds before reloading
          }

          function handleStreamLoad() {
              // If the stream loads successfully (e.g., after an error/reload)
              if (streamErrorTimeout) {
                  console.log("Stream image 'onload' event triggered, clearing potential reload timeout.");
                  clearTimeout(streamErrorTimeout);
                  streamErrorTimeout = null;
                  statusElement.textContent = 'Stream active.'; // Update status
              }
          }


          // --- Initialization ---
          document.addEventListener('DOMContentLoaded', () => {
              updateRecordButtonState(); // Set initial button state
              updateStatus(); // Fetch initial status
              // Start periodic status updates
              statusUpdateInterval = setInterval(() => {
                  // Only update if no blocking actions are happening
                  if (!isChangingResolution && !isTogglingRecording) {
                      updateStatus();
                  }
              }, 5000); // Update every 5 seconds
          });

          // Optional: Clear interval on page unload
          window.addEventListener('beforeunload', () => {
              if (statusUpdateInterval) clearInterval(statusUpdateInterval);
          });

        </script>
      </body>
    </html>
    """, resolution_text=resolution_text, current_w=current_w, current_h=current_h,
         err_msg=err_msg, digital_rec_state_initial=digital_rec_state_initial, batt_text_initial=batt_text_initial)


@app.route("/video_feed")
def video_feed():
    """Endpoint for the MJPEG video stream."""
    logging.info("Client connected to video feed.")
    # Returns a response that streams the output of generate_stream_frames
    return Response(generate_stream_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/status")
def status():
    """Returns the current status of the system as JSON."""
    global last_error, digital_recording_active, is_recording, battery_percentage, config_lock
    global video_writers, recording_paths # Access needed for recording status details

    status_text = "Streaming"
    rec_stat_detail = ""
    current_w, current_h = get_current_resolution()
    batt_perc = None # Default to None
    current_digital_state = False # Default

    # Safely read shared state variables
    with config_lock:
        current_digital_state = digital_recording_active
        batt_perc = battery_percentage
        # Need to read is_recording and paths inside lock if they could be modified concurrently?
        # For status read, might be okay if slightly stale, but safer to lock if write occurs elsewhere.
        # Let's assume capture loop manages is_recording/paths and we read it here.
        # No, stop_recording clears them, so access needs care. Re-read is_recording.
        current_is_recording = is_recording
        current_recording_paths = list(recording_paths) # Copy paths for safe access

    # Determine status text based on recording state
    if current_is_recording:
        if current_recording_paths: # Check if paths list is not empty
             rec_stat_detail = f" (Recording to {len(current_recording_paths)} USB(s))"
        else:
             # This indicates an inconsistent state (should have been cleared by stop_recording)
             rec_stat_detail = " (ERROR: Recording active but no paths!)"
             logging.warning("Status check found is_recording=True but recording_paths is empty.")
             if not last_error: last_error = "Inconsistent State: Recording active but no paths."
        status_text += rec_stat_detail
    # else: status_text = "Streaming (Not Recording)" # Be more explicit?

    # Handle error message and auto-clearing non-critical ones
    err_msg = last_error if last_error else ""
    # Auto-clear certain errors if the related component seems functional now
    # Example: If GPIO setup failed but switch object exists now (somehow?)
    # if switch is not None and err_msg and "GPIO Setup Error" in err_msg:
    #     logging.info("Auto-clearing previous GPIO setup error as switch object exists.")
    #     last_error = None; err_msg = ""
    # Example: If camera init failed but we are getting frames
    if output_frame is not None and err_msg and ("Init Error" in err_msg or "unavailable" in err_msg or "capture" in err_msg):
         logging.info("Auto-clearing previous camera/capture error as frames are being received.")
         last_error = None; err_msg = ""
    # Example: If battery monitor failed but we got a reading now
    if batt_perc is not None and err_msg and ("Battery Monitor" in err_msg or "INA219" in err_msg or "I2C Error" in err_msg):
         logging.info("Auto-clearing previous battery monitor error as a reading was successful.")
         last_error = None; err_msg = ""

    # Return JSON response
    return jsonify({
        'is_recording': current_is_recording,
        'digital_recording_active': current_digital_state, # State of the web UI toggle
        'resolution': f"{current_w}x{current_h}",
        'status_text': status_text,
        'error': err_msg, # Current error message
        'active_recordings': current_recording_paths, # List of files being written
        'battery_percent': batt_perc # Battery percentage (or null)
    })

@app.route("/set_resolution/<direction>", methods=['POST'])
def set_resolution(direction):
    """Endpoint to request a change in camera resolution."""
    global current_resolution_index, reconfigure_resolution_index, last_error, config_lock

    with config_lock:
        # Prevent queuing multiple reconfigurations
        if reconfigure_resolution_index is not None:
            logging.warning("Resolution change request ignored: Reconfiguration already in progress.")
            return jsonify({'success': False, 'message': 'Reconfiguration already in progress.'}), 429 # Too Many Requests

        # Validate current state (should always be valid if init worked)
        if not (0 <= current_resolution_index < len(SUPPORTED_RESOLUTIONS)):
             logging.error(f"Internal Error: Invalid current resolution index {current_resolution_index} before change!")
             return jsonify({'success': False, 'message': 'Internal state error: Invalid current resolution.'}), 500

        original_index = current_resolution_index
        new_index = current_resolution_index

        # Calculate the new index based on direction
        if direction == 'up':
            new_index += 1
        elif direction == 'down':
            new_index -= 1
        else:
            logging.warning(f"Invalid direction '{direction}' received for resolution change.")
            return jsonify({'success': False, 'message': 'Invalid direction specified (must be up or down).'}), 400

        # Clamp the index within the valid range
        new_index = max(0, min(len(SUPPORTED_RESOLUTIONS) - 1, new_index))

        # Check if the resolution is actually changing
        if new_index == original_index:
            msg = 'Already at highest resolution.' if direction == 'up' else 'Already at lowest resolution.'
            logging.info(f"Resolution change request ignored: {msg}")
            return jsonify({'success': False, 'message': msg}), 400 # Bad Request (or maybe 200 OK with message?)

        # If changing, set the reconfiguration flag for the capture loop
        new_w, new_h = SUPPORTED_RESOLUTIONS[new_index]
        logging.info(f"Web request received: change resolution index {original_index} -> {new_index} ({new_w}x{new_h})")
        reconfigure_resolution_index = new_index # Signal the capture thread
        last_error = None # Clear previous errors on user action

        # Return success, indicating the request was accepted
        return jsonify({
            'success': True,
            'message': 'Resolution change requested. Please wait for the stream to update.',
            'new_resolution': f"{new_w}x{new_h}" # Provide immediate feedback
        })

@app.route('/toggle_recording', methods=['POST'])
def toggle_recording():
    """Endpoint to toggle the digital (web UI) recording trigger."""
    global digital_recording_active, last_error, config_lock
    new_state = False
    with config_lock:
        # Flip the boolean state
        digital_recording_active = not digital_recording_active
        new_state = digital_recording_active
        logging.info(f"Digital recording trigger toggled via web UI to: {'ON' if new_state else 'OFF'}")
        # Clear recording-related errors when user manually toggles
        if last_error and ("Recording" in last_error or "writers" in last_error or "USB" in last_error or "sync" in last_error):
             logging.info(f"Clearing previous recording-related error due to user toggle: '{last_error}'")
             last_error = None

    # Return the new state
    return jsonify({'success': True, 'digital_recording_active': new_state})

# ===========================================================
# === FLASK ROUTES END HERE ===
# ===========================================================

# --- Signal Handling ---
def signal_handler(sig, frame):
    """Handles termination signals (SIGINT, SIGTERM)."""
    global shutdown_event
    if shutdown_event.is_set(): # Prevent double handling
        logging.warning("Shutdown already in progress, ignoring additional signal.")
        return
    logging.warning(f"Received signal {sig}. Initiating graceful shutdown...")
    shutdown_event.set() # Signal all loops to stop

# --- Main Execution ---
def main():
    global last_error, capture_thread, flask_thread, picam2, shutdown_event

    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)  # Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler) # kill command

    logging.info(" --- Starting Camera Stream & Record Service --- ")
    logging.info(f"--- Using Picamera2 and gpiozero ---")
    logging.info(f"--- Configured for {FRAME_RATE} FPS, {len(SUPPORTED_RESOLUTIONS)} resolutions ---")
    logging.info(f"--- Recording to {USB_BASE_PATH} using {RECORDING_FORMAT}{RECORDING_EXTENSION} ---")
    logging.info(f"--- Web UI on port {WEB_PORT} ---")
    if USE_NOIR_TUNING: logging.info(f"--- Attempting to use NoIR Tuning: {NOIR_TUNING_FILE_PATH} ---")
    if INA219_I2C_ADDRESS: logging.info(f"--- Battery Monitor Enabled (INA219 @ 0x{INA219_I2C_ADDRESS:X}) ---")
    if SWITCH_GPIO_PIN: logging.info(f"--- Physical Record Switch Enabled (GPIO {SWITCH_GPIO_PIN}) ---")


    # Main loop with restart capability
    while not shutdown_event.is_set():
        # Reset state variables for this run attempt
        last_error = None
        capture_thread = None
        flask_thread = None
        if picam2: # Pre-loop cleanup if restarting
            try:
                if picam2.started: picam2.stop()
                picam2.close()
            except Exception: pass # Ignore errors during cleanup
            picam2 = None

        try:
            # --- Initialize Hardware ---
            logging.info("Initializing Hardware Components...")
            if SWITCH_GPIO_PIN:
                if not setup_gpio():
                    logging.error(f"GPIO setup failed: {last_error}. Physical switch will be unavailable.")
                    # Continue without switch functionality
            else:
                logging.info("Physical switch disabled (SWITCH_GPIO_PIN not set).")

            if INA219_I2C_ADDRESS:
                if not setup_battery_monitor():
                    logging.warning(f"Battery monitor setup failed: {last_error}. Battery level will be unavailable.")
                    # Continue without battery monitoring
            else:
                logging.info("Battery monitor disabled (INA219_I2C_ADDRESS not set).")


            # --- Start Capture Thread (includes initial camera setup) ---
            logging.info("Starting frame capture thread...")
            capture_thread = threading.Thread(target=capture_and_process_loop, name="CaptureThread", daemon=True)
            capture_thread.start()
            # Wait a bit for the thread to attempt camera initialization
            time.sleep(4.0) # Increased wait time

            # Check if capture thread started and camera initialized successfully
            if not capture_thread.is_alive():
                raise RuntimeError(f"Capture thread failed to start or died during initialization: {last_error or 'Unknown thread error'}")
            if last_error and ("Init Error" in last_error or "failed fatally" in last_error):
                # If init specifically failed, raise error to trigger restart logic
                raise RuntimeError(f"Camera initialization failed within capture thread: {last_error}")
            logging.info("Capture thread appears to be running.")


            # --- Start Flask Web Server Thread ---
            logging.info(f"Starting Flask web server on 0.0.0.0:{WEB_PORT}...")
            # Run Flask in a separate thread. use_reloader=False is important for threaded mode.
            flask_thread = threading.Thread(
                target=lambda: app.run(host='0.0.0.0', port=WEB_PORT, debug=False, use_reloader=False, threaded=True),
                name="FlaskThread",
                daemon=True # Set as daemon so it exits when main thread exits
            )
            flask_thread.start()
            time.sleep(1.5) # Give Flask a moment to start
            if not flask_thread.is_alive():
                raise RuntimeError("Flask web server thread failed to start.")


            # --- System Running State ---
            logging.info("--- System Running ---")
            # Try to get local IP for user convenience (optional)
            try:
                import socket
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(("8.8.8.8", 80)) # Connect to external host to find local IP
                local_ip = s.getsockname()[0]
                s.close()
                logging.info(f"Access web interface at: http://{local_ip}:{WEB_PORT} (or use Pi's IP/hostname)")
            except Exception:
                logging.info(f"Access web interface at: http://<YOUR_PI_IP>:{WEB_PORT}")


            # Keep main thread alive, checking worker threads periodically
            while not shutdown_event.is_set():
                if not capture_thread.is_alive():
                    # Capture thread died unexpectedly
                    raise RuntimeError(last_error or "Capture thread terminated unexpectedly.")
                if not flask_thread.is_alive():
                    # Flask thread died unexpectedly
                    raise RuntimeError(last_error or "Flask web server thread terminated unexpectedly.")
                # Wait for shutdown signal or timeout
                shutdown_event.wait(timeout=5.0) # Check thread status every 5 seconds

            # If shutdown_event was set cleanly, break the outer loop
            break

        except RuntimeError as e:
            # Handle errors raised during startup or unexpected thread termination
            logging.error(f"!!! Runtime Error in Main Loop: {e}")
            logging.error("Attempting restart after 10 seconds...")
            # Signal threads to stop (if they haven't already)
            shutdown_event.set()
            # Wait briefly for threads to potentially exit (they are daemons, but try joining capture)
            if capture_thread and capture_thread.is_alive():
                capture_thread.join(timeout=3.0)
            # Reset shutdown event for the next restart attempt
            shutdown_event.clear()
            time.sleep(10.0) # Pause before restarting the loop
        except Exception as e:
            # Catch any other unexpected exceptions in the main setup/monitoring loop
            logging.exception(f"!!! Unhandled Exception in Main Loop: {e}")
            logging.error("Attempting restart after 10 seconds...")
            shutdown_event.set(); # Signal threads
            if capture_thread and capture_thread.is_alive():
                capture_thread.join(timeout=3.0)
            shutdown_event.clear() # Reset for restart
            time.sleep(10.0)

    # --- Final Cleanup (after shutdown_event is set and loop exits) ---
    logging.info("--- Shutdown initiated ---")
    shutdown_event.set() # Ensure flag is definitely set

    # Wait for capture thread to finish (it should handle its own cleanup)
    if capture_thread and capture_thread.is_alive():
        logging.info("Waiting for capture thread to exit...")
        capture_thread.join(timeout=5.0) # Wait up to 5 seconds
        if capture_thread.is_alive():
            logging.warning("Capture thread did not exit cleanly within timeout.")
        else:
            logging.info("Capture thread finished.")

    # Flask thread is a daemon, should exit automatically when main thread ends.

    # Final check to stop recording if somehow still active (should have been stopped by capture thread)
    if is_recording:
        logging.warning("Force stopping recording during final shutdown sequence.")
        stop_recording() # Call stop_recording one last time

    # Final camera cleanup (if capture thread didn't get it)
    if picam2:
        try:
            logging.info("Ensuring Picamera2 is closed during final shutdown...");
            if picam2.started: picam2.stop()
            picam2.close()
            logging.info("Picamera2 closed.")
        except Exception as e:
            logging.warning(f"Error during final Picamera2 close: {e}")

    # Cleanup GPIO resources
    if SWITCH_GPIO_PIN:
        cleanup_gpio()

    logging.info("--- Program Exit ---")

if __name__ == '__main__':
    main()