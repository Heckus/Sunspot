# -*- coding: utf-8 -*-
"""
hardware_manager.py

Manages hardware interactions for the Pi Camera Stream & Record application,
including GPIO switch, INA219 battery monitor, and Servo motor control via Sysfs PWM.
"""

import os
import time
import logging
import smbus
from gpiozero import Button, Device
from gpiozero.pins.native import NativeFactory

# Import configuration constants
import config

# ===========================================================
# === INA219 Battery Monitor Class START ===
# ===========================================================
# Register Addresses
_REG_CONFIG                      = 0x00
_REG_SHUNTVOLTAGE                = 0x01
_REG_BUSVOLTAGE                  = 0x02
_REG_POWER                       = 0x03
_REG_CURRENT                     = 0x04
_REG_CALIBRATION                 = 0x05

# Configuration Register Bits
_CONFIG_RESET                    = 0x8000 # Reset Bit

_CONFIG_BVOLTAGERANGE_MASK       = 0x2000 # Bus Voltage Range Mask
_CONFIG_BVOLTAGERANGE_16V        = 0x0000 # 0-16V Range
_CONFIG_BVOLTAGERANGE_32V        = 0x2000 # 0-32V Range (unused in default config)

_CONFIG_GAIN_MASK                = 0x1800 # Gain Mask
_CONFIG_GAIN_1_40MV              = 0x0000 # Gain 1, 40mV Range
_CONFIG_GAIN_2_80MV              = 0x0800 # Gain 2, 80mV Range
_CONFIG_GAIN_4_160MV             = 0x1000 # Gain 4, 160mV Range
_CONFIG_GAIN_8_320MV             = 0x1800 # Gain 8, 320mV Range

_CONFIG_BADCRES_MASK             = 0x0780 # Bus ADC Resolution Mask
_CONFIG_BADCRES_9BIT_1S          = 0x0000 # 9bit, 1 sample, 84us
_CONFIG_BADCRES_10BIT_1S         = 0x0080 # 10bit, 1 sample, 148us
_CONFIG_BADCRES_11BIT_1S         = 0x0100 # 11bit, 1 sample, 276us
_CONFIG_BADCRES_12BIT_1S         = 0x0180 # 12bit, 1 sample, 532us
_CONFIG_BADCRES_12BIT_2S         = 0x0480 # 12bit, 2 samples, 1.06ms
_CONFIG_BADCRES_12BIT_4S         = 0x0500 # 12bit, 4 samples, 2.13ms
_CONFIG_BADCRES_12BIT_8S         = 0x0580 # 12bit, 8 samples, 4.26ms
_CONFIG_BADCRES_12BIT_16S        = 0x0600 # 12bit, 16 samples, 8.51ms
_CONFIG_BADCRES_12BIT_32S        = 0x0680 # 12bit, 32 samples, 17.02ms
_CONFIG_BADCRES_12BIT_64S        = 0x0700 # 12bit, 64 samples, 34.05ms
_CONFIG_BADCRES_12BIT_128S       = 0x0780 # 12bit, 128 samples, 68.10ms

_CONFIG_SADCRES_MASK             = 0x0078 # Shunt ADC Resolution Mask
_CONFIG_SADCRES_9BIT_1S          = 0x0000 # 9bit, 1 sample, 84us
_CONFIG_SADCRES_10BIT_1S         = 0x0008 # 10bit, 1 sample, 148us
_CONFIG_SADCRES_11BIT_1S         = 0x0010 # 11bit, 1 sample, 276us
_CONFIG_SADCRES_12BIT_1S         = 0x0018 # 12bit, 1 sample, 532us
_CONFIG_SADCRES_12BIT_2S         = 0x0048 # 12bit, 2 samples, 1.06ms
_CONFIG_SADCRES_12BIT_4S         = 0x0050 # 12bit, 4 samples, 2.13ms
_CONFIG_SADCRES_12BIT_8S         = 0x0058 # 12bit, 8 samples, 4.26ms
_CONFIG_SADCRES_12BIT_16S        = 0x0060 # 12bit, 16 samples, 8.51ms
_CONFIG_SADCRES_12BIT_32S        = 0x0068 # 12bit, 32 samples, 17.02ms
_CONFIG_SADCRES_12BIT_64S        = 0x0070 # 12bit, 64 samples, 34.05ms
_CONFIG_SADCRES_12BIT_128S       = 0x0078 # 12bit, 128 samples, 68.10ms

_CONFIG_MODE_MASK                = 0x0007 # Operating Mode Mask
_CONFIG_MODE_POWERDOWN           = 0x0000
_CONFIG_MODE_SVOLT_TRIGGERED     = 0x0001
_CONFIG_MODE_BVOLT_TRIGGERED     = 0x0002
_CONFIG_MODE_SANDBVOLT_TRIGGERED = 0x0003
_CONFIG_MODE_ADCOFF              = 0x0004
_CONFIG_MODE_SVOLT_CONTINUOUS    = 0x0005
_CONFIG_MODE_BVOLT_CONTINUOUS    = 0x0006
_CONFIG_MODE_SANDBVOLT_CONTINUOUS= 0x0007


class INA219:
    """
    Driver for the INA219 current/voltage sensor.
    Handles I2C communication, configuration, and reading values.
    """
    def __init__(self, i2c_bus=config.INA219_I2C_BUS, addr=config.INA219_I2C_ADDRESS):
        """
        Initializes the INA219 sensor.

        Args:
            i2c_bus (int): The I2C bus number (default from config).
            addr (int): The I2C address of the sensor (default from config).

        Raises:
            FileNotFoundError: If the specified I2C bus does not exist.
            IOError: If communication with the sensor fails.
            Exception: For other initialization errors.
        """
        self.bus = None
        self.addr = addr
        self._cal_value = 0
        self._current_lsb = 0.0001 # Default LSB, will be updated by calibration
        self._power_lsb = 0.002    # Default LSB, will be updated by calibration

        try:
            self.bus = smbus.SMBus(i2c_bus)
            logging.info(f"INA219: Attempting to initialize sensor at address 0x{addr:X} on bus {i2c_bus}")
            # Check if device is present by trying to read config register
            self.read_register(_REG_CONFIG)
            logging.info(f"INA219: Found device at 0x{addr:X}.")

            # Configure based on config file setting
            if config.INA219_CALIBRATION_CONFIG == "16V_5A":
                self.set_calibration_16V_5A()
            elif config.INA219_CALIBRATION_CONFIG == "32V_2A":
                self.set_calibration_32V_2A()
            # Add elif for other custom configurations if needed
            else:
                logging.warning(f"INA219: Unknown calibration config '{config.INA219_CALIBRATION_CONFIG}'. Using default 16V 5A.")
                self.set_calibration_16V_5A()

            logging.info(f"INA219: Sensor initialized successfully (Config: {config.INA219_CALIBRATION_CONFIG}).")

        except FileNotFoundError:
            logging.error(f"!!! INA219: I2C bus {i2c_bus} not found. Check raspi-config or connections.")
            raise # Re-raise the specific error
        except IOError as e:
            logging.error(f"!!! INA219: Failed I/O operation with device at 0x{addr:X}. Check address and wiring. Error: {e}")
            raise # Re-raise the specific error
        except Exception as e:
            logging.error(f"!!! INA219: Unexpected error initializing sensor at 0x{addr:X}: {e}", exc_info=True)
            raise # Re-raise the general error

    def read_register(self, address):
        """Reads a 16-bit value from the specified register."""
        try:
            data = self.bus.read_i2c_block_data(self.addr, address, 2)
            return ((data[0] * 256) + data[1])
        except IOError as e:
            logging.error(f"INA219: I2C Read Error from register 0x{address:02X}: {e}")
            raise # Re-raise error for calling function to handle

    def write_register(self, address, data):
        """Writes a 16-bit value to the specified register."""
        try:
            temp = [0, 0]
            temp[0] = (data >> 8) & 0xFF # MSB
            temp[1] = data & 0xFF        # LSB
            self.bus.write_i2c_block_data(self.addr, address, temp)
        except IOError as e:
            logging.error(f"INA219: I2C Write Error to register 0x{address:02X} with value 0x{data:04X}: {e}")
            raise # Re-raise error

    def configure(self, voltage_range, gain, bus_adc, shunt_adc):
        """Configures the INA219 settings."""
        config_val = voltage_range | gain | bus_adc | shunt_adc | _CONFIG_MODE_SANDBVOLT_CONTINUOUS
        logging.debug(f"INA219: Writing configuration 0x{config_val:04X}")
        self.write_register(_REG_CONFIG, config_val)

    def set_calibration_16V_5A(self):
        """Configures the INA219 for 16V and 5A operation (0.01 Ohm shunt assumed)."""
        # Calibration value calculation depends on max expected current and shunt resistance.
        # For 16V, 5A, 0.01 Ohm shunt:
        # Max Possible Shunt Voltage = 5A * 0.01 Ohm = 0.05V = 50mV
        # Choose Gain 2 (80mV range) -> _CONFIG_GAIN_2_80MV
        # Current LSB = Max Expected Current / 32767 (roughly, adjusted for best resolution)
        # Using datasheet example/common values:
        self._current_lsb = 0.0001524 # Amps per bit (adjust if needed for your shunt/accuracy)
        self._cal_value = 26868       # Calibration register value for this config
        self._power_lsb = 0.003048    # Watts per bit

        self.write_register(_REG_CALIBRATION, self._cal_value)
        self.configure(_CONFIG_BVOLTAGERANGE_16V,
                       _CONFIG_GAIN_2_80MV,
                       _CONFIG_BADCRES_12BIT_32S, # High resolution/averaging for bus voltage
                       _CONFIG_SADCRES_12BIT_32S) # High resolution/averaging for shunt voltage
        logging.info("INA219: Calibrated for 16V 5A range (0.01 Ohm shunt assumed).")

    def set_calibration_32V_2A(self):
        """Configures the INA219 for 32V and 2A operation (0.1 Ohm shunt assumed)."""
        # For 32V, 2A, 0.1 Ohm shunt:
        # Max Possible Shunt Voltage = 2A * 0.1 Ohm = 0.2V = 200mV
        # Choose Gain 8 (320mV range) -> _CONFIG_GAIN_8_320MV
        self._current_lsb = 0.0001 # Amps per bit (100uA)
        self._cal_value = 10240    # Calibration register value for this config (adjust based on exact shunt)
        self._power_lsb = 0.002    # Watts per bit (2mW)

        self.write_register(_REG_CALIBRATION, self._cal_value)
        self.configure(_CONFIG_BVOLTAGERANGE_32V,
                       _CONFIG_GAIN_8_320MV,
                       _CONFIG_BADCRES_12BIT_32S,
                       _CONFIG_SADCRES_12BIT_32S)
        logging.info("INA219: Calibrated for 32V 2A range (0.1 Ohm shunt assumed).")

    def get_shunt_voltage_mV(self):
        """Reads the shunt voltage in millivolts."""
        value = self.read_register(_REG_SHUNTVOLTAGE)
        # Check sign bit (needs sign extension for negative values)
        if value > 32767:
            value -= 65536
        return value * 0.01 # LSB is 10uV = 0.01mV

    def get_bus_voltage_V(self):
        """Reads the bus voltage in volts."""
        raw_val = self.read_register(_REG_BUSVOLTAGE)
        # Shift to remove status bits (lower 3 bits), LSB is 4mV
        shifted_val = raw_val >> 3
        return shifted_val * 0.004

    def get_current_mA(self):
        """Reads the current in milliamps."""
        # Sometimes a sharp load change can cause the current register to overflow
        # or read invalid values. Reading calibration register can help reset this.
        # self.write_register(_REG_CALIBRATION, self._cal_value) # Uncomment if current readings are often wrong
        value = self.read_register(_REG_CURRENT)
        if value > 32767:
            value -= 65536
        return value * self._current_lsb * 1000 # Convert Amps to mA

    def get_power_W(self):
        """Reads the power in watts."""
        value = self.read_register(_REG_POWER)
        # Power register doesn't overflow the same way as current
        # if value > 32767: value -= 65536 # Usually not needed for power
        return value * self._power_lsb

# ===========================================================
# === INA219 Battery Monitor Class END ===
# ===========================================================


# ===========================================================
# === Servo Control Functions START ===
# ===========================================================

_servo_pwm_exported = False # Module-level state to track PWM export

def _servo_pwm_write(property, value):
    """Internal helper to write a value to a PWM sysfs file for the servo."""
    path_base = f"/sys/class/pwm/pwmchip{config.SERVO_PWM_CHIP}/"
    path_channel = f"{path_base}pwm{config.SERVO_PWM_CHANNEL}/"
    target_path = ""
    value_to_write = str(value)

    if property in ["period", "duty_cycle", "enable", "polarity"]:
        target_path = os.path.join(path_channel, property)
    elif property == "export":
        target_path = os.path.join(path_base, property)
        value_to_write = str(config.SERVO_PWM_CHANNEL) # Value is channel number
    elif property == "unexport":
        target_path = os.path.join(path_base, property)
        value_to_write = str(config.SERVO_PWM_CHANNEL)
    else:
        logging.error(f"SERVO: Unknown PWM property '{property}'")
        return False

    # Check if path exists before writing (except for export/unexport)
    if property not in ["export", "unexport"] and not os.path.exists(target_path):
        logging.error(f"SERVO: PWM path does not exist: {target_path}")
        logging.error("SERVO: Is the PWM channel exported? Is the chip/channel number correct?")
        return False

    try:
        with open(target_path, "w") as f:
            f.write(value_to_write)
        logging.debug(f"SERVO: Wrote '{value_to_write}' to '{target_path}'")
        return True
    except IOError as e:
        logging.error(f"SERVO: Error writing '{value_to_write}' to '{target_path}': {e}")
        logging.error("SERVO: Check permissions (run with sudo?) and path existence.")
        return False
    except Exception as e:
        logging.error(f"SERVO: Unexpected error writing to '{target_path}': {e}")
        return False

def servo_pwm_setup():
    """
    Exports and configures the PWM channel for servo control using sysfs.
    Must be called before using set_servo_angle. Requires root privileges.

    Returns:
        bool: True on success, False on failure.
    """
    global _servo_pwm_exported

    if not config.SERVO_ENABLED:
        logging.info("SERVO: Servo control is disabled in config. Skipping setup.")
        return False

    pwm_base_path = f"/sys/class/pwm/pwmchip{config.SERVO_PWM_CHIP}/"
    pwm_path = f"{pwm_base_path}pwm{config.SERVO_PWM_CHANNEL}/"

    logging.info(f"SERVO: Attempting setup: Chip={config.SERVO_PWM_CHIP}, Channel={config.SERVO_PWM_CHANNEL} (GPIO {config.SERVO_GPIO_PIN})")

    # Check root privileges (essential for sysfs)
    if os.geteuid() != 0:
        logging.error("SERVO: Setup requires root privileges (run with 'sudo'). Cannot control servo.")
        return False

    # Export the channel if not already exported
    if not os.path.exists(pwm_path):
        if not os.path.exists(pwm_base_path):
            logging.error(f"SERVO: PWM chip path does not exist: {pwm_base_path}")
            logging.error("SERVO: Is the SERVO_PWM_CHIP number correct?")
            return False

        logging.info(f"SERVO: Exporting PWM channel {config.SERVO_PWM_CHANNEL} on chip {config.SERVO_PWM_CHIP}...")
        if not _servo_pwm_write("export", config.SERVO_PWM_CHANNEL):
            logging.error("SERVO: Failed to export PWM channel.")
            return False
        time.sleep(0.2) # Give sysfs more time to create files reliably
        _servo_pwm_exported = True
    else:
        logging.info("SERVO: PWM channel already exported.")
        _servo_pwm_exported = True

    # Check again if path exists after attempting export
    if not os.path.exists(pwm_path):
        logging.error(f"SERVO: PWM path still does not exist after export attempt: {pwm_path}")
        _servo_pwm_exported = False
        return False

    # Configure PWM properties
    logging.info("SERVO: Setting PWM period...")
    if not _servo_pwm_write("period", config.SERVO_PERIOD_NS): return False
    logging.info("SERVO: Setting initial PWM duty cycle (center)...")
    # Calculate center duty cycle based on config min/max
    center_duty_ns = config.SERVO_MIN_DUTY_NS + (config.SERVO_MAX_DUTY_NS - config.SERVO_MIN_DUTY_NS) * (config.SERVO_CENTER_ANGLE - config.SERVO_MIN_ANGLE) / (config.SERVO_MAX_ANGLE - config.SERVO_MIN_ANGLE)
    if not _servo_pwm_write("duty_cycle", int(center_duty_ns)): return False
    logging.info("SERVO: Setting PWM polarity to normal...")
    if not _servo_pwm_write("polarity", "normal"): return False
    logging.info("SERVO: Enabling PWM output...")
    if not _servo_pwm_write("enable", 1): return False

    logging.info("SERVO: PWM setup complete.")
    return True

def set_servo_angle(angle):
    """
    Sets the servo position to the specified angle using sysfs PWM.

    Args:
        angle (float or int): The desired angle (clamped to configured min/max).
    """
    if not config.SERVO_ENABLED:
        # logging.debug("SERVO: Servo disabled, ignoring set_servo_angle.")
        return
    if not _servo_pwm_exported:
        logging.error("SERVO: PWM not setup or export failed. Call servo_pwm_setup() first.")
        return

    # Clamp angle to configured min/max range
    clamped_angle = max(config.SERVO_MIN_ANGLE, min(config.SERVO_MAX_ANGLE, float(angle)))

    # Map angle to duty cycle based on configured min/max duty and angle range
    angle_range = config.SERVO_MAX_ANGLE - config.SERVO_MIN_ANGLE
    duty_range_ns = config.SERVO_MAX_DUTY_NS - config.SERVO_MIN_DUTY_NS

    if angle_range <= 0: # Avoid division by zero
        duty_ns = config.SERVO_MIN_DUTY_NS
    else:
        proportion = (clamped_angle - config.SERVO_MIN_ANGLE) / angle_range
        duty_ns = config.SERVO_MIN_DUTY_NS + proportion * duty_range_ns

    # Clamp duty cycle to absolute min/max just in case
    clamped_duty_ns = int(max(config.SERVO_MIN_DUTY_NS, min(config.SERVO_MAX_DUTY_NS, duty_ns)))

    logging.debug(f"SERVO: Setting angle {clamped_angle:.1f} -> duty cycle {clamped_duty_ns} ns")
    if not _servo_pwm_write("duty_cycle", clamped_duty_ns):
        logging.error("SERVO: Failed to set duty cycle.")

def cleanup_servo_pwm():
    """
    Disables and unexports the servo PWM channel using sysfs.
    Call this during script shutdown.
    """
    global _servo_pwm_exported

    if not config.SERVO_ENABLED:
        logging.debug("SERVO: Servo disabled, skipping cleanup.")
        return

    logging.info("SERVO: Cleaning up PWM...")

    pwm_base_path = f"/sys/class/pwm/pwmchip{config.SERVO_PWM_CHIP}/"
    pwm_path = f"{pwm_base_path}pwm{config.SERVO_PWM_CHANNEL}/"

    # Only try to disable/unexport if the path exists and we think we exported it
    if _servo_pwm_exported and os.path.exists(pwm_path):
        logging.info("SERVO: Disabling PWM output...")
        _servo_pwm_write("enable", 0)
        time.sleep(0.1) # Short delay before unexporting
        logging.info(f"SERVO: Unexporting PWM channel {config.SERVO_PWM_CHANNEL} on chip {config.SERVO_PWM_CHIP}...")
        if _servo_pwm_write("unexport", config.SERVO_PWM_CHANNEL):
            _servo_pwm_exported = False # Mark as unexported on success
        else:
             logging.error("SERVO: Failed to unexport PWM channel during cleanup.")
    elif _servo_pwm_exported:
        logging.warning("SERVO: PWM channel path not found during cleanup, but was marked as exported. Skipping disable/unexport.")
        _servo_pwm_exported = False # Reset state anyway
    else:
        logging.info("SERVO: PWM channel not marked as exported, skipping cleanup actions.")

    logging.info("SERVO: PWM cleanup attempt complete.")

# ===========================================================
# === Servo Control Functions END ===
# ===========================================================


# ===========================================================
# === Hardware Manager Class START ===
# ===========================================================

class HardwareManager:
    """
    Encapsulates setup, access, and cleanup for hardware components.
    """
    def __init__(self):
        """Initializes hardware components based on config."""
        self.switch = None
        self.ina219_sensor = None
        self.battery_percentage = None
        self.last_battery_read_time = 0.0
        self.last_error = None # Store last hardware-related error

        # --- Setup GPIO Switch ---
        if config.SWITCH_GPIO_PIN is not None:
            self._setup_gpio_switch()
        else:
            logging.info("HW Manager: Physical switch disabled in config.")

        # --- Setup Battery Monitor ---
        if config.INA219_I2C_ADDRESS is not None:
            self._setup_battery_monitor()
        else:
            logging.info("HW Manager: Battery monitor disabled in config.")

        # --- Setup Servo ---
        if config.SERVO_ENABLED:
            if not servo_pwm_setup():
                self.last_error = "Servo PWM setup failed. Check logs and sudo privileges."
                logging.error(f"HW Manager: {self.last_error}")
                # Decide if this is fatal? For now, just log error.
            else:
                logging.info("HW Manager: Servo PWM setup successful.")
                # Set initial position
                set_servo_angle(config.SERVO_CENTER_ANGLE)
                time.sleep(0.5) # Allow time for servo to move
        else:
             logging.info("HW Manager: Servo control disabled in config.")

    def _setup_gpio_switch(self):
        """Initializes the GPIO switch using gpiozero."""
        logging.info(f"HW Manager: Setting up GPIO switch on pin {config.SWITCH_GPIO_PIN}...")
        try:
            # Attempt to use NativeFactory for potentially better performance/reliability
            try:
                Device.pin_factory = NativeFactory()
                logging.info("HW Manager: Using gpiozero NativeFactory.")
            except Exception as e:
                logging.warning(f"HW Manager: Could not set NativeFactory for gpiozero, using default: {e}")
                # Script will continue with the default factory if NativeFactory fails

            self.switch = Button(config.SWITCH_GPIO_PIN,
                                 pull_up=True, # Assume button connects pin to GND when pressed
                                 bounce_time=config.SWITCH_BOUNCE_TIME)
            logging.info(f"HW Manager: gpiozero Button on pin {config.SWITCH_GPIO_PIN} setup complete.")
            self.last_error = None # Clear previous GPIO errors on success
        except Exception as e:
            logging.error(f"!!! HW Manager: Failed to setup gpiozero Button: {e}", exc_info=True)
            self.last_error = f"GPIO Switch Setup Error: {e}"
            self.switch = None

    def _setup_battery_monitor(self):
        """Initializes the INA219 sensor."""
        logging.info("HW Manager: Setting up Battery Monitor (INA219)...")
        try:
            self.ina219_sensor = INA219(addr=config.INA219_I2C_ADDRESS, i2c_bus=config.INA219_I2C_BUS)
            # Perform an initial read
            self.read_battery_level()
            voltage = self.ina219_sensor.get_bus_voltage_V() # Read voltage again for logging
            logging.info(f"HW Manager: INA219 Sensor setup complete. Initial voltage: {voltage:.2f}V, Percentage: {self.battery_percentage:.1f}%")
            self.last_error = None # Clear previous battery errors on success
        except Exception as e:
            logging.error(f"!!! HW Manager: Failed to setup INA219 Battery Monitor: {e}", exc_info=False) # Log less verbosely initially
            self.last_error = f"Battery Monitor Setup Error: {e}"
            self.ina219_sensor = None

    def read_battery_level(self):
        """Reads the battery voltage and calculates the percentage."""
        if self.ina219_sensor is None:
            # logging.debug("HW Manager: Battery sensor not available, skipping read.")
            self.battery_percentage = None
            return

        current_time = time.monotonic()
        if current_time - self.last_battery_read_time < config.BATTERY_READ_INTERVAL:
            # logging.debug("HW Manager: Battery read interval not elapsed.")
            return # Don't read too frequently

        try:
            bus_voltage = self.ina219_sensor.get_bus_voltage_V()
            # Optional: Read current/power if needed for logging or state
            # current_ma = self.ina219_sensor.getCurrent_mA()
            # power_w = self.ina219_sensor.getPower_W()
            # logging.debug(f"INA219 Read: V={bus_voltage:.2f}V, I={current_ma:.1f}mA, P={power_w:.3f}W")

            voltage_range = config.BATTERY_MAX_VOLTAGE - config.BATTERY_MIN_VOLTAGE
            if voltage_range <= 0:
                logging.warning("HW Manager: Battery min/max voltages invalid (max <= min). Cannot calculate percentage.")
                percent = None
            else:
                percent = ((bus_voltage - config.BATTERY_MIN_VOLTAGE) / voltage_range) * 100.0
                percent = max(0.0, min(100.0, percent)) # Clamp 0-100

            self.battery_percentage = percent
            self.last_battery_read_time = current_time
            # Clear battery-related errors if read succeeds
            if self.last_error and ("Battery Monitor" in self.last_error or "INA219" in self.last_error or "I2C Error" in self.last_error):
                 logging.info(f"HW Manager: Successfully read battery, clearing previous error: '{self.last_error}'")
                 self.last_error = None

        except (IOError, OSError) as e:
            # Log I2C errors, potentially less verbosely after the first time?
            logging.error(f"!!! HW Manager: I2C Error reading INA219 sensor: {e}")
            self.last_error = f"Battery Read I2C Error: {e}"
            self.battery_percentage = None # Set to None on error
        except Exception as e:
            logging.error(f"!!! HW Manager: Unexpected error reading battery level: {e}", exc_info=True)
            self.last_error = f"Battery Read Error: {e}"
            self.battery_percentage = None

    def is_switch_pressed(self):
        """Checks if the physical switch is pressed."""
        if self.switch is None:
            return False
        try:
            return self.switch.is_pressed
        except Exception as e:
            logging.error(f"HW Manager: Error reading switch state: {e}")
            self.last_error = f"Switch Read Error: {e}"
            # Decide behavior: maybe assume OFF or keep last known state? Assume OFF for safety.
            return False

    def set_servo(self, angle):
        """Sets the servo angle (if servo is enabled and setup)."""
        set_servo_angle(angle) # Calls the module-level function

    def cleanup(self):
        """Cleans up hardware resources."""
        logging.info("HW Manager: Cleaning up hardware resources...")

        # Cleanup GPIO
        if self.switch:
            try:
                self.switch.close()
                logging.info("HW Manager: gpiozero Button closed.")
                self.switch = None
            except Exception as e:
                logging.warning(f"HW Manager: Error closing gpiozero Button: {e}")

        # Cleanup Servo PWM
        cleanup_servo_pwm() # Calls the module-level function

        # No explicit cleanup needed for INA219/smbus unless file handles were kept open

        logging.info("HW Manager: Hardware cleanup complete.")

# ===========================================================
# === Hardware Manager Class END ===
# ===========================================================

# Example Usage (for testing purposes)
if __name__ == "__main__":
    logging.basicConfig(level=config.LOG_LEVEL, format=config.LOG_FORMAT, datefmt=config.LOG_DATE_FORMAT)
    logging.info("--- Hardware Manager Test ---")

    try:
        hw = HardwareManager()

        # Test Switch (if enabled)
        if hw.switch:
            print("Press the switch (Ctrl+C to exit)...")
            while True:
                if hw.is_switch_pressed():
                    print("Switch is PRESSED")
                else:
                    print("Switch is RELEASED")
                time.sleep(0.2)
        else:
            print("Switch is disabled or failed to initialize.")

        # Test Battery (if enabled)
        if hw.ina219_sensor:
            print("\nReading battery level...")
            hw.read_battery_level()
            if hw.battery_percentage is not None:
                print(f"Battery: {hw.battery_percentage:.1f}%")
            else:
                print(f"Battery reading failed. Last Error: {hw.last_error}")
        else:
            print("\nBattery monitor is disabled or failed to initialize.")

        # Test Servo (if enabled)
        if config.SERVO_ENABLED and _servo_pwm_exported: # Check if setup succeeded
             print("\nTesting Servo...")
             angles = [config.SERVO_MIN_ANGLE, config.SERVO_CENTER_ANGLE, config.SERVO_MAX_ANGLE, config.SERVO_CENTER_ANGLE]
             for angle in angles:
                 print(f"Setting servo to {angle} degrees...")
                 hw.set_servo(angle)
                 time.sleep(1.5)
        else:
             print("\nServo is disabled or failed to initialize.")


    except KeyboardInterrupt:
        print("\nExiting test.")
    except Exception as e:
        print(f"\nAn error occurred during test: {e}")
    finally:
        if 'hw' in locals():
            hw.cleanup()
        print("--- Test Complete ---")
