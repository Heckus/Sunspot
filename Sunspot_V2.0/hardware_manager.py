# -*- coding: utf-8 -*-
"""
hardware_manager.py

Manages hardware interactions for the Pi Camera Stream & Record application,
including GPIO switch, INA219 battery monitor, and Servo motor control via Sysfs PWM.
Includes smooth servo movement logic.
"""

import os
import time
import logging
import smbus
import math # Import math for easing function
from gpiozero import Button, Device
from gpiozero.pins.native import NativeFactory

# Import configuration constants
import config

# ===========================================================
# === INA219 Battery Monitor Class START ===
# (INA219 Class remains unchanged)
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
# ... other BADCRES values ...
_CONFIG_BADCRES_12BIT_128S       = 0x0780 # 12bit, 128 samples, 68.10ms

_CONFIG_SADCRES_MASK             = 0x0078 # Shunt ADC Resolution Mask
_CONFIG_SADCRES_9BIT_1S          = 0x0000 # 9bit, 1 sample, 84us
# ... other SADCRES values ...
_CONFIG_SADCRES_12BIT_128S       = 0x0078 # 12bit, 128 samples, 68.10ms

_CONFIG_MODE_MASK                = 0x0007 # Operating Mode Mask
_CONFIG_MODE_POWERDOWN           = 0x0000
# ... other MODE values ...
_CONFIG_MODE_SANDBVOLT_CONTINUOUS= 0x0007


class INA219:
    """
    Driver for the INA219 current/voltage sensor.
    Handles I2C communication, configuration, and reading values.
    (Implementation unchanged from previous version)
    """
    def __init__(self, i2c_bus=config.INA219_I2C_BUS, addr=config.INA219_I2C_ADDRESS):
        """
        Initializes the INA219 sensor.
        (Code unchanged)
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
        if value > 32767: value -= 65536
        return value * 0.01 # LSB is 10uV = 0.01mV

    def get_bus_voltage_V(self):
        """Reads the bus voltage in volts."""
        raw_val = self.read_register(_REG_BUSVOLTAGE)
        shifted_val = raw_val >> 3
        return shifted_val * 0.004

    def get_current_mA(self):
        """Reads the current in milliamps."""
        value = self.read_register(_REG_CURRENT)
        if value > 32767: value -= 65536
        return value * self._current_lsb * 1000 # Convert Amps to mA

    def get_power_W(self):
        """Reads the power in watts."""
        value = self.read_register(_REG_POWER)
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
        # Don't log error for duty_cycle if it doesn't exist yet during initial setup
        if property != "duty_cycle" or _servo_pwm_exported:
            logging.error(f"SERVO: PWM path does not exist: {target_path}")
            logging.error("SERVO: Is the PWM channel exported? Is the chip/channel number correct?")
        return False

    try:
        with open(target_path, "w") as f:
            f.write(value_to_write)
        # Reduce log verbosity for frequent duty cycle writes
        if property != "duty_cycle":
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
        int or None: The calculated center duty cycle in nanoseconds on success, None on failure.
    """
    global _servo_pwm_exported

    if not config.SERVO_ENABLED:
        logging.info("SERVO: Servo control is disabled in config. Skipping setup.")
        return None # Indicate setup skipped

    pwm_base_path = f"/sys/class/pwm/pwmchip{config.SERVO_PWM_CHIP}/"
    pwm_path = f"{pwm_base_path}pwm{config.SERVO_PWM_CHANNEL}/"

    logging.info(f"SERVO: Attempting setup: Chip={config.SERVO_PWM_CHIP}, Channel={config.SERVO_PWM_CHANNEL} (GPIO {config.SERVO_GPIO_PIN})")

    # Check root privileges (essential for sysfs)
    if os.geteuid() != 0:
        logging.error("SERVO: Setup requires root privileges (run with 'sudo'). Cannot control servo.")
        return None # Indicate failure

    # Export the channel if not already exported
    if not os.path.exists(pwm_path):
        if not os.path.exists(pwm_base_path):
            logging.error(f"SERVO: PWM chip path does not exist: {pwm_base_path}")
            logging.error("SERVO: Is the SERVO_PWM_CHIP number correct?")
            return None # Indicate failure

        logging.info(f"SERVO: Exporting PWM channel {config.SERVO_PWM_CHANNEL} on chip {config.SERVO_PWM_CHIP}...")
        if not _servo_pwm_write("export", config.SERVO_PWM_CHANNEL):
            logging.error("SERVO: Failed to export PWM channel.")
            return None # Indicate failure
        time.sleep(0.2) # Give sysfs more time to create files reliably
        _servo_pwm_exported = True
    else:
        logging.info("SERVO: PWM channel already exported.")
        _servo_pwm_exported = True

    # Check again if path exists after attempting export
    if not os.path.exists(pwm_path):
        logging.error(f"SERVO: PWM path still does not exist after export attempt: {pwm_path}")
        _servo_pwm_exported = False
        return None # Indicate failure

    # Configure PWM properties
    logging.info("SERVO: Setting PWM period...")
    if not _servo_pwm_write("period", config.SERVO_PERIOD_NS): return None
    logging.info("SERVO: Setting initial PWM duty cycle (center)...")
    # Calculate center duty cycle based on config min/max
    try:
        center_duty_ns = config.SERVO_MIN_DUTY_NS + (config.SERVO_MAX_DUTY_NS - config.SERVO_MIN_DUTY_NS) * \
                         (config.SERVO_CENTER_ANGLE - config.SERVO_MIN_ANGLE) / (config.SERVO_MAX_ANGLE - config.SERVO_MIN_ANGLE)
        center_duty_ns = int(max(config.SERVO_MIN_DUTY_NS, min(config.SERVO_MAX_DUTY_NS, center_duty_ns))) # Clamp
    except ZeroDivisionError:
        logging.warning("SERVO: SERVO_MAX_ANGLE equals SERVO_MIN_ANGLE. Setting center duty to min duty.")
        center_duty_ns = int(config.SERVO_MIN_DUTY_NS)
    except Exception as e:
        logging.error(f"SERVO: Error calculating center duty cycle: {e}")
        return None

    if not _servo_pwm_write("duty_cycle", center_duty_ns): return None
    logging.info("SERVO: Setting PWM polarity to normal...")
    if not _servo_pwm_write("polarity", "normal"): return None
    logging.info("SERVO: Enabling PWM output...")
    if not _servo_pwm_write("enable", 1): return None

    logging.info("SERVO: PWM setup complete.")
    return center_duty_ns # Return the calculated center duty cycle


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
    Includes smooth servo control.
    """
    def __init__(self):
        """Initializes hardware components based on config."""
        self.switch = None
        self.ina219_sensor = None
        self.battery_percentage = None
        self.last_battery_read_time = 0.0
        self.last_error = None # Store last hardware-related error
        self.current_servo_duty_ns = None # Track last set servo duty cycle

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
            initial_duty_ns = servo_pwm_setup()
            if initial_duty_ns is None:
                self.last_error = "Servo PWM setup failed. Check logs and sudo privileges."
                logging.error(f"HW Manager: {self.last_error}")
            else:
                logging.info("HW Manager: Servo PWM setup successful.")
                # Store the initial duty cycle
                self.current_servo_duty_ns = initial_duty_ns
                logging.info(f"HW Manager: Initial servo duty cycle set to {self.current_servo_duty_ns} ns (Center: {config.SERVO_CENTER_ANGLE} deg).")
                time.sleep(0.5) # Allow time for servo to move to initial position
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
                logging.warning(f"HW Manager: Could not set NativeFactory for gpiozero ({e}), using default.")
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
            percent_str = f"{self.battery_percentage:.1f}%" if self.battery_percentage is not None else "N/A"
            logging.info(f"HW Manager: INA219 Sensor setup complete. Initial voltage: {voltage:.2f}V, Percentage: {percent_str}")
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
        # Only read if interval has passed OR first read failed
        if (current_time - self.last_battery_read_time < config.BATTERY_READ_INTERVAL) and self.battery_percentage is not None:
            # logging.debug("HW Manager: Battery read interval not elapsed.")
            return # Don't read too frequently if last read was successful

        try:
            bus_voltage = self.ina219_sensor.get_bus_voltage_V()
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

    def _angle_to_duty_ns(self, angle):
        """Converts an angle to the corresponding duty cycle in nanoseconds."""
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
        return clamped_duty_ns

    def _set_servo_angle_smooth(self, target_angle):
        """
        Moves the servo smoothly to the target angle using interpolation.
        This function blocks until the movement is complete.

        Args:
            target_angle (float or int): The desired angle.
        """
        if not _servo_pwm_exported:
            logging.error("SERVO: Cannot move, PWM not setup or export failed.")
            return
        if self.current_servo_duty_ns is None:
             logging.error("SERVO: Cannot move smoothly, current position unknown. Call setup first.")
             # Optionally, could try setting directly without smoothing here
             # target_duty = self._angle_to_duty_ns(target_angle)
             # _servo_pwm_write("duty_cycle", target_duty)
             # self.current_servo_duty_ns = target_duty
             return

        start_duty_ns = self.current_servo_duty_ns
        end_duty_ns = self._angle_to_duty_ns(target_angle)
        delta_duty_ns = end_duty_ns - start_duty_ns

        if abs(delta_duty_ns) < 100: # Threshold to avoid unnecessary movement for tiny changes
            logging.debug("SERVO: Target angle very close to current angle. Skipping smooth move.")
            if start_duty_ns != end_duty_ns: # Ensure final value is set if slightly different
                 _servo_pwm_write("duty_cycle", end_duty_ns)
                 self.current_servo_duty_ns = end_duty_ns
            return

        steps = config.SERVO_SMOOTH_MOVE_STEPS
        delay = config.SERVO_SMOOTH_MOVE_DELAY

        logging.debug(f"SERVO: Smooth move from {start_duty_ns}ns to {end_duty_ns}ns ({target_angle:.1f} deg) in {steps} steps.")

        for step in range(steps + 1):
            progress = step / steps
            # Use sinusoidal easing (ease-in/ease-out)
            ease_progress = 0.5 * (1.0 - math.cos(math.pi * progress))
            current_duty = int(start_duty_ns + delta_duty_ns * ease_progress)

            # Clamp intermediate duty cycle just in case
            current_duty = max(config.SERVO_MIN_DUTY_NS, min(config.SERVO_MAX_DUTY_NS, current_duty))

            if not _servo_pwm_write("duty_cycle", current_duty):
                logging.error("SERVO: Failed to set intermediate duty cycle during smooth move.")
                # Optionally break or try to recover? For now, continue.
            time.sleep(delay)

        # Ensure the final target duty cycle is set accurately
        if self.current_servo_duty_ns != end_duty_ns:
            if not _servo_pwm_write("duty_cycle", end_duty_ns):
                logging.error("SERVO: Failed to set final duty cycle after smooth move.")
            else:
                 logging.debug(f"SERVO: Smooth move complete. Final duty: {end_duty_ns}ns")

        # Update the stored current duty cycle
        self.current_servo_duty_ns = end_duty_ns


    def set_servo(self, angle):
        """
        Sets the servo angle. Uses smooth movement if enabled in config.

        Args:
            angle (float or int): The desired angle.
        """
        if not config.SERVO_ENABLED:
            # logging.debug("SERVO: Servo disabled, ignoring set_servo.")
            return
        if not _servo_pwm_exported:
            logging.error("SERVO: Cannot set angle, PWM not setup or export failed.")
            return

        if config.SERVO_SMOOTH_MOVE:
            self._set_servo_angle_smooth(angle)
        else:
            # Direct move (original behavior)
            target_duty_ns = self._angle_to_duty_ns(angle)
            logging.debug(f"SERVO: Setting angle {angle:.1f} -> duty cycle {target_duty_ns} ns (direct)")
            if _servo_pwm_write("duty_cycle", target_duty_ns):
                self.current_servo_duty_ns = target_duty_ns # Update state even for direct move
            else:
                logging.error("SERVO: Failed to set duty cycle directly.")


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
    logging.info("--- Hardware Manager Test (Smooth Servo) ---")

    hw = None
    try:
        hw = HardwareManager()

        # Test Switch (if enabled)
        if hw.switch:
            print("\n--- Testing Switch (Press Ctrl+C to skip/exit) ---")
            print("Press the switch...")
            try:
                start_time = time.time()
                while time.time() - start_time < 10: # Test for 10 seconds
                    if hw.is_switch_pressed():
                        print("Switch is PRESSED")
                    else:
                        print("Switch is RELEASED")
                    time.sleep(0.2)
            except KeyboardInterrupt:
                print("Skipping rest of switch test.")
        else:
            print("\nSwitch is disabled or failed to initialize.")

        # Test Battery (if enabled)
        if hw.ina219_sensor:
            print("\n--- Testing Battery ---")
            hw.read_battery_level()
            percent_str = f"{hw.battery_percentage:.1f}%" if hw.battery_percentage is not None else "Read Failed"
            print(f"Battery: {percent_str}")
            if hw.battery_percentage is None: print(f"Last Error: {hw.last_error}")
        else:
            print("\nBattery monitor is disabled or failed to initialize.")

        # Test Servo (if enabled)
        if config.SERVO_ENABLED and hw.current_servo_duty_ns is not None: # Check if setup succeeded
             print("\n--- Testing Servo ---")
             angles_to_test = [config.SERVO_MIN_ANGLE, config.SERVO_MAX_ANGLE, config.SERVO_CENTER_ANGLE]
             for angle in angles_to_test:
                 print(f"Setting servo to {angle} degrees...")
                 hw.set_servo(angle) # This will use smooth move if enabled
                 print(f"Move to {angle} initiated. Waiting...")
                 # Wait based on steps/delay - add a buffer
                 wait_time = (config.SERVO_SMOOTH_MOVE_STEPS * config.SERVO_SMOOTH_MOVE_DELAY) + 0.5 if config.SERVO_SMOOTH_MOVE else 1.0
                 time.sleep(wait_time)
                 print(f"Move to {angle} should be complete.")
        elif config.SERVO_ENABLED:
             print("\nServo enabled but failed to initialize.")
        else:
             print("\nServo is disabled.")


    except KeyboardInterrupt:
        print("\nExiting test due to KeyboardInterrupt.")
    except Exception as e:
        print(f"\nAn error occurred during test: {e}")
        logging.exception("Test error")
    finally:
        if hw: # Ensure hw exists before cleanup
            hw.cleanup()
        print("--- Test Complete ---")
