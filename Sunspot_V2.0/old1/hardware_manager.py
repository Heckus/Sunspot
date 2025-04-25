# -*- coding: utf-8 -*-
"""
hardware_manager.py

Manages hardware interactions for the Pi Camera Stream & Record application,
including GPIO switch, INA219 battery monitor, and Servo motor control via Sysfs PWM.
Now includes state tracking for the servo angle.
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
# (INA219 Class code remains unchanged)
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
        self.bus = None
        self.addr = addr
        self._cal_value = 0
        self._current_lsb = 0.0001 # Default LSB, will be updated by calibration
        self._power_lsb = 0.002    # Default LSB, will be updated by calibration
        try:
            self.bus = smbus.SMBus(i2c_bus)
            logging.info(f"INA219: Attempting to initialize sensor at address 0x{addr:X} on bus {i2c_bus}")
            self.read_register(_REG_CONFIG) # Check presence
            logging.info(f"INA219: Found device at 0x{addr:X}.")
            if config.INA219_CALIBRATION_CONFIG == "16V_5A": self.set_calibration_16V_5A()
            elif config.INA219_CALIBRATION_CONFIG == "32V_2A": self.set_calibration_32V_2A()
            else: logging.warning(f"INA219: Unknown calibration config '{config.INA219_CALIBRATION_CONFIG}'. Using default 16V 5A."); self.set_calibration_16V_5A()
            logging.info(f"INA219: Sensor initialized successfully (Config: {config.INA219_CALIBRATION_CONFIG}).")
        except FileNotFoundError: logging.error(f"!!! INA219: I2C bus {i2c_bus} not found."); raise
        except IOError as e: logging.error(f"!!! INA219: Failed I/O operation with device at 0x{addr:X}. Error: {e}"); raise
        except Exception as e: logging.error(f"!!! INA219: Unexpected error initializing sensor at 0x{addr:X}: {e}", exc_info=True); raise

    def read_register(self, address):
        try: data = self.bus.read_i2c_block_data(self.addr, address, 2); return ((data[0] * 256) + data[1])
        except IOError as e: logging.error(f"INA219: I2C Read Error from register 0x{address:02X}: {e}"); raise

    def write_register(self, address, data):
        try: temp = [0, 0]; temp[0] = (data >> 8) & 0xFF; temp[1] = data & 0xFF; self.bus.write_i2c_block_data(self.addr, address, temp)
        except IOError as e: logging.error(f"INA219: I2C Write Error to register 0x{address:02X} with value 0x{data:04X}: {e}"); raise

    def configure(self, voltage_range, gain, bus_adc, shunt_adc):
        config_val = voltage_range | gain | bus_adc | shunt_adc | _CONFIG_MODE_SANDBVOLT_CONTINUOUS
        logging.debug(f"INA219: Writing configuration 0x{config_val:04X}")
        self.write_register(_REG_CONFIG, config_val)

    def set_calibration_16V_5A(self):
        self._current_lsb = 0.0001524; self._cal_value = 26868; self._power_lsb = 0.003048
        self.write_register(_REG_CALIBRATION, self._cal_value)
        self.configure(_CONFIG_BVOLTAGERANGE_16V, _CONFIG_GAIN_2_80MV, _CONFIG_BADCRES_12BIT_32S, _CONFIG_SADCRES_12BIT_32S)
        logging.info("INA219: Calibrated for 16V 5A range (0.01 Ohm shunt assumed).")

    def set_calibration_32V_2A(self):
        self._current_lsb = 0.0001; self._cal_value = 10240; self._power_lsb = 0.002
        self.write_register(_REG_CALIBRATION, self._cal_value)
        self.configure(_CONFIG_BVOLTAGERANGE_32V, _CONFIG_GAIN_8_320MV, _CONFIG_BADCRES_12BIT_32S, _CONFIG_SADCRES_12BIT_32S)
        logging.info("INA219: Calibrated for 32V 2A range (0.1 Ohm shunt assumed).")

    def get_shunt_voltage_mV(self):
        value = self.read_register(_REG_SHUNTVOLTAGE);
        if value > 32767: value -= 65536
        return value * 0.01

    def get_bus_voltage_V(self):
        raw_val = self.read_register(_REG_BUSVOLTAGE); shifted_val = raw_val >> 3; return shifted_val * 0.004

    def get_current_mA(self):
        value = self.read_register(_REG_CURRENT)
        if value > 32767: value -= 65536
        return value * self._current_lsb * 1000

    def get_power_W(self):
        value = self.read_register(_REG_POWER); return value * self._power_lsb
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

    if property in ["period", "duty_cycle", "enable", "polarity"]: target_path = os.path.join(path_channel, property)
    elif property == "export": target_path = os.path.join(path_base, property); value_to_write = str(config.SERVO_PWM_CHANNEL)
    elif property == "unexport": target_path = os.path.join(path_base, property); value_to_write = str(config.SERVO_PWM_CHANNEL)
    else: logging.error(f"SERVO: Unknown PWM property '{property}'"); return False

    if property not in ["export", "unexport"] and not os.path.exists(target_path):
        logging.error(f"SERVO: PWM path does not exist: {target_path}. Is channel exported?")
        return False

    try:
        with open(target_path, "w") as f: f.write(value_to_write)
        logging.debug(f"SERVO: Wrote '{value_to_write}' to '{target_path}'")
        return True
    except IOError as e: logging.error(f"SERVO: Error writing '{value_to_write}' to '{target_path}': {e}. Check permissions."); return False
    except Exception as e: logging.error(f"SERVO: Unexpected error writing to '{target_path}': {e}"); return False

def servo_pwm_setup():
    """Exports and configures the PWM channel for servo control using sysfs."""
    global _servo_pwm_exported
    if not config.SERVO_ENABLED: logging.info("SERVO: Servo control disabled. Skipping setup."); return False

    pwm_base_path = f"/sys/class/pwm/pwmchip{config.SERVO_PWM_CHIP}/"
    pwm_path = f"{pwm_base_path}pwm{config.SERVO_PWM_CHANNEL}/"
    logging.info(f"SERVO: Attempting setup: Chip={config.SERVO_PWM_CHIP}, Channel={config.SERVO_PWM_CHANNEL} (GPIO {config.SERVO_GPIO_PIN})")
    if os.geteuid() != 0: logging.error("SERVO: Setup requires root privileges (sudo)."); return False

    if not os.path.exists(pwm_path):
        if not os.path.exists(pwm_base_path): logging.error(f"SERVO: PWM chip path does not exist: {pwm_base_path}"); return False
        logging.info(f"SERVO: Exporting PWM channel {config.SERVO_PWM_CHANNEL}...")
        if not _servo_pwm_write("export", config.SERVO_PWM_CHANNEL): logging.error("SERVO: Failed to export PWM channel."); return False
        time.sleep(0.2); _servo_pwm_exported = True
    else: logging.info("SERVO: PWM channel already exported."); _servo_pwm_exported = True

    if not os.path.exists(pwm_path): logging.error(f"SERVO: PWM path still does not exist after export attempt: {pwm_path}"); _servo_pwm_exported = False; return False

    logging.info("SERVO: Configuring PWM..."); ok = True
    ok &= _servo_pwm_write("period", config.SERVO_PERIOD_NS)
    center_duty_ns = config.SERVO_MIN_DUTY_NS + (config.SERVO_MAX_DUTY_NS - config.SERVO_MIN_DUTY_NS) * (config.SERVO_CENTER_ANGLE - config.SERVO_MIN_ANGLE) / (config.SERVO_MAX_ANGLE - config.SERVO_MIN_ANGLE)
    ok &= _servo_pwm_write("duty_cycle", int(center_duty_ns))
    ok &= _servo_pwm_write("polarity", "normal")
    ok &= _servo_pwm_write("enable", 1)

    if ok: logging.info("SERVO: PWM setup complete.")
    else: logging.error("SERVO: PWM setup failed during configuration."); return False
    return True

def set_servo_angle(angle):
    """
    Sets the servo position to the specified angle using sysfs PWM.
    Returns the clamped angle that was actually set.
    """
    if not config.SERVO_ENABLED: return None # Return None if disabled
    if not _servo_pwm_exported: logging.error("SERVO: PWM not setup. Call servo_pwm_setup() first."); return None

    clamped_angle = max(config.SERVO_MIN_ANGLE, min(config.SERVO_MAX_ANGLE, float(angle)))
    angle_range = config.SERVO_MAX_ANGLE - config.SERVO_MIN_ANGLE
    duty_range_ns = config.SERVO_MAX_DUTY_NS - config.SERVO_MIN_DUTY_NS

    if angle_range <= 0: duty_ns = config.SERVO_MIN_DUTY_NS
    else: proportion = (clamped_angle - config.SERVO_MIN_ANGLE) / angle_range; duty_ns = config.SERVO_MIN_DUTY_NS + proportion * duty_range_ns

    clamped_duty_ns = int(max(config.SERVO_MIN_DUTY_NS, min(config.SERVO_MAX_DUTY_NS, duty_ns)))

    logging.debug(f"SERVO: Setting angle {clamped_angle:.1f} -> duty cycle {clamped_duty_ns} ns")
    if not _servo_pwm_write("duty_cycle", clamped_duty_ns):
        logging.error("SERVO: Failed to set duty cycle.")
        return None # Indicate failure
    return clamped_angle # Return the angle that was set

def cleanup_servo_pwm():
    """Disables and unexports the servo PWM channel using sysfs."""
    global _servo_pwm_exported
    if not config.SERVO_ENABLED: logging.debug("SERVO: Servo disabled, skipping cleanup."); return

    logging.info("SERVO: Cleaning up PWM...")
    pwm_base_path = f"/sys/class/pwm/pwmchip{config.SERVO_PWM_CHIP}/"
    pwm_path = f"{pwm_base_path}pwm{config.SERVO_PWM_CHANNEL}/"

    if _servo_pwm_exported and os.path.exists(pwm_path):
        logging.info("SERVO: Disabling PWM output...")
        _servo_pwm_write("enable", 0)
        time.sleep(0.1)
        logging.info(f"SERVO: Unexporting PWM channel {config.SERVO_PWM_CHANNEL}...")
        if _servo_pwm_write("unexport", config.SERVO_PWM_CHANNEL): _servo_pwm_exported = False
        else: logging.error("SERVO: Failed to unexport PWM channel during cleanup.")
    elif _servo_pwm_exported: logging.warning("SERVO: PWM channel path not found during cleanup, but was marked exported."); _servo_pwm_exported = False
    else: logging.info("SERVO: PWM channel not marked as exported, skipping cleanup actions.")
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
    Tracks current servo angle.
    """
    def __init__(self):
        """Initializes hardware components based on config."""
        self.switch = None
        self.ina219_sensor = None
        self.battery_percentage = None
        self.last_battery_read_time = 0.0
        self.last_error = None # Store last hardware-related error
        self.current_servo_angle = config.SERVO_CENTER_ANGLE # Initialize servo angle state

        # --- Setup GPIO Switch ---
        if config.SWITCH_GPIO_PIN is not None: self._setup_gpio_switch()
        else: logging.info("HW Manager: Physical switch disabled in config.")

        # --- Setup Battery Monitor ---
        if config.INA219_I2C_ADDRESS is not None: self._setup_battery_monitor()
        else: logging.info("HW Manager: Battery monitor disabled in config.")

        # --- Setup Servo ---
        if config.SERVO_ENABLED:
            if not servo_pwm_setup():
                self.last_error = "Servo PWM setup failed. Check logs and sudo privileges."
                logging.error(f"HW Manager: {self.last_error}")
            else:
                logging.info("HW Manager: Servo PWM setup successful.")
                # Set initial position using the class method now
                self.set_servo(config.SERVO_CENTER_ANGLE)
                time.sleep(0.5) # Allow time for servo to move
        else:
             logging.info("HW Manager: Servo control disabled in config.")
             self.current_servo_angle = None # Mark as None if disabled

    def _setup_gpio_switch(self):
        """Initializes the GPIO switch using gpiozero."""
        logging.info(f"HW Manager: Setting up GPIO switch on pin {config.SWITCH_GPIO_PIN}...")
        try:
            try: Device.pin_factory = NativeFactory(); logging.info("HW Manager: Using gpiozero NativeFactory.")
            except Exception as e: logging.warning(f"HW Manager: Could not set NativeFactory: {e}")
            self.switch = Button(config.SWITCH_GPIO_PIN, pull_up=True, bounce_time=config.SWITCH_BOUNCE_TIME)
            logging.info(f"HW Manager: gpiozero Button on pin {config.SWITCH_GPIO_PIN} setup complete.")
            if self.last_error and "Switch" in self.last_error: self.last_error = None # Clear previous switch errors
        except Exception as e:
            logging.error(f"!!! HW Manager: Failed to setup gpiozero Button: {e}", exc_info=True)
            self.last_error = f"GPIO Switch Setup Error: {e}"; self.switch = None

    def _setup_battery_monitor(self):
        """Initializes the INA219 sensor."""
        logging.info("HW Manager: Setting up Battery Monitor (INA219)...")
        try:
            self.ina219_sensor = INA219(addr=config.INA219_I2C_ADDRESS, i2c_bus=config.INA219_I2C_BUS)
            self.read_battery_level() # Perform initial read
            voltage = self.ina219_sensor.get_bus_voltage_V()
            logging.info(f"HW Manager: INA219 Sensor setup complete. Initial voltage: {voltage:.2f}V, Percentage: {self.battery_percentage:.1f}%")
            if self.last_error and "Battery" in self.last_error: self.last_error = None # Clear previous battery errors
        except Exception as e:
            logging.error(f"!!! HW Manager: Failed to setup INA219 Battery Monitor: {e}", exc_info=False)
            self.last_error = f"Battery Monitor Setup Error: {e}"; self.ina219_sensor = None

    def read_battery_level(self):
        """Reads the battery voltage and calculates the percentage."""
        if self.ina219_sensor is None: self.battery_percentage = None; return
        current_time = time.monotonic()
        # Removed interval check here - let caller decide frequency if needed

        try:
            bus_voltage = self.ina219_sensor.get_bus_voltage_V()
            voltage_range = config.BATTERY_MAX_VOLTAGE - config.BATTERY_MIN_VOLTAGE
            if voltage_range <= 0: logging.warning("HW Manager: Battery min/max voltages invalid."); percent = None
            else: percent = ((bus_voltage - config.BATTERY_MIN_VOLTAGE) / voltage_range) * 100.0; percent = max(0.0, min(100.0, percent))
            self.battery_percentage = percent
            self.last_battery_read_time = current_time
            if self.last_error and ("Battery" in self.last_error or "INA219" in self.last_error or "I2C Error" in self.last_error):
                 logging.info(f"HW Manager: Successfully read battery, clearing previous error: '{self.last_error}'"); self.last_error = None
        except (IOError, OSError) as e: logging.error(f"!!! HW Manager: I2C Error reading INA219 sensor: {e}"); self.last_error = f"Battery Read I2C Error: {e}"; self.battery_percentage = None
        except Exception as e: logging.error(f"!!! HW Manager: Unexpected error reading battery level: {e}", exc_info=True); self.last_error = f"Battery Read Error: {e}"; self.battery_percentage = None

    def is_switch_pressed(self):
        """Checks if the physical switch is pressed."""
        if self.switch is None: return False
        try: return self.switch.is_pressed
        except Exception as e: logging.error(f"HW Manager: Error reading switch state: {e}"); self.last_error = f"Switch Read Error: {e}"; return False

    def set_servo(self, angle):
        """
        Sets the servo angle (if servo is enabled and setup) and updates internal state.
        """
        if not config.SERVO_ENABLED:
            logging.debug("HW Manager: Servo disabled, ignoring set_servo.")
            return
        if not _servo_pwm_exported:
             logging.error("HW Manager: Cannot set servo angle, PWM not exported/setup.")
             self.last_error = "Servo Set Error: PWM not ready"
             return

        # Call the module-level function to set the angle via sysfs
        # It returns the clamped angle on success, or None on failure
        actual_angle_set = set_servo_angle(angle)

        if actual_angle_set is not None:
            # Update internal state only on success
            self.current_servo_angle = actual_angle_set
            logging.debug(f"HW Manager: Servo angle state updated to {self.current_servo_angle:.1f}")
            # Clear previous servo errors on success
            if self.last_error and "Servo" in self.last_error:
                logging.info(f"HW Manager: Successfully set servo, clearing previous error: '{self.last_error}'")
                self.last_error = None
        else:
            # set_servo_angle failed (already logged error)
            self.last_error = self.last_error or "Servo Set Error: Failed to write duty cycle" # Ensure error is set

    def get_current_servo_angle(self):
        """
        Returns the last known servo angle stored in the manager's state.
        Returns None if servo is disabled or state is unknown.
        """
        return self.current_servo_angle

    def cleanup(self):
        """Cleans up hardware resources."""
        logging.info("HW Manager: Cleaning up hardware resources...")
        if self.switch:
            try: self.switch.close(); logging.info("HW Manager: gpiozero Button closed."); self.switch = None
            except Exception as e: logging.warning(f"HW Manager: Error closing gpiozero Button: {e}")
        cleanup_servo_pwm() # Calls the module-level function
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
        print(f"Initial Servo Angle State: {hw.get_current_servo_angle()}")

        # Test Switch (if enabled)
        # ... (switch test code - unchanged) ...

        # Test Battery (if enabled)
        # ... (battery test code - unchanged) ...

        # Test Servo (if enabled)
        if config.SERVO_ENABLED and _servo_pwm_exported:
             print("\nTesting Servo...")
             angles_to_test = [config.SERVO_MIN_ANGLE, config.SERVO_CENTER_ANGLE + 30, config.SERVO_MAX_ANGLE, config.SERVO_CENTER_ANGLE]
             for angle in angles_to_test:
                 print(f"Setting servo to {angle} degrees...")
                 hw.set_servo(angle) # Use class method
                 time.sleep(1.0)
                 print(f"  Current Angle State: {hw.get_current_servo_angle()}")
                 time.sleep(0.5)
        else:
             print("\nServo is disabled or failed to initialize.")


    except KeyboardInterrupt:
        print("\nExiting test.")
    except Exception as e:
        print(f"\nAn error occurred during test: {e}")
        logging.exception("Test failed")
    finally:
        # Ensure hw exists before cleanup
        if 'hw' in locals() and isinstance(hw, HardwareManager):
            hw.cleanup()
        else:
             # If hw init failed, still try module-level cleanup
             cleanup_servo_pwm()
        print("--- Test Complete ---")

