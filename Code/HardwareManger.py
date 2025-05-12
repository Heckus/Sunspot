# hardware_manager.py

import os
import time
import logging
import Config # Import project-specific Config

# Attempt to import smbus for INA219 if battery monitoring is enabled
if Config.BATTERY_MONITOR_ENABLED:
    try:
        import smbus
        SMBUS_AVAILABLE = True
    except ImportError:
        SMBUS_AVAILABLE = False
        logging.warning("HardwareManager: smbus library not found. INA219 battery monitoring will be disabled.")
        # Override Config if library is missing
        Config.BATTERY_MONITOR_ENABLED = False
else:
    SMBUS_AVAILABLE = False


# Conditional INA219 Class (only if enabled and smbus available)
if Config.BATTERY_MONITOR_ENABLED and SMBUS_AVAILABLE:
    # Register Addresses (from sample HardwareManager)
    _REG_Config = 0x00
    _REG_SHUNTVOLTAGE = 0x01
    _REG_BUSVOLTAGE = 0x02
    # _REG_POWER = 0x03 # Not strictly needed for voltage/percentage
    # _REG_CURRENT = 0x04 # Not strictly needed for voltage/percentage
    _REG_CALIBRATION = 0x05

    # Configuration Register Bits (simplified for this context, more in sample)
    _Config_BVOLTAGERANGE_16V = 0x0000
    _Config_BVOLTAGERANGE_32V = 0x2000
    _Config_GAIN_1_40MV = 0x0000 # Example gain
    _Config_GAIN_8_320MV = 0x1800 # Example gain for 32V
    _Config_BADCRES_12BIT_1S = 0x0180 # Example ADC resolution
    _Config_SADCRES_12BIT_1S = 0x0018 # Example ADC resolution
    _Config_MODE_SANDBVOLT_CONTINUOUS = 0x0007


    class INA219:
        """
        Simplified driver for the INA219 current/voltage sensor,
        adapted for battery voltage monitoring.
        """
        def __init__(self, i2c_bus=Config.INA219_I2C_BUS, addr=Config.INA219_I2C_ADDRESS):
            self.bus = None
            self.addr = addr
            self.is_present = False

            try:
                self.bus = smbus.SMBus(i2c_bus)
                # Basic check: try reading Config register
                self.read_register(_REG_Config)
                self.is_present = True
                logging.info(f"INA219: Found device at 0x{addr:X} on bus {i2c_bus}.")
                self.Configure_sensor()
            except FileNotFoundError:
                logging.error(f"INA219: I2C bus {i2c_bus} not found. Ensure I2C is enabled.")
            except IOError:
                logging.error(f"INA219: Failed I/O with device at 0x{addr:X}. Check address and wiring.")
            except Exception as e:
                logging.error(f"INA219: Unexpected error initializing sensor: {e}")

        def read_register(self, address):
            data = self.bus.read_i2c_block_data(self.addr, address, 2)
            return (data[0] << 8) | data[1]

        def write_register(self, address, data):
            temp = [(data >> 8) & 0xFF, data & 0xFF]
            self.bus.write_i2c_block_data(self.addr, address, temp)

        def Configure_sensor(self):
            """Configures the INA219 for typical voltage reading scenarios."""
            # This is a generic Configuration. For precise current/power, calibration is key.
            # For battery voltage, a basic Config focusing on bus voltage range is often sufficient.
            # Example: 16V range, Gain 1 (40mV shunt range), 12-bit ADC, continuous mode
            if Config.INA219_CALIBRATION_Config == "32V_1A_OR_SIMILAR": # Example for higher voltage
                 Config_val = (_Config_BVOLTAGERANGE_32V |
                               _Config_GAIN_8_320MV | # Max shunt voltage range
                               _Config_BADCRES_12BIT_1S |
                               _Config_SADCRES_12BIT_1S |
                               _Config_MODE_SANDBVOLT_CONTINUOUS)
                 # For 32V_1A, shunt might be 0.1 ohm. Cal value example: (0.04096 / (0.00005 * 0.1)) = 8192
                 # This calibration is more for current. For voltage only, it's less critical.
                 self.write_register(_REG_CALIBRATION, 8192) # Example, may not be optimal
            else: # Default to 16V_X_AMPS (e.g. 16V_5A from sample)
                Config_val = (_Config_BVOLTAGERANGE_16V |
                              _Config_GAIN_1_40MV | # Default gain for up to 40mV across shunt
                              _Config_BADCRES_12BIT_1S |
                              _Config_SADCRES_12BIT_1S |
                              _Config_MODE_SANDBVOLT_CONTINUOUS)
                # For 16V_5A, shunt might be 0.01 ohm. Cal value example: (0.04096 / (0.0001 * 0.01)) = 40960
                # (Sample used different LSBs for current, leading to different cal)
                # For voltage monitoring only, a default calibration might be okay if not reading current.
                self.write_register(_REG_CALIBRATION, 4096) # A common default-ish value if not fine-tuned

            self.write_register(_REG_Config, Config_val)
            logging.info(f"INA219: Configured with value 0x{Config_val:04X} for {Config.INA219_CALIBRATION_Config} profile.")


        def get_bus_voltage_V(self):
            if not self.is_present: return None
            try:
                raw_val = self.read_register(_REG_BUSVOLTAGE)
                # Check if "Conversion Ready" bit (CNVR, bit 1) is set, if needed. Not usually for continuous.
                # Shift right 3 bits and multiply by LSB (4mV)
                voltage = (raw_val >> 3) * 0.004
                return voltage
            except Exception as e:
                logging.warning(f"INA219: Error reading bus voltage: {e}")
                return None
else:
    # Define a dummy INA219 class if disabled or unavailable
    class INA219:
        def __init__(self, *args, **kwargs):
            self.is_present = False
            logging.info("INA219: Monitoring is disabled or smbus not available.")
        def get_bus_voltage_V(self):
            return None


class HardwareManager:
    """
    Manages hardware interactions for the 3D Volleyball Tracking project.
    Currently focused on battery monitoring if enabled.
    """
    def __init__(self):
        self.ina219_sensor = None
        self.battery_percentage = None
        self.last_battery_read_time = 0.0
        self.last_error = None

        if Config.BATTERY_MONITOR_ENABLED:
            logging.info("HardwareManager: Initializing INA219 Battery Monitor...")
            self.ina219_sensor = INA219(i2c_bus=Config.INA219_I2C_BUS,
                                        addr=Config.INA219_I2C_ADDRESS)
            if not self.ina219_sensor.is_present:
                self.last_error = "INA219 sensor not detected or failed to initialize."
                logging.error(f"HardwareManager: {self.last_error}")
                # Ensure it's None if not present, so checks for self.ina219_sensor work
                self.ina219_sensor = None
            else:
                self.read_battery_level() # Initial read
        else:
            logging.info("HardwareManager: Battery monitoring is disabled in Config.")

        logging.info("HardwareManager initialized.")

    def read_battery_level(self):
        """Reads the battery voltage and calculates the percentage, if INA219 is available."""
        if not Config.BATTERY_MONITOR_ENABLED or self.ina219_sensor is None:
            self.battery_percentage = None
            return

        current_time = time.monotonic()
        # Only read if interval has passed or if previous read failed (battery_percentage is None)
        if (current_time - self.last_battery_read_time < Config.BATTERY_READ_INTERVAL) and \
           self.battery_percentage is not None:
            return # Don't read too frequently

        bus_voltage = self.ina219_sensor.get_bus_voltage_V()
        self.last_battery_read_time = current_time

        if bus_voltage is not None:
            voltage_range = Config.BATTERY_MAX_VOLTAGE - Config.BATTERY_MIN_VOLTAGE
            if voltage_range <= 0:
                logging.warning("HardwareManager: Battery min/max voltages invalid. Cannot calculate percentage.")
                self.battery_percentage = None
                self.last_error = "Invalid battery voltage range in Config."
            else:
                percent = ((bus_voltage - Config.BATTERY_MIN_VOLTAGE) / voltage_range) * 100.0
                self.battery_percentage = max(0.0, min(100.0, percent)) # Clamp 0-100
                logging.debug(f"Battery Voltage: {bus_voltage:.2f}V, Percentage: {self.battery_percentage:.1f}%")
                if self.last_error and "INA219" in self.last_error: # Clear previous INA error if read succeeds
                    self.last_error = None
        else:
            self.battery_percentage = None
            self.last_error = "Failed to read voltage from INA219."
            logging.warning(f"HardwareManager: {self.last_error}")
        return self.battery_percentage


    def get_status(self):
        """Returns a dictionary of hardware status."""
        status = {
            "battery_percentage": self.battery_percentage,
            "battery_monitor_enabled": Config.BATTERY_MONITOR_ENABLED,
            "last_error": self.last_error
        }
        if self.ina219_sensor and self.ina219_sensor.is_present:
            status["ina219_detected"] = True
        else:
            status["ina219_detected"] = False
        return status

    def cleanup(self):
        """Cleans up hardware resources (if any)."""
        logging.info("HardwareManager: Cleaning up hardware resources...")
        # No specific cleanup needed for INA219 via smbus typically,
        # unless GPIO pins were directly managed for other components.
        logging.info("HardwareManager: Cleanup complete.")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format=Config.LOG_FORMAT, datefmt=Config.LOG_DATE_FORMAT)
    logging.info("--- HardwareManager Test ---")

    # Ensure Config has BATTERY_MONITOR_ENABLED = True for this test to be meaningful
    if not Config.BATTERY_MONITOR_ENABLED:
        logging.warning("Battery monitoring is disabled in Config.py. Test will be limited.")
        # Temporarily enable for test if you want to force it, but better to edit Config.py
        # Config.BATTERY_MONITOR_ENABLED = True # Not recommended to change Config at runtime here

    hw_manager = HardwareManager()

    try:
        for i in range(5):
            if Config.BATTERY_MONITOR_ENABLED:
                batt_percent = hw_manager.read_battery_level()
                if batt_percent is not None:
                    logging.info(f"Test Read {i+1}: Battery Level = {batt_percent:.1f}%")
                else:
                    logging.warning(f"Test Read {i+1}: Failed to read battery level. Error: {hw_manager.last_error}")
            else:
                logging.info("Test Read: Battery monitoring is disabled.")
                break # No point looping if disabled

            if i < 4: # Don't sleep after the last read
                 # Wait less than BATTERY_READ_INTERVAL to test throttling, then more.
                sleep_duration = Config.BATTERY_READ_INTERVAL / 2 if i % 2 == 0 else Config.BATTERY_READ_INTERVAL + 1
                logging.info(f"Sleeping for {sleep_duration}s...")
                time.sleep(sleep_duration)

        status_info = hw_manager.get_status()
        logging.info(f"Final Hardware Status: {status_info}")

    except KeyboardInterrupt:
        logging.info("Test interrupted by user.")
    finally:
        hw_manager.cleanup()
        logging.info("--- HardwareManager Test Complete ---")