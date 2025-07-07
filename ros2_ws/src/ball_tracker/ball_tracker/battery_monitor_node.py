#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import BatteryState

# Attempt to import the I2C library
try:
    import smbus2
    SMBUS2_AVAILABLE = True
except ImportError:
    SMBUS2_AVAILABLE = False

# --- INA219 Constants (from your HardwareManager.py) ---
_INA219_I2C_ADDRESS = 0x42 # Default address for many UPS boards
_REG_BUSVOLTAGE = 0x02

class BatteryMonitorNode(Node):
    """
    A node that monitors battery status by reading voltage from an INA219
    sensor over I2C, as is common on Waveshare UPS hats.
    """
    def __init__(self):
        super().__init__('battery_monitor_node')

        if not SMBUS2_AVAILABLE:
            self.get_logger().error("The 'smbus2' library is not installed. "
                                   "Please install it: pip install smbus2. Shutting down.")
            rclpy.shutdown()
            return
            
        # Declare parameters
        self.declare_parameter('i2c_bus', 1) # Default I2C bus for Raspberry Pi
        self.declare_parameter('publish_rate_hz', 0.5) # Read from hardware less frequently
        self.declare_parameter('voltage_full', 8.4)  # Common for 2-cell Li-Ion
        self.declare_parameter('voltage_empty', 6.0) # Common for 2-cell Li-Ion

        # Get parameters
        self.i2c_bus_num = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.voltage_full = self.get_parameter('voltage_full').get_parameter_value().double_value
        self.voltage_empty = self.get_parameter('voltage_empty').get_parameter_value().double_value

        # Publisher for the battery state
        self.battery_publisher_ = self.create_publisher(BatteryState, 'battery_state', 10)

        # Initialize I2C bus
        self.i2c_bus = None
        self.is_device_present = self.initialize_i2c()

        # Timer to periodically publish the battery state
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_battery_state)

        self.get_logger().info('Battery monitor node (INA219 Hardware) started.')
        if not self.is_device_present:
            self.get_logger().error(f"INA219 device not found at address 0x{_INA219_I2C_ADDRESS:X} "
                                   f"on I2C bus {self.i2c_bus_num}. Will not publish data.")

    def initialize_i2c(self):
        """Initializes the smbus2 instance and checks for the device."""
        try:
            self.i2c_bus = smbus2.SMBus(self.i2c_bus_num)
            # A simple check to see if the device is responsive.
            # This will raise an IOError if the device is not found.
            self.i2c_bus.read_word_data(_INA219_I2C_ADDRESS, _REG_BUSVOLTAGE)
            self.get_logger().info(f"Successfully connected to INA219 at 0x{_INA219_I2C_ADDRESS:X}")
            return True
        except (FileNotFoundError, IOError) as e:
            self.get_logger().error(f"Failed to initialize I2C bus or find device: {e}")
            return False
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred during I2C initialization: {e}")
            return False

    def read_battery_voltage(self):
        """Reads the battery voltage from the INA219 sensor."""
        if not self.is_device_present or self.i2c_bus is None:
            return None
        
        try:
            # The INA219 returns the bus voltage in a specific format.
            # Reading word data handles the byte swapping for us.
            raw_val = self.i2c_bus.read_word_data(_INA219_I2C_ADDRESS, _REG_BUSVOLTAGE)
            # The actual voltage is in the top 13 bits. Shift right by 3 and multiply by the LSB (4mV).
            voltage = (raw_val >> 3) * 0.004
            return voltage
        except Exception as e:
            self.get_logger().warn(f"Error reading from INA219 sensor: {e}")
            return None

    def publish_battery_state(self):
        """Reads the voltage, calculates the percentage, and publishes."""
        voltage = self.read_battery_voltage()
        
        if voltage is None:
            # If we fail to read, publish an "unknown" state
            battery_msg = BatteryState()
            battery_msg.header.stamp = self.get_clock().now().to_msg()
            battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            battery_msg.present = False
            self.battery_publisher_.publish(battery_msg)
            return

        # Calculate percentage, clamping between 0 and 1
        voltage_range = self.voltage_full - self.voltage_empty
        if voltage_range <= 0:
            percentage = 0.0
        else:
            percentage = (voltage - self.voltage_empty) / voltage_range
            percentage = max(0.0, min(1.0, percentage)) # Clamp to [0, 1] range

        # Create the BatteryState message
        battery_msg = BatteryState()
        battery_msg.header.stamp = self.get_clock().now().to_msg()
        battery_msg.voltage = voltage
        battery_msg.percentage = percentage
        battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        battery_msg.present = True

        self.battery_publisher_.publish(battery_msg)

def main(args=None):
    rclpy.init(args=args)
    battery_node = BatteryMonitorNode()
    
    if SMBUS2_AVAILABLE:
        try:
            rclpy.spin(battery_node)
        except KeyboardInterrupt:
            pass
        finally:
            battery_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()