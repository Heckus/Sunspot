INA219 use example
```cpp
#include "INA219.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

int main() {
    try {
        INA219 ina219(1, 0x41);  // Use I2C bus 1, address 0x41

        while (true) {
            float bus_voltage = ina219.getBusVoltage_V();
            float current = ina219.getCurrent_mA();
            float power = ina219.getPower_W();
            float percentage = ina219.getBatteryPercentage();

            std::cout << "Load Voltage: " << std::fixed << std::setprecision(3) 
                      << bus_voltage << " V" << std::endl;
            std::cout << "Current: " << std::fixed << std::setprecision(6) 
                      << current/1000.0f << " A" << std::endl;
            std::cout << "Power: " << std::fixed << std::setprecision(3) 
                      << power << " W" << std::endl;
            std::cout << "Battery: " << std::fixed << std::setprecision(1) 
                      << percentage << "%" << std::endl;
            std::cout << std::endl;

            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
```