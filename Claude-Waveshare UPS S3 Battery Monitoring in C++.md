## Prompt:
i have python code for a waveshare ups s3 which uses ic2 to determine battery statistics. However i need this code in C++ not python and the waveshare example only has python. Convert this code to C++ and assume it will be on a pi 5 with ic2 enabled

## Response:
I'll help you convert this Python code to C++ for your Raspberry Pi 5. The code will use the Linux I2C interface to communicate with the INA219 power monitor chip.

INA219 C++ Implementation

Click to open code

I've converted the Python code to C++, maintaining the same functionality while adapting it to use Linux I2C interfaces. Here are some key points about the conversion:

1.  The code uses the Linux I2C device interface (`/dev/i2c-X`) instead of Python's SMBus
2.  All the enums and constants have been converted to C++ style
3.  Error handling uses C++ exceptions instead of Python's try/except
4.  The I2C read/write operations use direct file operations instead of SMBus functions
5.  The main loop uses C++ chrono for timing instead of Python's time.sleep

To compile this code on your Raspberry Pi 5, you'll need to:

1.  Save it to a file (e.g., `ina219.cpp`)
2.  Compile with:

```bash
g++ -o ina219 ina219.cpp -std=c++11
```

3.  Run with:

```bash
sudo ./ina219
```

Note that you need to run it with sudo because accessing the I2C bus requires root privileges.

Let me know if you need any clarification or adjustments to the code!
```cpp
#include <iostream>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include <chrono>
#include <thread>

// Register addresses
constexpr uint8_t REG_CONFIG = 0x00;
constexpr uint8_t REG_SHUNTVOLTAGE = 0x01;
constexpr uint8_t REG_BUSVOLTAGE = 0x02;
constexpr uint8_t REG_POWER = 0x03;
constexpr uint8_t REG_CURRENT = 0x04;
constexpr uint8_t REG_CALIBRATION = 0x05;

class INA219 {
public:
    // Constants for bus voltage range
    enum class BusVoltageRange {
        RANGE_16V = 0x00,
        RANGE_32V = 0x01
    };

    // Constants for gain
    enum class Gain {
        DIV_1_40MV = 0x00,
        DIV_2_80MV = 0x01,
        DIV_4_160MV = 0x02,
        DIV_8_320MV = 0x03
    };

    // Constants for ADC resolution
    enum class ADCResolution {
        ADCRES_9BIT_1S = 0x00,
        ADCRES_10BIT_1S = 0x01,
        ADCRES_11BIT_1S = 0x02,
        ADCRES_12BIT_1S = 0x03,
        ADCRES_12BIT_2S = 0x09,
        ADCRES_12BIT_4S = 0x0A,
        ADCRES_12BIT_8S = 0x0B,
        ADCRES_12BIT_16S = 0x0C,
        ADCRES_12BIT_32S = 0x0D,
        ADCRES_12BIT_64S = 0x0E,
        ADCRES_12BIT_128S = 0x0F
    };

    // Constants for operating mode
    enum class Mode {
        POWERDOWN = 0x00,
        SVOLT_TRIGGERED = 0x01,
        BVOLT_TRIGGERED = 0x02,
        SANDBVOLT_TRIGGERED = 0x03,
        ADCOFF = 0x04,
        SVOLT_CONTINUOUS = 0x05,
        BVOLT_CONTINUOUS = 0x06,
        SANDBVOLT_CONTINUOUS = 0x07
    };

    INA219(int bus = 1, uint8_t address = 0x40) {
        char filename[20];
        snprintf(filename, sizeof(filename), "/dev/i2c-%d", bus);
        
        fd = open(filename, O_RDWR);
        if (fd < 0) {
            throw std::runtime_error("Failed to open I2C bus");
        }
        
        if (ioctl(fd, I2C_SLAVE, address) < 0) {
            close(fd);
            throw std::runtime_error("Failed to acquire bus access or talk to slave");
        }

        cal_value = 0;
        current_lsb = 0;
        power_lsb = 0;
        setCalibration_32V_2A();
    }

    ~INA219() {
        if (fd >= 0) {
            close(fd);
        }
    }

    void setCalibration_32V_2A() {
        // Configure for 32V, 2A
        current_lsb = 0.1;  // Current LSB = 100uA per bit
        cal_value = 4096;   // Calculated calibration value
        power_lsb = 0.002;  // Power LSB = 2mW per bit

        // Write calibration value
        writeRegister(REG_CALIBRATION, cal_value);

        // Configure settings
        uint16_t config = static_cast<uint16_t>(BusVoltageRange::RANGE_32V) << 13 |
                         static_cast<uint16_t>(Gain::DIV_8_320MV) << 11 |
                         static_cast<uint16_t>(ADCResolution::ADCRES_12BIT_32S) << 7 |
                         static_cast<uint16_t>(ADCResolution::ADCRES_12BIT_32S) << 3 |
                         static_cast<uint16_t>(Mode::SANDBVOLT_CONTINUOUS);

        writeRegister(REG_CONFIG, config);
    }

    float getShuntVoltage_mV() {
        writeRegister(REG_CALIBRATION, cal_value);
        int16_t value = readRegister(REG_SHUNTVOLTAGE);
        return value * 0.01f;
    }

    float getBusVoltage_V() {
        writeRegister(REG_CALIBRATION, cal_value);
        uint16_t value = readRegister(REG_BUSVOLTAGE);
        return (value >> 3) * 0.004f;
    }

    float getCurrent_mA() {
        int16_t value = readRegister(REG_CURRENT);
        return value * current_lsb;
    }

    float getPower_W() {
        writeRegister(REG_CALIBRATION, cal_value);
        int16_t value = readRegister(REG_POWER);
        return value * power_lsb;
    }

private:
    int fd;
    uint16_t cal_value;
    float current_lsb;
    float power_lsb;

    void writeRegister(uint8_t reg, uint16_t value) {
        uint8_t buf[3] = {
            reg,
            static_cast<uint8_t>(value >> 8),
            static_cast<uint8_t>(value & 0xFF)
        };
        if (write(fd, buf, 3) != 3) {
            throw std::runtime_error("Failed to write to I2C device");
        }
    }

    uint16_t readRegister(uint8_t reg) {
        uint8_t buf[2];
        
        // Write register address
        if (write(fd, &reg, 1) != 1) {
            throw std::runtime_error("Failed to write to I2C device");
        }
        
        // Read data
        if (read(fd, buf, 2) != 2) {
            throw std::runtime_error("Failed to read from I2C device");
        }
        
        return (buf[0] << 8) | buf[1];
    }
};

int main() {
    try {
        INA219 ina219(1, 0x41);  // Use I2C bus 1, address 0x41

        while (true) {
            float bus_voltage = ina219.getBusVoltage_V();
            float shunt_voltage = ina219.getShuntVoltage_mV() / 1000.0f;
            float current = ina219.getCurrent_mA();
            float power = ina219.getPower_W();
            float percentage = (bus_voltage - 9.0f) / 3.6f * 100.0f;
            
            percentage = std::min(std::max(percentage, 0.0f), 100.0f);

            std::cout << "Load Voltage: " << std::fixed << std::setprecision(3) 
                      << bus_voltage << " V" << std::endl;
            std::cout << "Current: " << std::fixed << std::setprecision(6) 
                      << current/1000.0f << " A" << std::endl;
            std::cout << "Power: " << std::fixed << std::setprecision(3) 
                      << power << " W" << std::endl;
            std::cout << "Percent: " << std::fixed << std::setprecision(1) 
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
## Prompt:
ok can you package this c++ code with a header so i can call the functions in another c++ file.

## Response:
I'll split the code into a header (.h) and implementation (.cpp) file for better modularity.

INA219.h

Click to open code

INA219.cpp

Click to open code

example.cpp

Click to open code

I've split the code into three files:

1.  `INA219.h`: The header file containing the class declaration
2.  `INA219.cpp`: The implementation file containing all the method definitions
3.  `example.cpp`: An example program showing how to use the library

To use this in your own project:

1.  Copy `INA219.h` and `INA219.cpp` to your project directory
2.  Include `INA219.h` in your source files where you need to use the INA219 class
3.  Compile all the files together

Here's how to compile the example:

```bash
g++ -c INA219.cpp -o INA219.o
g++ -c example.cpp -o example.o
g++ INA219.o example.o -o example
```

Some key improvements in this version:

1.  Added proper header guards
2.  Separated interface from implementation
3.  Made the class non-copyable (since it manages a file descriptor)
4.  Added a convenience method `getBatteryPercentage()` for the UPS-specific calculation
5.  Improved encapsulation of private members
6.  Added proper includes in each file

To use the INA219 class in your own code, you can do something like this:

```cpp
#include "INA219.h"

void myFunction() {
    INA219 ina219(1, 0x41);  // bus 1, address 0x41
    
    float voltage = ina219.getBusVoltage_V();
    float current = ina219.getCurrent_mA();
    float battery = ina219.getBatteryPercentage();
    
    // Use the values as needed...
}
```

Let me know if you need any clarification or have questions about using the library in your project!
```cpp
#ifndef INA219_H
#define INA219_H

#include <cstdint>

class INA219 {
public:
    // Constants for bus voltage range
    enum class BusVoltageRange {
        RANGE_16V = 0x00,
        RANGE_32V = 0x01
    };

    // Constants for gain
    enum class Gain {
        DIV_1_40MV = 0x00,
        DIV_2_80MV = 0x01,
        DIV_4_160MV = 0x02,
        DIV_8_320MV = 0x03
    };

    // Constants for ADC resolution
    enum class ADCResolution {
        ADCRES_9BIT_1S = 0x00,
        ADCRES_10BIT_1S = 0x01,
        ADCRES_11BIT_1S = 0x02,
        ADCRES_12BIT_1S = 0x03,
        ADCRES_12BIT_2S = 0x09,
        ADCRES_12BIT_4S = 0x0A,
        ADCRES_12BIT_8S = 0x0B,
        ADCRES_12BIT_16S = 0x0C,
        ADCRES_12BIT_32S = 0x0D,
        ADCRES_12BIT_64S = 0x0E,
        ADCRES_12BIT_128S = 0x0F
    };

    // Constants for operating mode
    enum class Mode {
        POWERDOWN = 0x00,
        SVOLT_TRIGGERED = 0x01,
        BVOLT_TRIGGERED = 0x02,
        SANDBVOLT_TRIGGERED = 0x03,
        ADCOFF = 0x04,
        SVOLT_CONTINUOUS = 0x05,
        BVOLT_CONTINUOUS = 0x06,
        SANDBVOLT_CONTINUOUS = 0x07
    };

    // Constructor
    INA219(int bus = 1, uint8_t address = 0x40);
    
    // Destructor
    ~INA219();

    // Delete copy constructor and assignment operator
    INA219(const INA219&) = delete;
    INA219& operator=(const INA219&) = delete;

    // Configuration method
    void setCalibration_32V_2A();

    // Measurement methods
    float getShuntVoltage_mV();
    float getBusVoltage_V();
    float getCurrent_mA();
    float getPower_W();

    // Battery percentage calculation (specific to Waveshare UPS S3)
    float getBatteryPercentage() {
        float bus_voltage = getBusVoltage_V();
        float percentage = (bus_voltage - 9.0f) / 3.6f * 100.0f;
        return std::min(std::max(percentage, 0.0f), 100.0f);
    }

private:
    int fd;
    uint16_t cal_value;
    float current_lsb;
    float power_lsb;

    void writeRegister(uint8_t reg, uint16_t value);
    uint16_t readRegister(uint8_t reg);
};

#endif // INA219_H
```
```cpp
#include "INA219.h"
#include <stdexcept>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include <algorithm>

// Register addresses
constexpr uint8_t REG_CONFIG = 0x00;
constexpr uint8_t REG_SHUNTVOLTAGE = 0x01;
constexpr uint8_t REG_BUSVOLTAGE = 0x02;
constexpr uint8_t REG_POWER = 0x03;
constexpr uint8_t REG_CURRENT = 0x04;
constexpr uint8_t REG_CALIBRATION = 0x05;

INA219::INA219(int bus, uint8_t address) {
    char filename[20];
    snprintf(filename, sizeof(filename), "/dev/i2c-%d", bus);
    
    fd = open(filename, O_RDWR);
    if (fd < 0) {
        throw std::runtime_error("Failed to open I2C bus");
    }
    
    if (ioctl(fd, I2C_SLAVE, address) < 0) {
        close(fd);
        throw std::runtime_error("Failed to acquire bus access or talk to slave");
    }

    cal_value = 0;
    current_lsb = 0;
    power_lsb = 0;
    setCalibration_32V_2A();
}

INA219::~INA219() {
    if (fd >= 0) {
        close(fd);
    }
}

void INA219::setCalibration_32V_2A() {
    // Configure for 32V, 2A
    current_lsb = 0.1;  // Current LSB = 100uA per bit
    cal_value = 4096;   // Calculated calibration value
    power_lsb = 0.002;  // Power LSB = 2mW per bit

    // Write calibration value
    writeRegister(REG_CALIBRATION, cal_value);

    // Configure settings
    uint16_t config = static_cast<uint16_t>(BusVoltageRange::RANGE_32V) << 13 |
                     static_cast<uint16_t>(Gain::DIV_8_320MV) << 11 |
                     static_cast<uint16_t>(ADCResolution::ADCRES_12BIT_32S) << 7 |
                     static_cast<uint16_t>(ADCResolution::ADCRES_12BIT_32S) << 3 |
                     static_cast<uint16_t>(Mode::SANDBVOLT_CONTINUOUS);

    writeRegister(REG_CONFIG, config);
}

float INA219::getShuntVoltage_mV() {
    writeRegister(REG_CALIBRATION, cal_value);
    int16_t value = readRegister(REG_SHUNTVOLTAGE);
    return value * 0.01f;
}

float INA219::getBusVoltage_V() {
    writeRegister(REG_CALIBRATION, cal_value);
    uint16_t value = readRegister(REG_BUSVOLTAGE);
    return (value >> 3) * 0.004f;
}

float INA219::getCurrent_mA() {
    int16_t value = readRegister(REG_CURRENT);
    return value * current_lsb;
}

float INA219::getPower_W() {
    writeRegister(REG_CALIBRATION, cal_value);
    int16_t value = readRegister(REG_POWER);
    return value * power_lsb;
}

void INA219::writeRegister(uint8_t reg, uint16_t value) {
    uint8_t buf[3] = {
        reg,
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value & 0xFF)
    };
    if (write(fd, buf, 3) != 3) {
        throw std::runtime_error("Failed to write to I2C device");
    }
}

uint16_t INA219::readRegister(uint8_t reg) {
    uint8_t buf[2];
    
    // Write register address
    if (write(fd, &reg, 1) != 1) {
        throw std::runtime_error("Failed to write to I2C device");
    }
    
    // Read data
    if (read(fd, buf, 2) != 2) {
        throw std::runtime_error("Failed to read from I2C device");
    }
    
    return (buf[0] << 8) | buf[1];
}
```
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
