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