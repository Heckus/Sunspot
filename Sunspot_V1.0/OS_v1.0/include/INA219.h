#ifndef INA219_H
#define INA219_H

#include <cstdint>
#include <algorithm>

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