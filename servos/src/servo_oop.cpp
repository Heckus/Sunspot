#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

// Pin definitions - adjust these according to your wiring
const int SERVO1_PIN = 18;  // GPIO 18 (Pin 12)
const int SERVO2_PIN = 19;  // GPIO 19 (Pin 35)

// Servo configuration
const int PWM_FREQ = 50;    // 50Hz frequency
const int MIN_PULSE = 500;  // Minimum pulse width in microseconds (0 degrees)
const int MAX_PULSE = 2500; // Maximum pulse width in microseconds (180 degrees)

// Class to manage servo operations
class ServoController {
private:
    int pin;
    float currentAngle;

    // Convert angle (degrees) to PWM pulse width
    int angleToPulse(float angle) {
        // Constrain angle between 0 and 180 degrees
        angle = std::min(std::max(angle, 0.0f), 180.0f);
        return MIN_PULSE + (angle / 180.0f) * (MAX_PULSE - MIN_PULSE);
    }

public:
    ServoController(int servoPin) : pin(servoPin), currentAngle(90.0f) {
        // Initialize the PWM pin
        pinMode(pin, OUTPUT);
        softPwmCreate(pin, 0, 200);  // Range: 0-200 (20ms period at 50Hz)
    }

    // Set servo angle (0-180 degrees)
    void setAngle(float angle) {
        currentAngle = angle;
        int pulse = angleToPulse(angle);
        // Convert pulse width to PWM value (0-200 range)
        int pwmValue = (pulse * 200) / 20000;
        softPwmWrite(pin, pwmValue);
    }

    float getCurrentAngle() const {
        return currentAngle;
    }
};

class DualServoSystem {
private:
    ServoController servo1;
    ServoController servo2;

public:
    DualServoSystem() : 
        servo1(SERVO1_PIN),
        servo2(SERVO2_PIN) {
    }

    // Set angles for both servos
    void setAngles(float theta, float beta) {
        servo1.setAngle(theta);
        servo2.setAngle(beta);
        
        // Small delay to allow servos to reach position
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Get current angles
    void getCurrentAngles(float& theta, float& beta) {
        theta = servo1.getCurrentAngle();
        beta = servo2.getCurrentAngle();
    }
};

int main() {
    // Initialize WiringPi
    if (wiringPiSetupGpio() == -1) {
        std::cerr << "Failed to initialize WiringPi!" << std::endl;
        return 1;
    }

    try {
        DualServoSystem servoSystem;
        
        while (true) {
            float theta, beta;
            std::cout << "Enter theta angle (0-180): ";
            std::cin >> theta;
            std::cout << "Enter beta angle (0-180): ";
            std::cin >> beta;

            if (std::cin.fail()) {
                std::cin.clear();
                std::cin.ignore(10000, '\n');
                std::cout << "Invalid input. Please enter numbers between 0 and 180." << std::endl;
                continue;
            }

            servoSystem.setAngles(theta, beta);
            
            // Print current positions
            float currentTheta, currentBeta;
            servoSystem.getCurrentAngles(currentTheta, currentBeta);
            std::cout << "Current positions - Theta: " << currentTheta 
                      << "°, Beta: " << currentBeta << "°" << std::endl;
            
            char continue_prompt;
            std::cout << "Continue? (y/n): ";
            std::cin >> continue_prompt;
            if (continue_prompt != 'y' && continue_prompt != 'Y') {
                break;
            }
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
