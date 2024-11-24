#include <wiringPi.h>
#include <softPwm.h>
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>

// Pin definitions - adjust these according to your wiring
#define SERVO1_PIN 18  // GPIO 18 (Pin 12)
#define SERVO2_PIN 19  // GPIO 19 (Pin 35)

// Servo configuration
#define PWM_FREQ 50     // 50Hz frequency
#define MIN_PULSE 500   // Minimum pulse width in microseconds (0 degrees)
#define MAX_PULSE 2500  // Maximum pulse width in microseconds (180 degrees)

// Function to convert angle to PWM value
int angleToPulse(float angle) {
    // Constrain angle between 0 and 180 degrees
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    return MIN_PULSE + (angle / 180.0f) * (MAX_PULSE - MIN_PULSE);
}

// Function to set servo angle
void setServoAngle(int pin, float angle) {
    int pulse = angleToPulse(angle);
    // Convert pulse width to PWM value (0-200 range for 20ms period)
    int pwmValue = (pulse * 200) / 20000;
    softPwmWrite(pin, pwmValue);
}

// Function to initialize servo
void initServo(int pin) {
    pinMode(pin, OUTPUT);
    softPwmCreate(pin, 0, 200);  // Range: 0-200 (20ms period at 50Hz)
}

int main() {
    // Initialize WiringPi
    if (wiringPiSetupGpio() == -1) {
        std::cerr << "Failed to initialize WiringPi!" << std::endl;
        return 1;
    }

    // Initialize both servos
    initServo(SERVO1_PIN);
    initServo(SERVO2_PIN);

    // Set initial position (90 degrees)
    setServoAngle(SERVO1_PIN, 90);
    setServoAngle(SERVO2_PIN, 90);

    // Main control loop
    while (true) {
        float theta, beta;

        // Get user input for angles
        std::cout << "Enter theta angle (0-180): ";
        std::cin >> theta;
        std::cout << "Enter beta angle (0-180): ";
        std::cin >> beta;

        // Check for valid input
        if (std::cin.fail()) {
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            std::cout << "Invalid input. Please enter numbers between 0 and 180." << std::endl;
            continue;
        }

        // Set servo positions
        setServoAngle(SERVO1_PIN, theta);
        setServoAngle(SERVO2_PIN, beta);

        // Small delay to allow servos to reach position
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        // Print current positions
        std::cout << "Current positions - Theta: " << theta 
                  << "°, Beta: " << beta << "°" << std::endl;

        // Ask to continue
        char continue_prompt;
        std::cout << "Continue? (y/n): ";
        std::cin >> continue_prompt;
        if (continue_prompt != 'y' && continue_prompt != 'Y') {
            break;
        }
    }

    // Center servos before exit
    setServoAngle(SERVO1_PIN, 90);
    setServoAngle(SERVO2_PIN, 90);
    
    return 0;
}
