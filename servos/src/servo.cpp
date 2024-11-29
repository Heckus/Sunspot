// #include <iostream>
// #include <cmath>
// #include <thread>
// #include <chrono>
// #include <wiringPi.h>
// #include <softPwm.h>

#include <iostream>
#include <wiringPi.h>
#include <softPwm.h>

// Pin definitions - adjust these according to your wiring
#define SERVO1_PIN 1  // WiringPi pin 1 (GPIO 18, Pin 12)
#define SERVO2_PIN 24 // WiringPi pin 24 (GPIO 19, Pin 35)

// Servo configuration
#define MIN_PULSE 5    // Minimum pulse width in milliseconds (0 degrees)
#define MAX_PULSE 25   // Maximum pulse width in milliseconds (180 degrees)

// Function to convert angle to PWM value
int angleToPulse(float angle) {
    // Constrain angle between 0 and 180 degrees
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    return static_cast<int>(MIN_PULSE + (angle / 180.0f) * (MAX_PULSE - MIN_PULSE));
}

// Function to set servo angle
void setServoAngle(int pin, float angle) {
    int pulseWidth = angleToPulse(angle);
    softPwmWrite(pin, pulseWidth);
}

int main() {
    // Initialize WiringPi
    if (wiringPiSetup() == -1) {
        std::cerr << "Failed to initialize WiringPi" << std::endl;
        return 1;
    }

    // Initialize software PWM
    if (softPwmCreate(SERVO1_PIN, 0, 200) != 0 || softPwmCreate(SERVO2_PIN, 0, 200) != 0) {
        std::cerr << "Failed to initialize software PWM" << std::endl;
        return 1;
    }

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
        delay(500);

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