#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <lgpio.h>

// Pin definitions - adjust these according to your wiring
#define SERVO1_PIN 18  // GPIO 18 (Pin 12)
#define SERVO2_PIN 19  // GPIO 19 (Pin 35)

// Servo configuration
#define MIN_PULSE 0.5   // Minimum pulse width in milliseconds (0 degrees)
#define MAX_PULSE 2.5   // Maximum pulse width in milliseconds (180 degrees)
#define SERVO_FREQUENCY 50 // Servo frequency in Hz
#define SERVO_OFFSET 0 // Servo offset
#define SERVO_CYCLES 0 // Infinite cycles

// Function to convert angle to PWM value
float angleToPulse(float angle) {
    // Constrain angle between 0 and 180 degrees
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    return MIN_PULSE + (angle / 180.0f) * (MAX_PULSE - MIN_PULSE);
}

// Function to set servo angle
void setServoAngle(int h, int pin, float angle) {
    float pulse = angleToPulse(angle);
    int pulseWidth = static_cast<int>(pulse * 1000); // Convert to microseconds
    lgTxServo(h, pin, pulseWidth, SERVO_FREQUENCY, SERVO_OFFSET, SERVO_CYCLES);
}

int main() {
    int h;
    h = lgGpiochipOpen(0); // Open GPIO chip 0
    if (h < 0) {
        std::cerr << "Failed to open GPIO chip" << std::endl;
        return 1;
    }

    // Set initial position (90 degrees)
    setServoAngle(h, SERVO1_PIN, 90);
    setServoAngle(h, SERVO2_PIN, 90);

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
        setServoAngle(h, SERVO1_PIN, theta);
        setServoAngle(h, SERVO2_PIN, beta);

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
    setServoAngle(h, SERVO1_PIN, 90);
    setServoAngle(h, SERVO2_PIN, 90);

    lgGpiochipClose(h); // Close GPIO chip
    
    return 0;
}
