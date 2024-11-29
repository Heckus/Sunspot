#include <iostream>
#include <gpiod.h>
#include <chrono>
#include <thread>

// Pin definitions - adjust these according to your wiring
#define SERVO1_PIN 18  // GPIO 18 (Pin 12)
#define SERVO2_PIN 19  // GPIO 19 (Pin 35)

// Servo configuration
#define MIN_PULSE 500   // Minimum pulse width in microseconds (0 degrees)
#define MAX_PULSE 2500  // Maximum pulse width in microseconds (180 degrees)

// Function to convert angle to PWM value
int angleToPulse(float angle) {
    // Constrain angle between 0 and 180 degrees
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    return static_cast<int>(MIN_PULSE + (angle / 180.0f) * (MAX_PULSE - MIN_PULSE));
}

// Function to set servo angle
void setServoAngle(gpiod_line* line, float angle) {
    int pulseWidth = angleToPulse(angle);
    // auto chip = gpiod_line_get_chip(line); // Removed to avoid closing the chip
    gpiod_line_request_output(line, "servo", 0);

    // Generate PWM signal
    for (int i = 0; i < 50; ++i) { // 50 Hz PWM signal
        gpiod_line_set_value(line, 1);
        std::this_thread::sleep_for(std::chrono::microseconds(pulseWidth));
        gpiod_line_set_value(line, 0);
        std::this_thread::sleep_for(std::chrono::microseconds(20000 - pulseWidth));
    }

    // gpiod_chip_close(chip); // Removed to avoid closing the chip
}

int main() {
    // Open GPIO chip
    gpiod_chip* chip = gpiod_chip_open_by_name("gpiochip0");
    if (!chip) {
        std::cerr << "Failed to open gpiochip0" << std::endl;
        return 1;
    }

    // Get GPIO lines
    gpiod_line* servo1_line = gpiod_chip_get_line(chip, SERVO1_PIN);
    gpiod_line* servo2_line = gpiod_chip_get_line(chip, SERVO2_PIN);
    if (!servo1_line || !servo2_line) {
        std::cerr << "Failed to get GPIO lines" << std::endl;
        gpiod_chip_close(chip);
        return 1;
    }

    // Set initial position (90 degrees)
    setServoAngle(servo1_line, 90);
    setServoAngle(servo2_line, 90);

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
        setServoAngle(servo1_line, theta);
        setServoAngle(servo2_line, beta);

        // Small delay to allow servos to reach position
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

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
    setServoAngle(servo1_line, 90);
    setServoAngle(servo2_line, 90);

    // Close GPIO chip
    gpiod_chip_close(chip);

    return 0;
}
