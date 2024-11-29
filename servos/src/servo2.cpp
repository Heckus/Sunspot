#include <iostream>
#include <gpiod.h>
#include <chrono>
#include <thread>
#include <csignal>

// Pin definitions - adjust these according to your wiring
#define SERVO1_PIN 18  // GPIO 18 (Pin 12)
#define SERVO2_PIN 19  // GPIO 19 (Pin 35)

// Servo configuration
#define MIN_PULSE 500   // Minimum pulse width in microseconds (0 degrees)
#define MAX_PULSE 2500  // Maximum pulse width in microseconds (180 degrees)

// Function to convert angle to PWM value
#include <map>

// Define maps to customize setpoints for each servo
std::map<float, int> servobetaAngleToPulseMap = {
    {-90, 500},
    {-45, 1051},
    {0, 1600},
    {45, 2149},
    {90, 2500}
};

std::map<float, int> servothetaAngleToPulseMap = {
    {-90, 500},
    {-45, 900},
    {0, 1300},
    {45, 1700},
    {90, 2200}
};

int angleToPulse(float angle, const std::map<float, int>& angleToPulseMap) {
    // Constrain angle between 0 and 180 degrees
    if (angle < -90) angle = -90;
    if (angle > 90) angle = 90;

    // Find the closest setpoints in the map
    auto lower = angleToPulseMap.lower_bound(angle);
    if (lower == angleToPulseMap.end()) return angleToPulseMap.rbegin()->second;
    if (lower == angleToPulseMap.begin()) return lower->second;

    auto upper = lower--;
    float lowerAngle = lower->first;
    float upperAngle = upper->first;
    int lowerPulse = lower->second;
    int upperPulse = upper->second;

    // Interpolate between the closest setpoints
    return static_cast<int>(lowerPulse + (angle - lowerAngle) * (upperPulse - lowerPulse) / (upperAngle - lowerAngle));
}

// Function to set servo angle
void setServoAngle(gpiod_line* line,int number, float angle) {
    int pulseWidth = angleToPulse(angle, number == 1 ? servothetaAngleToPulseMap : servobetaAngleToPulseMap);
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

gpiod_chip* chip;
gpiod_line* servo1_line;
gpiod_line* servo2_line;

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received. Centering servos and exiting..." << std::endl;

    // Center servos before exit
    setServoAngle(servo1_line, 1, 0);
    setServoAngle(servo2_line, 2, 0);

    // Close GPIO chip
    gpiod_chip_close(chip);

    exit(signum);
}

int main() {
    // Register signal handler
    signal(SIGINT, signalHandler);

    // Open GPIO chip
    chip = gpiod_chip_open_by_name("gpiochip0");
    if (!chip) {
        std::cerr << "Failed to open gpiochip0" << std::endl;
        return 1;
    }

    // Get GPIO lines
    servo1_line = gpiod_chip_get_line(chip, SERVO1_PIN);
    servo2_line = gpiod_chip_get_line(chip, SERVO2_PIN);
    if (!servo1_line || !servo2_line) {
        std::cerr << "Failed to get GPIO lines" << std::endl;
        gpiod_chip_close(chip);
        return 1;
    }

    // Set initial position (90 degrees)
    setServoAngle(servo1_line, 1, 0);
    setServoAngle(servo2_line, 2, 0);

    // Main control loop
    while (true) {
        float theta, beta;

        // Get user input for angles
        std::cout << "Enter theta angle ((-90)-90): ";
        std::cin >> theta;
        std::cout << "Enter beta angle ((-90)-90): ";
        std::cin >> beta;

        // Check for valid input
        if (std::cin.fail()) {
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            std::cout << "Invalid input. Please enter numbers between -90 and 90." << std::endl;
            continue;
        }

        // Set servo positions
        setServoAngle(servo1_line, 1, theta);
        setServoAngle(servo2_line, 2, beta);

        // Small delay to allow servos to reach position
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Print current positions
        std::cout << "Current positions - Theta: " << theta 
                  << "°, Beta: " << beta << "°" << std::endl;
    }

    return 0;
}
