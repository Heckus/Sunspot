#include "data.h"

// Constructor
Data::Data() : thetaAngle(0.0), betaAngle(0.0), batteryLevel(100), mode(0) {
    for (int i = 0; i < 3; ++i) {
        buttonStates[i] = false;
    }
}

// Getters
double Data::getThetaAngle() {
    std::lock_guard<std::mutex> lock(mtx);
    return thetaAngle;
}

double Data::getBetaAngle() {
    std::lock_guard<std::mutex> lock(mtx);
    return betaAngle;
}

int Data::getBatteryLevel() {
    std::lock_guard<std::mutex> lock(mtx);
    return batteryLevel;
}

int Data::getMode() {
    std::lock_guard<std::mutex> lock(mtx);
    return mode;
}

bool Data::getButtonState(int buttonIndex) {
    if (buttonIndex < 0 || buttonIndex >= 3) {
        throw std::out_of_range("Button index out of range");
    }
    std::lock_guard<std::mutex> lock(mtx);
    return buttonStates[buttonIndex];
}

// Setters
void Data::setThetaAngle(double angle) {
    std::lock_guard<std::mutex> lock(mtx);
    thetaAngle = angle;
}

void Data::setBetaAngle(double angle) {
    std::lock_guard<std::mutex> lock(mtx);
    betaAngle = angle;
}

void Data::setBatteryLevel(int level) {
    std::lock_guard<std::mutex> lock(mtx);
    batteryLevel = level;
}

void Data::setMode(int mode) {
    std::lock_guard<std::mutex> lock(mtx);
    this->mode = mode;
}

void Data::setButtonState(int buttonIndex, bool state) {
    if (buttonIndex < 0 || buttonIndex >= 3) {
        throw std::out_of_range("Button index out of range");
    }
    std::lock_guard<std::mutex> lock(mtx);
    buttonStates[buttonIndex] = state;
}

// Print function
void Data::printData() {
    std::lock_guard<std::mutex> lock(mtx);
    std::cout << "Theta Angle: " << thetaAngle << std::endl;
    std::cout << "Beta Angle: " << betaAngle << std::endl;
    std::cout << "Battery Level: " << batteryLevel << std::endl;
    std::cout << "Mode: " << mode << std::endl;
    std::cout << "Button States: ";
    for (int i = 0; i < 3; ++i) {
        std::cout << buttonStates[i] << " ";
    }
    std::cout << std::endl;
}