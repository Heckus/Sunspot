#pragma once // include guard
#ifndef SWITCH_H
#define SWITCH_H

#include <Arduino.h>

class ThreeWaySwitch {
private:
    const int PIN;
    const int THRESHOLD_LOWER_1 = 600;
    const int THRESHOLD_UPPER_1 = 750;
    const int THRESHOLD_LOWER_2 = 0;
    const int THRESHOLD_UPPER_2 = 599;
    const int THRESHOLD_LOWER_3 = 900;
    const int THRESHOLD_UPPER_3 = 1023;

    const unsigned long DEBOUNCE_DELAY = 50;
    int lastStablePosition = -1;
    unsigned long lastDebounceTime = 0;

public:
    ThreeWaySwitch(int pin);
    void calibrate();
    int getPosition();
    void printState();
};

#endif // SWITCH_H

// Pin definition
// const int SWITCH_PIN = A0;

// // Create switch object
// ThreeWaySwitch threeWaySwitch(SWITCH_PIN);

// void setup() {
//     Serial.begin(9600);
    
//     // Optional: Uncomment to run calibration
//     // threeWaySwitch.calibrate();
// }

// void loop() {
//     // Read and print switch state
//     threeWaySwitch.printState();
    
//     delay(1000);  // Small delay to prevent serial flooding
// }
