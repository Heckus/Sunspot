#pragma once
#ifndef BUTTONS_H
#define BUTTONS_H

#include <Arduino.h>

class CapacitiveButton {
private:   
    int pinSend;
    int pinReceive;
    uint16_t threshold;

public:
    CapacitiveButton(int sendPin,long thresholdValue);
    void printSensorValue();
    bool isPressed();
};

class RegularButton {
private:
    int pin;
    bool lastState;

public:
    RegularButton(int buttonPin);
    bool isPressed(); //untested
};

#endif // BUTTONS_H


// CapacitiveButton button(4, 2, 10000);
// CapacitiveButton button2(13, 12, 10000);
// void setup() {
//     Serial.begin(9600);
    
// }

// void loop() {
//     //button.printSensorValue();
//     if (button.isPressed()) {
//         Serial.println("Button1 is pressed!");
//     } 

//     if (button2.isPressed()) {
//         Serial.println("Button2 is pressed!");
//     } 
//     delay(100);
// }