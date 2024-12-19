#include "buttons.h"


// CapacitiveButton class implementation

CapacitiveButton::CapacitiveButton(int sendPin, long thresholdValue)
    : pinSend(sendPin), threshold(thresholdValue) {
}


void CapacitiveButton::printSensorValue() {
    long sensorValue = touchRead(pinSend);
    Serial.print("Sensor value: ");
    Serial.println(sensorValue);
}

bool CapacitiveButton::isPressed() {
    long sensorValue = touchRead(pinSend);
    return sensorValue > threshold;
}

// RegularButton class implementation

RegularButton::RegularButton(int buttonPin) : pin(buttonPin), lastState(LOW) {
    pinMode(pin, INPUT);
}

bool RegularButton::isPressed() {
    bool currentState = digitalRead(pin);
    if (currentState == HIGH && lastState == LOW) {
        lastState = HIGH;
        return true;
    } else if (currentState == LOW) {
        lastState = LOW;
    }
    return false;
}
