#include "buttons.h"

#include <CapacitiveSensor.h>

// CapacitiveButton class implementation

CapacitiveButton::CapacitiveButton(int sendPin, int receivePin, long thresholdValue)
    : pinSend(sendPin), pinReceive(receivePin), threshold(thresholdValue) {
    sensor = new CapacitiveSensor(pinSend, pinReceive);
}

CapacitiveButton::~CapacitiveButton() {
    delete sensor;
}

void CapacitiveButton::printSensorValue() {
    long sensorValue = sensor->capacitiveSensor(30);
    Serial.print("Sensor value: ");
    Serial.println(sensorValue);
}

bool CapacitiveButton::isPressed() {
    long sensorValue = sensor->capacitiveSensor(30);
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
