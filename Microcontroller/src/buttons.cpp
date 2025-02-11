#include "buttons.h"


// CapacitiveButton class implementation

CapacitiveButton::CapacitiveButton(int sendPin, long thresholdValue)
    : pinSend(sendPin), threshold(thresholdValue) {
}


void CapacitiveButton::printSensorValue(void) {
    long sensorValue = touchRead(pinSend);
    Serial.print("Sensor value: ");
    Serial.println(sensorValue);
}

long CapacitiveButton::returnsensorvalue(void){
    return touchRead(pinSend);
}

bool CapacitiveButton::isPressed(void) {
    uint16_t sensorValue = touchRead(pinSend);
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
