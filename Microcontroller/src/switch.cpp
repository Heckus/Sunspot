#include "switch.h"
// 3-Way Switch Reader with Debounce

ThreeWaySwitch::ThreeWaySwitch(int pin) : PIN(pin) {
    pinMode(pin, INPUT);
}

// Calibrate switch thresholds
void ThreeWaySwitch::calibrate() {
    Serial.println("3-Way Switch Calibration");
    Serial.println("Move switch to each position and note the readings");
    
    for (int pos = 1; pos <= 3; pos++) {
        Serial.print("Move switch to position ");
        Serial.println(pos);
        delay(5000);  // Give time to move switch
        
        int totalReading = 0;
        const int SAMPLES = 10;
        
        for (int i = 0; i < SAMPLES; i++) {
            totalReading += analogRead(PIN);
            delay(50);
        }
        
        int avgReading = totalReading / SAMPLES;
        Serial.print("Position ");
        Serial.print(pos);
        Serial.print(" Average Reading: ");
        Serial.println(avgReading);
    }
    
    Serial.println("Calibration complete. Update threshold values in code if needed.");
}

// Read switch position with debounce
int ThreeWaySwitch::getPosition() {
    int reading = analogRead(PIN);
    int currentPosition = -1;

    // Determine switch position based on reading
    if (reading >= THRESHOLD_LOWER_1 && reading <= THRESHOLD_UPPER_1) {
        currentPosition = 1;
    } else if (reading >= THRESHOLD_LOWER_2 && reading <= THRESHOLD_UPPER_2) {
        currentPosition = 2;
    } else if (reading >= THRESHOLD_LOWER_3 && reading <= THRESHOLD_UPPER_3) {
        currentPosition = 3;
    }

    // Debounce logic
    if (currentPosition != lastStablePosition) {
        if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
            lastStablePosition = currentPosition;
        }
        lastDebounceTime = millis();
    }

    return lastStablePosition;
}

// Print current switch state with raw analog value
void ThreeWaySwitch::printState() {
    int pos = getPosition();
    int rawReading = analogRead(PIN);
    
    if (pos == -1) {
        Serial.print("Switch Position: invalid ");
        Serial.print(" | Raw Analog Reading: ");
        Serial.println(rawReading);
    } else {
        Serial.print("Switch Position: ");
        Serial.print(pos);
        Serial.print(" | Raw Analog Reading: ");
        Serial.println(rawReading);
    }
}

// connect one side of switch to 3.3V, the other side to 5v
// both with protection resisitors. connect middle of switch to A0
// and at junction add capacitor to ground to remove noise(long leg on ground)