#include <Arduino.h>
void setup() {
    Serial.begin(9600);
    while (!Serial) {
        // Wait for serial connection
    }
    Serial.println("Arduino ready.");
}

void loop() {
    // Check if data is available from Raspberry Pi
    if (Serial.available() > 0) {
        String message = Serial.readStringUntil('\n'); // Read until newline
        Serial.print("I got this message: ");
        Serial.println(message); // Send acknowledgment
    }
}
