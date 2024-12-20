#include <Arduino.h>

void setup() {
    // Initialize serial communication at 115200 baud rate
    Serial.begin(115200);
    while (!Serial) {
        ; // Wait for serial port to connect. Needed for native USB
    }
    Serial.println("ESP32-S3 Serial Communication Example");
    Serial1.begin(115200, SERIAL_8N1, D7, D6);  // For Pi communication using hardware UART1
}

void loop() {
    // Print a message every second
    Serial.println("Hello from ESP32-S3! serial0");
    Serial1.println("Hello from ESP32-S3! serail1");
    delay(1000);
}