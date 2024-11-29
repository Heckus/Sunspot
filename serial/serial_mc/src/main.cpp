#include <Arduino.h>

// Configuration
const long BAUD_RATE = 115200;
const byte START_MARKER = 0x7E;  // '~' character
const byte END_MARKER = 0x7F;    // '' character

void setup() {
    Serial.begin(BAUD_RATE);
}

// Function to encode 32-bit message
// Bit breakdown:
// Bits 0-6: Servo 1 Angle (0-127 degrees)
// Bits 7-13: Servo 2 Angle (0-127 degrees)
// Bits 14-20: Some other parameter
// Bits 21-31: Reserved/Checksum
uint32_t encodeMessage(uint8_t servo1Angle, uint8_t servo2Angle, uint8_t param) {
    uint32_t message = 0;
    message |= (servo1Angle & 0x7F);  // First 7 bits
    message |= (servo2Angle & 0x7F) << 7;  // Next 7 bits
    message |= (param & 0x7F) << 14;  // Next 7 bits
    
    // Optional: Add simple checksum
    uint8_t checksum = servo1Angle ^ servo2Angle ^ param;
    message |= (checksum & 0x7FF) << 21;
    
    return message;
}

void sendMessage(uint32_t message) {
    Serial.write(START_MARKER);
    
    // Send 32-bit message as 4 bytes
    Serial.write((message >> 24) & 0xFF);
    Serial.write((message >> 16) & 0xFF);
    Serial.write((message >> 8) & 0xFF);
    Serial.write(message & 0xFF);
    
    Serial.write(END_MARKER);
}

void loop() {
    // Example: Send servo angles and a parameter
    uint32_t message = encodeMessage(45, 90, 25);
    sendMessage(message);
    
    delay(100);  // Adjust transmission frequency as needed
}