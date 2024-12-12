#pragma once

// led.h
#ifndef LED_H
#define LED_H

#include <FastLED.h>

class LEDBoard {
private:
  CRGB* boardLEDs;
  int numLEDs;
  int dataPin;

public:
  LEDBoard(int pin, int numLedsOnBoard);
  ~LEDBoard();

  LEDBoard(const LEDBoard&) = delete;
  LEDBoard& operator=(const LEDBoard&) = delete;

  void setLED(int index, uint8_t r, uint8_t g, uint8_t b);
  void setLED(int index, CRGB color);
  void show();
  void clear();

  void handleLowBattery();
  CRGB calculateColorGradient(int index);
  void setBatteryLevel(int percentage);

static CRGB STCRGB(String color);
};

#endif // LED_H



// EXAMPLE USAGE

//   // Create LED boards
//   LEDBoard statusBoard(7, 1);   // 16 LEDs on pin 3
//   LEDBoard batteryBoard(6, 8);   // 8 LEDs on pin 2


// void setup() {
//   // Initialize FastLED global settings
//   FastLED.setBrightness(50);  // Set overall brightness

//   // Demonstrate individual LED control
//   statusBoard.setLED(0, 255, 255, 255);    // Set first LED to red
//   delay(1000);
  
// }

// void loop() {
//   // Simulate battery level changes for demonstration
  
//     batteryBoard.setBatteryLevel(10);
    

// }
