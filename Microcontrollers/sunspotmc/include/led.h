#pragma once

// led.h
#ifndef LED_H
#define LED_H

#include <Adafruit_NeoPixel.h>

class LEDBoard {
private:
  Adafruit_NeoPixel* boardLEDs;
  int numLEDs;
  int dataPin;

public:
  LEDBoard(int pin, int numLedsOnBoard);
  ~LEDBoard();

  LEDBoard(const LEDBoard&) = delete;
  LEDBoard& operator=(const LEDBoard&) = delete;

  void setLED(int index, uint8_t r, uint8_t g, uint8_t b);
  void setLED(int index, uint32_t color);
  void show();
  void clear();

  void handleLowBattery();
  uint32_t calculateColorGradient(int index);
  void setBatteryLevel(int percentage);

  uint32_t STC(String color) const; // Add const qualifier
};

#endif // LED_H