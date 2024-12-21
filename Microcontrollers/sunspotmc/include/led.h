#pragma once

// led.h
#ifndef LED_H
#define LED_H

#include <Adafruit_NeoPixel.h>
void handleLowBattery(Adafruit_NeoPixel* boardLEDs);
uint32_t calculateColorGradient(Adafruit_NeoPixel* boardLEDs, int index);
void setBatteryLevel(Adafruit_NeoPixel* boardLEDs, int percentage);
void setLEDColor(Adafruit_NeoPixel* boardLEDs, int index, String color);
uint32_t stringToColor(Adafruit_NeoPixel* boardLEDs, const String& color);
#endif // LED_H