#include "led.h"

unsigned long previousMillis = 0;
const long blinkInterval = 1000; // Interval at which to blink (milliseconds)
bool ledState = false;
const int numLEDs = 8; // Number of LEDs on the board

uint32_t stringToColor(Adafruit_NeoPixel* boardLEDs, const String& color) {
  struct ColorMap {
    const char* name;
    uint8_t r, g, b;
  };
  
  static const ColorMap colors[] = {
    {"red",     255, 0,   0  },
    {"green",   0,   255, 0  },
    {"blue",    0,   0,   255},
    {"white",   255, 255, 255},
    {"black",   0,   0,   0  },
    {"yellow",  255, 255, 0  },
    {"purple",  128, 0,   128},
    {"cyan",    0,   255, 255}
  };
  
  for (const auto& c : colors) {
    if (color == c.name) {
      return boardLEDs->Color(c.r, c.g, c.b);
    }
  }
  
  return boardLEDs->Color(0, 0, 0); // Default to black
}


void setLEDColor(Adafruit_NeoPixel* boardLEDs, int index, String color){
  uint32_t colorValue = stringToColor(boardLEDs, color);
  boardLEDs->setPixelColor(index, colorValue);
  boardLEDs->show();
}

void handleLowBattery(Adafruit_NeoPixel* boardLEDs) {
  // Use millis() for non-blocking blink
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= blinkInterval) {
      // Save the last time you blinked the LED
      previousMillis = currentMillis;
      
      // Toggle LED state
      ledState = !ledState;
      
      if (ledState) {
          // When ledState is true, turn on the first LED in red
          boardLEDs->setPixelColor(0, boardLEDs->Color(255, 0, 0));  // Bright red
      } else {
          // When ledState is false, turn off the LED
          boardLEDs->setPixelColor(0, 0);
      }
  }
  boardLEDs->show();
}

uint32_t calculateColorGradient(Adafruit_NeoPixel* boardLEDs, int index) {
  float ratio = (float)index / (numLEDs - 1);
  
  // Color transitions:
  // 0-0.5 ratio: Red to Orange
  // 0.5-1 ratio: Orange to Green
  if (ratio <= 0.5) {
      // Red to Orange
      uint8_t r = 255;
      uint8_t g = map(ratio * 2 * 255, 0, 255, 0, 165);
      uint8_t b = 0;
      return boardLEDs->Color(r, g, b);
  } else {
      // Orange to Green
      uint8_t r = map((ratio - 0.5) * 2 * 255, 0, 255, 165, 0);
      uint8_t g = 255;
      uint8_t b = 0;
      return boardLEDs->Color(r, g, b);
  }
}

void setBatteryLevel(Adafruit_NeoPixel* boardLEDs, int percentage) {
  // Constrain percentage to 0-100
  percentage = constrain(percentage, 0, 100);
  
  // Clear the board first
  boardLEDs->clear();
  
  // Special handling for very low battery (10% or lower)
  if (percentage <= 10) {
      handleLowBattery(boardLEDs);
      return;  // Exit the function after handling low battery
  }
  
  // Calculate number of LEDs to light up
  int ledsToLight = map(percentage, 0, 100, 1, numLEDs);  // Changed to start from 1
  
  // Color gradient from red (low) to green (high)
  for (int i = 0; i < ledsToLight; i++) {
      boardLEDs->setPixelColor(i, calculateColorGradient(boardLEDs, i));
  }
  boardLEDs->show();
}


