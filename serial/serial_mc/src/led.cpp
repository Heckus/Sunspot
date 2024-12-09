
#include "led.h"

#include <FastLED.h>
#define Board1pin 7
#define Board2pin 6
#define Board3pin 5

unsigned long previousMillis = 0;
bool ledState = false;
const unsigned long blinkInterval = 500;

// Constructor
LEDBoard::LEDBoard(int pin, int numLedsOnBoard) : 
  numLEDs(numLedsOnBoard),dataPin(pin)
{
  // Dynamically allocate LED array for this board
  boardLEDs = new CRGB[numLEDs];
  
  // Add this board's LEDs to FastLED
  if(dataPin == Board1pin){
    FastLED.addLeds<WS2812B, Board1pin, GRB>(boardLEDs, numLEDs);
  }
  else if(dataPin == Board2pin){
    FastLED.addLeds<WS2812B, Board2pin, GRB>(boardLEDs, numLEDs);
  }
  else if(dataPin == Board3pin){
    FastLED.addLeds<WS2812B, Board3pin, GRB>(boardLEDs, numLEDs);
  }
}

// Destructor to free dynamically allocated memory
LEDBoard::~LEDBoard() {
  delete[] boardLEDs;
}

// Set color of a specific LED
void LEDBoard::setLED(int index, uint8_t r, uint8_t g, uint8_t b) {
  if (index >= 0 && index < numLEDs) {
    boardLEDs[index] = CRGB(r, g, b);
  }
  show();
}

// Overloaded setLED with CRGB input
void LEDBoard::setLED(int index, CRGB color) {
  if (index >= 0 && index < numLEDs) {
    boardLEDs[index] = color;
  }
  show();
}

// Show the current LED states
void LEDBoard::show() {
  FastLED.show();
}

// Clear all LEDs
void LEDBoard::clear() {
  for (int i = 0; i < numLEDs; i++) {
    boardLEDs[i] = CRGB::Black;
  }
  show();
}

void LEDBoard::setBatteryLevel(int percentage) {
  // Constrain percentage to 0-100
  percentage = constrain(percentage, 0, 100);
  
  // Clear the board first
  clear();
  
  // Special handling for very low battery (10% or lower)
  if (percentage <= 10) {
      // Use millis() for non-blocking blink
      unsigned long currentMillis = millis();
      
      if (currentMillis - previousMillis >= blinkInterval) {
          // Save the last time you blinked the LED
          previousMillis = currentMillis;
          
          // Toggle LED state
          ledState = !ledState;
          
          if (ledState) {
              // When ledState is true, turn on the first LED in red
              boardLEDs[0] = CRGB(255, 0, 0);  // Bright red
          } else {
              // When ledState is false, turn off the LED
              boardLEDs[0] = CRGB(0, 0, 0);
          }
      }
      
      show();
      delay(100);// Delay to prevent flickering
      return;  // Exit the function after handling low battery
  }
  
  // Calculate number of LEDs to light up
  int ledsToLight = map(percentage, 0, 100, 1, numLEDs);  // Changed to start from 1
  
  // Color gradient from red (low) to green (high)
  for (int i = 0; i < ledsToLight; i++) {
      // Calculate color gradient based on actual LED position
      float ratio = (float)i / (numLEDs - 1);
      
      // Color transitions:
      // 0-0.5 ratio: Red to Orange
      // 0.5-1 ratio: Orange to Green
      if (ratio <= 0.5) {
          // Red to Orange
          uint8_t r = 255;
          uint8_t g = map(ratio * 2 * 255, 0, 255, 0, 165);
          uint8_t b = 0;
          boardLEDs[i] = CRGB(r, g, b);
      } else {
          // Orange to Green
          uint8_t r = map((ratio - 0.5) * 2 * 255, 0, 255, 165, 0);
          uint8_t g = 255;
          uint8_t b = 0;
          boardLEDs[i] = CRGB(r, g, b);
      }
  }
  show();
}
