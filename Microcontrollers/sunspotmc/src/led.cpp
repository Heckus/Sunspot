#include "led.h"

unsigned long previousMillis = 0;
bool ledState = false;
const unsigned long blinkInterval = 500;

// Constructor
LEDBoard::LEDBoard(int pin, int numLedsOnBoard) : 
  dataPin(pin), numLEDs(numLedsOnBoard) 
{
  // Dynamically allocate LED array for this board
  boardLEDs = new Adafruit_NeoPixel(numLEDs, dataPin, NEO_GRB + NEO_KHZ800);
  
  // Initialize the NeoPixel library
  boardLEDs->begin();
  boardLEDs->show(); // Initialize all pixels to 'off'
}

// Destructor to free dynamically allocated memory
LEDBoard::~LEDBoard() {
  delete boardLEDs;
}

// Show the current LED states
void LEDBoard::show() {
  boardLEDs->show();
}

// Clear all LEDs
void LEDBoard::clear() {
  for (int i = 0; i < numLEDs; i++) {
    boardLEDs->setPixelColor(i, 0);
  }
  show();
}


// Set color of a specific LED
void LEDBoard::setLED(int index, uint8_t r, uint8_t g, uint8_t b) {
  if (index >= 0 && index < numLEDs) {
    boardLEDs->setPixelColor(index, boardLEDs->Color(r, g, b));
  }
  show();
}

// Set color of a specific LED using a 32-bit color value
void LEDBoard::setLED(int index, uint32_t color) {
  if (index >= 0 && index < numLEDs) {
    boardLEDs->setPixelColor(index, color);
  }
  show();
}


void LEDBoard::handleLowBattery() {
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
  show();
}

uint32_t LEDBoard::calculateColorGradient(int index) {
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

void LEDBoard::setBatteryLevel(int percentage) {
  // Constrain percentage to 0-100
  percentage = constrain(percentage, 0, 100);
  
  // Clear the board first
  clear();
  
  // Special handling for very low battery (10% or lower)
  if (percentage <= 10) {
      handleLowBattery();
      return;  // Exit the function after handling low battery
  }
  
  // Calculate number of LEDs to light up
  int ledsToLight = map(percentage, 0, 100, 1, numLEDs);  // Changed to start from 1
  
  // Color gradient from red (low) to green (high)
  for (int i = 0; i < ledsToLight; i++) {
      boardLEDs->setPixelColor(i, calculateColorGradient(i));
  }
  show();
}

uint32_t LEDBoard::STC(String color) const {
  if (color == "red") {
      return boardLEDs->Color(255, 0, 0);
  } else if (color == "green") {
      return boardLEDs->Color(0, 255, 0);
  } else if (color == "blue") {
      return boardLEDs->Color(0, 0, 255);
  } else if (color == "white") {
      return boardLEDs->Color(255, 255, 255);
  } else if (color == "black") {
      return boardLEDs->Color(0, 0, 0);
  } else if (color == "yellow") {
      return boardLEDs->Color(255, 255, 0);
  } else if (color == "purple") {
      return boardLEDs->Color(128, 0, 128);
  } else if (color == "cyan") {
      return boardLEDs->Color(0, 255, 255);
  } else {
      return boardLEDs->Color(0, 0, 0); // Default to black if color is unknown
  }
}