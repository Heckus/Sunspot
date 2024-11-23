#include <FastLED.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

// LED strip configuration
#define LED_PIN     18
#define NUM_LEDS    60
#define BRIGHTNESS  64
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define UPDATES_PER_SECOND 100

// Create LED array
CRGB leds[NUM_LEDS];

// Helper function for smooth transitions
void fadeToBlackBy(int fadeBy) {
    for(int i = 0; i < NUM_LEDS; i++) {
        leds[i].fadeToBlackBy(fadeBy);
    }
}

// Animation patterns
void rainbow() {
    static uint8_t hue = 0;
    for(int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CHSV(hue + (i * 255 / NUM_LEDS), 255, 255);
    }
    hue++;
}

void breathing(CRGB color) {
    static uint8_t brightness = 0;
    static bool increasing = true;
    
    for(int i = 0; i < NUM_LEDS; i++) {
        leds[i] = color;
        leds[i].fadeToBlackBy(255 - brightness);
    }
    
    if(increasing) {
        brightness++;
        if(brightness == 255) increasing = false;
    } else {
        brightness--;
        if(brightness == 0) increasing = true;
    }
}

void wave(CRGB color) {
    static uint8_t position = 0;
    fadeToBlackBy(20);
    
    float center = sin(position * (M_PI / 128)) * (NUM_LEDS - 1);
    for(int i = 0; i < 3; i++) {
        leds[(int)center + i] = color;
        if((int)center - i >= 0) {
            leds[(int)center - i] = color;
        }
    }
    position++;
}

void meteor(CRGB color) {
    static uint8_t position = 0;
    fadeToBlackBy(64);
    
    for(int i = 0; i < 5; i++) {
        if(position - i >= 0 && position - i < NUM_LEDS) {
            leds[position - i] = color;
            leds[position - i].fadeToBlackBy(i * 50);
        }
    }
    
    position++;
    if(position > NUM_LEDS + 5) position = 0;
}

void twinkle() {
    fadeToBlackBy(10);
    if(random8() < 50) {
        int pos = random16(NUM_LEDS);
        leds[pos] += CRGB(random8(), random8(), random8());
    }
}

// Pattern control class
class PatternController {
public:
    enum Pattern {
        SOLID,
        RAINBOW,
        BREATHING,
        WAVE,
        METEOR,
        TWINKLE
    };
    
    PatternController() : currentPattern(RAINBOW), color(CRGB::Red) {}
    
    void setPattern(Pattern pattern) {
        currentPattern = pattern;
    }
    
    void setColor(CRGB newColor) {
        color = newColor;
    }
    
    void update() {
        switch(currentPattern) {
            case SOLID:
                fill_solid(leds, NUM_LEDS, color);
                break;
            case RAINBOW:
                rainbow();
                break;
            case BREATHING:
                breathing(color);
                break;
            case WAVE:
                wave(color);
                break;
            case METEOR:
                meteor(color);
                break;
            case TWINKLE:
                twinkle();
                break;
        }
        FastLED.show();
    }
    
private:
    Pattern currentPattern;
    CRGB color;
};

int main() {
    // Initialize FastLED
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.clear();
    
    PatternController controller;
    
    std::cout << "LED Strip Controller" << std::endl;
    std::cout << "Available commands:" << std::endl;
    std::cout << "1: Solid Color" << std::endl;
    std::cout << "2: Rainbow" << std::endl;
    std::cout << "3: Breathing" << std::endl;
    std::cout << "4: Wave" << std::endl;
    std::cout << "5: Meteor" << std::endl;
    std::cout << "6: Twinkle" << std::endl;
    std::cout << "q: Quit" << std::endl;
    
    char input;
    bool running = true;
    
    while(running) {
        if(std::cin.get(input)) {
            switch(input) {
                case '1':
                    controller.setPattern(PatternController::SOLID);
                    controller.setColor(CRGB::Red);
                    break;
                case '2':
                    controller.setPattern(PatternController::RAINBOW);
                    break;
                case '3':
                    controller.setPattern(PatternController::BREATHING);
                    controller.setColor(CRGB::Blue);
                    break;
                case '4':
                    controller.setPattern(PatternController::WAVE);
                    controller.setColor(CRGB::Green);
                    break;
                case '5':
                    controller.setPattern(PatternController::METEOR);
                    controller.setColor(CRGB::Purple);
                    break;
                case '6':
                    controller.setPattern(PatternController::TWINKLE);
                    break;
                case 'q':
                    running = false;
                    break;
            }
        }
        
        controller.update();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / UPDATES_PER_SECOND));
    }
    
    // Clear LEDs before exit
    FastLED.clear();
    FastLED.show();
    
    return 0;
}
