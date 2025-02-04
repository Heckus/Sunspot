// #include <Adafruit_NeoPixel.h>

// #define LED_PIN 7
// #define NUM_LEDS 8
// #define TOUCH_PIN T1 // Corresponding to TOUCH_PAD_NUM1

// Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// bool ledswitch = false;

// void setAllLeds(uint32_t color) {
//     for (int i = 0; i < NUM_LEDS; i++) {
//         strip.setPixelColor(i, color);
//     }
//     strip.show();
// }

// void setup() {
//     //Serial.begin(115200);
//     strip.begin();
//     strip.show(); // Initialize all pixels to 'off'
//     setAllLeds(strip.Color(255, 255, 0)); // Red
// }

// void loop() {
//     uint16_t touchValue = touchRead(TOUCH_PIN);

//     if (touchValue < 500) { // Adjust threshold as needed
//         ledswitch = !ledswitch;
//         if (ledswitch) {
//             setAllLeds(strip.Color(255, 0, 0)); // Red
//         } else {
//             setAllLeds(strip.Color(0, 255, 0)); // Green
//         }
//         delay(500); // Debounce delay
//     }
// }


