#include <ws2811.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

// LED strip configuration
#define LED_COUNT 1
#define LED_PIN 18
#define LED_FREQ_HZ 800000
#define LED_DMA 10
#define LED_BRIGHTNESS 65
#define LED_CHANNEL 0

ws2811_t ledstrip = {
    .freq = LED_FREQ_HZ,
    .dmanum = LED_DMA,
    .channel = {
        [0] = {
            .gpionum = LED_PIN,
            .invert = 0,
            .count = LED_COUNT,
            .brightness = LED_BRIGHTNESS,
        }
    }
};

// Initialize the LED strip
int init_led_strip() {
    ws2811_return_t ret;
    
    if ((ret = ws2811_init(&ledstrip)) != WS2811_SUCCESS) {
        fprintf(stderr, "ws2811_init failed: %s\n", ws2811_get_return_t_str(ret));
        return -1;
    }
    
    return 0;
}

// Helper function to set color for all LEDs
void color_wipe(uint32_t color) {
    for(int i = 0; i < LED_COUNT; i++) {
        ledstrip.channel[0].leds[i] = color;
        ws2811_render(&ledstrip);
        usleep(50000); // 50ms delay
    }
}

// Color helper functions
uint32_t create_color(uint8_t r, uint8_t g, uint8_t b) {
    return (uint32_t)((r << 16) | (g << 8) | b);
}

// LED control functions
void led_red() {
    color_wipe(create_color(255, 0, 0));
}

void led_blue() {
    color_wipe(create_color(0, 0, 255));
}

void led_green() {
    color_wipe(create_color(0, 255, 0));
}

void led_yellow() {
    color_wipe(create_color(255, 255, 0));
}

void led_clear() {
    color_wipe(create_color(0, 0, 0));
}

// Cleanup function
void cleanup_led_strip() {
    ws2811_fini(&ledstrip);
}

// Example usage
int main() {
    if (init_led_strip() == -1) {
        return -1;
    }

    printf("Testing LED strip...\n");
    
    // Test red color
    printf("Setting RED color\n");
    led_red();
    sleep(3);
    
    // Test blue color
    printf("Setting BLUE color\n");
    led_blue();
    sleep(3);
    
    // Clear LEDs before exit
    led_clear();
    
    // Cleanup
    cleanup_led_strip();
    
    return 0;
}
