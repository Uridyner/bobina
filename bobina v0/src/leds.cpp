#include "leds.hpp"

#include "config.hpp"

constexpr uint8_t ledPins[] = {-1, -2};
constexpr size_t numLeds = sizeof(pins) / sizeof(pins[0]);

void initLeds() {
    for (size_t i = 0; i < numLeds; i++) {
        pinMode(ledPins[i], OUTPUT);
    }
}

void showConfig(Config c) {
    
    for (size_t i = 0; i < numLeds; i++) {
        
    }
}