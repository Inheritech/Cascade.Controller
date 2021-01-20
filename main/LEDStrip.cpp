#include "LEDStrip.h"

LEDStrip::LEDStrip(int size) : leds(nullptr), size(size) {
    leds = new CRGB[size];
}

int LEDStrip::getSize() {
    return size;
}

CRGB* LEDStrip::getLeds() {
    return leds;
}

void LEDStrip::setLed(int index, CRGB color) {
    if (index >= 0 && index < size) {
        leds[index] = color;
    }
}