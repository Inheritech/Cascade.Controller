#include <FastLED.h>

class LEDStrip {
public:
    LEDStrip(int size);
    int getSize();
    CRGB* getLeds();
    void setLed(int index, CRGB color);
private:
    CRGB* leds;
    int size;
};