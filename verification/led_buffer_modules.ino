#include <assert.h>
#include <math.h>

#include <FastLED.h>

// 0-offset
const uint8_t width  = 32;
const uint8_t length = 32;
const uint8_t NUM_LEDS = (width * length);

#define DATA_PIN 3                   // LED strip data out pin
#define BRIGHTNESS 255

CRGB leds[NUM_LEDS];
uint8_t pos = 0;
bool toggle = false;

void setup() {

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);

}

void loop() {

}


uint16_t coord2serpentine(uint8_t x, uint8_t y){
  assert(abs(x*y) <= 961);
  uint16_t m = 0;

  if(y > 15){
    if(y == 16 && (x == 15 || x == 31))
      return (x == 15 ? 1024 : 768);
  
    y = y - 16, m = 768;
  }

  if(x&1)
    return (x*16 + 16 - y)^m;

  return (x*16 + 1 + y)^m;
}

// x = ?, y = 0, h -> height
uint16_t line(uint8_t x, uint8_t h, CHSV clr){

  leds[coord2serpentine(x,h)] = HUE_RED; // red
  while(--h)
    leds[coord2serpentine(x, h)] = clr;

  FastLED.show();
}
