#include <assert.h>
#include <math.h>

#include <FastLED.h>

// 0-offset
const uint8_t MATRIX_WIDTH  = 32,
              MATRIX_HEIGHT = 32;

const uint8_t NUM_LEDS = (MATRIX_WIDTH * MATRIX_HEIGHT),
              INCREASE_BRIGHTNESS =27,  // increase brightness button -> GPIO27
              DECREASE_BRIGHTNESS =14;  // decrease brightness button -> GPIO14

#define DATA_PIN 3                   // LED strip data out pin
uint8_t BRIGHTNESS = 15,             // brightness is between 0-255, unsigned 8-bit integer
        unpressed  = 1;

CRGB leds[NUM_LEDS],
     colors[MATRIX_HEIGHT];

void setup() {

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);

  pinMode(INCREASE_BRIGHTNESS, INPUT_PULLUP);
  pinMode(DECREASE_BRIGHTNESS, INPUT_PULLUP);


  // define starting and ending level rgb values
  uint8_t rs = 135, // rgb values on lowest level
          gs = 120,
          bs = 0,

          rx = 255, // rgb values on upper-most level
          gx = 0,
          bx = 0;

  // delta rgb values between lowest and upper-most levels
  float dr = float(rx - rs)/(MATRIX_HEIGHT - 1),
        dg = float(gx - gs)/(MATRIX_HEIGHT - 1),
        db = float(bx - bs)/(MATRIX_HEIGHT - 1),
        r=rs,g=gs,b=bs;


  for(uint8_t i = 0; i < MATRIX_HEIGHT; ++i, r += dr, g += dg, b += db)
    colors[i] = CRGB(r, g, b);

}

void loop() {

  if(unpressed){

    if(!digitalRead(INCREASE_BRIGHTNESS) && BRIGHTNESS < 255)    // GPIO27
      FastLED.setBrightness(++BRIGHTNESS);//, Serial.println(BRIGHTNESS);

    else if (!digitalRead(DECREASE_BRIGHTNESS) && BRIGHTNESS)    // GPIO14
      FastLED.setBrightness(--BRIGHTNESS);//, Serial.println(BRIGHTNESS);

  }


  // ...


  unpressed = digitalRead(INCREASE_BRIGHTNESS) && digitalRead(DECREASE_BRIGHTNESS);
}

//        ->^
// [4][3] ^<-
// [1][2] ->^
uint16_t xyToIndex(uint8_t x, uint8_t y){
	uint16_t m = 256*(x < 16);
	
	if(y&1)
		return (271 + 16*y - x%16)^m;
	
	return (256 + 16*y + x%16)^m;
}

// [4][3] ^->|
// [1][2] |  v->
uint16_t coord2serp(uint8_t x, uint8_t y){
	if(x==15 and y==16)
		return 1023;

	uint16_t m = 768*(y > 15);
	
	if(x&1)
		return (16*x + 15 - y%16)^m;

	return (16*x + y%16)^m;
}

// [1][2] ^->|
// [4][3] |  v->
uint16_t coord2serp(uint8_t x, uint8_t y){
	if(!y && x==15)
		return 1023;

	uint16_t m = 768*(y < 16);
	
	if(x&1)
		return (16*x + 15 - y%16)^m;

	return (16*x + y%16)^m;
}


// [3][4]   |<-^
// [2][1] <-v  |
uint16_t coord2serpentine(uint8_t x, uint8_t y){
  assert(abs(x*y) <= 961);
  uint16_t m = 0;

  if(y > 15){
    if(y == 16 && (x == 15 || x == 31))
      return (x == 15 ? 1023 : 768);
  
    y = y - 16, m = 768;
  }

  if(x&1)
    return (x*16 + 15 - y)^m;

  return (x*16 + y)^m;
}

// x = ?, y = 0, h -> height
uint16_t line(uint8_t x, uint8_t h){

  leds[xyToIndex(x,h)] = HUE_RED; // red
  while(--h)
    leds[xyToIndex(x, h)] = colors[h];

  FastLED.show();
}
