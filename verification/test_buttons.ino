
#include <FastLED.h>

const uint8_t MATRIX_HEIGHT       =32,
              MATRIX_WIDTH        =32;

const uint8_t NUM_LEDS            =MATRIX_HEIGHT*MATRIX_WIDTH,
              DATA_PIN            =5,
              INCREASE_BRIGHTNESS =27,  // increase brightness button -> GPIO27
              DECREASE_BRIGHTNESS =14,  // decrease brightness button -> GPIO14
              INCREASE_HEIGHT     =0,   // increase height of bar -> GPIO0
              DECREASE_HEIGHT     =2;   // decrease height of bar -> GPIO2

uint8_t BRIGHTNESS        = 15,
        HEIGHT            = 0,
        BRIGHT_UNPRESSED  = 1,
        HEIGHT_UNPRESSED  = 1;

CRGB leds[NUM_LEDS];

void setup() {
  Serial.begin(115200);
  
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();

  pinMode(INCREASE_BRIGHTNESS, INPUT_PULLUP);
  pinMode(DECREASE_BRIGHTNESS, INPUT_PULLUP);

  pinMode(INCREASE_HEIGHT, INPUT_PULLUP);
  pinMode(DECREASE_HEIGHT, INPUT_PULLUP);
}

void loop() {

  
  if(BRIGHT_UNPRESSED){

    if(!digitalRead(INCREASE_BRIGHTNESS) && BRIGHTNESS < 255)    // GPIO27
      FastLED.setBrightness(++BRIGHTNESS);

    else if (!digitalRead(DECREASE_BRIGHTNESS) && BRIGHTNESS)    // GPIO14
      FastLED.setBrightness(--BRIGHTNESS);
      
    Serial.println(BRIGHTNESS);
  }

  if(HEIGHT_UNPRESSED){

    if(!digitalRead(INCREASE_HEIGHT) && HEIGHT < MATRIX_HEIGHT)    // GPIO0
      fhline(++HEIGHT - 1, CRGB(255,255,255));

    else if (!digitalRead(DECREASE_HEIGHT) && HEIGHT)              // GPIO2
      fhline(HEIGHT-- - 1, CRGB(0,0,0));
  }


  BRIGHT_UNPRESSED = digitalRead(INCREASE_BRIGHTNESS) && digitalRead(DECREASE_BRIGHTNESS);
  HEIGHT_UNPRESSED = digitalRead(INCREASE_HEIGHT) && digitalRead(DECREASE_HEIGHT);
  FastLED.show();
}

void fhline(uint8_t y, CRGB clr){
  for(uint8_t x = 0; x < 32; ++x)
    leds[xyToIndex(x, y)] = clr;
}


uint16_t xyToIndex(uint8_t x, uint8_t y){
	uint16_t m = 256*(x < 16);
	
	if(y&1)
		return (271 + 16*y - x%16)^m;
	
	return (256 + 16*y + x%16)^m;
}
