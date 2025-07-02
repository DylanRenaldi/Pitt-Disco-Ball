
#include <FastLED.h>

const uint8_t INCREASE_BRIGHTNESS =27,  // increase brightness button -> GPIO27
              DECREASE_BRIGHTNESS =14;  // decrease brightness button -> GPIO14

uint8_t BRIGHTNESS = 15,
        unpressed  = 1;



void setup() {
  //Serial.begin(115200);

  pinMode(INCREASE_BRIGHTNESS, INPUT_PULLUP);
  pinMode(DECREASE_BRIGHTNESS, INPUT_PULLUP);
}

void loop() {

  
  if(unpressed){

    if(!digitalRead(INCREASE_BRIGHTNESS) && BRIGHTNESS < 100)    // GPIO27
      FastLED.setBrightness(++BRIGHTNESS);//, Serial.println(BRIGHTNESS);

    else if (!digitalRead(DECREASE_BRIGHTNESS) && BRIGHTNESS > 1)  // GPIO14
      FastLED.setBrightness(--BRIGHTNESS);//, Serial.println(BRIGHTNESS);

  }


  unpressed = digitalRead(INCREASE_BRIGHTNESS) && digitalRead(DECREASE_BRIGHTNESS);
}
