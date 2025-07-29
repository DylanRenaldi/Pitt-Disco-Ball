
#include "disco.hpp"

CRGB leds[NUM_LEDS];
CRGB colors[MATRIX_HEIGHT];

void setup() {
  disco::init();

  // disco::gradient(colors, CRGB::Aqua, CRGB::Fuchsia);
}

void loop() {
  
  static uint8_t diff = 0,mode = 3;
  if (disco::frameInterval(diff)) return;
  disco::debounceButtons(diff);

  static unsigned long lastf = millis();
  if (millis() - lastf > 5000) {
    lastf = millis();

    mode = random(5, 9);
    disco::checkBluetooth(diff, mode);
  }

  // modes
  // etc.  
}
