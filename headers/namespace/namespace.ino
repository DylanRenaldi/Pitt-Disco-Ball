
#include "disco.hpp"

void setup() {
  disco::init();
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

// Convert (x, y) coordinate to 1D LED index based on matrix layout
void setxyLEDColor(int x, int y, CRGB color) {
	uint16_t index = disco::xyIndexTable[x][y];
	disco::leds[index] = color;
}
