
/* -------- REQUIRED EXTERNAL LIBRARIES --------

  - FastLED.h         <- https://github.com/FastLED/FastLED
  - arduinoFFT.h      <- https://github.com/kosme/arduinoFFT

*/

#include <disco.hpp>

// =============================================================================
// Initialize system
// =============================================================================
void setup() {

  // Serial.begin(115200);
  disco::init();         // initialize disco mode to 3
}


// =============================================================================
// Main Loop
// =============================================================================
void loop() {

  // ---------------------------------------
  // Check Framerate and Debounce Buttons
  // ---------------------------------------
  static uint8_t diff = 0;
  static bool update = false;
  if (disco::frameInterval(diff)) return;
  disco::debounceButtons(diff);

  // ---------------------------------------
  // Check if Bluetooth is connected to display the Bluetooth LED
  // ---------------------------------------
  disco::checkBluetooth(diff);

  if (dlv::ESP_BT.available()) {
    disco::readBluetooth(update, false);
  }
  
  disco::update(diff, update);
}
