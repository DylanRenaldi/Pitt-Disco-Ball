
/* -------- REQUIRED EXTERNAL LIBRARIES --------

  - FastLED.h         <- https://github.com/FastLED/FastLED
  - arduinoFFT.h      <- https://github.com/kosme/arduinoFFT

*/

#include <disco.hpp>


// =============================================================================
// Initialize system
// =============================================================================
void setup() {

  //Serial.begin(115200);
  disco::init();
}



// =============================================================================
// Main Loop
// =============================================================================
void loop() {

  // ---------------------------------------
  // Check Framerate and Debounce Buttons
  // ---------------------------------------
  static uint8_t diff = 0,mode = 3;
  if (disco::frameInterval(diff)) return;
  disco::debounceButtons(diff);

  // ---------------------------------------
  // Check if Bluetooth is connected to display the Bluetooth LED
  // ---------------------------------------
  disco::checkBluetooth(diff, mode);

  if (disco::ESP_BT.available()) {
    disco::readBluetooth(mode, false);
  }
  
  switch (mode) {

    case 1: {
      
      discoModes::solidColorFill();
      break;
    }

    case 2: {

      discoModes::colorWave();
      break;
    }

    case 3: {

      discoModes::audioReactiveVisualizer();
      break;
    }

    case 4: {

      discoModes::lowFrequencyPulseSync();
      break;
    }
    
    case 5: {

      discoModes::rippleBeat();
      break;
    }

    case 6: {
      
      discoModes::fireGlow();
      break;
    }

    case 7: {
      
      discoModes::energySnake();
      break;
    }
  }
  
  disco::show();
}