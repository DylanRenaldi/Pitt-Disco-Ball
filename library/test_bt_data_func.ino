
#include <disco.hpp>

void setup() {
  Serial.begin(115200);
  disco::init();
}


void loop() {

  static uint8_t diff = 0, mode = 3;
  if (disco::frameInterval(diff)) return;
  disco::debounceButtons(diff);

  disco::checkBluetooth(diff, mode);
  if(disco::ESP_BT.available()) {
    disco::readBluetooth(mode);
  }

}
