// will blink the BLUETOOTH_LED light for 1 second if the system is either connected or disconnected
// further logic will be implemented to indicate modes:
// 3-second blink: your bluetooth device has connected
// 3x 1-second blinks:  your bluetooth device has disconnected
// 1 second blink:  bluetooth device has successfully updated mode
// etc



// #include "../disco_peripherals.h"  -> #define BLUETOOTH_LED 27

#define BLUETOOTH_LED 27

void setup() {
  Serial.begin(115200);
  
  pinMode(BLUETOOTH_LED, OUTPUT);
}

void loop() {

  static int8_t connected = false;
  static uint16_t BLUETOOTH_BUFFER = 0;
  static int lastFrame = millis(), diff;

  diff = millis() - lastFrame;
  lastFrame = millis();

  if(BLUETOOTH_BUFFER) {
    if(!digitalRead(BLUETOOTH_LED))
      digitalWrite(BLUETOOTH_LED, HIGH);

    if(BLUETOOTH_BUFFER > diff)
      BLUETOOTH_BUFFER -= diff;
    else {
      BLUETOOTH_BUFFER = 0;
      digitalWrite(BLUETOOTH_LED, LOW);
    }
  } 

  if (isClient(millis())) {
    if (!connected) {
      connected = true;
      BLUETOOTH_BUFFER = 1000;      // 1000ms
    }

  } else if (connected) {
    connected = false;
    BLUETOOTH_BUFFER = 1000;        // 1000ms
  }
}

// generic isClient function to resemble BluetoothSerial isClient function
bool isClient(int milis){
  return (milis/5000)&1;    // every 5000 milliseconds
}
