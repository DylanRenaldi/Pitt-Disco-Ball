
#include <BluetoothSerial.h>

#include "../disco_peripherals.h"
// #include "../disco_modules.h"


BluetoothSerial ESP_BT;

void setup() {
  Serial.begin(115200);
  ESP_BT.begin("ESP-Controller");
  
  pinMode(INCREASE_BRIGHTNESS, INPUT_PULLUP);
  pinMode(DECREASE_BRIGHTNESS, INPUT_PULLUP);

  pinMode(BLUETOOTH_LED, OUTPUT);
}

uint8_t BRIGHTNESS = 5;


void loop() {
  
  static uint8_t diff = 0;
	static uint8_t mode = 7;
	static unsigned long lastf = millis();


  if (frameInterval(diff)) return;
	checkBluetooth(ESP_BT, diff, mode);

	if (millis() - lastf > 5000) {		// every 5 seconds, synchronously
		lastf = millis();
		mode = random(4, 10);						// between 4 and 9
	}

}


bool frameInterval(uint8_t &diff) {

	static unsigned long lastFrame = millis();
	static const uint8_t frameInterval = 25;          		// Target ~40 FPS

	diff = millis() - lastFrame;            			// save diff because it may change in midst of below comparison and cause an erroneous error

	if (diff < frameInterval) return true;
	lastFrame = millis();
	
	return false;
}

void checkBluetooth(BluetoothSerial &ESP_BT, const uint8_t &diff, const uint8_t &mode) {

	static bool connected = false;
	static uint8_t iteration = 0,update = mode;                              // set to 0 and mode, respectively so nothing occurs first iteration
	static uint16_t ON_BUFFER  = 0,
									OFF_BUFFER = 0;

	if (update != mode) {
		OFF_BUFFER = 100 * (iteration > 0 || ON_BUFFER > 0);				// 100 ms if BLUETOOTH_LED is logically on in any facet
		iteration  = update = mode;
	}
	
	if (ESP_BT.hasClient() != connected) {
		connected = !connected;
		ON_BUFFER = 1000;																			// 1000 ms
	}

	if (ON_BUFFER) {	// ON_BUFFER takes priority
		ON_BUFFER = (ON_BUFFER > diff ? ON_BUFFER - diff : 0);

		if (ON_BUFFER != digitalRead(BLUETOOTH_LED))
			digitalWrite(BLUETOOTH_LED, ON_BUFFER > 0);

	} else if (OFF_BUFFER) {		// necessary between mode-update blink iterations
		// if (digitalRead(BLUETOOTH_LED))
		// 	digitalWrite(BLUETOOTH_LED, LOW);

		OFF_BUFFER = (OFF_BUFFER > diff ? OFF_BUFFER - diff : 0);

	} else if (iteration) {
		ON_BUFFER = OFF_BUFFER = 100;
		--iteration;
	}
}

void debounceButtons(uint8_t &BRIGHTNESS, const uint8_t &diff) {

	static uint8_t debounce = 0;

	if(!debounce) {
		uint8_t b = BRIGHTNESS;
		if(digitalRead(INCREASE_BRIGHTNESS) && BRIGHTNESS < 100)    // GPIO17
			++BRIGHTNESS;//FastLED.setBrightness(++BRIGHTNESS);

		else if (digitalRead(DECREASE_BRIGHTNESS) && BRIGHTNESS)    // GPIO16
			--BRIGHTNESS;//FastLED.setBrightness(--BRIGHTNESS);

		if(b != BRIGHTNESS){
			Serial.println(BRIGHTNESS);
			debounce = 100;
		}
	} else {
		debounce = (debounce >= diff ? debounce - diff : 0);
	}
}
