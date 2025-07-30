# To use headers:
(1) Create a folder `Pitt-DiscoLamp` in the `Documents/Arduino/libraries` folder	

(2) push the `library.properties` files in this repo directory to the `Pitt-DiscoLamp` folder	

(3) create `src` folder in the `Pitt-DiscoLamp` folder			

(4) push `disco.cpp`, `disco.hpp`, and `disco.h` to the `src` folder			

		
include `#include <disco.hpp>` at the top of the arduino .ino file

(will need to add comments and parameter stuff to each module definition and implementation)


# NOTE:
`DATA_PIN` was changed to `MATRIX_DATA_PIN` in the disco_peripherals.h file because `DATA_PIN` is declared somewhere generically inside the FastLED library

(recent update compiled properly, has not been fully tested on physical esp32)

     
# variable changes:

- The variables now included in the disco namespace: (referenced via disco::)
  - double vReal, vImag
  - BluetoothSerial ESP_BT
  - CRGB leds, gradient (previously *colors*)
  - ArduinoFFT<double> FFT
  - uint8_t logBins
  - uint16_t xyIndexTable


# modules changes:

## disco::init module
Initializes disco namespace objects
```
void disco::init() {

	disco::FFT = ArduinoFFT<double>(disco::vReal, disco::vImag, SAMPLES, SAMPLING_FREQ);
	disco::ESP_BT.begin("Pitt-DiscoLamp");
	setupI2S();

	FastLED.addLeds<WS2812B, MATRIX_DATA_PIN, GRB>(disco::leds, NUM_LEDS);
  FastLED.setBrightness(10);
	FastLED.clear();

	pinMode(INCREASE_BRIGHTNESS, INPUT_PULLDOWN);
	pinMode(DECREASE_BRIGHTNESS, INPUT_PULLDOWN);
	pinMode(BLUETOOTH_LED, OUTPUT);

	disco::setGradient(CRGB::Aqua, CRGB::Fuchsia);			// -> disco:gradient <- colors
}
```

## check bluetooth client & bluetooth led buffer function
will turn on the bluetooth led if a client connects/disconnects or sends an update.
if the mode is updated, the bluetooth led will blink according to the mode-number
`void disco::checkBluetooth(const uint8_t &diff, const uint8_t &mode) {...}`

## gradient module
Only requires begin and end CRGB values to calculate gradient between CRGB values
`void disco::setGradient(CRGB begin, CRGB end) {...}`


## Frame interval & brightness button check
the syntax and such for calling this method could be changed via preference        
`bool disco::frameInterval(const uint8_t& diff) {...}`
```
static uint8_t diff = 0,mode = 3;
if (disco::frameInterval(diff)) return;      // if frameInterval is too fast
```

`void disco::debounceButtons(const uint8_t& diff) {...}`
```
// after frameInterval check
disco::debounceButtons(diff);
```


## i2s fft data module
FFT, vReal, and vImag are in the disco namespace, and hence aren't required for the function call
since the same operation occurs each call
`void disco::I2S_FFT_data() {...}`       
