if these header files are contained within the same directory as the .ino file, the include statement would be as follows:

`#include "disco_... .h"`

if the header flies are contained one folder outside the directory of the .ino file, the include statement would be as follows:

`#include "../disco... .h"`

etc.          

<muy importante!!!!>        

include the headers in the following order:
`  disco_peripherals.h -> disco_modules.h -> disco_display_modules.h  `        

`disco_display_modules.h` uses functions and globals from `disco_modules.h` and `disco_peripherals.h`, respectively.        
`disco_modules.h` uses globals from `disco_peripherals.h`        

## modules changes:          
xyIndexTable is created in the disco_modules.h header so it is in the same global scope of the xyToIndex function and the setupXYTable function. That way, when calling the xyToIndex function, only the x,y parameters are required, instead of needing to have the table as a parameter since these modules are included in the header file.

# Frame interval & brightness button check
the syntax and such for calling this method could be changed via preference        
`bool frameInterval(uint8_t &)`
```
// Debounce Buttons
bool isFrameFast;
isFrameFast = frameInterval(BRIGHTNESS);

if(isFrameFast) return;
```

# bluetooth reading module        
`void readBluetooth(BluetoothSerial &ESP_BT, uint8_t &mode, CRGB &colorFill)`        
colorFill is used in mode 1
```
// Read incoming Bluetooth command
static uint8_t mode = 3;
static CRGB colorFill = CRGB(0,0,255);
if(ESP_BT.available()) readBluetooth(ESP_BT, mode, colorFill);
```

# i2s fft data module          
`void I2S_FFT_data(aruinoFFT<double> &FFT, CRGB peakColor, CRGB *leds)`          
`I2S_FFT_data(FFT, CRGB::White, leds);`          
