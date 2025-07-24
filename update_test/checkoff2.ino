#include <FastLED.h>                // Library to control WS2812B LED matrix
#include <arduinoFFT.h>            // Library for performing FFT (Fast Fourier Transform)
#include <driver/i2s.h>            // I2S driver for audio input on ESP32
#include <math.h>
#include "BluetoothSerial.h"       // Bluetooth communication for control
#include <array>  

#include "../disco_peripherals.h"
#include "../disco_modules.h"
#include "../disco_display_modules.h"

uint8_t BRIGHTNESS        =5;     // Initial Brightness

CRGB leds[NUM_LEDS],                          // LED matrix buffer
     colors[MATRIX_HEIGHT];                   // CRGB color gradient array
     
double vReal[SAMPLES];                        // Real part of FFT input
double vImag[SAMPLES];                        // Imaginary part of FFT input
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);                       // FFT object

// Bluetooth serial for remote control
BluetoothSerial ESP_BT;

// Beat detection variables
double prevLowEnergy = 0;
unsigned long lastBeatTime = 0;
int estimatedBPM = 120;           // Estimated beats per minute
int dynamicFrameInterval = 25;    // Controls how frequently LED updates (adaptive)

auto logBins = logspace(2, SAMPLES/2, MATRIX_WIDTH+1);


// =============================================================================
// Initialize system
// =============================================================================
void setup() {

  setupXYTable();
  ESP_BT.begin("ESP-Controller");    // Start Bluetooth for control input
  Serial.begin(115200);
  setupI2S();                        // Initialize audio input

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();

  pinMode(INCREASE_BRIGHTNESS, INPUT_PULLUP);
  pinMode(DECREASE_BRIGHTNESS, INPUT_PULLUP);
  pinMode(BLUETOOTH_LED, OUTPUT);
}

// =============================================================================
// Main Loop
// =============================================================================
void loop() {

  // ---------------------------------------
  // Debounce Buttons
  // ---------------------------------------
  bool isFrameFast;
  isFrameFast = frameInterval(BRIGHTNESS);

  if(isFrameFast) return;

  // ---------------------------------------
  // Read incoming Bluetooth command
  // ---------------------------------------
  static uint8_t mode = 3;
  static CRGB colorFill = CRGB(0,0,255);
  if(ESP_BT.available()) readBluetooth(ESP_BT, mode, colorFill);

  // ---------------------------------------
  // Handle different display modes
  // ---------------------------------------
  if (mode == 1) {

    fill_solid(leds, NUM_LEDS, colorFill);
  }

  // ..............................
  // Mode 2: Color Wave
  // ..............................
  else if (mode == 2) {
    FastLED.clear();

    static int offset = -1;
    offset++;
    for (int y = 0; y < MATRIX_HEIGHT; y++) {
      for (int x = 0; x < MATRIX_WIDTH; x++) {
        float wave = sin((x + offset) * 0.3 + millis() * 0.005);
        uint8_t brightness = map(wave * 100, -100, 100, 50, 255);
        leds[xyToIndex(x, y)] = CHSV((x * 10 + offset) % 255, 255, brightness);
      }
    }
  }

  // ..............................
  // Audio reactive FFT visualization
  // ..............................
  else if (mode == 3) {

    I2S_FFT_data(FFT, vReal, vImag);
    
    // Attenuate high-frequency bins
    for (int i = 3; i < 36; i++) {
      vReal[i] *= 0.2;
    }

    // Beat detection from low-frequency energy
    double lowFreqEnergy = 0;
    for (int i = 1; i <= 10; i++) {
      lowFreqEnergy += vReal[i];
    }
    lowFreqEnergy /= 10.0;

    unsigned long now = millis();
    if (lowFreqEnergy > prevLowEnergy * 1.5 && now - lastBeatTime > 300) {
      // Beat detected
      unsigned long interval = now - lastBeatTime;
      lastBeatTime = now;
      estimatedBPM = 60000 / interval;
      dynamicFrameInterval = constrain(interval / 4, 15, 60);  // Adapt update rate
    }

    // Smooth energy to avoid jitter
    prevLowEnergy = 0.8 * prevLowEnergy + 0.2 * lowFreqEnergy;

    // Visualize the FFT bands on the LED matrix
    displayReactiveBands(vReal, CRGB::White, leds);
  }

  // ..............................
  // Mode 4: Pulse in sync with low frequencies
  // ..............................
  else if (mode == 4) {
      
    I2S_FFT_data(FFT, vReal, vImag);

    // Compute low frequency average
    double lowEnergy = 0;
    for (int i = 1; i <= 10; i++) {
      lowEnergy += vReal[i];
    }
    lowEnergy /= 10.0;

    // Smooth the pulse to avoid flickering
    static double smoothed = 0;
    smoothed = 0.8 * smoothed + 0.2 * lowEnergy;

    // Map to brightness or saturation
    uint8_t pulseVal = constrain(smoothed * 100, 10, 255);
    CRGB color = CHSV((millis() / 10) % 255, 255, pulseVal);

    fill_solid(leds, NUM_LEDS, color);
  }
    
  // ..............................
  // Mode 5: Ripple Beat
  // ..............................
  else if (mode == 5) {
      
    static int rippleRadius = -1;
    static unsigned long rippleTime = 0;

    if (millis() - rippleTime > 300 && prevLowEnergy * 1.5 < vReal[2]) {
      rippleRadius = 0;
      rippleTime = millis();
    }

    fill_solid(leds, NUM_LEDS, CRGB::Black);
    if (rippleRadius >= 0) {
      for (int y = 0; y < MATRIX_HEIGHT; y++) {
        for (int x = 0; x < MATRIX_WIDTH; x++) {
          int dx = x - MATRIX_WIDTH / 2;
          int dy = y - MATRIX_HEIGHT / 2;
          int dist = sqrt(dx * dx + dy * dy);
          if (dist == rippleRadius) {
            leds[xyToIndex(x, y)] = CHSV((millis() / 5) % 255, 255, 255);
          }
        }
      }
      rippleRadius++;
      if (rippleRadius > max(MATRIX_WIDTH, MATRIX_HEIGHT)) rippleRadius = -1;
    }
  }

  // ..............................
  // Mode 6: Fire Glow (bass-driven flicker)
  // ..............................
  else if (mode == 6) {
      
    for (int y = 0; y < MATRIX_HEIGHT; y++) {
      for (int x = 0; x < MATRIX_WIDTH; x++) {
        float bass = vReal[2] * 10;
        uint8_t heat = random8(constrain((int)(bass * 255), 30, 255));
        leds[xyToIndex(x, y)] = CHSV(map(heat, 0, 255, 0, 40), 255, heat);
      }
    }
  }

  // ..............................
  // Mode 7: Energy Snake
  // ..............................
  else if (mode == 7) {
      
    static int headX = 0;
    static int direction = 1;
    double amp = 0;
    for (int i = 3; i < 10; i++) amp += vReal[i];
    amp /= 7.0;
    int trailLength = constrain((int)(amp * 10), 1, MATRIX_WIDTH);

    fill_solid(leds, NUM_LEDS, CRGB::Black);
    for (int t = 0; t < trailLength; t++) {
      int x = headX - t * direction;
      if (x >= 0 && x < MATRIX_WIDTH) {
        for (int y = 0; y < MATRIX_HEIGHT; y++) {
          leds[xyToIndex(x, y)] = CHSV((millis() / 5 + y * 5) % 255, 255, 255 - t * 20);
        }
      }
    }

    headX += direction;
    if (headX >= MATRIX_WIDTH || headX < 0) {
      direction *= -1;
      headX += direction;
    }
  }

  // ..............................
  // Mode 8: Gradient Spectrum Reactive
  // ..............................
  else if (mode == 8) {
    
    I2S_FFT_data(FFT, vReal, vImag);

    // Could also heavily suppress ambient noise via bins 1 and 2 (perhaps just bin 1)
    // vReal[1] *= 0.8;
    // vReal[2] *= 0.8;

    // Suppress vocal range (300–3400 Hz)
    for (int i = 3; i < 36; i++) {
      vReal[i] *= 0.2;  // Reduce vocal band magnitude
    }

    displayWaveform(vReal, CRGB::White, leds);
  }

  // if(BRIGHTNESS>100){
  //   BRIGHTNESS = 100;
  // }
  FastLED.show();
}



