#include <FastLED.h>             // For controlling WS2812B LED strips
#include <arduinoFFT.h>          // For performing FFT on audio data
#include <driver/i2s.h>          // For ESP32 I2S audio input
#include <math.h>                // For logarithmic and math functions
#include "BluetoothSerial.h"     // For Bluetooth communication

// LED matrix and visualization parameters
#define MATRIX_WIDTH      32
#define MATRIX_HEIGHT     32
#define NUM_LEDS          (MATRIX_WIDTH * MATRIX_HEIGHT)
#define DATA_PIN          5
#define BRIGHTNESS        5

// Audio sampling settings
#define SAMPLES           512
#define SAMPLING_FREQ     48000

// I2S pin assignments
#define I2S_WS            25  // Word Select (LRCLK)
#define I2S_SD            33  // Serial Data input
#define I2S_SCK           26  // Bit Clock (BCLK)

// Global variables
CRGB leds[NUM_LEDS];                   // LED matrix buffer
ArduinoFFT<double> FFT;                // FFT object
double vReal[SAMPLES];                 // Real part of FFT input
double vImag[SAMPLES];                 // Imaginary part of FFT input

BluetoothSerial ESP_BT;                // Bluetooth serial object

// Visualization state
double smoothedBands[32] = {0};        // Smoothed FFT band levels
double noiseFloor[32] = {0.05};        // Adaptive noise floor per band
int peakHeights[32] = {0};             // Peak level hold per band
unsigned long lastPeakUpdate[32] = {0}; // Time of last peak update
const int peakHoldTime = 150;          // Time in ms to hold peak before decay
const int peakFallSpeed = 1;           // Amount to reduce peak per update

// Initializes I2S for microphone input
void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLING_FREQ,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 256,
    .use_apll = false
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM_0);
}

// Arduino setup function
void setup() {
  ESP_BT.begin("ESP-Controller");          // Initialize Bluetooth with device name
  Serial.begin(115200);                    // Initialize Serial for debugging
  setupI2S();                              // Initialize I2S microphone input
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS); // Init LED matrix
  FastLED.setBrightness(BRIGHTNESS);       // Set LED brightness
  FastLED.clear();
  FastLED.show();
}

int incoming = 3350;  // Default Bluetooth command value

// Arduino main loop
void loop() {
  static unsigned long lastFrame = 0;
  const int frameInterval = 25; // Target ~40 FPS
  if (millis() - lastFrame < frameInterval) return;
  lastFrame = millis();

  // Read 2-byte command over Bluetooth if available
  if (ESP_BT.available() >= 2) {
    uint8_t highByte = ESP_BT.read();
    uint8_t lowByte = ESP_BT.read();
    incoming = (highByte << 8) | lowByte;
  }

  // Extract mode and color from the command
  int mode = incoming / 1000;           // 1 = solid, 2 = bar, 3 = FFT
  int colorCode = (incoming % 1000) / 100;

  int r = 0, g = 0, b = 0;
  if (colorCode == 1) r = 255;
  else if (colorCode == 2) g = 255;
  else if (colorCode == 3) b = 255;

  switch (mode) {
    case 1:  // Solid color fill
      fill_solid(leds, NUM_LEDS, CRGB(r, g, b));
      FastLED.show();
      break;

    case 2: {  // Moving horizontal bar animation
      static int frame = 0;
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      int y = frame % MATRIX_HEIGHT;
      for (int x = 0; x < MATRIX_WIDTH; x++) {
        leds[xyToIndex(x, y)] = CRGB(r, g, b);
      }
      FastLED.show();
      frame++;
      break;
    }

    case 3: {  // Audio reactive FFT display
      int32_t samples[SAMPLES];
      size_t bytes_read = 0;
      unsigned long startMicros = micros(); // Track latency

      i2s_read(I2S_NUM_0, (char*)samples, SAMPLES * sizeof(int32_t), &bytes_read, portMAX_DELAY);

      // Convert raw I2S samples to normalized floats
      for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = (samples[i] >> 14) / 2048.0;
        vImag[i] = 0.0;
      }

      // Apply windowing and perform FFT
      FFT.windowing(vReal, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward);
      FFT.compute(vReal, vImag, SAMPLES, FFTDirection::Forward);
      FFT.complexToMagnitude(vReal, vImag, SAMPLES);

      // Suppress vocal range (300–3400 Hz)
      for (int i = 3; i < 36 && i < SAMPLES / 2; i++) {
        vReal[i] *= 0.2;  // Reduce vocal band magnitude
      }

      // Display FFT result
      displayReactiveBands(vReal);

      // Print latency
      unsigned long endMicros = micros();
      Serial.print("Audio-to-LED Latency: ");
      Serial.print(endMicros - startMicros);
      Serial.println(" us");
      break;
    }
  }
}

// Visualize FFT result on LED matrix
void displayReactiveBands(double *magnitudes) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].fadeToBlackBy(40);  // Fading trail effect
  }

  // Logarithmically spaced frequency bands
  static const int logBins[33] = {
    1, 2, 3, 4, 5, 6, 7, 9, 11, 13, 15, 18, 21, 24, 28, 32,
    37, 43, 50, 58, 67, 77, 89, 103, 119, 137, 158, 182, 209, 239, 273, 311, 352
  };

  unsigned long now = millis();
  for (int band = 0; band < 32; band++) {
    int startBin = logBins[band];
    int endBin = logBins[band + 1];
    int binCount = endBin - startBin;

    double sum = 0;
    for (int i = startBin; i < endBin && i < SAMPLES / 2; i++) {
      sum += magnitudes[i];
    }
    double avg = sum / binCount;

    // Update adaptive noise floor
    if (avg < noiseFloor[band]) {
      noiseFloor[band] = noiseFloor[band] * 0.95 + avg * 0.05;
    } else {
      noiseFloor[band] = noiseFloor[band] * 0.9995 + avg * 0.0005;
    }

    // Remove noise floor and smooth band value
    double adjusted = avg - noiseFloor[band];
    if (adjusted < 0) adjusted = 0;

    smoothedBands[band] = 0.7 * smoothedBands[band] + 0.3 * adjusted;
    double scaled = log10(smoothedBands[band] + 1) * 10;

    int height = map((int)scaled, 0, 20, 0, MATRIX_HEIGHT);
    height = constrain(height, 0, MATRIX_HEIGHT);

    // Peak hold logic
    if (height >= peakHeights[band]) {
      peakHeights[band] = height;
      lastPeakUpdate[band] = now;
    } else if (now - lastPeakUpdate[band] > peakHoldTime) {
      peakHeights[band] = max(0, peakHeights[band] - peakFallSpeed);
    }

    // Color mapping based on height
    uint8_t hue = map(height, 0, MATRIX_HEIGHT, 160, 0);
    CRGB barColor = CHSV(hue, 255, 255);
    int flippedBand = 31 - band;  // Flip horizontally for visual symmetry

    // Draw vertical bar
    for (int y = 0; y < height; y++) {
      int index = xyToIndex(flippedBand, MATRIX_HEIGHT - 1 - y);
      leds[index] = barColor;
    }

    // Draw peak marker
    if (peakHeights[band] > 0) {
      int peakY = MATRIX_HEIGHT - 1 - peakHeights[band];
      int peakIndex = xyToIndex(flippedBand, peakY);
      leds[peakIndex] = CRGB::White;
    }
  }

  FastLED.show();  // Push LED buffer to display
}

// Converts 2D matrix coordinates to 1D LED index for 4-tile layout
int xyToIndex(int x, int y) {
  const int tileSize = 16;
  int tileX = x / tileSize;
  int tileY = y / tileSize;
  int inTileX = x % tileSize;
  int inTileY = y % tileSize;

  int tileNumber;
  if (tileY == 0) {
    tileNumber = tileX;           // Top row: left to right
  } else {
    tileNumber = 3 - tileX;       // Bottom row: right to left
  }

  int baseIndex = tileNumber * tileSize * tileSize;
  int localIndex;

  // Zigzag layout within tiles
  if (inTileY % 2 == 0) {
    localIndex = inTileY * tileSize + inTileX;
  } else {
    localIndex = inTileY * tileSize + (tileSize - 1 - inTileX);
  }

  return baseIndex + localIndex;
}
