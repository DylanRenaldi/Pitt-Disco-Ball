#include <FastLED.h>
#include <arduinoFFT.h>
#include <driver/i2s.h>
#include <math.h>
#include "BluetoothSerial.h"

// ========================
// Matrix & Audio Settings
// ========================
#define MATRIX_WIDTH      32
#define MATRIX_HEIGHT     32
#define NUM_LEDS          (MATRIX_WIDTH * MATRIX_HEIGHT)
#define DATA_PIN          5
#define BRIGHTNESS        5

#define SAMPLES           512
#define SAMPLING_FREQ     48000

// ========================
// Microphone I2S Pins
// ========================
#define I2S_WS            25
#define I2S_SD            33
#define I2S_SCK           26

// ========================
// Globals
// ========================
CRGB leds[NUM_LEDS];
ArduinoFFT<double> FFT = ArduinoFFT<double>();
double vReal[SAMPLES];
double vImag[SAMPLES];

BluetoothSerial ESP_BT;

// ========================
// Setup I2S Microphone
// ========================
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

// ========================
// Setup
// ========================
void setup() {
  ESP_BT.begin("ESP-Controller");
  Serial.begin(115200);
  setupI2S();

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();
}

// ========================
// Main Loop
// ========================
int incoming = 3350;

void loop() {
  // Read Bluetooth input if available
  if (ESP_BT.available() >= 2) {
    uint8_t highByte = ESP_BT.read();
    uint8_t lowByte = ESP_BT.read();
    incoming = (highByte << 8) | lowByte;
  }

  // Parse mode and color
  int mode = incoming / 1000;
  int colorCode = (incoming % 1000) / 100;

  int r = 0, g = 0, b = 0;
  if (colorCode == 1) r = 255;
  else if (colorCode == 2) g = 255;
  else if (colorCode == 3) b = 255;

  switch (mode) {
    case 1: // Static Color Fill
      fill_solid(leds, NUM_LEDS, CRGB(r, g, b));
      FastLED.show();
      break;

    case 2: { // Moving Horizontal Line
      static int frame = 0;
      static unsigned long lastUpdate = 0;
      unsigned long now = millis();

      if (now - lastUpdate > 80) {
        lastUpdate = now;
        fill_solid(leds, NUM_LEDS, CRGB::Black);

        int y = frame % MATRIX_HEIGHT;
        for (int x = 0; x < MATRIX_WIDTH; x++) {
          int i = xyToIndex(x, y);
          leds[i] = CRGB(r, g, b);
        }

        FastLED.show();
        frame++;
      }
      break;
    }

    case 3: { // FFT Visualizer
      int32_t samples[SAMPLES];
      size_t bytes_read = 0;

      i2s_read(I2S_NUM_0, (char*)samples, SAMPLES * sizeof(int32_t), &bytes_read, portMAX_DELAY);

      for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = (samples[i] >> 14) / 2048.0;
        vImag[i] = 0.0;
      }

      FFT.windowing(vReal, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward);
      FFT.compute(vReal, vImag, SAMPLES, FFTDirection::Forward);
      FFT.complexToMagnitude(vReal, vImag, SAMPLES);

      displayReactiveBands(vReal);
      break;
    }
  }
}

// ========================
// Show 32 vertical bands
// ========================
void displayReactiveBands(double *magnitudes) {
  fill_solid(leds, NUM_LEDS, CRGB::Black);

  int startBin = 1;
  for (int band = 0; band < 32; band++) {
    int binCount = 2;
    int endBin = startBin + binCount;

    double sum = 0;
    for (int i = startBin; i <= endBin && i < SAMPLES / 2; i++) {
      sum += magnitudes[i];
    }

    double avg = sum / binCount;
    double scaled = log10(avg + 1) * 10;
    int height = map((int)scaled, 0, 20, 0, MATRIX_HEIGHT);
    height = constrain(height, 0, MATRIX_HEIGHT);

    for (int y = 0; y < height; y++) {
      int index = xyToIndex(band, MATRIX_HEIGHT - 1 - y);
      leds[index] = CRGB(0, 0, 255);
    }

    startBin = endBin + 1;
  }

  FastLED.show();
}

// ========================
// Matrix Coordinate Mapping
// ========================
int xyToIndex(int x, int y) {
  const int tileSize = 16;
  int tileX = x / tileSize;         // 0 or 1
  int tileY = y / tileSize;         // 0 or 1
  int inTileX = x % tileSize;
  int inTileY = y % tileSize;

  int tileNumber;

  // Serpentine layout: alternate row direction
  if (tileY == 0) {
    tileNumber = tileX;           // Top row: 0 (left), 1 (right)
  } else {
    tileNumber = 3 - tileX;       // Bottom row: 3 (left), 2 (right)
  }

  int baseIndex = tileNumber * tileSize * tileSize;

  // Serpentine within tile
  int localIndex;
  if (inTileY % 2 == 0) {
    localIndex = inTileY * tileSize + inTileX;
  } else {
    localIndex = inTileY * tileSize + (tileSize - 1 - inTileX);
  }

  return baseIndex + localIndex;
}
