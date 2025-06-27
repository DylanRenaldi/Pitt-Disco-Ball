#include <arduinoFFT.h>
#include <Adafruit_NeoPixel.h>
#include <driver/i2s.h>
#include <math.h>
#include "BluetoothSerial.h"

// ========================
// Matrix & Audio Settings
// ========================
#define MATRIX_WIDTH      32
#define MATRIX_HEIGHT     32
#define NUM_LEDS          (MATRIX_WIDTH * MATRIX_HEIGHT)
#define MATRIX_PIN        5
#define BRIGHTNESS        32

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
Adafruit_NeoPixel matrix(NUM_LEDS, MATRIX_PIN, NEO_GRB + NEO_KHZ800);
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

  matrix.begin();
  matrix.setBrightness(BRIGHTNESS);
  matrix.show();
}

// ========================
// Main Loop
// ========================
int incoming = 0;
int temp = 0;

void loop() {
  // Read Bluetooth input if available
  if (ESP_BT.hasClient()) {
    incoming = ESP_BT.read();
    temp = incoming;
  } else {
    incoming = 3250; // Default fallback
  }

  // Parse mode, color, and brightness
  int mode = incoming / 1000;
  int newBrightness = incoming % 100;
  int colorCode = (incoming % 1000) / 100;

  matrix.setBrightness(newBrightness);

  // Set color based on colorCode
  int r = 0, g = 0, b = 0;
  if (colorCode == 1) r = 255;
  else if (colorCode == 2) g = 255;
  else if (colorCode == 3) b = 255;

  // ======= MODE HANDLING ========
  switch (mode) {

    // -----------------------------
    case 1: // Static Color Fill
    // -----------------------------
      for (int i = 0; i < NUM_LEDS; i++) {
        matrix.setPixelColor(i, matrix.Color(r, g, b));
      }
      matrix.show();
      break;

    // -----------------------------
    case 2: { // Dynamic - Horizontal Moving Line
    // -----------------------------
      static int frame = 0;
      static unsigned long lastUpdate = 0;
      unsigned long now = millis();

      if (now - lastUpdate > 80) { // Adjust speed here
        lastUpdate = now;

        matrix.clear();
        int y = frame % MATRIX_HEIGHT;

        for (int x = 0; x < MATRIX_WIDTH; x++) {
          int i = xyToIndex(x, y);
          matrix.setPixelColor(i, matrix.Color(r, g, b));
        }

        matrix.show();
        frame++;
      }
      break;
    }

    // -----------------------------
    case 3: // Reactive FFT Visualizer
    // -----------------------------
      int32_t samples[SAMPLES];
      size_t bytes_read = 0;

      // Read raw audio data from mic
      i2s_read(I2S_NUM_0, (char*)samples, SAMPLES * sizeof(int32_t), &bytes_read, portMAX_DELAY);

      // Normalize & prepare for FFT
      for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = (samples[i] >> 14) / 2048.0;
        vImag[i] = 0.0;
      }

      // Perform FFT
      FFT.windowing(vReal, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward);
      FFT.compute(vReal, vImag, SAMPLES, FFTDirection::Forward);
      FFT.complexToMagnitude(vReal, vImag, SAMPLES);

      // Display the FFT results
      displayReactiveBands(vReal);
      delay(30);
      break;
  }
}

// ========================
// Show 32 vertical bars
// ========================
void displayReactiveBands(double *magnitudes) {
  matrix.clear();

  int startBin = 1;
  for (int band = 0; band < 32; band++) {
    int binCount = 2; // ~85 bins / 32 bands ≈ 2.6
    int endBin = startBin + binCount;

    double sum = 0;
    for (int i = startBin; i <= endBin && i < SAMPLES / 2; i++) {
      sum += magnitudes[i];
    }

    double avg = sum / binCount;
    double scaled = log10(avg + 1) * 10;
    int height = map((int)scaled, 0, 20, 0, MATRIX_HEIGHT);
    height = constrain(height, 0, MATRIX_HEIGHT);

    // Light up from bottom
    for (int y = 0; y < height; y++) {
      int index = xyToIndex(band, MATRIX_HEIGHT - 1 - y);
      matrix.setPixelColor(index, matrix.Color(0, 0, 255));
    }

    startBin = endBin + 1;
  }

  matrix.show();
}

// ========================
// Matrix Coordinate Mapping
// ========================
int xyToIndex(int x, int y) {
  // Assumes zigzag/serpentine layout
  if (y % 2 == 0) {
    return y * MATRIX_WIDTH + x;
  } else {
    return y * MATRIX_WIDTH + (MATRIX_WIDTH - 1 - x);
  }
}
