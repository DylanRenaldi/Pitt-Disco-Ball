#include <arduinoFFT.h>
#include <Adafruit_NeoPixel.h>
#include <driver/i2s.h>

#define SAMPLES         512           // Must be a power of 2
#define SAMPLING_FREQ   48000         // In Hz (must be around double the max frequency in audio spectrum (~20kHz))
#define NUM_BANDS       16
#define LED_PIN         5
#define MATRIX_WIDTH    32
#define MATRIX_HEIGHT   32
#define NUM_LEDS        (MATRIX_WIDTH * MATRIX_HEIGHT)
#define BRIGHTNESS      32

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
ArduinoFFT<double> FFT = ArduinoFFT<double>();

double vReal[SAMPLES];
double vImag[SAMPLES];

// I2S Pin Configuration
#define I2S_WS   25  // Word Select (LRCL)
#define I2S_SD   33  // Serial Data In (DOUT)
#define I2S_SCK  26  // Serial Clock (BCLK)

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

void setup() {
  Serial.begin(115200);
  setupI2S();
  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.show();
}

void loop() {
  int32_t samples[SAMPLES];
  size_t bytes_read = 0;

  // Read from INMP441 via I2S
  i2s_read(I2S_NUM_0, (char*)samples, SAMPLES * sizeof(int32_t), &bytes_read, portMAX_DELAY);

  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = (samples[i] >> 14) / 2048.0;  // Convert and normalize
    vImag[i] = 0.0;
  }

  // Perform FFT
  FFT.windowing(vReal, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(vReal, vImag, SAMPLES, FFTDirection::Forward);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  // Display result
  displaySpectrum(vReal);
  delay(30);  // Frame rate control
}

void displaySpectrum(double *magnitudes) {
  strip.clear();

  const int binsPerBand = (SAMPLES / 2) / NUM_BANDS;

  for (int band = 0; band < NUM_BANDS; band++) {
    double avg = 0;
    for (int i = 0; i < binsPerBand; i++) {
      avg += magnitudes[band * binsPerBand + i];
    }
    avg /= binsPerBand;

    // Convert average to height (0–32)
    int barHeight = constrain(map(avg, 0, 100, 0, MATRIX_HEIGHT), 0, MATRIX_HEIGHT);
    drawBar(band * 2, barHeight);  // Each band spans 2 pixels width
  }

  strip.show();
}

void drawBar(int xStart, int height) {
  uint32_t color = getColorForHeight(height);
  for (int x = xStart; x < xStart + 2; x++) {
    for (int y = 0; y < height; y++) {
      int pixelIndex = getPixelIndex(x, MATRIX_HEIGHT - 1 - y);
      strip.setPixelColor(pixelIndex, color);
    }
  }
}

uint32_t getColorForHeight(int height) {
  // Blue → Green → Yellow → Red gradient
  uint8_t r = 0, g = 0, b = 0;

  if (height <= 8) {
    r = 0;
    g = map(height, 0, 8, 0, 255);
    b = 255;  // Blue to Cyan
  } else if (height <= 16) {
    r = 0;
    g = 255;
    b = map(height, 8, 16, 255, 0);  // Cyan to Green
  } else if (height <= 24) {
    r = map(height, 16, 24, 0, 255);
    g = 255;
    b = 0;  // Green to Yellow
  } else {
    r = 255;
    g = map(height, 24, 32, 255, 0);
    b = 0;  // Yellow to Red
  }

  return strip.Color(r, g, b);
}

int getPixelIndex(int x, int y) {
  // Zigzag layout (serpentine)
  if (y % 2 == 0) {
    return y * MATRIX_WIDTH + x;
  } else {
    return y * MATRIX_WIDTH + (MATRIX_WIDTH - 1 - x);
  }
}
