#include <arduinoFFT.h>
#include <Adafruit_NeoPixel.h>
#include <driver/i2s.h>
#include <math.h> // for log10

#define SAMPLES         512
#define SAMPLING_FREQ   48000
#define BRIGHTNESS      32
#define LEDS_PER_STRIP  4

// GPIO pins for LED strips
#define STRIP1_PIN 5    // Sub-bass
#define STRIP2_PIN 18   // Bass
#define STRIP3_PIN 19   // Mids
#define STRIP4_PIN 21   // Treble

Adafruit_NeoPixel strip1(LEDS_PER_STRIP, STRIP1_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2(LEDS_PER_STRIP, STRIP2_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip3(LEDS_PER_STRIP, STRIP3_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip4(LEDS_PER_STRIP, STRIP4_PIN, NEO_GRB + NEO_KHZ800);

ArduinoFFT<double> FFT = ArduinoFFT<double>();
double vReal[SAMPLES];
double vImag[SAMPLES];

// I2S microphone pins
#define I2S_WS   25
#define I2S_SD   33
#define I2S_SCK  26

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

  strip1.begin(); strip1.setBrightness(BRIGHTNESS); strip1.show();
  strip2.begin(); strip2.setBrightness(BRIGHTNESS); strip2.show();
  strip3.begin(); strip3.setBrightness(BRIGHTNESS); strip3.show();
  strip4.begin(); strip4.setBrightness(BRIGHTNESS); strip4.show();
}

void loop() {
  int32_t samples[SAMPLES];
  size_t bytes_read = 0;

  // Read from I2S mic
  i2s_read(I2S_NUM_0, (char*)samples, SAMPLES * sizeof(int32_t), &bytes_read, portMAX_DELAY);

  // Convert raw data
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = (samples[i] >> 14) / 2048.0;
    vImag[i] = 0.0;
  }

  // Perform FFT
  FFT.windowing(vReal, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(vReal, vImag, SAMPLES, FFTDirection::Forward);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  // Visualize
  displayStrips(vReal);
  delay(30);
}

void displayStrips(double *magnitudes) {
  strip1.clear();
  strip2.clear();
  strip3.clear();
  strip4.clear();

  // Frequency bins (based on ~93.75 Hz per bin)
  int level1 = computeLevel(magnitudes, 1, 1);     // ~20–60 Hz (Sub-bass)
  int level2 = computeLevel(magnitudes, 1, 2);     // ~60–250 Hz (Bass)
  int level3 = computeLevel(magnitudes, 3, 21);    // ~250–2000 Hz (Mids)
  int level4 = computeLevel(magnitudes, 22, 85);   // ~2000–8000 Hz (Treble)

  lightStrip(strip1, level1);
  lightStrip(strip2, level2);
  lightStrip(strip3, level3);
  lightStrip(strip4, level4);

  strip1.show();
  strip2.show();
  strip3.show();
  strip4.show();
}

// More sensitive + logarithmic scaling
int computeLevel(double *data, int startBin, int endBin) {
  double sum = 0;
  for (int i = startBin; i <= endBin; i++) {
    sum += data[i];
  }
  double avg = sum / (endBin - startBin + 1);
  double logAvg = log10(avg + 1) * 10;  // Add 1 to prevent log(0)

  // Adjust sensitivity scale as needed
  int level = map((int)logAvg, 0, 20, 0, 4);
  return constrain(level, 0, 4);
}

// Light up LEDs based on level
void lightStrip(Adafruit_NeoPixel& strip, int level) {
  uint32_t color = strip.Color(0, 255, 0); // Green
  for (int i = 0; i < LEDS_PER_STRIP; i++) {
    if (i < level) {
      strip.setPixelColor(i, color);
    } else {
      strip.setPixelColor(i, 0); // Off
    }
  }
}
