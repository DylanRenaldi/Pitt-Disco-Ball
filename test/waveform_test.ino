
#include <FastLED.h>             // For controlling WS2812B LED strips
#include <arduinoFFT.h>          // For performing FFT on audio data
#include <driver/i2s.h>          // For ESP32 I2S audio input
#include <math.h>                // For logarithmic and math functions

// LED matrix and visualization parameters
#define MATRIX_WIDTH          32
#define MATRIX_HEIGHT         16
#define NUM_LEDS              (MATRIX_WIDTH * MATRIX_HEIGHT)
#define DATA_PIN              5

// Brightness Interaction Variables (const uint8_t instead of #define so they are only declared in this scope)
const uint8_t 
  INCREASE_BRIGHTNESS   =27,      // increase brightness button -> GPIO27
  DECREASE_BRIGHTNESS   =14,      // decrease brightness button -> GPIO14
              
// I2S pin assignments
  I2S_WS                =25,      // Word Select (LRCLK)
  I2S_SD                =33,      // Serial Data input
  I2S_SCK               =26,      // Bit Clock (BCLK)
              
// Audio sampling settings
  SAMPLES               =512,     // Quantity of Samples
  SAMPLING_FREQ         =48000;   // Sampling Frequency (center frequency is SAMPLING_FREQ / 2)

uint8_t BRIGHTNESS        =5;

// Global variables
XYMap xymap(MATRIX_WIDTH, MATRIX_HEIGHT);     // XYMap object
CRGB leds[NUM_LEDS],                          // LED matrix buffer
     colors[MATRIX_HEIGHT];                   // CRGB color gradient array
ArduinoFFT<double> FFT;                       // FFT object
double vReal[SAMPLES];                        // Real part of FFT input
double vImag[SAMPLES]{};                      // Imaginary part of FFT input

// Visualization state
double smoothedBands[MATRIX_WIDTH] = {0};        // Smoothed FFT band levels
double noiseFloor[MATRIX_WIDTH] = {0.05};        // Adaptive noise floor per band
// int peakHeights[MATRIX_WIDTH] = {0};             // Peak level hold per band
// unsigned lastPeakUpdate[MATRIX_WIDTH] = {0};     // Time of last peak update
// const int peakHoldTime = 150;          // Time in ms to hold peak before decay
// const int peakFallSpeed = 1;           // Amount to reduce peak per update

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
  
  Serial.begin(115200);                    // Initialize Serial for debugging
  setupI2S();                              // Initialize I2S microphone input

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS); // Init LED matrix
  FastLED.setBrightness(BRIGHTNESS);       // Set LED brightness
  FastLED.clear();
  FastLED.show();

  pinMode(INCREASE_BRIGHTNESS, INPUT_PULLUP);
  pinMode(DECREASE_BRIGHTNESS, INPUT_PULLUP);
  //pinMode(0, INPUT_PULLUP);                           // reset average calculation

  gradient(colors, CRGB(0,0,125), CRGB(0,0,255));
}

//int incoming = 3350;  // Default Bluetooth command value

// Arduino main loop
void loop() {

  static uint8_t debounce = 0;

  if(!debounce) {
    uint8_t b = BRIGHTNESS;
    if(!digitalRead(INCREASE_BRIGHTNESS) && BRIGHTNESS < 100)    // GPIO27
      FastLED.setBrightness(++BRIGHTNESS);

    else if (!digitalRead(DECREASE_BRIGHTNESS) && BRIGHTNESS)    // GPIO14
      FastLED.setBrightness(--BRIGHTNESS);

    if(b != BRIGHTNESS){
      Serial.println(BRIGHTNESS);
      debounce = 100;
    }
  }

  static unsigned lastFrame = 0, diff;
  const int frameInterval = 25;           // Target ~40 FPS

  diff = millis() - lastFrame;            // save diff because it may change in midst of below comparison and cause an erroneous error

  if (diff < frameInterval) return;
  debounce -= (debounce >= diff ? diff : debounce);     // only change debounce after frameInterval (otherwise, debounce will be compoundingly reduced at the clock rate)
  lastFrame = millis();

  // Audio reactive FFT display
  int32_t samples[SAMPLES];
  size_t bytes_read = 0;

  i2s_read(I2S_NUM_0, (char*)samples, SAMPLES * sizeof(int32_t), &bytes_read, portMAX_DELAY);

  // Convert raw I2S samples to normalized floats
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = double(samples[i] >> 14) / 2048.0;         // idk how the results will change with the values actually being double
  }

  // Apply windowing and perform FFT
  FFT.windowing(vReal, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(vReal, vImag, SAMPLES, FFTDirection::Forward);
  // FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  // Could also heavily suppress ambient noise via bins 1 and 2 (perhaps just bin 1)
  // vReal[1] *= 0.8;
  // vReal[2] *= 0.8;

  // Suppress vocal range (300–3400 Hz)
  for (int i = 3; i < 36; i++) {
    vReal[i] *= 0.2;  // Reduce vocal band magnitude
  }

  displayWaveform(vReal);
  FastLED.show();
}

// Visualize FFT result on LED matrix
void displayWaveform(double *values) {

  // Logarithmically spaced frequency bands
  static const int logBins[MATRIX_WIDTH + 1] = {
    1, 2, 3, 4, 5, 6, 7, 9, 11, 13, 15, 18, 21, 24, 28, 32,                           // vocal range (300 Hz - 3400 Hz)
    37, 43, 50, 58, 67, 77, 89, 103, 119, 137, 158, 182, 209, 239, 273, 311, 352
  };

  // unsigned now = millis();
  for (int band = 0; band < MATRIX_WIDTH; band++) {
    int startBin = logBins[band];
    int endBin = logBins[band + 1];
    int binCount = endBin - startBin;

    double sum = 0;
    for (int i = startBin; i < endBin && i < SAMPLES; i++) {
      sum += values[i];
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
    int8_t parity = (smoothedBands[band] < 0 ? -1 : 1);

    smoothedBands[band] *= parity;
    double scaled =  10 * log10(smoothedBands[band] + 1);

    int height = map((int)scaled, 0, 20, 1, MATRIX_HEIGHT >> 1);
    height = constrain(height, 1, MATRIX_HEIGHT >> 1);
    height = (MATRIX_HEIGHT >> 1) + height * parity;

    // Color mapping based on height
    //uint8_t hue = map(height, 0, MATRIX_HEIGHT, 160, 0);
    //CRGB barColor = CHSV(hue, 255, 255);
    int flippedBand = MATRIX_WIDTH - band - 1;  // Flip horizontally for visual symmetry

    // Draw vertical bar (starting 1 because height map is 1-offset)
    for (int y = 1, index; y < height; y++) {
      index = xymap(flippedBand, y);
      //index = xymap(band, y);
      leds[index] = colors[y - 1];
    }

    // Draw peak marker
    // int peakY = MATRIX_HEIGHT - peakHeights[band];
    int peakIndex = xymap(flippedBand, height);
    leds[peakIndex] = CRGB::White;
  }

}

void gradient(CRGB *out, CRGB begin, CRGB end){

  float dr = float(end.r - begin.r)/(MATRIX_HEIGHT - 1),
        dg = float(end.g - begin.g)/(MATRIX_HEIGHT - 1),
        db = float(end.b - begin.b)/(MATRIX_HEIGHT - 1);  

  // last color doesn't matter (if unsigned overflow) because of peak marker
  for(uint8_t i = 0; i < MATRIX_HEIGHT; ++i, begin.r += dr, begin.g += dg, begin.b += db)
    out[i] = begin;
}
