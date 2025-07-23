#include <FastLED.h>                // Library to control WS2812B LED matrix
#include <arduinoFFT.h>            // Library for performing FFT (Fast Fourier Transform)
#include <driver/i2s.h>            // I2S driver for audio input on ESP32
#include <math.h>
#include "BluetoothSerial.h"       // Bluetooth communication for control
#include <vector>  

// =============================================================================
// Matrix & Audio Settings
// =============================================================================

#define MATRIX_WIDTH      32      // Width of LED panel
#define MATRIX_HEIGHT     16      // Height of LED panel
#define NUM_LEDS          (MATRIX_WIDTH * MATRIX_HEIGHT)  // Total number of LEDs
#define DATA_PIN          5       // Pin connected to the LED strip

// =============================================================================
// Global Variables
// =============================================================================

const uint8_t 
  INCREASE_BRIGHTNESS   =27,      // increase brightness button -> GPIO27
  DECREASE_BRIGHTNESS   =14,      // decrease brightness button -> GPIO14
  I2S_WS                =25,      // Word Select (LRCLK)
  I2S_SD                =33,      // Serial Data input
  I2S_SCK               =26,      // Bit Clock (BCLK)
  SAMPLES               =512,     // Quantity of Samples
  SAMPLING_FREQ         =48000;   // Sampling Frequency (center frequency is SAMPLING_FREQ / 2)
uint8_t BRIGHTNESS        =5;     // Initial Brightness

XYMap xymap(MATRIX_WIDTH, MATRIX_HEIGHT);     // XYMap object
CRGB leds[NUM_LEDS],                          // LED matrix buffer
     colors[MATRIX_HEIGHT];                   // CRGB color gradient array
ArduinoFFT<double> FFT;                       // FFT object
double vReal[SAMPLES];                        // Real part of FFT input
double vImag[SAMPLES]{};                      // Imaginary part of FFT input
int xyIndexTable[MATRIX_HEIGHT][MATRIX_WIDTH];


// Bluetooth serial for remote control
BluetoothSerial ESP_BT;

// Audio visualization variables
double smoothedBands[MATRIX_WIDTH] = {0};   // Smoothed FFT magnitudes for each band
double noiseFloor[MATRIX_WIDTH] = {0.05};   // Dynamic noise floor per band
int peakHeights[MATRIX_WIDTH] = {0};        // Keeps track of peak levels per band
unsigned long lastPeakUpdate[MATRIX_WIDTH] = {0};
const int peakHoldTime = 50;     // Hold time for peaks in milliseconds
const int peakFallSpeed = 1;      // How fast the peak indicator drops

// Beat detection variables
double prevLowEnergy = 0;
unsigned long lastBeatTime = 0;
int estimatedBPM = 120;           // Estimated beats per minute
int dynamicFrameInterval = 25;    // Controls how frequently LED updates (adaptive)

// =============================================================================
// Calculate logarithmic bin edges for frequency bands
// =============================================================================
std::vector<int> logspace(double start, double stop, int numbins) {
    std::vector<int> bins(numbins);
    double ratio = log10(stop / start);

    for (int i = 0; i < numbins; ++i) {
        bins[i] = start * pow(10.0, ratio * i / (numbins - 1));
        if (i && bins[i] <= bins[i - 1]) bins[i] = 1 + bins[i - 1];
    }

    return bins;
}

std::vector<int> logBins = logspace(2, 500, MATRIX_WIDTH+1);

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

  
}

int incoming = 3350; // Default mode and color input value
String incomingData = "";


// =============================================================================
// Main Loop
// =============================================================================
void loop() {

  // ---------------------------------------
  // Debounce Buttons
  // ---------------------------------------
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

  static unsigned long lastFrame = 0;
  if (millis() - lastFrame < dynamicFrameInterval) return;
  lastFrame = millis();

  // ---------------------------------------
  // Read incoming Bluetooth command
  // ---------------------------------------
  while(ESP_BT.available()){
    char c = ESP_BT.read();
    if (c == '\n') {
      incoming = incomingData.toInt();
      incomingData = "";
    }else{
      incomingData += c;
    }
  }

  // Parse control command
  int mode = incoming / 1000;
  int colorCode = (incoming % 1000) / 100;

  // ---------------------------------------
  // Map color code to RGB values
  // ---------------------------------------
  int r = 0, g = 0, b = 0;
  if (colorCode == 1) r = 255;
  else if (colorCode == 2) g = 255;
  else if (colorCode == 3) b = 255;

  // ---------------------------------------
  // Handle different display modes
  // ---------------------------------------
  switch (mode) {

    // ..............................
    // Mode 1: Solid color fill
    // ..............................
    case 1:
      fill_solid(leds, NUM_LEDS, CRGB(r, g, b));
      break;

    // ..............................
    // Mode 2: Color Wave
    // ..............................
    case 2: {

      static int offset = -1;
      offset++;
      for (int y = 0; y < MATRIX_HEIGHT; y++) {
        for (int x = 0; x < MATRIX_WIDTH; x++) {
          float wave = sin((x + offset) * 0.3 + millis() * 0.005);
          uint8_t brightness = map(wave * 100, -100, 100, 50, 255);
          leds[xyToIndex(x, y)] = CHSV((x * 10 + offset) % 255, 255, brightness);
        }
      }
      break;
    }

    // ..............................
    // Audio reactive FFT visualization
    // ..............................
    case 3: {

      int32_t samples[SAMPLES];
      size_t bytes_read = 0;

      // Read audio samples from I2S
      i2s_read(I2S_NUM_0, (char*)samples, SAMPLES * sizeof(int32_t), &bytes_read, portMAX_DELAY);

      // Convert raw samples to floating-point and prepare for FFT
      for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = (samples[i] >> 14) / 2048.0;  // Normalize
        vImag[i] = 0.0;
      }

      // Apply FFT
      FFT.windowing(vReal, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward);
      FFT.compute(vReal, vImag, SAMPLES, FFTDirection::Forward);
      FFT.complexToMagnitude(vReal, vImag, SAMPLES);

      // Attenuate high-frequency bins
      for (int i = 3; i < 36 && i < SAMPLES / 2; i++) {
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
      displayReactiveBands(vReal);
      break;
    }

    // ..............................
    // Mode 4: Pulse in sync with low frequencies
    // ..............................
    case 4: {
      
      int32_t samples[SAMPLES];
      size_t bytes_read = 0;

      // Read samples from I2S
      i2s_read(I2S_NUM_0, (char*)samples, SAMPLES * sizeof(int32_t), &bytes_read, portMAX_DELAY);

      // Prepare for FFT
      for (int i = 0; i < SAMPLES; i++) {
        vReal[i] = (samples[i] >> 14) / 2048.0;
        vImag[i] = 0.0;
      }

      FFT.windowing(vReal, SAMPLES, FFTWindow::Hamming, FFTDirection::Forward);
      FFT.compute(vReal, vImag, SAMPLES, FFTDirection::Forward);
      FFT.complexToMagnitude(vReal, vImag, SAMPLES);

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
      break;
    }
    
    // ..............................
    // Mode 5: Ripple Beat
    // ..............................
    case 5: {
      
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
      break;
    }

    // ..............................
    // Mode 6: Fire Glow (bass-driven flicker)
    // ..............................
    case 6: {
      
      for (int y = 0; y < MATRIX_HEIGHT; y++) {
        for (int x = 0; x < MATRIX_WIDTH; x++) {
          float bass = vReal[2] * 10;
          uint8_t heat = random8(constrain((int)(bass * 255), 30, 255));
          leds[xyToIndex(x, y)] = CHSV(map(heat, 0, 255, 0, 40), 255, heat);
        }
      }
      break;
    }

    // ..............................
    // Mode 7: Energy Snake
    // ..............................
    case 7: {
      
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
      break;
    }

    // ..............................
    // Mode 8: Gradient Spectrum Reactive
    // ..............................
    case 8: {
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
    }
  }
  if(BRIGHTNESS>100){
    BRIGHTNESS = 100;
  }
  FastLED.show();
}

// =============================================================================
// Initialize and Configure I2S audio input
// =============================================================================
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

// =============================================================================
// Visualize audio frequency bands as vertical bars with peak indicators
// =============================================================================
void displayReactiveBands(double *magnitudes) {
  // Dim all LEDs for trail effect
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].fadeToBlackBy(40);
  }

  unsigned long now = millis();
  for (int band = 0; band <32; band++) {
    int startBin = logBins[band];
    int endBin = logBins[band + 1];
    int binCount = endBin - startBin;

    // Average the magnitudes in the band
    double sum = 0;
    for (int i = startBin; i < endBin && i < SAMPLES / 2; i++) {
      sum += magnitudes[i];
    }
    double avg = sum / binCount;

    // Adjust noise floor dynamically
    if (avg < noiseFloor[band]) {
      noiseFloor[band] = noiseFloor[band] * 0.95 + avg * 0.05;
    } else {
      noiseFloor[band] = noiseFloor[band] * 0.9995 + avg * 0.0005;
    }

    double adjusted = avg - noiseFloor[band];
    if (adjusted < 0) adjusted = 0;

    smoothedBands[band] = 0.7 * smoothedBands[band] + 0.3 * adjusted;

    // Convert to visual height
    double scaled = log10(smoothedBands[band] + 1) * 10;
    int height = map((int)scaled, 0, 20, 0, MATRIX_HEIGHT);
    height = constrain(height, 0, MATRIX_HEIGHT);

    // Update peak height
    if (height >= peakHeights[band]) {
      peakHeights[band] = height;
      lastPeakUpdate[band] = now;
    } else if (now - lastPeakUpdate[band] > peakHoldTime) {
      peakHeights[band] = max(0, peakHeights[band] - peakFallSpeed);
    }

    // Draw bar and peak
    uint8_t hue = map(height, 0, MATRIX_HEIGHT, 160, 0);
    CRGB barColor = CHSV(hue, 255, 255);
    int flippedBand = 31 - band;  // Flip for visual symmetry

    for (int y = 0; y < height; y++) {
      int index = xyToIndex(flippedBand, MATRIX_HEIGHT - 1 - y);
      leds[index] = barColor;
    }

    if (peakHeights[band] > 0) {
      int peakY = MATRIX_HEIGHT - 1 - peakHeights[band];
      int peakIndex = xyToIndex(flippedBand, peakY);
      leds[peakIndex] = CRGB::White;
    }
  }
}



// =============================================================================
// Convert (x, y) coordinate to 1D LED index based on matrix layout
// =============================================================================
inline int xyToIndex(int x, int y) {
  return xyIndexTable[y][x];
}

void setupXYTable() {
  for (int y = 0; y < MATRIX_HEIGHT; y++) {
    for (int x = 0; x < MATRIX_WIDTH; x++) {
      const int tileSize = 16;
      int tileX = x / tileSize;
      int tileY = y / tileSize;
      int inTileX = x % tileSize;
      int inTileY = y % tileSize;

      int tileNumber;
      if (tileY == 0) {
        tileNumber = tileX;
      } else {
        tileNumber = 3 - tileX;
      }

      int baseIndex = tileNumber * tileSize * tileSize;
      int localIndex;

      // Zigzag wiring handling
      if (inTileY % 2 == 0) {
        localIndex = inTileY * tileSize + inTileX;
      } else {
        localIndex = inTileY * tileSize + (tileSize - 1 - inTileX);
      }

      xyIndexTable[y][x] = baseIndex + localIndex;
    }
  }
}

// Visualize FFT result on LED matrix
void displayWaveform(double *values) {

  gradient(colors, CRGB(0,0,125), CRGB(0,0,255));
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

