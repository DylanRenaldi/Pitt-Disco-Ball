#include <FastLED.h>                // Library to control WS2812B LED matrix
#include <arduinoFFT.h>            // Library for performing FFT (Fast Fourier Transform)
#include <driver/i2s.h>            // I2S driver for audio input on ESP32
#include <math.h>
#include "BluetoothSerial.h"       // Bluetooth communication for control
#include <vector>  
#include <disco.hpp>
#include "disco_peripherals.h"

// =============================================================================
// Global Variables
// =============================================================================

uint8_t BRIGHTNESS        =10;     // Initial Brightness

XYMap xymap(MATRIX_WIDTH, MATRIX_HEIGHT);     // XYMap object
CRGB leds[NUM_LEDS],                          // LED matrix buffer
     colors[MATRIX_HEIGHT];                   // CRGB color gradient array
ArduinoFFT<double> FFT;                       // FFT object
double vReal[SAMPLES];                        // Real part of FFT input
double vImag[SAMPLES];                      // Imaginary part of FFT input
int xyIndexTable[MATRIX_HEIGHT][MATRIX_WIDTH];


// Bluetooth serial for remote control
BluetoothSerial ESP_BT;

// Audio visualization variables
double smoothedBands[MATRIX_WIDTH] = {0};   // Smoothed FFT magnitudes for each band
double noiseFloor[MATRIX_WIDTH] = {0.05};   // Dynamic noise floor per band
int peakHeights[MATRIX_WIDTH] = {0};        // Keeps track of peak levels per band
unsigned long lastPeakUpdate[MATRIX_WIDTH] = {0};
const int peakHoldTime = 150;     // Hold time for peaks in milliseconds
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

std::vector<int> logBins = logspace(2, SAMPLES/2, MATRIX_WIDTH+1);

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

  pinMode(INCREASE_BRIGHTNESS, INPUT_PULLDOWN);
  pinMode(DECREASE_BRIGHTNESS, INPUT_PULLDOWN);
  pinMode(BLUETOOTH_LED, OUTPUT);
}

String incomingData = "";
int r1 = 255;
int g1 = 0;
int b1 = 0;
int r2 = 0;
int g2 = 0;
int b2 = 0;
int mode = 3;
int selectedBrightness = 20;



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
    if(digitalRead(INCREASE_BRIGHTNESS) && BRIGHTNESS < 100)    // GPIO27
      FastLED.setBrightness(++BRIGHTNESS);

    else if (digitalRead(DECREASE_BRIGHTNESS) && BRIGHTNESS)    // GPIO14
      FastLED.setBrightness(--BRIGHTNESS);

    if(b != BRIGHTNESS){
      Serial.println(BRIGHTNESS);
      debounce = 100;
    }
  }

  // ---------------------------------------
  // Check if Bluetooth is connected to display the Bluetooth LED
  // ---------------------------------------
  static int8_t connected = false;
  static uint16_t BLUETOOTH_BUFFER = 0;
  static int lastTime = millis(), difference;

  difference = millis() - lastTime;
  lastTime = millis();

  if(BLUETOOTH_BUFFER) {
    if(!digitalRead(BLUETOOTH_LED))
      digitalWrite(BLUETOOTH_LED, HIGH);

    if(BLUETOOTH_BUFFER > difference)
      BLUETOOTH_BUFFER -= difference;
    else {
      BLUETOOTH_BUFFER = 0;
      digitalWrite(BLUETOOTH_LED, LOW);
    }
  } 

  if (ESP_BT.hasClient()) {
    if (!connected) {
      connected = true;
      BLUETOOTH_BUFFER = 3000;      // 1000ms
    } else if (ESP_BT.available()) {
      BLUETOOTH_BUFFER = 3000;
    }
  }else if (connected) {
    connected = false;
    BLUETOOTH_BUFFER = 3000;        // 1000ms
  }

  // ---------------------------------------
  // Framerate and timing variables
  // ---------------------------------------
  static unsigned lastFrame = 0, diff;
  const int frameInterval = 25;           // Target ~40 FPS

  diff = millis() - lastFrame;            // save diff because it may change in midst of below comparison and cause an erroneous error

  if (diff < frameInterval) return;
  debounce -= (debounce >= diff ? diff : debounce);     // only change debounce after frameInterval (otherwise, debounce will be compoundingly reduced at the clock rate)
  lastFrame = millis();

  // ---------------------------------------
  // Read incoming Bluetooth command
  // ---------------------------------------
  while(ESP_BT.available()){
    char c = ESP_BT.read();
    if (c == '\n') {
      // Split string by commas
      std::vector<String> tokens;
      int lastIndex = 0;

      for (int i = 0; i < incomingData.length(); i++) {
        if (incomingData.charAt(i) == ',') {
          tokens.push_back(incomingData.substring(lastIndex, i));
          lastIndex = i + 1;
        }
      }

      // Add the last token after the final comma
      if (lastIndex < incomingData.length()) {
        tokens.push_back(incomingData.substring(lastIndex));
      }

      // Expecting 8 values: r1, g1, b1, r2, g2, b2, mode, brightness
      if (tokens.size() == 8) {
        r1 = tokens[0].toInt();
        g1 = tokens[1].toInt();
        b1 = tokens[2].toInt();
        r2 = tokens[3].toInt();
        g2 = tokens[4].toInt();
        b2 = tokens[5].toInt();
        mode = tokens[6].toInt();
        selectedBrightness = tokens[7].toInt();
        BRIGHTNESS = selectedBrightness;
        FastLED.setBrightness(BRIGHTNESS);
      }

      incomingData = "";
    }else{
      incomingData += c;
    }

  }

  // ---------------------------------------
  // Handle different display modes
  // ---------------------------------------
  switch (mode) {

    // ..............................
    // Mode 1: Solid color fill
    // ..............................
    case 1: {
      if (r1 == r2 && g1 == g2 && b1 == b2){
        fill_solid(leds, NUM_LEDS, CRGB(r1, g1, b1));
      }else{
        CRGB topColor = CRGB(r1, g1, b1);
        CRGB bottomColor = CRGB(r2, g2, b2);
        CRGB gradientRow[MATRIX_HEIGHT];

        gradient(gradientRow, topColor, bottomColor);

        for (int y = 0; y < MATRIX_HEIGHT; y++) {
          for (int x = 0; x < MATRIX_WIDTH; x++) {
            leds[xyToIndex(x, y)] = gradientRow[y];
          }
        }
      }
      break;
    }
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
        int32_t shifted = samples[i] >> 14;
        shifted = constrain(shifted, -4096, 4095);  // Clamp extreme values
        vReal[i] = shifted / 2048.0;
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

      // Determine whether to use solid or gradient bar colors
      bool useGradient = !(r1 == r2 && g1 == g2 && b1 == b2);
      CRGB gradientColors[MATRIX_HEIGHT];
      if (useGradient) {
        gradient(gradientColors, CRGB(r1, g1, b1), CRGB(r2, g2, b2));
      }

      // Display the bands with custom colors
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i].fadeToBlackBy(40);
      }

      for (int band = 0; band < MATRIX_WIDTH; band++) {
        int startBin = logBins[band];
        int endBin = logBins[band + 1];
        int binCount = endBin - startBin;

        double sum = 0;
        for (int i = startBin; i < endBin && i < SAMPLES / 2; i++) {
          sum += vReal[i];
        }
        double avg = sum / binCount;

        if (avg < noiseFloor[band]) {
          noiseFloor[band] = noiseFloor[band] * 0.95 + avg * 0.05;
        } else {
          noiseFloor[band] = noiseFloor[band] * 0.9995 + avg * 0.0005;
        }

        double adjusted = avg - noiseFloor[band];
        if (adjusted < 0) adjusted = 0;

        smoothedBands[band] = 0.7 * smoothedBands[band] + 0.3 * adjusted;

        double scaled = log10(smoothedBands[band] + 1) * 10;
        int height = map((int)scaled, 0, 20, 1, MATRIX_HEIGHT);  
        height = constrain(height, 1, MATRIX_HEIGHT);
        if(height==1 && !peakHeights[band]) continue;
      
      
      // Peak hold logic
        if (height > 1 && height >= peakHeights[band]) {
            peakHeights[band] = height;
            lastPeakUpdate[band] = now;
        } else if (now - lastPeakUpdate[band] > peakHoldTime) {
            if(peakHeights[band] <= peakFallSpeed) {
                peakHeights[band] = 0;
                continue;
            }
      
            peakHeights[band] -= peakFallSpeed;
        } // else continue;
      
        // Color mapping based on height
        //uint8_t hue = map(height, 0, MATRIX_HEIGHT, 160, 0);
        //CRGB barColor = CHSV(hue, 255, 255);
        int flippedBand = MATRIX_WIDTH - 1 - band;  // Flip horizontally for visual symmetry
      
        // Draw vertical bar (starting 1 because height map is 1-offset)
        for (int y = 1, index; y < height; y++) {
            index = xyToIndex(flippedBand, MATRIX_HEIGHT - y);
            leds[index] = useGradient ? gradientColors[y-1] : CRGB(r1, g1, b1);
        }
      
        // Draw peak marker
        int peakY = MATRIX_HEIGHT - peakHeights[band];
        int peakIndex = xyToIndex(flippedBand, peakY);
        leds[peakIndex] = CRGB::White;
      }
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
  }
  if(BRIGHTNESS>100){
    BRIGHTNESS = 100;
  }
  FastLED.show();
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

void gradient(CRGB *out, CRGB begin, CRGB end){

  float dr = float(end.r - begin.r)/(MATRIX_HEIGHT - 1),
        dg = float(end.g - begin.g)/(MATRIX_HEIGHT - 1),
        db = float(end.b - begin.b)/(MATRIX_HEIGHT - 1);  

  // last color doesn't matter (if unsigned overflow) because of peak marker
  for(uint8_t i = 0; i < MATRIX_HEIGHT; ++i, begin.r += dr, begin.g += dg, begin.b += db)
    out[i] = begin;
}
