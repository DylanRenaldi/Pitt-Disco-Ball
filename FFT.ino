#include <arduinoFFT.h>

const int adcPin = 34;
const int ledPins[6] = {2, 4, 5, 12, 13, 14};
const int numLeds = 6;

#define SAMPLES 64              // Must be a power of 2
#define SAMPLING_FREQ 8000    // In Hz 

double vReal[SAMPLES];
double vImag[SAMPLES];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

// Noise filtering
const int noiseThresholds[6] = {40, 40, 50, 60, 70, 80}; // Tune per band
const float bandGains[6] = {1.2, 1.2, 1.0, 1.0, 0.9, 0.8};


// Smoothing
float smoothingFactor = 0.2;
float smoothedLevels[6] = {0};

void setup() {
  analogReadResolution(12); // 12-bit ADC
  for (int i = 0; i < numLeds; i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }
}

void loop() {
  // Sample audio
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = analogRead(adcPin);
    vImag[i] = 0;
    delayMicroseconds(1000000 / SAMPLING_FREQ);
  }

  // Remove DC bias (center around 0)
  double dcOffset = 0;
  for (int i = 0; i < SAMPLES; i++) dcOffset += vReal[i];
  dcOffset /= SAMPLES;
  for (int i = 0; i < SAMPLES; i++) vReal[i] -= dcOffset;

  // FFT
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // Split into 6 frequency bands
  int rawLevels[6] = {0};

  for (int i = 2; i < SAMPLES / 2; i++) {
    double freq = i * ((double)SAMPLING_FREQ / SAMPLES);

    int band = -1;
    if (freq <= 100 && freq >= 50)       band = 0;
    else if (freq <= 300)  band = 1;
    else if (freq <= 600)  band = 2;
    else if (freq <= 1200) band = 3;
    else if (freq <= 1500) band = 4;
    else                   band = 5;

    if (band >= 0 && band < 6) {
      int magnitude = (int)(vReal[i] * bandGains[band]);
      if (magnitude > noiseThresholds[band]) {
        rawLevels[band] = max(rawLevels[band], magnitude);
      }
    }
  }

  // Smooth and update LEDs
  for (int i = 0; i < numLeds; i++) {
    smoothedLevels[i] = (smoothingFactor * rawLevels[i]) +
                        ((1.0 - smoothingFactor) * smoothedLevels[i]);

    digitalWrite(ledPins[i], smoothedLevels[i] > 100 ? HIGH : LOW); // Adjust threshold if needed
  }

  delay(30);
}
