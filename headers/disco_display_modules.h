
// Visualize FFT result on LED matrix
unsigned displayReactiveBands(double *magnitudes) {

  // Logarithmically spaced frequency bands
  static const auto logBins = logspace(1, SAMPLES/2, MATRIX_WIDTH + 1);

  unsigned now = millis(), quantity = 0;
  for (int band = 0; band < MATRIX_WIDTH; band++) {
    int startBin = logBins[band];
    int endBin = logBins[band + 1];
    int binCount = endBin - startBin;

    double sum = 0;
    for (int i = startBin; i < endBin; i++) {
      sum += magnitudes[i];
    }
    double avg = sum / binCount;

    if (sum < 0) sum = -sum;
    int dB = -20 * log10(sum);
    int height = constrain(dB, 1, 120);
    height = map(height, 1, 120, 1, MATRIX_HEIGHT);

    // Serial.printf("%f | %i dB, %i \n", sum, dB, height);

    quantity += height - 1;

    // Peak hold logic
    if (height > 1 && height >= peakHeights[band]) {
      peakHeights[band] = height;
      lastPeakUpdate[band] = now;
    } else if (now - lastPeakUpdate[band] > peakHoldTime) {
      if(peakHeights[band] <= peakFallSpeed){
        peakHeights[band] = 0;
        continue;
      }

      peakHeights[band] -= peakFallSpeed;
    } // else continue;

    // Color mapping based on height
    //uint8_t hue = map(height, 0, MATRIX_HEIGHT, 160, 0);
    //CRGB barColor = CHSV(hue, 255, 255);
    int flippedBand = MATRIX_WIDTH - band - 1;  // Flip horizontally for visual symmetry

    // Draw vertical bar (starting 1 because height map is 1-offset)
    for (int y = 0, index; y < height; y++) {
      index = xyToIndex(flippedBand, MATRIX_HEIGHT - y);
      leds[index] = CRGB::Aqua; //colors[y - 1];
    }

    // Draw peak marker
    int peakY = MATRIX_HEIGHT - peakHeights[band];
    int peakIndex = xyToIndex(flippedBand, peakY);
    leds[peakIndex] = CRGB::Pink;
  }
  
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].fadeToBlackBy(40);  // Fading trail effect
  }

  return quantity;
}


// Visualize FFT result on LED matrix
void displayWaveform(double *values) {

  // Logarithmically spaced frequency bands
  static const auto logBins = logspace(1, SAMPLES/2, MATRIX_WIDTH + 1);

  // unsigned now = millis();
  for (int band = 0; band < MATRIX_WIDTH; band++) {
    int startBin = logBins[band];
    int endBin = logBins[band + 1];
    int binCount = endBin - startBin;

    double sum = 0;
    for (int i = startBin; i < endBin; i++) {
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
      index = xyToIndex(band, MATRIX_HEIGHT - y);
      leds[index] = colors[y - 1];
    }

    // Draw peak marker
    // int peakY = MATRIX_HEIGHT - peakHeights[band];
    int peakIndex = xyToIndex(flippedBand, height);
    leds[peakIndex] = peakColor;
  }

}


// Visualize FFT result on LED matrix
unsigned displayWaveSpectrum(double *magnitudes) {

  // Logarithmically spaced frequency bands
  static const auto logBins = logspace(1, SAMPLES/2, MATRIX_WIDTH + 1);

  unsigned now = millis(), quantity = 0;
  for (int band = 0; band < MATRIX_WIDTH; band++) {
    int startBin = logBins[band];
    int endBin = logBins[band + 1];
    int binCount = endBin - startBin;

    double sum = 0;
    for (int i = startBin; i < endBin; i++) {
      sum += magnitudes[i];
    }
    double avg = sum / binCount;

    if (sum < 0) sum = -sum;
    int dB = -20 * log10(sum);

    int height = constrain(dB, 1, 120);
    height = map(height, 1, 120, 1, MATRIX_HEIGHT >> 1);

    // Serial.printf("%f | %i dB, %i \n", sum, dB, height);

    quantity += 2*height - 1;

    // Color mapping based on height
    //uint8_t hue = map(height, 0, MATRIX_HEIGHT, 160, 0);
    //CRGB barColor = CHSV(hue, 255, 255);
    int flippedBand = MATRIX_WIDTH - band - 1;  // Flip horizontally for visual symmetry
    int lower = (MATRIX_HEIGHT >> 1) - height + 1;
    int upper = (MATRIX_HEIGHT >> 1) + height;

    // Draw vertical bar (starting 1 because height map is 1-offset)
    for (int index; lower < upper; ++lower) {
      index = xyToIndex(flippedBand, MATRIX_HEIGHT - lower);
      leds[index] = CRGB::Aqua; //colors[y - 1];
    }

    if(height == 1) continue;

    // Draw peak marker
    int peakY = MATRIX_HEIGHT - peakHeights[band];
    int peakIndex = xyToIndex(flippedBand, upper);
    int minIndex  = xyToIndex(flippedBand, (MATRIX_HEIGHT >> 1) - height);
    leds[peakIndex] = CRGB::Magenta;
    leds[minIndex] = CRGB::Magenta;
  }

  // Serial.println("\n");
  return quantity;
}