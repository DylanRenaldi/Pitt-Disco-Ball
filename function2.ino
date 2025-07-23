void displayReactiveBands(double *magnitudes) {
  // Dim all LEDs for trail effect
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].fadeToBlackBy(40);
  }

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

    // Remove noise floor and smooth band value
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