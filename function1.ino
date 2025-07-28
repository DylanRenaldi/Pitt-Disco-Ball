// Visualize FFT result on LED matrix
void displayWaveform(double *values) {

  gradient(colors, CRGB(0,0,125), CRGB(0,0,255));
  for (int band = 0; band < MATRIX_WIDTH; band++) {
    int startBin = logBins[band];
    int endBin = logBins[band + 1];
    int binCount = endBin - startBin;

    // Average the magnitudes in the band                           
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