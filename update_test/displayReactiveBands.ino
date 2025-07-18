// is marginally more efficient, but mainly (hopefully) resolves the peak marker stopping before the lowest bin

scaled = constrain(scaled, 0, MATRIX_HEIGHT);
int height = map((int)scaled, 0, 20, 1, MATRIX_HEIGHT);
if(height==1 && !peakHeights[band]) continue;

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
int flippedBand = 31 - band;  // Flip horizontally for visual symmetry

// Draw vertical bar (starting 1 because height map is 1-offset)
for (int y = 1, index; y < height; y++) {
  index = xyToIndex(flippedBand, MATRIX_HEIGHT - y);
  leds[index] = colors[y - 1];
}

// Draw peak marker
int peakY = MATRIX_HEIGHT - peakHeights[band];
int peakIndex = xyToIndex(flippedBand, peakY);
leds[peakIndex] = CRGB::White;
