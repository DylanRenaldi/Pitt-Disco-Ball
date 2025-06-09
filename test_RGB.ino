#include <Adafruit_Protomatter.h>

// Matrix size
#define WIDTH 32
#define HEIGHT 16

// Your specified GPIO pins
int rgbPins[] = {25, 33, 27, 14, 12, 13}; // R1, G1, B1, R2, G2, B2
int addrPins[] = {16, 2, 32};             // A, B, C
int clockPin = 18;
int latchPin = 4;
int oePin = 22;

// Create matrix
Adafruit_Protomatter matrix(
  WIDTH, // width
  4,     // bit depth (brightness resolution)
  1,     // number of chained panels
  rgbPins, sizeof(rgbPins) / sizeof(rgbPins[0]),
  addrPins, sizeof(addrPins) / sizeof(addrPins[0]),
  clockPin, latchPin, oePin,
  false // no double buffering
);

void setup() {
  Serial.begin(115200);
  delay(500);

  // Start matrix
  ProtomatterStatus status = matrix.begin();
  if (status != PROTOMATTER_OK) {
    Serial.print("Matrix begin() failed: ");
    Serial.println(status);
    while (true);
  }

  matrix.setBrightness(128); // 0-255

  // Step 1: Fill screen with green
  matrix.fillScreen(matrix.color565(0, 255, 0));
  matrix.show();
  delay(1000);  // Pause 1 second

  // Step 2: Draw yellow border
  matrix.drawRect(0, 0, WIDTH, HEIGHT, matrix.color565(255, 255, 0));
  matrix.show();
  delay(1000);  // Pause 1 second

  // Step 3: Draw red X (diagonals)
  matrix.drawLine(0, 0, WIDTH - 1, HEIGHT - 1, matrix.color565(255, 0, 0));
  matrix.show();
  delay(500);  // Shorter pause before next line

  matrix.drawLine(WIDTH - 1, 0, 0, HEIGHT - 1, matrix.color565(255, 0, 0));
  matrix.show();
  delay(1000);  // Final pause
}

void loop() {
  // Nothing repeating
}
