#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <math.h>

#define MIC_PIN 36
#define SAMPLES 64
#define SAMPLING_FREQ 4000  // 4 kHz

float vReal[SAMPLES];
float vImag[SAMPLES];

// Matrix panel setup
HUB75_I2S_CFG mxconfig(32, 16, 1);
MatrixPanel_I2S_DMA matrix(mxconfig);

// Bar colors
uint16_t colors[] = {
  matrix.color565(255, 0, 0),
  matrix.color565(255, 128, 0),
  matrix.color565(255, 255, 0),
  matrix.color565(0, 255, 0),
  matrix.color565(0, 255, 255),
  matrix.color565(0, 0, 255),
  matrix.color565(128, 0, 255),
  matrix.color565(255, 0, 255)
};

// --- Basic Radix-2 FFT implementation ---
void simpleFFT(float *real, float *imag, int N) {
  int i, j, k, n, m;
  int M = log2(N);
  float tReal, tImag, uReal, uImag, angle;

  // Bit reversal
  j = 0;
  for (i = 0; i < N - 1; i++) {
    if (i < j) {
      tReal = real[i];
      real[i] = real[j];
      real[j] = tReal;
      tImag = imag[i];
      imag[i] = imag[j];
      imag[j] = tImag;
    }
    k = N / 2;
    while (k <= j) {
      j -= k;
      k /= 2;
    }
    j += k;
  }

  // FFT
  for (m = 1; m <= M; m++) {
    int step = pow(2, m);
    float theta = -2.0 * PI / step;
    for (k = 0; k < N; k += step) {
      for (n = 0; n < step / 2; n++) {
        angle = theta * n;
        float cosA = cos(angle);
        float sinA = sin(angle);
        int i1 = k + n;
        int i2 = k + n + step / 2;

        tReal = cosA * real[i2] - sinA * imag[i2];
        tImag = sinA * real[i2] + cosA * imag[i2];
        uReal = real[i1];
        uImag = imag[i1];

        real[i1] = uReal + tReal;
        imag[i1] = uImag + tImag;
        real[i2] = uReal - tReal;
        imag[i2] = uImag - tImag;
      }
    }
  }
}
// --- End FFT ---

void setupMatrixPins() {
  mxconfig.gpio.r1 = 25;
  mxconfig.gpio.g1 = 33;
  mxconfig.gpio.b1 = 27;
  mxconfig.gpio.r2 = 14;
  mxconfig.gpio.g2 = 12;
  mxconfig.gpio.b2 = 13;
  mxconfig.gpio.a  = 0;
  mxconfig.gpio.b  = 2;
  mxconfig.gpio.c  = 32;
  mxconfig.gpio.clk = 18;
  mxconfig.gpio.lat = 4;
  mxconfig.gpio.oe  = 15;
}

void setup() {
  setupMatrixPins();
  matrix.begin();
  matrix.setBrightness8(64);
  Serial.begin(115200);
}

void loop() {
  // Sample mic
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = analogRead(MIC_PIN);
    vImag[i] = 0;
    delayMicroseconds(1000000 / SAMPLING_FREQ);
  }

  // Run FFT
  simpleFFT(vReal, vImag, SAMPLES);

  // Clear matrix
  matrix.fillScreen(0);

  // Visualize FFT bins
  for (int i = 2; i < 18; i++) { // skip DC and low bins
    float magnitude = sqrt(vReal[i] * vReal[i] + vImag[i] * vImag[i]);
    int height = map((int)magnitude, 0, 3000, 0, 16);
    height = constrain(height, 0, 16);
    int x = (i - 2) * 2;
    for (int y = 0; y < height; y++) {
      matrix.drawPixel(x, 15 - y, colors[y / 2]);
      matrix.drawPixel(x + 1, 15 - y, colors[y / 2]);
    }
  }

  delay(20);
}
