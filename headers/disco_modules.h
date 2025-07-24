
#pragma once

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

// Converts 2D matrix coordinates to 1D LED index for 4-tile layout
uint16_t xyToIndex(uint8_t x, uint8_t y){
	uint16_t m = 256*(x < 16);
	
	if(y&1)
		return (271 + 16*y - x%16)^m;
	
	return (256 + 16*y + x%16)^m;
}

void gradient(CRGB *out, CRGB begin, CRGB end){

  float dr = float(end.r - begin.r)/(MATRIX_HEIGHT - 1),
        dg = float(end.g - begin.g)/(MATRIX_HEIGHT - 1),
        db = float(end.b - begin.b)/(MATRIX_HEIGHT - 1);  

  // last color doesn't matter (if unsigned overflow) because of peak marker
  for(uint8_t i = 0; i < MATRIX_HEIGHT; ++i, begin.r += dr, begin.g += dg, begin.b += db)
    out[i] = begin;
}

const std::array<uint16_t, MATRIX_WIDTH + 1> logspace(double start, double stop, int numbins){
	
	double ratio = log10(stop / start);
	std::array<uint16_t, MATRIX_WIDTH + 1> bins;

	for (int i = 0; i < numbins; ++i) {
    bins[i] = start * pow(10.0, ratio * i / (numbins - 1));
    if (i && bins[i] <= bins[i - 1]) bins[i] = 1 + bins[i - 1];
  }

  return bins;
}

bool frameInterval(uint8_t BRIGHTNESS) {
	
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

	static unsigned lastFrame = 0, diff;
	static const int frameInterval = 25;          		// Target ~40 FPS

	diff = millis() - lastFrame;            				// save diff because it may change in midst of below comparison and cause an erroneous error

	if (diff < frameInterval) return true;	
	debounce -= (debounce >= diff ? diff : debounce);     // only change debounce after frameInterval (otherwise, debounce will be compoundingly reduced at the clock rate)
	lastFrame = millis();
	
	return false;
}

void I2S_FFT_data(ArduinoFFT<double> FFT, double *vReal, double *vImag) {
	int32_t samples[SAMPLES];
    size_t bytes_read = 0;
    // unsigned startMicros = micros(); // Track latency

    i2s_read(I2S_NUM_0, (void*)samples, SAMPLES * sizeof(int32_t), &bytes_read, portMAX_DELAY);
    int samples_read = bytes_read / sizeof(int32_t);

    // Convert raw I2S samples to normalized floats
    for (int i = 0; i < samples_read; i++) {
      // peak amplitude
      vReal[i] = samples[i] / double((1 << 31) - 1);
      vImag[i] = 0; // set to 0 to avoid erroneous errors and overflow
    }

	// assuming FFT object: FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

    // Apply windowing and perform FFT
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();
}
