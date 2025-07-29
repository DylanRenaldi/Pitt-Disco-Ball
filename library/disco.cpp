
#include "disco.hpp"

#include <driver/i2s.h>
#include <FastLED.h>

// Initializes I2S for microphone input
constexpr void setupI2S() {

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


void disco::init() {

	disco::FFT = ArduinoFFT<double>(disco::vReal, disco::vImag, SAMPLES, SAMPLING_FREQ);
	disco::ESP_BT.begin("Pitt-DiscoLamp");
	setupI2S();

	FastLED.addLeds<WS2812B, MATRIX_DATA_PIN, GRB>(disco::leds, NUM_LEDS);
	FastLED.setBrightness(10);
	FastLED.clear();

	pinMode(INCREASE_BRIGHTNESS, INPUT_PULLDOWN);
	pinMode(DECREASE_BRIGHTNESS, INPUT_PULLDOWN);
	pinMode(BLUETOOTH_LED, OUTPUT);

	disco::setGradient(CRGB::Aqua, CRGB::Fuchsia);			// -> disco:gradient <- colors
}

void disco::setGradient(CRGB begin, CRGB end){

	float dr = float(end.r - begin.r)/(MATRIX_HEIGHT - 1),
		  dg = float(end.g - begin.g)/(MATRIX_HEIGHT - 1),
          db = float(end.b - begin.b)/(MATRIX_HEIGHT - 1);  

	// last color doesn't matter (if unsigned overflow) because of peak marker
	for(uint8_t i = 0; i < MATRIX_HEIGHT; ++i, begin.r += dr, begin.g += dg, begin.b += db)
		disco::gradient[i] = begin;
}


void disco::debounceButtons(const uint8_t &diff) {

	static uint8_t debounce = 0, brightness = 10;

	if(!debounce) {
		uint8_t b = brightness;
		if(digitalRead(INCREASE_BRIGHTNESS) && brightness < 100)    // GPIO17
			FastLED.setBrightness(++brightness);

		else if (digitalRead(DECREASE_BRIGHTNESS) && brightness)    // GPIO16
			FastLED.setBrightness(--brightness);

		if(b != brightness){
			//Serial.println(brightness);
			debounce = 100;
		}
	} else {
		debounce = (debounce > diff ? debounce - diff : 0);
	}
}


void disco::checkBluetooth(const uint8_t &diff, const uint8_t &mode) {

	static bool connected = false;
	static uint8_t iterations = 0,update = mode;							// initialize to avoid blinking on first iteration
	static uint16_t ON_BUFFER  = 0,
									OFF_BUFFER = 0;

	if (update != mode) {
		OFF_BUFFER = 100 * bool(iterations | ON_BUFFER);				// 100 ms if BLUETOOTH_LED is logically on in any facet
		iterations = update = mode;
	}
	
	if (disco::ESP_BT.hasClient() != connected) {
		connected = !connected;
		ON_BUFFER = 1000;																			// 1000 ms
	}

	if (ON_BUFFER) {	// ON_BUFFER takes priority
		ON_BUFFER = (ON_BUFFER > diff ? ON_BUFFER - diff : 0);
		digitalWrite(BLUETOOTH_LED, ON_BUFFER > 0);

	} else if (OFF_BUFFER) {		// necessary between mode-update blink iterations
		OFF_BUFFER = (OFF_BUFFER > diff ? OFF_BUFFER - diff : 0);

	} else if (iterations) {
		ON_BUFFER = OFF_BUFFER = 100;
		--iterations;
	}
}

bool disco::frameInterval(uint8_t &diff) {

	static unsigned long lastFrame = millis();
	static const uint8_t frameInterval = 25;          		// Target ~40 FPS

	diff = millis() - lastFrame;            			// save diff because it may change in midst of below comparison and cause an erroneous error

	if (diff < frameInterval) return true;
	lastFrame = millis();
	
	return false;
}


void disco::I2S_FFT_data() {

	static int32_t samples[SAMPLES];
	static size_t bytes_read = 0;

	i2s_read(I2S_NUM_0, (void*)samples, SAMPLES * sizeof(int32_t), &bytes_read, portMAX_DELAY);

	// Convert raw I2S samples to normalized floats
	for (uint16_t i = 0, samples_read = bytes_read / sizeof(int32_t); i < samples_read; ++i) {
		// peak amplitude
		// vReal[i] = samples[i] / double((1 << 31) - 1);
		disco::vReal[i] = (samples[i] >> 14) / 2048;
		disco::vImag[i] = 0; // set to 0 to avoid erroneous errors and overflow
	}

	// Apply windowing and perform FFT
	disco::FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
	disco::FFT.compute(FFTDirection::Forward);
	disco::FFT.complexToMagnitude();
}


