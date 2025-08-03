
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
	
	//disco::mode = mode_init;
	//disco::brightness = 10;

	dlv::FFT = ArduinoFFT<double>(dlv::vReal, dlv::vImag, SAMPLES, SAMPLING_FREQ);
	dlv::ESP_BT.begin("Pitt-DiscoLamp");
	setupI2S();

	FastLED.addLeds<WS2812B, MATRIX_DATA_PIN, GRB>(dlv::leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
	FastLED.setBrightness(10);

	pinMode(INCREASE_BRIGHTNESS, INPUT_PULLDOWN);
	pinMode(DECREASE_BRIGHTNESS, INPUT_PULLDOWN);
	pinMode(BLUETOOTH_LED, OUTPUT);

	disco::setColorProfile(CRGB::Aqua, CRGB::Fuchsia);			// -> disco:gradient <- colors
}

void disco::update(const uint8_t& diff, bool& update) {	
	
	switch (disco::mode) {

		case 1: discoModes::solidColorFill();
			break;

		case 2: discoModes::colorWave();
			break;
			
		case 3: discoModes::audioReactiveVisualizer();
			break;
			
		case 4: discoModes::lowFrequencyPulseSync();
			break;
		
		case 5: discoModes::fireMatrix();
			break;

		case 6: discoModes::fireGlow();
			break;

		case 7: discoModes::energySnake();
			break;

		case 8: discoModes::waveSpectrumVisualizer();
			break;

		case 9: discoModes::imageFrame(diff, update);	// calls FastLED.show() when update = true
			return;		// image is static -> dont need to update each iteration
	}
	
	FastLED.show();
}

void disco::setColorProfile(struct CRGB begin, const struct CRGB& end){

	float dr = float(end.r - begin.r)/(MATRIX_HEIGHT - 1),
		  dg = float(end.g - begin.g)/(MATRIX_HEIGHT - 1),
          db = float(end.b - begin.b)/(MATRIX_HEIGHT - 1);  

	for(uint8_t i = 0; i < MATRIX_HEIGHT; ++i, begin.r += dr, begin.g += dg, begin.b += db)
		dlv::colorProfile[i] = begin;
	
	dlv::colorProfile[MATRIX_HEIGHT - 1] = end;	// manually set end CRGB value because of float->int round-off error
}

void disco::readBluetooth(bool& update, bool debug) {

	static String incomingData = "";
	static CRGB RGB1,RGB2;

	for (char c, ix{},temp; dlv::ESP_BT.available() && c != '\n';) {
		c = dlv::ESP_BT.read();

		if (c != ',') {
			incomingData += c;
		} else {
			temp = incomingData.toInt();

			if (ix < 6) {
				(ix < 3 ? RGB1 : RGB2)[ix%3] = temp;
			} else if (ix == 6) {
				update = (disco::mode != temp);					// indicate mode update
				disco::mode = temp;
			} else {
				temp = constrain(temp, 0, 100);					// assert brightnes 0 < brightness < 100
				update = update || (disco::brightness != temp);
				disco::brightness = temp;
				FastLED.setBrightness(temp);					// brightness updated here, can be read via FastLED.getBrightness()
			}	  

			++ix;
			incomingData = "";
		}
	}
	
	if (update) {
		FastLED.clear();
	}

	// set gradient if gradient bounds are different
	if(RGB1 != dlv::colorProfile[0] || RGB2 != dlv::colorProfile[MATRIX_HEIGHT - 1]) {
		disco::setColorProfile(RGB1, RGB2);
	}

	incomingData = "";
	while (dlv::ESP_BT.available()) dlv::ESP_BT.read();       // ignore extra data if an update was sent too quickly
}

void disco::debounceButtons(const uint8_t& diff, bool& update) {

	static uint8_t debounce = 0;

	if(!debounce) {
		uint8_t b = disco::brightness;
		if (digitalRead(INCREASE_BRIGHTNESS) && digitalRead(DECREASE_BRIGHTNESS)) {
			update = true;
			FastLED.clear();
			debounce = 250;
			disco::mode = (disco::mode%9) + 1;
		}
		
		else if(digitalRead(INCREASE_BRIGHTNESS) && b < 100)    // GPIO17
			FastLED.setBrightness(++b);

		else if (digitalRead(DECREASE_BRIGHTNESS) && b)    // GPIO16
			FastLED.setBrightness(--b);

		if(b != disco::brightness){
			disco::brightness = b;
			// Serial.println(b);
			debounce = 50;
		}
	} else {
		debounce = (debounce > diff ? debounce - diff : 0);
	}
}


void disco::checkBluetooth(const uint8_t &diff) {

	static bool connected = false;
	static uint8_t iterations = 0,update = disco::mode;							// initialize to avoid blinking on first iteration
	static uint16_t ON_BUFFER  = 0,
					OFF_BUFFER = 0;

	if (update != disco::mode) {
		OFF_BUFFER = BLINK_INTERVAL * bool(iterations | ON_BUFFER);			// 100 ms if BLUETOOTH_LED is logically on in any facet
		iterations = update = disco::mode;
	}
	
	if (dlv::ESP_BT.hasClient() != connected) {
		connected = !connected;
		ON_BUFFER = BT_CONNECTION;
	}

	if (ON_BUFFER) {	// ON_BUFFER takes priority
		ON_BUFFER = (ON_BUFFER > diff ? ON_BUFFER - diff : 0);
		digitalWrite(BLUETOOTH_LED, ON_BUFFER > 0);

	} else if (OFF_BUFFER) {		// necessary between mode-update blink iterations
		OFF_BUFFER = (OFF_BUFFER > diff ? OFF_BUFFER - diff : 0);

	} else if (iterations) {
		ON_BUFFER = OFF_BUFFER = BLINK_INTERVAL;
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
		dlv::vReal[i] = (samples[i] >> 14) / 2048.0;
		dlv::vImag[i] = 0; // set to 0 to avoid erroneous errors and overflow
	}

	// Apply windowing and perform FFT
	dlv::FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
	dlv::FFT.compute(FFTDirection::Forward);
	dlv::FFT.complexToMagnitude();
}



