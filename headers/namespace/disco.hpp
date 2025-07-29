
#pragma once

#include "disco_peripherals.h"

#include <arduinoFFT.h>
#include <crgb.h>
#include <array>

namespace disco {
	const std::array<uint16_t, MATRIX_WIDTH + 1> logspace(double, double, int);
	void I2S_FFT_data(ArduinoFFT<double>&, double*, double*);
	void checkBluetooth(const uint8_t&, const uint8_t&);
	//void readBluetooth
	void debounceButtons(const uint8_t&);
	void gradient(CRGB *, CRGB, CRGB);
	bool frameInterval(uint8_t&);
	
	//etc.

	void init();
};