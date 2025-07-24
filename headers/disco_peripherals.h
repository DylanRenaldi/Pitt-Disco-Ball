
#pragma once

// LED matrix and visualization parameters
#define MATRIX_WIDTH      	32
#define MATRIX_HEIGHT     	16
#define NUM_LEDS          	(MATRIX_WIDTH * MATRIX_HEIGHT)
#define DATA_PIN          	5

// Peripherals
#define INCREASE_BRIGHTNESS 17			// increase brightness button -> GPIO17
#define DECREASE_BRIGHTNESS 16			// decrease brightness button -> GPIO16
#define BLUETOOTH_LED       27			// bluetooth LED indicator    -> GPIO27

// I2S pin assignments
#define I2S_WS            	25			// Word Select (LRCLK)
#define I2S_SD            	32 			// Serial Data input
#define I2S_SCK           	33			// Bit Clock (BCLK)

// Audio sampling settings
#define SAMPLES				512			// sample buffer size 	(bandwidth is 0 < SAMPLES/2)
#define SAMPLING_FREQ		44100		// sample rate 			(bandwidth is SAMPLING_FREQ/2)

// Visualization state variables
#define peakHoldTime 		150			// Time in ms to hold peak before decay
#define peakFallSpeed  		1			// Amount to reduce peak per update
