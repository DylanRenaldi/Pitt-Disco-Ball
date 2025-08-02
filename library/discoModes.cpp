
#include "disco.hpp"			// disco namespace
#include "discoModes.hpp"		// discoModes implementation
#include "pxImage.hpp"			// for mode 8

#include <FastLED.h>
#include "fl/fill.h"			// fl::fill_solid
#include "lib8tion/random8.h"	// random8

#include <math.h>				// for log10


DEFINE_GRADIENT_PALETTE(firepal){
    0,   0,   0,   0,  // black (bottom of fire)
    32,  255, 0,   0,  // red (base of flames)
    190, 255, 255, 0,  // yellow (middle of flames)
    255, 255, 255, 255 // white (hottest part/tips of flames)
};


void setxyLEDColor(const uint8_t x, const uint8_t y, const struct CRGB& color) {
	const uint16_t index = dlv::xyIndexTable[x][y];
	dlv::leds[index] = color;
}


// Audio visualization variables
double smoothedBands[MATRIX_WIDTH] = {0};   // Smoothed FFT magnitudes for each band
double noiseFloor[MATRIX_WIDTH] = {0.05};   // Dynamic noise floor per band
int peakHeights[MATRIX_WIDTH] = {0};        // Keeps track of peak levels per band
unsigned long lastPeakUpdate[MATRIX_WIDTH] = {0};

// Beat detection variables
double prevLowEnergy = 0;
unsigned long lastBeatTime = 0;
int estimatedBPM = 120;           // Estimated beats per minute
int dynamicFrameInterval = 25;    // Controls how frequently LED updates (adaptive)


// mode 1 <- color fill based on dlv::colorProfile
void discoModes::solidColorFill() {
	if(dlv::colorProfile[0] == dlv::colorProfile[MATRIX_HEIGHT - 1]) {
		fl::fill_solid(dlv::leds, NUM_LEDS, dlv::colorProfile[0]);
	} else {
		for (uint8_t y = 0,x; y < MATRIX_HEIGHT; ++y) {
			for (x = 0; x < MATRIX_WIDTH; ++x) {
				setxyLEDColor(x, y, dlv::colorProfile[y]);
			}
		}
	}
}

// mode 2 <- color wave
void discoModes::colorWave() {
	static int offset = -1;
    offset++;
	for (int y = 0; y < MATRIX_HEIGHT; y++) {
		for (int x = 0; x < MATRIX_WIDTH; x++) {
			float wave = sin((x + offset) * 0.3 + millis() * 0.005);
			uint8_t brightness = map(wave * 100, -100, 100, 50, 255);
			setxyLEDColor(x, y, CHSV((x * 10 + offset) % 255, 255, brightness));
		}
	}
}


// mode 3 <- Audio Reactive FFT Visualization
void discoModes::audioReactiveVisualizer() {
	disco::I2S_FFT_data();

	// Attenuate high-frequency bins
	for (int i = 3; i < 36; i++) {
		dlv::vReal[i] *= 0.2;
	}

	// Beat detection from low-frequency energy
	double lowFreqEnergy = 0;
	for (int i = 1; i <= 10; i++) {
		lowFreqEnergy += dlv::vReal[i];
	}
	lowFreqEnergy /= 10.0;

	unsigned long now = millis();
	if (lowFreqEnergy > prevLowEnergy * 1.5 && now - lastBeatTime > 300) {
		// Beat detected
		unsigned long interval = now - lastBeatTime;
		lastBeatTime = now;
		estimatedBPM = 60000 / interval;
		dynamicFrameInterval = constrain(interval / 4, 15, 60);  // Adapt update rate
	}

	// Smooth energy to avoid jitter
	prevLowEnergy = 0.8 * prevLowEnergy + 0.2 * lowFreqEnergy;

	// Display the bands with custom colors
	for (int i = 0; i < NUM_LEDS; i++) {
		dlv::leds[i].fadeToBlackBy(40);
	}

	for (int band = 0; band < MATRIX_WIDTH; band++) {
		int startBin = dlv::logBins[band];
		int endBin = dlv::logBins[band + 1];
		int binCount = endBin - startBin;

		double sum = 0;
		for (int i = startBin; i < endBin; i++) {
			sum += dlv::vReal[i];
		}
		double avg = sum / binCount;

		if (avg < noiseFloor[band]) {
			noiseFloor[band] = noiseFloor[band] * 0.95 + avg * 0.05;
		} else {
			noiseFloor[band] = noiseFloor[band] * 0.9995 + avg * 0.0005;
		}

		double adjusted = avg - noiseFloor[band];
		if (adjusted < 0) adjusted = 0;

		smoothedBands[band] = 0.7 * smoothedBands[band] + 0.3 * adjusted;

		double scaled = log10(smoothedBands[band] + 1) * 10;
		int height = map((int)scaled, 0, 20, 1, MATRIX_HEIGHT);  
		height = constrain(height, 1, MATRIX_HEIGHT);
		if(height==1 && !peakHeights[band]) continue;


		// Peak hold logic
		if (height > 1 && height >= peakHeights[band]) {
			peakHeights[band] = height;
			lastPeakUpdate[band] = now;
		} else if (now - lastPeakUpdate[band] > peakHoldTime) {
			if(peakHeights[band] <= peakFallSpeed) {
				peakHeights[band] = 0;
				continue;
			}

			peakHeights[band] -= peakFallSpeed;
		} // else continue;

		// Color mapping based on height
		//uint8_t hue = map(height, 0, MATRIX_HEIGHT, 160, 0);
		//CRGB barColor = CHSV(hue, 255, 255);
		int flippedBand = MATRIX_WIDTH - 1 - band;  // Flip horizontally for visual symmetry

		// Draw vertical bar (starting 1 because height map is 1-offset)
		for (int y = 1, index; y < height; y++) {
		  setxyLEDColor(flippedBand, MATRIX_HEIGHT - y, dlv::colorProfile[y - 1]);
		}

		// Draw peak marker
		setxyLEDColor(flippedBand, MATRIX_HEIGHT - peakHeights[band], dlv::peakColor);
	}
}


// mode 4 <- Pulse in sync with low frequencies
void discoModes::lowFrequencyPulseSync() {
	disco::I2S_FFT_data();

	// Compute low frequency average
	double lowEnergy = 0;
	for (int i = 1; i <= 10; i++) {
		lowEnergy += dlv::vReal[i];
	}
	lowEnergy /= 10.0;

	// Smooth the pulse to avoid flickering
	static double smoothed = 0;
	smoothed = 0.8 * smoothed + 0.2 * lowEnergy;

	// Map to brightness or saturation
	uint8_t pulseVal = constrain(smoothed * 100, 10, 255);
	struct CRGB color = CHSV((millis() / 10) % 255, 255, pulseVal);

	fl::fill_solid(dlv::leds, NUM_LEDS, color);
}

uint8_t scalexy = 60;
uint8_t speed = 20;

// mode 5 <- Fire Matrix
void discoModes::fireMatrix() {
	CRGBPalette16 myPal = firepal;
    uint32_t now = millis();
    
    // Loop through every LED in our matrix
    for (uint8_t y,x = 0; x < MATRIX_WIDTH; x++) {
        for (y = 0; y < MATRIX_HEIGHT; y++) {
            
            uint8_t minuend    = inoise8(x * scalexy, y * scalexy + now, now / speed);
            uint8_t subtrahend = abs8(y - (MATRIX_HEIGHT - 1)) * 255 / (MATRIX_HEIGHT - 1);

			setxyLEDColor(x, y, ColorFromPalette(myPal, qsub8(minuend, subtrahend), disco::brightness));
        }
    }
}


// mode 6 <- Fire Glow (bass-driven flicker)
void discoModes::fireGlow() {
	disco::I2S_FFT_data();

	float bass = dlv::vReal[2] * 10;
	for (int y = 0; y < MATRIX_HEIGHT; y++) {
        for (int x = 0; x < MATRIX_WIDTH; x++) {
			uint8_t heat = random8(constrain((int)(bass * 255), 30, 255));
			setxyLEDColor(x, y, CHSV(map(heat, 0, 255, 0, 40), 255, heat));
        }
	}
}


// mode 7 <- Energy Snake
void discoModes::energySnake() {
	static int headX = 0;
	static int direction = 1;
	double amp = 0;
	for (int i = 3; i < 10; i++) amp += dlv::vReal[i];
	amp /= 7.0;
	int trailLength = constrain((int)(amp * 10), 1, MATRIX_WIDTH);
	
	fl::fill_solid(dlv::leds, NUM_LEDS, CRGB::Black);
	for (int t = 0; t < trailLength; t++) {
		int x = headX - t * direction;
		if (x >= 0 && x < MATRIX_WIDTH) {
			for (int y = 0; y < MATRIX_HEIGHT; y++) {
				setxyLEDColor(x, y, CHSV((millis() / 5 + y * 5) % 255, 255, 255 - t * 20));
			}
		}	
	}

	headX += direction;
	if (headX >= MATRIX_WIDTH || headX < 0) {
		direction *= -1;
		headX += direction;
	}
}



// mode 8 <- Image Frame
void discoModes::imageFrame(const uint8_t& diff, bool& update) {
	
	// initialize the index variable and the time variable with the correct typename size
	static auto ix   = pxi::IMAGE_COUNT;
	static auto time = pxi::IMAGE_TIME;
	
	if (update || !time) {
		update = false;
		time = pxi::IMAGE_TIME;				// reset variables
		++ix %= pxi::IMAGE_COUNT;
		
		pxi::drawImage(pxi::images[ix]);	// calls FastLED.show()
	} else {
		time = (time > diff ? time - diff : 0);
	}
}

// Visualize FFT result on LED matrix
void discoModes::waveSpectrumVisualizer() {
	disco::I2S_FFT_data();

	// Attenuate high-frequency bins
	for (int i = 3; i < 36; i++) {
		dlv::vReal[i] *= 0.2;
	}

	// Beat detection from low-frequency energy
	double lowFreqEnergy = 0;
	for (int i = 1; i <= 10; i++) {
		lowFreqEnergy += dlv::vReal[i];
	}
	lowFreqEnergy /= 10.0;

	unsigned long now = millis();
	if (lowFreqEnergy > prevLowEnergy * 1.5 && now - lastBeatTime > 300) {
		// Beat detected
		unsigned long interval = now - lastBeatTime;
		lastBeatTime = now;
		estimatedBPM = 60000 / interval;
		dynamicFrameInterval = constrain(interval / 4, 15, 60);  // Adapt update rate
	}

	// Smooth energy to avoid jitter
	prevLowEnergy = 0.8 * prevLowEnergy + 0.2 * lowFreqEnergy;


  // Display the bands with custom colors
	for (int i = 0; i < NUM_LEDS; i++) {
		dlv::leds[i].fadeToBlackBy(40);
	}

	for (int band = 0; band < MATRIX_WIDTH; band++) {
		int startBin = dlv::logBins[band];
		int endBin = dlv::logBins[band + 1];
		int binCount = endBin - startBin;

		double sum = 0;
		for (int i = startBin; i < endBin; i++) {
			sum += dlv::vReal[i];
		}
		double avg = sum / binCount;

		if (avg < noiseFloor[band]) {
			noiseFloor[band] = noiseFloor[band] * 0.95 + avg * 0.05;
		} else {
			noiseFloor[band] = noiseFloor[band] * 0.9995 + avg * 0.0005;
		}

		double adjusted = avg - noiseFloor[band];
		if (adjusted < 0) adjusted = 0;
    

		smoothedBands[band] = 0.7 * smoothedBands[band] + 0.3 * adjusted;
		double scaled = 20 * log10(smoothedBands[band] + 1);

		int height = constrain((int)scaled, 1, 20);
		height = map(height, 1, 20, 1, MATRIX_HEIGHT >> 1);

		if(height == 1) continue;

		int flippedBand = MATRIX_WIDTH - band - 1;  // Flip horizontally for visual symmetry
		int lower = (MATRIX_HEIGHT >> 1) - height;
		int upper = (MATRIX_HEIGHT >> 1) + height - 1;

		// lower: (16 - 1 -> 15), (16 - 16 -> 0)           -> {15, 0}
		// upper: (16 + 1 - 1 -> 16), (16 + 16 - 1 -> 31)  -> {16, 31}

		// Draw vertical bar (starting 1 because height map is 1-offset)
		for (int index = lower; index <= upper; ++index) {
			setxyLEDColor(flippedBand, index, CRGB::Aqua);
		}

		// Draw peak marker
		setxyLEDColor(flippedBand, upper, CRGB::Magenta);
		setxyLEDColor(flippedBand, lower, CRGB::LawnGreen);
	}
}
