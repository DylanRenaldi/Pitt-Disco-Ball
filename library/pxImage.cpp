
#include "pxImage.hpp"
#include "disco.hpp"
#include <FastLED.h>

void pxi::drawImage(const uint8_t* image) {

	for (int c,y = 0,x,i = 0; y < MATRIX_HEIGHT; ++y) {
		for (x = 0; x < MATRIX_WIDTH; ++x) {
			for(c = 0; c < 3; ++c, ++i) {
				dlv::leds[dlv::xyIndexTable[x][y]][c] = image[i];
			}
		}
	}
	
	FastLED.show();
}