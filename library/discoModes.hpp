
#pragma once

#include <crgb.h>

namespace discoModes {
	
	void audioReactiveVisualizer();
	void lowFrequencyPulseSync();
	void solidColorFill();
	void energySnake();
	void rippleBeat();
	void colorWave();
	void fireGlow();
	
	inline struct CRGB peakColor = CRGB::White;
	
}