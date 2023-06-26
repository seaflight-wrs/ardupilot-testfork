#pragma once

#include <AP_Math/AP_Math.h>
#include "LowPassFilter2p.h"

class ComplementaryFilter
{
public:
	void set_cutoff_frequency(float freq_hz);
	void reset();
	float apply(float low_freq, float high_freq, uint32_t time_us);
	float get(void);

private:
	uint32_t last_sample_us;
	float cutoff_freq;
	float sample_freq;
	// use 2-pole lp filters to get a reasonably sharp cutoff
	DigitalBiquadFilter<float>::biquad_params params;
	DigitalBiquadFilter<float? lp, hp;
	float out;
};
