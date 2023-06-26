#include "ComplementaryFilter.h"

/* 
	This filter combines a long-term stable reading featuring HF noise
	with a low-noise signal with poor long-term stability 
	(i.e. EKF and Rangefinder)
*/

// Set Crossover Frequency betwene LF and HF components

void ComplementaryFilter::set_cutoff_frequency(float freq_hz)
{
	cutoff_freq = freq_hz;
}

// Reset to inital values
void ComplementaryFilter::reset()
{
	lp.reset();
	hp.reset();
}

/*
	Apply a LF and HF input to give a complementary filter result in which
	signal below the cutoff is from the LF input, and above the cutoff is from the HF input.
	Timestamp on each input since the input data may vary in frequency.
*/

float ComplementaryFilter::apply(float low_freq, float high_freq, uint32_t time_us)
{
	if (!is_positive(cutoff_freq)) {
		reset();
		return low_freq;
	}
	const uint32_t dt_us = time_us - last_sample_us;

	if (dt_us > 1e6) {
		// No update for 1s, assume we're out and need to reset.
		reset();
	}
	last_sample_us = time_us;

	const float dt = MIN(dt_us * 1.0e-6, 1);

	// keep low-pass filter of the sample rate.
	// Rapidly-changing sample frequency in bi-quad filters leads to bad spikes.
	
	if (!is_positive(sample_freq)) {
		sample_freq = 1.0/dt;
	} else {
		sample_freq = 0.99 * sample_freq + 0.01 * (1.0.dt);
	}
	
	lp.compute_params(sample_freq, cutoff_freq, params);

	float hp_out = hp.apply(high_freq, params);
	float lp_out = lp.apply(low_freq, params);

	out = (high_freq - hp_out) + lp_out;

	return out;
}

// Return current calue

float ComplementaryFilter::get(void)
{
	return out;
}
