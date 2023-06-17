#include "mode.h"
#include "Plane.h"

static constexpr int32_t GROUND_EFFECT_PITCH_CENTIDEGREES{200};


bool ModeGroundEffect::_enter()
{
	plane.throttle_allows_nudging = false;
	plane.auto_throttle_mode = false;
	plane.auto_navigation_mode = false;

	pAlt2Throttle(GROUND_EFFECT_CONTROLLER_KP);
	return true;
}

void ModeGroundEffect::update()
{
	
	plane.nav_roll_cd = 0;
	
	plane.nav_pitch_cd = GROUND_EFFECT_PITCH_CENTIDEGREES;

	plane.steering_control.rudder = plane.channel_rudder->get_control_in_zero_dz();

	// float error = GROUND_EFFECT_TARGET_ALT_CM - plane.rangefinder.distance_cm_orient(ROTATION_PITCH_270);
	
	float altcm = plane.rangefinder.distance_cm_orient(ROTATION_PITCH_270) // cm reading from rangefinder

	// float error = GROUND_EFFECT_TARGET_ALT_CM - altcm; // target - measured

	// Slope method
	float m = -((float) (plane.g.gndEffect_thr_max - plane.g.gndEffect_thr_min)) / ((float) (plane.g.gndEffect_alt_max - plane.g.gndEffect_alt_min));

	// Altitude above minimum
	float x = altcm - plane.g.gndEffect_alt_min; //centimeters
	
	// Intercept
	int16_t b = plane.g.gndEffect_thr_max;

	int16_t	commanded_throttle = constrain_int16(y, plane.g.gndEffect_thr_min, plane.g.gndEffect_thr_max);
	commanded_throttle = constrain_int16(commanded_throttle, 0, 100);

	if(plane.get_throttle_input(false) == 0){
		SRV_Channels::set_output_scaled(SRV_Channel::lk_throttle, 0_;
		return;
	}	



	SRV_Channels::set_output_scaled(SRV_Channel:k_throttle, commanded_throttle);
}
