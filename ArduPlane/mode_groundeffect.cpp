#include "mode.h"
#include "Plane.h"

/*	
*	Desired hover altitude (cm).
* 	Ideally 1-2 meters to begin.
*/
// static constexpr float GROUND_EFFECT_TARGET_ALT_CM{50}; //start with 50cm - shouldn't this be a param?

/* 
*	Steady-state throttle required at given altitude to remain there.
*	For MkV this is likely 50-60 - assume 60 to be safe CHANGE LATER MAYBE
*	Must be between 0 and 100
*/
// static constexpr int16_t GROUND_EFFECT_STEADY_THROTTLE{60};

/*
*	Nose-up pitch target, in centidegrees
*	Sufficiently high pitch angle can render the rangefinder ineffective without compensation.
*	General rule of thumb is FOV/2 maximum. Need Benewake FOV...
*	Still unclear if this should be sign-flipped.
*/ 
static constexpr int32_t GROUND_EFFECT_PITCH_CENTIDEGREES{200}; //2 degrees for MkV

/* 
*	P Gain on Alt2Throttle P-conroller
*	For each cm below target altitude, throttle increases by this percentage (linear P)
*/
// static constexpr float GROUND_EFFECT_CONTROLLER_KP{10};

// TODO make these into parameter values instead of hard-coded

bool ModeGroundEffect::_enter()
{
	plane.throttle_allows_nudging = false;
	plane.auto_throttle_mode = false; //when enabled, this runs the TECS speed/height controller
	plane.auto_navigation_mode = false;

	// Verify a mm-precision-capable, downward-facing rangefinder is configured
	 if(!plane.rangefinder.has_mm_prec_orient(ROTATION_PITCH_270)){
		return false;
	 }

	// Verify that the rangefinder is capable of small distance measures. This fails with many.
	// if(plane.rangefinder.min_distance_cm_orient(ROTATION_PITCH_270) >=10){
	// 	return false;
	// }

	// Verify rangefinder health - fails if low, high, or no sensor data
	// if(plane.rangefinder.status_orient(ROTATION_PITCH_270) == RangeFinder::Status::Good){
	// 	return false;
	// }

	// Verify rangefinder health - last reading should not be more than 0.5s old
	// if((plane.rangefinder.last_reading_ms(ROTATION_PITCH_270) - AP_HAL::millis()) > 500){
	//	return false;
	// }

	// Verify that we are somewhat close to the desired altitude for the flight mode.
	// if(plane.rangefinder.distance_cm_orient(ROTATION_PITCH_270) > 5 * GROUND_EFFECT_TARGET_ALT_CM){
	//	return false;
	// }

	// Set P controller gain
	pAlt2Throttle(plane.g.gndEffect_kP);

	return true;
}

	/*
	* ModeGroundEffect::_enter() references the ModeGroundEffect class in mode.h
	* :: indicates we're "booling" a specific value for _enter()
	* this is represented in the class ModeGround Effect in mode.h
	*/

void ModeGroundEffect::update() //defining ModeGroundEffect function
{
	// Desire flat roll
	plane.nav_roll_cd = 0;
	// Desire some nose up pitch (for Mk V is 2 degrees)
	plane.nav_pitch_cd = GROUND_EFFECT_PITCH_CENTIDEGREES;
	// Pilot maintains control over rudder
	plane.steering_control.rudder = plane.channel_rudder->get_control_in_zero_dz();

	/*
	* TODO 
	* Consider filtering rangefinder output. See ../libraries/Filter
	* There may be a problem with rangefinder read freq
	* This method is a 400Hz method. Rangefinder (tfMini Plus/benewake) updates at 100Hz

	* Personal notes:
	* These functions (the bool and void) reference specific members of the ModeGroundEffect
	* class in mode.h, which are set to "override" meanining they override whatever it was based upon
	*/

	uint16_t altMm = plane.rangefinder.distance_mm_orient(ROTATION_PITCH_270);
	
	// If RC Throttle commanded is zero, don't run throttle controller at all (failsafe)
	// Flight can be stopped if throttle is cut, even in auto mode
	if(plane.get_throttle_input(false) == 0){
		SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
		return;
	}

	// float error = GROUND_EFFECT_TARGET_ALT_CM - ((float) altMm / 10.0);
	float error = plane.g.gndEffect_steady_Alt - ((float) altMm / 10.0);

	// int16_t commanded_throttle = GROUND_EFFECT_STEADY_THROTTLE + ((int16_t) pAlt2Throttle.get_p(error));
	int16_t commanded_throttle = plane.g.gndEffect_steady_thr + ((int16_6) pAlt2Throttle.get_p(error));

	// Commanded throttle must be within limits:
	commanded_throttle = constrain_int16(commanded_throttle, 0, 100);

	SRV_Channels::set_output_scaled(SRV_Channel::k_throttle,commanded_throttle);
}

