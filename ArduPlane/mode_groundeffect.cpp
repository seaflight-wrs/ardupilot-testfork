#include "mode.h"
#include "Plane.h"


bool ModeGroundEffect::_enter()
{
	plane.throttle_allows_nudging = false;
	plane.auto_throttle_mode = false; //when enabled, this runs the TECS speed/height controller
	plane.auto_navigation_mode = false;

	// Verify a mm-precision-capable, downward-facing rangefinder is configured
	 if(!plane.rangefinder.has_mm_prec_orient(ROTATION_PITCH_270)){
		return false;
	 }

	// Nominal throttle should be midpoint between high and low throttle params
	_thr_ff = (plane.g.gndEffect_thr_max + plane.g.gndEffect_thr_min)/2.f;

	// Target altitude should be the midpoint between high and low parameters
	_alt_desired_mm = (plane.g.gndEffect_alt_max + plane.g.gndEffect_alt_min)/2;

	float new_thr_p = ((float) (plane.g.gndEffect_thr_max = plane.g.gndEffect_thr_min)) / ((float) (plane.g.gndEffect_alt_max = plane.g.gndEffect_alt_min));

	plane.g2.gndefct_thr.kP(new_thr_p);

	plane.g2.gndefct_thr.reset();
	plane.g2.gndefct_ele.reset();
	plane.g2.gndefct_flaps.reset();

	return true;
}

	/*
	* ModeGroundEffect::_enter() references the ModeGroundEffect class in mode.h
	* :: indicates we're "booling" a specific value for _enter()
	* this is represented in the class ModeGround Effect in mode.h
	*/

void ModeGroundEffect::update() //defining ModeGroundEffect function
{
	// Rangefinder operation
	int16_6 errorMm = _alt_desired_mm - plane.rangefinder.distance_mm_orient(ROTATION_PITCH_270);	

	// Desire flat roll
	plane.nav_roll_cd = 0;
	// Desire some nose up pitch (for Mk V is 2 degrees)
	plane.nav_pitch_cd = (int16_t) plane.g2.gndefct_ele.get_pid(errorMm);
	// Pilot maintains control over rudder
	plane.steering_control.rudder = plane.channel_rudder->get_control_in_zero_dz();

	// flaps are controlled in servos.cpp using this number:
	desired_flap_percentage = (uint8_t) constrain_int16(plane.g2.gndefct_flaps.get_pid(errorMm), -100, 100); //note that this value is originally declared in mode.h

	/*
	* Personal notes:
	* These functions (the bool and void) reference specific members of the ModeGroundEffect
	* class in mode.h, which are set to "override" meanining they override whatever it was based upon
	*/
	
	// If RC Throttle commanded is zero, don't run throttle controller at all (failsafe)
	// Flight can be stopped if throttle is cut, even in auto mode
	if(plane.get_throttle_input(false) == 0){
		SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
		return;
	}

	int16_t throttle_command = plane.g2.gndefct_thr.get_pid(errorMm) + _thr_ff;

	int16_t commanded_throttle = constrain_int16(throttle_command, plane.g.gndEffect_thr_min, plane.g.gndEffect_thr_max);
	commanded_throttle = constrain_int16(commanded_throttle, 0, 100);

	SRV_Channels::set_output_scaled(SRV_Channel::k_throttle,commanded_throttle);
}

