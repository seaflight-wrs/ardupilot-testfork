#include "mode.h"
#include "Plane.h"

bool ModeGroundEffect::_enter()
{
	// plane.throttle_allows_nudging = false; // apparently not in Plane anymore?
	// plane.auto_throttle_mode = false;
	// plane.auto_navigation_mode = false;

	_thr_ff = (plane.g.gndEffect_thr_max + plane.g.gndEffect_thr_min)/2.f;

	_alt_desired_cm = (plane.g.gndEffect_alt_max + plane.g.gndEffect_alt_min)/2;

	float new_thr_p = ((float) (plane.g.gndEffect_thr_max - plane.g.gndEffect_thr_min)) / ((float) (plane.g.gndEffect_alt_max = plane.g.gndEffect_alt_min));

	plane.g2.gndefct_thr.kP(new_thr_p);

	plane.g2.gndefct_thr.reset();
	plane.g2.gndefct_ele.reset();
	plane.g2.gndefct_flaps.reset();
	
	//pAlt2Throttle(GROUND_EFFECT_CONTROLLER_KP);
	return true;
}

void ModeGroundEffect::update()
{
	float altcm = plane.rangefinder.distance_cm_orient(ROTATION_PITCH_270); // cm reading from rangefinder
	int16_t errorcm = _alt_desired_cm - altcm;
	
	plane.nav_roll_cd = 0;
	
	plane.nav_pitch_cd = (int16_t) plane.g2.gndefct_ele.get_pid(errorcm);

	plane.steering_control.rudder = plane.channel_rudder->get_control_in_zero_dz();

	desired_flap_percentage = (int8_t) constrain_int16(plane.g2.gndefct_flaps.get_pid(errorcm), -100, 100);


	if(plane.get_throttle_input(false) == 0){
		SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
		return;
	}	

	int16_t throttle_command = plane.g2.gndefct_thr.get_pid(errorcm) + _thr_ff;

	int16_t commanded_throttle = constrain_int16(throttle_command, plane.g.gndEffect_thr_min, plane.g.gndEffect_thr_max);
	commanded_throttle = constrain_int16(commanded_throttle, 0, 100);

	SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, commanded_throttle);
}
