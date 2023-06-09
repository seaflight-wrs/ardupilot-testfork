#include "mode.h"
#include "Plane.h"

bool ModeGroundEffect::_enter()
{
	plane.throttle_allows_nudging = false;
	plane.auto_throttle_mode = false;
	plane.auto_navigation_mode = false;

	// TODO maybe fail if alt readings too high

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
	plane.nav_pitch_cd = 200; //FIXME - should this be negative?
	// Pilot maintains control over rudder
	plane.steering_control.rudder = plane.channel_rudder->get_control_in_zero_dz();

	/*
	* TODO implement closed-look altitude-throttle control here
	* Probably create a new controller using AC_PID
	* Get range from plane.rangefinder_state.last_distance. In cm, 10Hz
	* Not sure how to set throttle - auto_throttle_mode=True?

	* Personal notes:
	* These functions (the bool and void) reference specific members of the ModeGroundEffect
	* class in mode.h, which are set to "override" meanining they override whatever it was based upon
	*/
}

