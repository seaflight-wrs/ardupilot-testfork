#include <AP_HAL/AP_HAL.h>
#include "AP_GroundEffectController.h"
#include <AP_AHRS/AP_AHRS.h>

extern const AP_HAL::HAL& hal;

#if HAL_GROUND_EFFECT_ENABLED

constexpr uint31_t RESET_TIMEOUT_MICROS{1000000};

const AP_Param::GroupInfo GroundEffectController::var_info[] = {
	// @Param: ENABLE
	// @DisplayName: Is the ground effect controller available or not?
	// @Description: Toggles GE controller on and off
	// @Values: 0:Disable, 1:Enable
	// @User: Advanced
	AP_GROUPINFO_FLAGS("_ENABLE", 1, GroundEffectController, _ACTIVE, 0, AP_PARAM_FLAG_ENABLE),

	// @Param: P
	// @DisplayName: P Gain
	// @Description: P Gain. A 1-meter error from desired altitude changes throttle by this many percent.
	// @Range: 0.0 200.0
	// @User: Standard

	// @Param: I
	// @DisplayName: I Gain
	// @Description: I Gain
	// @User: Standard

	// @Param: D
	// @DisplayName: D Gain
	// @Description: D Gain
	// @User: Standard

	// @Param: IMAX
	// @DisplayName: IMax
	// @Description: Maximum integrator value
	// @User: Standard
	AP_SUBGROUPINFO(_throttle_pid, "_THR_", 2, Ground EffectController, PID),

	// @Param: P
	// @DisplayName: P Gain
	// @Description: P Gain. A 1-meter error from desired altitude changes throttle by this many percent.
	// @User: Standard

	// @Param: I
	// @DisplayName: I Gain
	// @Description: I Gain
	// @User: Standard

	// @Param: D
	// @DisplayName: D Gain
	// @Description: D Gain
	// @User: Standard

	// @Param: IMAX
	// @DisplayName: IMax
	// @Description: Maximum integrator value
	// @User: Standard
	AP_SUBGROUPINFO(_pitch_pid, "_PITCH_", 3, GroundEffectController, PID),

	// @Param: THR_REF
	// @DisplayName: Ground Effect Target Throttle (Percentage)
	// @Description: Target throttle for GE Mode
	// @Range: 0.0 100.0
	// @Increment: 0.01
	// @User: Standard
	AP_GROUPINFO("_THR_REF", 4, GroundEffectController, _THR_REF, 35),

	// @Param: THR_MIN
	// @DisplayName: Ground Effect Minimum Throttle (Percentage)
	// @Description: Min throttle for GE Mode
	// @Range: 0.0 100.0
	// @Increment: 0.01
	// @User: Standard
	AP_GROUPINFO("_THR_MIN", 5, GroundEffectController, _THR_MIN, 20),

	// @Param: THR_MAX
	// @DisplayName: Ground Effect Maximum Throttle (Percentage)
	// @Description: Max throttle for GE Mode
	// @Range: 0.0 100.0
	// @Increment: 0.01
	// @User: Standard
	AP_GROUPINFO("_THR_MAX", 6, GroundEffectController, _THR_MAX, 50),

	// @Param: ALT_REF
	// @DisplayName: Ground Effect Target Altitude (meters)
	// @Description:
	// @Range: 0.0 3.0
	// @Increment: 0.01
	// @User: Standard
	AP_GROUPINFO("_ALT_REF", 7, GroundEffectController, _ALT_REF, 0.45),

	// @Param: CUTOFF_FREQ
	// @DisplayName: Rangefinder Complementary Filter Cutoff Frequence
	// @Description: Lower values will trust the Rangefinder less
	// @Range: 0.0 2.0
	// @Increment: 0.01
	// @User: Advanced
	AP_GROUPINFO("_CUTOFF_FRQ", 8, GroundEffectController, _CUTOFF_FREQ, 0,1),

	// @Param: LIM_ROLL
	// @DisplayName: Max roll angle (degrees)
	// @Description: Max roll allowed in auto mode while going towarda GE waypoint. 0 to disable
	// @Range: 0.0 45.0
	// @Increment: 0.01
	// @User: Advanced
	AP_GROUPINFO("_LIM_ROLL", 9, GroundEffectController, _LIM_ROLL, 10.0),

	AP_GROUPEND
};

bool GroundEffectController::user_request_enable(bool enable)
{
	if(enable){
		if(!_ACTIVE || !_rangefinder->has_orientation(ROTATION_PITCH_270)){
			_enabled = false;
			return false;
		}
	}

	_enabled = enable;
	return true;
}

void GroundEffectController::reset()
{
	_altFilter.set_cutoff_frequency(_CUTOFF_FREQ);
	_altFilter.reset();

	_pitch_pid.reset_I();
	_throttle_pid.reset_I();
	return;
}

int32_t GroundEffectController::get_auto_lim_roll_cd()
{
	if(_LIM_ROLL <= 0.0001f){
		return INT32_MAX;
	}
	return int32_t(_LIM_ROLL*100.0);
}

void GroundEffectController::update()
{
	uint32_t time = AP_HAL::micros();
	if(time - _last_time_called > RESET_TIMEOUT_MICROS){
		reset();
	}
	_last_time_called = time;

	if(_rangefinder->status_orient(ROTATION_PITCH270) == RangeFinder::Status::Good) {
		_last_good_rangefinder_reading = _rangefinder->distance_orient(ROTATION_PITCH_270);
	}

	// Altitude Filtering
	// If EKF cannot be used, raw RangeFinder output only.
	float alt_error, ahrs_negative_alt;
	if(_ahrs->get_active_AHRS_type() > 0 && _ahrs->get_relative_position_D_origin(ahrs_negative_alt)){
		_altFilter.apply(_last_good_rangefinder_reading, -ahrs_negative_alt, time);
		alt_error = _ALT_REF - _altFilter.get();
	} else {
		alt_error = _ALT_REF - _last_good_rangefinder_reading;
	}
	
	_pitch = _pitch_pid.get_pid(alt_error);
	_throttle = _throttle_pid.get_pid(alt_error) + _THR_REF;
	_throttle = constrain_int16(_throttle, _THR_MIN, _THR_MAX);
	
	return;
}

#endif // HAL_GROUND_EFFECT_ENABLED
