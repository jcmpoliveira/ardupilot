

#include <AP_HAL/AP_HAL.h>
#include "AP_RollController.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_RollController::var_info[] = {
	// @Param: TCONST
	// @DisplayName: Roll Time Constant
	// @Description: This controls the time constant in seconds from demanded to achieved bank angle. A value of 0.5 is a good default and will work with nearly all models. Advanced users may want to reduce this time to obtain a faster response but there is no point setting a time less than the aircraft can achieve.
	// @Range: 0.4 1.0
	// @Units: seconds
	// @Increment: 0.1
	// @User: Advanced
	AP_GROUPINFO("TCONST",      0, AP_RollController, gains.tau,       0.5f),

	// @Param: P
	// @DisplayName: Proportional Gain
	// @Description: This is the gain from bank angle error to aileron.
	// @Range: 0.1 4.0
	// @Increment: 0.1
	// @User: User
	AP_GROUPINFO("P",        1, AP_RollController, gains.P,        0.6f),

	// @Param: D
	// @DisplayName: Damping Gain
	// @Description: This is the gain from roll rate to aileron. This adjusts the damping of the roll control loop. It has the same effect as RLL2SRV_D in the old PID controller but without the spikes in servo demands. This gain helps to reduce rolling in turbulence. It should be increased in 0.01 increments as too high a value can lead to a high frequency roll oscillation that could overstress the airframe.
	// @Range: 0 0.1
	// @Increment: 0.01
	// @User: User
	AP_GROUPINFO("D",        2, AP_RollController, gains.D,        0.02f),

	// @Param: I
	// @DisplayName: Integrator Gain
	// @Description: This is the gain from the integral of bank angle to aileron. It has the same effect as RLL2SRV_I in the old PID controller. Increasing this gain causes the controller to trim out steady offsets due to an out of trim aircraft.
	// @Range: 0 1.0
	// @Increment: 0.05
	// @User: User
	AP_GROUPINFO("I",        3, AP_RollController, gains.I,        0.1f),

	// @Param: RMAX
	// @DisplayName: Maximum Roll Rate
	// @Description: This sets the maximum roll rate that the controller will demand (degrees/sec). Setting it to zero disables the limit. If this value is set too low, then the roll can't keep up with the navigation demands and the plane will start weaving. If it is set too high (or disabled by setting to zero) then ailerons will get large inputs at the start of turns. A limit of 60 degrees/sec is a good default.
	// @Range: 0 180
	// @Units: degrees/second
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("RMAX",   4, AP_RollController, gains.rmax,       0),

	// @Param: IMAX
	// @DisplayName: Integrator limit
	// @Description: This limits the number of degrees of aileron in centi-degrees over which the integrator will operate. At the default setting of 3000 centi-degrees, the integrator will be limited to +- 30 degrees of servo travel. The maximum servo deflection is +- 45 centi-degrees, so the default value represents a 2/3rd of the total control throw which is adequate unless the aircraft is severely out of trim.
	// @Range: 0 4500
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("IMAX",      5, AP_RollController, gains.imax,        3000),

	// @Param: FF
	// @DisplayName: Feed forward Gain
	// @Description: This is the gain from demanded rate to aileron output. 
	// @Range: 0.1 4.0
	// @Increment: 0.1
	// @User: User
	AP_GROUPINFO("FF",        6, AP_RollController, gains.FF,          0.0f),

	AP_GROUPEND
};



int32_t AP_RollController::_get_rate_out(float desired_rate, float scaler, bool disable_integrator)
{
	uint32_t tnow = AP_HAL::millis();
	uint32_t dt = tnow - _last_t;
	if (_last_t == 0 || dt > 1000) {
		dt = 0;
	}
	_last_t = tnow;
	

	float ki_rate = gains.I * gains.tau;
    float eas2tas = _ahrs.get_EAS2TAS();
	float kp_ff = MAX((gains.P - gains.I * gains.tau) * gains.tau  - gains.D , 0) / eas2tas;
    float k_ff = gains.FF / eas2tas;
	float delta_time    = (float)dt * 0.001f;
	
	// Limit the demanded roll rate
	if (gains.rmax && desired_rate < -gains.rmax) {
        desired_rate = - gains.rmax;
    } else if (gains.rmax && desired_rate > gains.rmax) {
        desired_rate = gains.rmax;
    }
	
    // Get body rate vector (radians/sec)
	float omega_x = _ahrs.get_gyro().x;
	
	// Calculate the roll rate error (deg/sec) and apply gain scaler
    float achieved_rate = ToDeg(omega_x);
	float rate_error = (desired_rate - achieved_rate) * scaler;
	
	// Get an airspeed estimate - default to zero if none available
	float aspeed;
	if (!_ahrs.airspeed_estimate(&aspeed)) {
        aspeed = 0.0f;
    }


	if (!disable_integrator && ki_rate > 0) {
		//only integrate if gain and time step are positive and airspeed above min value.
		if (dt > 0 && aspeed > float(aparm.airspeed_min)) {
		    float integrator_delta = rate_error * ki_rate * delta_time * scaler;
			// prevent the integrator from increasing if surface defln demand is above the upper limit
			if (_last_out < -45) {
                integrator_delta = MAX(integrator_delta , 0);
            } else if (_last_out > 45) 
            {
 
                 integrator_delta = MIN(integrator_delta, 0);
            }
			_pid_info.I += integrator_delta;
		}
	} else {
		_pid_info.I = 0;
	}
	
    // Scale the integration limit
    float intLimScaled = gains.imax * 0.01f;

    // Constrain the integrator state
    _pid_info.I = constrain_float(_pid_info.I, -intLimScaled, intLimScaled);
	

    _pid_info.D = rate_error * gains.D * scaler;
    _pid_info.P = desired_rate * kp_ff * scaler;
    _pid_info.FF = desired_rate * k_ff * scaler;
    _pid_info.desired = desired_rate;

	_last_out = _pid_info.FF + _pid_info.P + _pid_info.D;

    if (autotune.running && aspeed > aparm.airspeed_min) {

        autotune.update(desired_rate, achieved_rate, _last_out);
    }

	_last_out += _pid_info.I;
	
	// Convert to centi-degrees and constrain
	return constrain_float(_last_out * 100, -4500, 4500);
}


int32_t AP_RollController::get_rate_out(float desired_rate, float scaler)
{
    return _get_rate_out(desired_rate, scaler, false);
}


int32_t AP_RollController::get_servo_out(int32_t angle_err, float scaler, bool disable_integrator)
{
    if (gains.tau < 0.1f) {
        gains.tau.set(0.1f);
    }
	
	// Calculate the desired roll rate (deg/sec) from the angle error
	float desired_rate = angle_err * 0.01f / gains.tau;

    return _get_rate_out(desired_rate, scaler, disable_integrator);
}

void AP_RollController::reset_I()
{
	_pid_info.I = 0;
}

