#include "copter.h"

#define GRAVITY					9.81

#define KP_THRUST				1
#define KD_THRUST				1

#define KP_PHI					1
#define KD_PHI					1

#define KP_THETTA				1
#define KD_THETTA				1

#define KP_PSI					1
#define KD_PSI					1

#define ROLL_SPEED_DESIRED		1
#define PITCH_SPEED_DESIRED		1
#define YAW_SPEED_DESIRED		1

#define MOTOR_MAX_RPM			25000
#define MOTOR_MAX_THRUST_N		0.15
#define LIFT_CONSTANT			MOTOR_MAX_THRUST_N / (MOTOR_MAX_RPM * MOTOR_MAX_RPM)
#define DRAG_CONSTANT			0.1 * LIFT_CONSTANT
#define ROTOR_HALF_DISTANCE		0.055

#define I_XX					0.5E-4
#define I_YY					0.5E-4
#define I_ZZ					0.5E-4


void copter_init(TIM_HandleTypeDef* htim, uint16_t rf, uint16_t lf, uint16_t lb, uint16_t rb)
{
	copter.rf.htim = htim;
	copter.lf.htim = htim;
	copter.lb.htim = htim;
	copter.rb.htim = htim;
	
	copter.rf.channel = rf;
	copter.lf.channel = lf;
	copter.lb.channel = lb;
	copter.rb.channel = rb;
	
	motor_set_speed(&copter.rf, 0);
	motor_set_speed(&copter.lf, 0);
	motor_set_speed(&copter.lb, 0);
	motor_set_speed(&copter.rb, 0);
	
	copter.orientation.quat.w = 1.0;
	copter.orientation.quat.x = 0.0;
	copter.orientation.quat.y = 0.0;
	copter.orientation.quat.z = 0.0;
	
	copter.mode = FLY_MODE_MANUAL;
}


void handle_takeoff()
{
	
}

void handle_land()
{
	
}

void handle_manual()
{
	motor_set_speed(&copter.rf, (uint8_t)(1 * copter.actuator.thrust));
	motor_set_speed(&copter.lf, (uint8_t)(1 * copter.actuator.thrust));
	motor_set_speed(&copter.lb, (uint8_t)(1 * copter.actuator.thrust));
	motor_set_speed(&copter.rb, (uint8_t)(1 * copter.actuator.thrust));
//	
//	float thrust_d		= (MOTOR_MAX_THRUST_N * 4 * copter.actuator.thrust) / 100;
//	float tau_phi		= KP_PHI	* (copter.trimm.roll	+ copter.orientation.euler.roll)	 * I_XX;
//	float tau_thetta	= KP_THETTA * (copter.trimm.pitch	- copter.orientation.euler.pitch)	* I_YY;
//	float tau_psi		= KP_PSI	* copter.trimm.yaw	 * I_ZZ;
//	
//		
//	float thrust_d		= (MOTOR_MAX_THRUST_N * 4 * copter.actuator.thrust) / 100;
//	float tau_phi		= KP_PHI	* copter.trimm.roll		* I_XX;
//	float tau_thetta	= KP_THETTA * copter.trimm.pitch	* I_YY;
//	float tau_psi		= KP_PSI	* copter.trimm.yaw		* I_ZZ;
//	
//	 ... and motor speeds
//	 rf & lb - CW
//	 rb & lf - CCW
//	int rf_motor_speed_d = (float)sqrt((thrust_d / (4 * LIFT_CONSTANT)) - (tau_thetta	/ (2 * LIFT_CONSTANT * ROTOR_HALF_DISTANCE))	- (tau_psi / (4 * DRAG_CONSTANT)));
//	int rb_motor_speed_d = (float)sqrt((thrust_d / (4 * LIFT_CONSTANT)) - (tau_phi		/ (2 * LIFT_CONSTANT * ROTOR_HALF_DISTANCE))	+ (tau_psi / (4 * DRAG_CONSTANT)));
//	int lb_motor_speed_d = (float)sqrt((thrust_d / (4 * LIFT_CONSTANT)) + (tau_thetta	/ (2 * LIFT_CONSTANT * ROTOR_HALF_DISTANCE))	- (tau_psi / (4 * DRAG_CONSTANT)));
//	int lf_motor_speed_d = (float)sqrt((thrust_d / (4 * LIFT_CONSTANT)) + (tau_phi		/ (2 * LIFT_CONSTANT * ROTOR_HALF_DISTANCE))	+ (tau_psi / (4 * DRAG_CONSTANT)));
//	
//	int rf_motor_speed_d = (float)sqrt((thrust_d / (4 * LIFT_CONSTANT)) - (tau_phi		/ (2 * LIFT_CONSTANT * ROTOR_HALF_DISTANCE))	+ (tau_thetta	/ (2 * LIFT_CONSTANT * ROTOR_HALF_DISTANCE)) 	+ (tau_psi / (4 * DRAG_CONSTANT)));
//	int rb_motor_speed_d = (float)sqrt((thrust_d / (4 * LIFT_CONSTANT)) - (tau_phi		/ (2 * LIFT_CONSTANT * ROTOR_HALF_DISTANCE))	- (tau_thetta	/ (2 * LIFT_CONSTANT * ROTOR_HALF_DISTANCE))	- (tau_psi / (4 * DRAG_CONSTANT)));
//	int lb_motor_speed_d = (float)sqrt((thrust_d / (4 * LIFT_CONSTANT)) + (tau_phi		/ (2 * LIFT_CONSTANT * ROTOR_HALF_DISTANCE))	- (tau_thetta	/ (2 * LIFT_CONSTANT * ROTOR_HALF_DISTANCE))	+ (tau_psi / (4 * DRAG_CONSTANT)));
//	int lf_motor_speed_d = (float)sqrt((thrust_d / (4 * LIFT_CONSTANT)) + (tau_phi		/ (2 * LIFT_CONSTANT * ROTOR_HALF_DISTANCE))	+ (tau_thetta	/ (2 * LIFT_CONSTANT * ROTOR_HALF_DISTANCE))	- (tau_psi / (4 * DRAG_CONSTANT)));
//
//		
//	 Assume that max speed of motor is 100 (timer counter period), so transform the value and assign it to the corresponding motor
//	motor_set_speed(&copter.rf, (uint8_t)((100 * rf_motor_speed_d)	/ MOTOR_MAX_RPM));
//	motor_set_speed(&copter.lf, (uint8_t)((100 * lf_motor_speed_d)	/ MOTOR_MAX_RPM));
//	motor_set_speed(&copter.lb, (uint8_t)((100 * lb_motor_speed_d)	/ MOTOR_MAX_RPM));
//	motor_set_speed(&copter.rb, (uint8_t)((100 * rb_motor_speed_d)	/ MOTOR_MAX_RPM));
	
}

void copter_handle_actuators()
{
	switch (copter.mode)
	{
	case FLY_MODE_STABILIZED:
		copter_update_attitude();
	case FLY_MODE_MANUAL:
		//copter_update_attitude();
		handle_manual();
		break;
	case FLY_MODE_TAKEOFF:
		handle_takeoff();
		break;
	case FLY_MODE_LAND:
		handle_land();
		break;
	}
}


void copter_update_attitude()
{
	// Now calculate the desired moments of inertia and summary thrust of our creation
	
	float thrust_d		= (MOTOR_MAX_THRUST_N * 4 * copter.actuator.thrust) / 100;
	float tau_phi		= (KD_PHI		* (ROLL_SPEED_DESIRED	- copter.speed.rollspeed)	+ KP_PHI * (copter.actuator.roll	- copter.orientation.euler.roll))	* I_XX;
	float tau_thetta	= (KD_THETTA	* (PITCH_SPEED_DESIRED	- copter.speed.pitchspeed)	+ KP_PHI * (copter.actuator.pitch	- copter.orientation.euler.pitch))  * I_YY;
	float tau_psi		= (KD_PSI		* (YAW_SPEED_DESIRED	- copter.speed.yawspeed)	+ KP_PHI * (copter.actuator.yaw		- copter.orientation.euler.yaw))	* I_ZZ;
	
	// ... and motor speeds
	int rf_motor_speed_d = (float)sqrt((thrust_d / (4 * LIFT_CONSTANT)) - (tau_thetta	/ (2 * LIFT_CONSTANT * ROTOR_HALF_DISTANCE))	- (tau_psi / (4 * DRAG_CONSTANT)));
	int rb_motor_speed_d = (float)sqrt((thrust_d / (4 * LIFT_CONSTANT)) - (tau_phi		/ (2 * LIFT_CONSTANT * ROTOR_HALF_DISTANCE))	+ (tau_psi / (4 * DRAG_CONSTANT)));
	int lb_motor_speed_d	= (float)sqrt((thrust_d / (4 * LIFT_CONSTANT)) + (tau_thetta	/ (2 * LIFT_CONSTANT * ROTOR_HALF_DISTANCE))	- (tau_psi / (4 * DRAG_CONSTANT)));
	int lf_motor_speed_d	= (float)sqrt((thrust_d / (4 * LIFT_CONSTANT)) + (tau_phi		/ (2 * LIFT_CONSTANT * ROTOR_HALF_DISTANCE))	+ (tau_psi / (4 * DRAG_CONSTANT)));
	
	// Assume that max speed of motor is 100 (timer counter period), so transform the value and assign it to the corresponding motor
	motor_set_speed(&copter.rf, (uint8_t)((100 * rf_motor_speed_d)	/ MOTOR_MAX_RPM));
	motor_set_speed(&copter.lf, (uint8_t)((100 * lf_motor_speed_d)	/ MOTOR_MAX_RPM));
	motor_set_speed(&copter.lb, (uint8_t)((100 * lb_motor_speed_d)	/ MOTOR_MAX_RPM));
	motor_set_speed(&copter.rb, (uint8_t)((100 * rb_motor_speed_d)	/ MOTOR_MAX_RPM));
}