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
#define ROTOR_HALF_DISTANCE		5.5

#define I_XX					1E-5
#define I_YY					1E-5
#define I_ZZ					0.5E-4


void copter_motor_init(motor_t *f, motor_t *r, motor_t *b, motor_t *l)
{
	copter.front = *f;
	copter.right = *r;
	copter.back = *b;
	copter.left = *l;
	
	motor_set_speed(&copter.front, 0);
	motor_set_speed(&copter.right, 0);
	motor_set_speed(&copter.back, 0);
	motor_set_speed(&copter.left, 0);
	
	copter.orientation.quat.w = 1.0;
	copter.orientation.quat.x = 0.0;
	copter.orientation.quat.y = 0.0;
	copter.orientation.quat.z = 0.0;
}


void handle_takeoff()
{
	
}

void handle_land()
{
	
}

void handle_manual()
{
	
}

void copter_handle_actuators()
{
	switch (copter.mode)
	{
	case FLY_MODE_STABILIZED:
	case FLY_MODE_MANUAL:
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
	
	int front_motor_speed_d = (float)sqrt((thrust_d / (4 * LIFT_CONSTANT)) - (tau_thetta	/ (2 * LIFT_CONSTANT * ROTOR_HALF_DISTANCE))	- (tau_psi / (4 * DRAG_CONSTANT)));
	int right_motor_speed_d = (float)sqrt((thrust_d / (4 * LIFT_CONSTANT)) - (tau_phi		/ (2 * LIFT_CONSTANT * ROTOR_HALF_DISTANCE))	+ (tau_psi / (4 * DRAG_CONSTANT)));
	int back_motor_speed_d	= (float)sqrt((thrust_d / (4 * LIFT_CONSTANT)) + (tau_thetta	/ (2 * LIFT_CONSTANT * ROTOR_HALF_DISTANCE))	- (tau_psi / (4 * DRAG_CONSTANT)));
	int left_motor_speed_d	= (float)sqrt((thrust_d / (4 * LIFT_CONSTANT)) + (tau_phi		/ (2 * LIFT_CONSTANT * ROTOR_HALF_DISTANCE))	+ (tau_psi / (4 * DRAG_CONSTANT)));
	
	// Assume that max speed of motor is 100 (timer counter period), so transform the value and assign it to the corresponding motor
	motor_set_speed(&copter.front,	(uint8_t)((100 * front_motor_speed_d)	/ MOTOR_MAX_RPM));
	motor_set_speed(&copter.right,	(uint8_t)((100 * right_motor_speed_d)	/ MOTOR_MAX_RPM));
	motor_set_speed(&copter.back,	(uint8_t)((100 * back_motor_speed_d)	/ MOTOR_MAX_RPM));
	motor_set_speed(&copter.left,	(uint8_t)((100 * left_motor_speed_d)	/ MOTOR_MAX_RPM));
}