#include "copter.h"


void copter_motor_init(motor_t *lf, motor_t *rf, motor_t *rb, motor_t *lb)
{
	copter.lf = *lf;
	copter.rf = *rf;
	copter.rb = *rb;
	copter.lb = *lb;
	
	copter.lf.speed = 0;
	copter.rf.speed = 0;
	copter.rb.speed = 0;
	copter.lb.speed = 0;
	
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