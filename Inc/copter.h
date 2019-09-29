#pragma once

#ifndef __copter_H
#define __copter_H

#include "stm32f1xx_hal.h"
#include "motor.h"

typedef enum
{
	FLY_MODE_MANUAL,
	FLY_MODE_STABILIZED,
	FLY_MODE_TAKEOFF,
	FLY_MODE_LAND
} FLY_MODE;

typedef struct
{
	uint8_t thrust;
	int8_t pitch;
	int8_t roll;
	int8_t yaw;
} actuators_t;

typedef struct 
{
	motor_t lf;
	motor_t rf;
	motor_t rb;
	motor_t lb;
	
	FLY_MODE mode;
	
} copter_t;


#endif /*__copter_H */
