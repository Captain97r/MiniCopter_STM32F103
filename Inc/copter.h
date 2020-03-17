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
	float roll;
	float pitch;
	float yaw;
} euler_t;
typedef struct
{
	float w;
	float x;
	float y;
	float z;
} quaternion_t;

typedef struct
{
	euler_t euler;
	quaternion_t quat;
	float heading;
	float altitude;
} orientation_t;

typedef struct 
{
	float rollspeed;
	float pitchspeed;
	float yawspeed;
} speed_t;

typedef struct 
{
	motor_t rf;
	motor_t lf;
	motor_t lb;
	motor_t rb;
	
	FLY_MODE mode;
	actuators_t actuator;
	orientation_t orientation;
	speed_t speed;
} copter_t;

copter_t copter;

void copter_init(TIM_HandleTypeDef* htim, uint16_t rf, uint16_t lf, uint16_t lb, uint16_t rb);

void copter_handle_actuators();

void copter_update_attitude();

#endif /*__copter_H */