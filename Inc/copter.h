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
	motor_t front;
	motor_t right;
	motor_t back;
	motor_t left;
	
	FLY_MODE mode;
	actuators_t actuator;
	orientation_t orientation;
	speed_t speed;
} copter_t;

copter_t copter;

void copter_motor_init(motor_t *f, motor_t *r, motor_t *b, motor_t *l);

void copter_handle_actuators();

#endif /*__copter_H */