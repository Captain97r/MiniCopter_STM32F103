#ifndef __motor_H
#define __motor_H

#include "stm32f1xx_hal.h"

typedef struct 
{
	TIM_HandleTypeDef *htim;
	uint16_t channel;
	uint8_t speed;
} motor_t;


void motor_set_speed(motor_t *motor, uint8_t speed);

uint8_t motor_get_speed(motor_t *motor);

#endif /*__motor_H */