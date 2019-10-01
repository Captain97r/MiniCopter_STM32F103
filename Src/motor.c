#include "motor.h"

void motor_set_speed(motor_t *motor, uint8_t speed)
{
	if (speed > 99)
		speed = 99;
	
	switch (motor->channel)
	{
	case TIM_CHANNEL_1:
		motor->htim->Instance->CCR1 = speed;
		break;
	case TIM_CHANNEL_2:
		motor->htim->Instance->CCR2 = speed;
		break;
	case TIM_CHANNEL_3:
		motor->htim->Instance->CCR3 = speed;
		break;
	case TIM_CHANNEL_4:
		motor->htim->Instance->CCR4 = speed;
		break;
	}
	motor->speed = speed;
}

uint8_t motor_get_speed(motor_t *motor)
{
	return motor->speed;
}