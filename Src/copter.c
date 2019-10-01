#include "copter.h"


void copter_motor_init(copter_t *copter, motor_t *lf, motor_t *rf, motor_t *rb, motor_t *lb)
{
	copter->lf = *lf;
	copter->rf = *rf;
	copter->rb = *rb;
	copter->lb = *lb;
	
	copter->lf.speed = 0;
	copter->rf.speed = 0;
	copter->rb.speed = 0;
	copter->lb.speed = 0;
}

void copter_handle_actuators()
{
	// Oh shit, here we go again...
}

//void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
//	{
//		copter_handle_actuators();
//	}
//}
