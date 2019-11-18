#include "sheduler.h"

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Triggers 1000 times per second (once per millisecond)
	static uint16_t cnt = 0;
	static uint16_t tick = 0;
	
	if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		cnt++;
		if (!(cnt % ADC_HANLING_INTERVAL_MS))
		{
			battery_calculate_voltage();
		}
		if (!(cnt %  BT_MSG_RECEIVE_INTERVAL_MS))
		{
			copter_handle_actuators();
		}if (!(cnt % BT_MSG_SEND_INTERVAL_MS))
		{
			bt_message_send();
		}
		if (!(cnt % IMU_HANDLING_INTERVAL_MS))
		{
			tick = HAL_GetTick();
			calculate_orientation();
			uint16_t a = HAL_GetTick() - tick;
			uint16_t b = 0;
		}
		if (!(cnt % CNT_RESET))
		{
			cnt = 0;
		}
	}
}