#include "bt_msg_handler.h"
#include "copter.h"

extern copter_t copter;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	copter.mode				= message.buf[7];
	
	copter.actuator.thrust	= message.buf[6];
	copter.actuator.roll	= message.buf[5];
	copter.actuator.pitch	= message.buf[4];
	copter.actuator.yaw		= message.buf[3];
}