#include "bt_msg_handler.h"
#include "copter.h"
#include "MPU9250.h"
#include "main.h"

extern copter_t copter;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	copter.mode				= message.buf[7];
	
	copter.actuator.thrust	= message.buf[6];
	copter.actuator.roll	= message.buf[5];
	copter.actuator.pitch	= message.buf[4];
	copter.actuator.yaw		= message.buf[3];
}

void bt_message_send()
{
	uint8_t msg[7] = {
			6,
		(int8_t)copter.orientation.euler.pitch,
		(int8_t)copter.orientation.euler.roll,
		(int8_t)copter.orientation.euler.yaw,
		(int8_t)mpu9250.magnetometer.data.x,
		(int8_t)mpu9250.magnetometer.data.y,
		(int8_t)mpu9250.magnetometer.data.z,
	};
	
	HAL_UART_Transmit(message.huart, (uint8_t *)msg, 7, 1000);
}