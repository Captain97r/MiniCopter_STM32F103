#include "bt_msg_handler.h"
#include "copter.h"
#include "MPU9250.h"
#include "main.h"

extern copter_t copter;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	copter.mode				= message.buf[0];
	
	copter.actuator.thrust	= message.buf[1];
	copter.actuator.roll	= message.buf[2];
	copter.actuator.pitch	= message.buf[3];
	copter.actuator.yaw		= message.buf[4];
	
	HAL_UART_Receive_DMA(huart, (uint8_t *)&message.buf, MESSAGE_BUFFER_SIZE);
}

void bt_message_send()
{
	uint16_t msg[15] = {
			9,
		(int16_t)(mpu9250.magnetometer.data.x * 100),
		(int16_t)(mpu9250.magnetometer.data.y * 100),
		(int16_t)(mpu9250.magnetometer.data.z * 100),
	};
	
	HAL_UART_Transmit(message.huart, (uint8_t *)msg, 30, 1000);
}