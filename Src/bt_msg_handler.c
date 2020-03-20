#include "bt_msg_handler.h"
#include "copter.h"
#include "MPU9250.h"
#include "main.h"
#include "battery.h"

extern copter_t copter;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	int8_t bias;
	if (message.buf[2] == 'R')
		bias = 3;
	
	if (message.buf[1] == 'R')
		bias = 2;
	
	if (message.buf[0] == 'R')
		bias = 1;
	
	if (message.buf[3] == 'R')
		bias = 4;
	
	if (message.buf[4] == 'R')
		bias = 5;
	
	copter.mode				= message.buf[bias + 0];
	
	copter.actuator.thrust	= message.buf[bias + 1];
	copter.actuator.roll	= message.buf[bias + 2];
	copter.actuator.pitch	= message.buf[bias + 3];
	copter.actuator.yaw		= message.buf[bias + 4];
	
	copter.trimm.roll		= message.buf[bias + 5] - 63;
	copter.trimm.pitch		= message.buf[bias + 6] - 63;
	copter.trimm.yaw		= message.buf[bias + 7] - 63;
	
	HAL_UART_Receive_DMA(huart, (uint8_t *)&message.buf, MESSAGE_BUFFER_SIZE);
}

void bt_message_send()
{
	uint16_t msg[15] = {
			9,
		(int16_t)(battery.voltage_mult_1000 / 10.0),
	};
	
	HAL_UART_Transmit(message.huart, (uint8_t *)msg, 30, 1000);
}