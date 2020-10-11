#ifndef __radio_handler_H
#define __radio_handler_H

#define MESSAGE_BUFFER_SIZE		32

#include "stm32f1xx_hal.h"

/**
 * Message structure transmitting to the actuator device:
 * msg[0..1]	- BAT Voltage x 1000 (uint16_t)
 * 
 * msg[2]		- module initialization bitmap:
 *					bit 7..2	- not used, '000000';
 *					bit 1		- MPU9250 presence flag
 *								  1: MPU9250 is present
 *								  0: MPU9250 is absent
 *					bit 0		- BMP280 presence flag
 *								  1: BMP280 is present
 *								  0: BMP280 is absent
 *						  
 * msg[3..6]	- roll angle calculated by the on-board IMU unit (float32_t)
 * msg[7..10]	- pitch angle calculated by the on-board IMU unit (float32_t)
 * msg[11..14]	- yaw angle calculated by the on-board IMU unit (float32_t)
 * 
*/
//typedef struct
//{
//	uint8_t buf[MESSAGE_BUFFER_SIZE];
//	UART_HandleTypeDef *huart;
//} bt_message_t;

//bt_message_t message;

void radio_message_send();

#endif /*__radio_handler_H */