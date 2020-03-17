#ifndef __msg_handler_H
#define __msg_handler_H

#define MESSAGE_BUFFER_SIZE		8

#include "stm32f1xx_hal.h"


/**
 * Message structure for the copter control:
 * msg[7]		= fly mode value based on FLY_MODE enum in copter.h
 * msg[6]		= thrust actuator value in range 0..99
 * msg[5]		= roll actuator value in range -99..99
 * msg[4]		= pitch actuator value in range -99..99
 * msg[3]		= yaw actuator value in range -99..99
 * msg[2..1]	= reserved
 * msg[0]		= CRC8? (honestly idk, reserved)
 * 
*/
typedef struct
{
	uint8_t buf[MESSAGE_BUFFER_SIZE];
	UART_HandleTypeDef *huart;
} bt_message_t;

bt_message_t message;

void bt_message_send();

#endif /*__msg_handler_H */