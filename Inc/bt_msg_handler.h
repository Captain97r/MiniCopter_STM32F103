#ifndef __msg_handler_H
#define __msg_handler_H

#define MESSAGE_BUFFER_SIZE		16

#include "stm32f1xx_hal.h"


/**
 * Message structure for the copter control:
 * msg[15]		= "Q" - message start (hardcoded chars)
 * msg[14]		= "m" - mode frame (hardcoded char)
 * msg[13]      = fly mode value based on FLY_MODE enum in copter.h
 * msg[12..10]	= "act" (hardcoded chars)
 * msg[9..8]	= thrust actuator value in range 0..99
 * msg[7..6]	= roll actuator value in range -99..99
 * msg[5..4]	= pitch actuator value in range -99..99
 * msg[3..2]	= yaw actuator value in range -99..99
 * msg[1]		= reserved
 * msg[0]		= CRC8? (honestly idk, reserved)
 * 
*/
typedef struct
{
	uint8_t buf[MESSAGE_BUFFER_SIZE];
} bt_message_t;

bt_message_t message;

#endif /*__msg_handler_H */