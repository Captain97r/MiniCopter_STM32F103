#ifndef __HC06_H
#define __HC06_H

//#include "usart.h"
#include "bt_msg_handler.h"


typedef enum
{
	HC06_BAUD_1200   = 1,
	HC06_BAUD_2400   = 2,
	HC06_BAUD_4800   = 3,
	HC06_BAUD_9600   = 4,
	HC06_BAUD_19200  = 5,
	HC06_BAUD_38400  = 6,
	HC06_BAUD_57600  = 7,
	HC06_BAUD_115200 = 8,
	HC06_BAUD_230400 = 9
} HC06_RATE;

typedef enum
{
	HC06_PARITY_NONE,
	HC06_PARITY_ODD,
	HC06_PARITY_EVEN
} HC06_PARITY;

HAL_StatusTypeDef HC06_check(HC06_t *hc06);

HAL_StatusTypeDef HC06_set_name(HC06_t *hc06);

HAL_StatusTypeDef HC06_set_pass(HC06_t *hc06);

HAL_StatusTypeDef HC06_get_version(HC06_t *hc06);

// 0 = no; 1 = odd; 2 = even
HAL_StatusTypeDef HC06_set_parity(HC06_t *hc06, uint8_t mode);

HAL_StatusTypeDef HC06_set_baud(HC06_t *hc06, HC06_RATE baudrate);

#endif /*__HC06_H */