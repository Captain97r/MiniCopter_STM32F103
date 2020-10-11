#ifndef __sheduler_H
#define __sheduler_H

#include "stm32f1xx_hal.h"
#include "battery.h"
#include "copter.h"
#include "IMU.h"
#include "radio_handler.h"

#define ADC_HANLING_INTERVAL_MS			1000
#define BT_MSG_SEND_INTERVAL_MS			100
#define BT_MSG_RECEIVE_INTERVAL_MS		5
#define IMU_HANDLING_INTERVAL_MS		5
#define CNT_RESET						1000

#endif /*__sheduler_H */