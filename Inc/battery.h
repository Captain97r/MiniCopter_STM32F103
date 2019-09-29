#pragma once

#ifndef __battery_H
#define __battery_H

#define ADC_BUFFER_SIZE		3

#include "stm32f1xx_hal.h"

typedef struct 
{
	uint32_t buf[ADC_BUFFER_SIZE];
	uint16_t voltage_mult_1000;
} battery_t;

battery_t battery;
static uint16_t median = 0;
static uint32_t ADC_counter = 0;
static uint32_t TIM_counter = 0;

uint16_t MedianFilter(uint16_t datum);

float get_battery_voltage();

#endif /*__battery_H */
