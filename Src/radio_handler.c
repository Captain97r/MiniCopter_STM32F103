#include "radio_handler.h"
#include "copter.h"
#include "MPU9250.h"
#include "BMP280.h"
#include "main.h"
#include "battery.h"
#include "nRF24L01.h"

extern copter_t copter;

uint8_t nRF24_payload[32];    // buffer for payload
uint8_t payload_length;    // variable to store a length of received payload
uint8_t pipe;    // pipe number
uint8_t light_value;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static uint8_t pkt_count = 0;
	if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY) {
		
		uint8_t module_bimap = 0x00;
		if (bmp280.isPresent == HAL_OK)
			module_bimap |= 0x01;
		
		if (mpu9250.isPresent == HAL_OK)
			module_bimap |= 0x02;
		
		
		uint8_t ack_payload[32] = {(uint8_t)(battery.voltage_mult_1000 & 0xFF), (uint8_t)((battery.voltage_mult_1000 >> 8) & 0xFF), module_bimap, 
			(*(int32_t *)(&copter.orientation.euler.roll)) & 0xFF, ((*(int32_t *)(&copter.orientation.euler.roll)) >> 8) & 0xFF,
			((*(int32_t *)(&copter.orientation.euler.roll)) >> 16) & 0xFF, ((*(int32_t *)(&copter.orientation.euler.roll)) >> 24) & 0xFF,
			(*(int32_t *)(&copter.orientation.euler.pitch)) & 0xFF, ((*(int32_t *)(&copter.orientation.euler.pitch)) >> 8) & 0xFF,
			((*(int32_t *)(&copter.orientation.euler.pitch)) >> 16) & 0xFF, ((*(int32_t *)(&copter.orientation.euler.pitch)) >> 24) & 0xFF,
			(*(int32_t *)(&copter.orientation.euler.yaw)) & 0xFF, ((*(int32_t *)(&copter.orientation.euler.yaw)) >> 8) & 0xFF,
			((*(int32_t *)(&copter.orientation.euler.yaw)) >> 16) & 0xFF, ((*(int32_t *)(&copter.orientation.euler.yaw)) >> 24) & 0xFF, 
			0};
		
		nRF24_WriteAckPayload(nRF24_PIPE1, ack_payload, 32); 
		
		pipe = nRF24_ReadPayload(nRF24_payload, &payload_length); 
		
		nRF24_FlushRX();	
		nRF24_ClearIRQFlags();
		
		light_value = nRF24_payload[1];
		  
		if (nRF24_payload[0] == 0)
		{
			TIM4->CCR2 = light_value;
			TIM4->CCR3 = 0;
			TIM4->CCR1 = 0; 
		}
		else if (nRF24_payload[0] == 1)
		{
			TIM4->CCR2 = 0;
			TIM4->CCR3 = light_value;
			TIM4->CCR1 = 0; 
		}
	}
}

void radio_message_send()
{

}