#include "nRF24L01_hal.h"
#include "stm32f1xx_hal_spi.h"

extern SPI_HandleTypeDef hspi2;

// Low level SPI transmit/receive function (hardware depended)
// input:
//   data - value to transmit via SPI
// return: value received from SPI
uint8_t nRF24_LL_RW(uint8_t data) {
	// Wait until TX buffer is empty
	uint8_t result = 0;
	while(!__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE));
	// Send byte to SPI (TXE cleared)
	HAL_SPI_TransmitReceive(&hspi1, &data, &result, 1, 1000);
	return result;
}