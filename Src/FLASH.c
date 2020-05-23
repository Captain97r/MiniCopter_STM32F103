#include "FLASH.h"

void FLASH_page_erase(uint32_t pageAddress)
{
	while (FLASH->SR & FLASH_SR_BSY) ;
	if (FLASH->SR & FLASH_SR_EOP) {
		FLASH->SR = FLASH_SR_EOP;
	}

	FLASH->CR |= FLASH_CR_PER;
	FLASH->AR = pageAddress;
	FLASH->CR |= FLASH_CR_STRT;
	while (!(FLASH->SR & FLASH_SR_EOP)) ;
	FLASH->SR = FLASH_SR_EOP;
	FLASH->CR &= ~FLASH_CR_PER;
}

void FLASH_write(uint8_t* data, uint32_t address, uint32_t count) 
{
	uint32_t i;

	while (FLASH->SR & FLASH_SR_BSY) ;
	if (FLASH->SR & FLASH_SR_EOP) {
		FLASH->SR = FLASH_SR_EOP;
	}

	FLASH->CR |= FLASH_CR_PG;

	for (i = 0; i < count; i += 2) {
		*(__IO uint16_t*)(address + i) = (((uint16_t)data[i + 1]) << 8) + data[i];
		while (!(FLASH->SR & FLASH_SR_EOP)) ;
		FLASH->SR = FLASH_SR_EOP;
	}

	FLASH->CR &= ~(FLASH_CR_PG);
}

void FLASH_read(uint32_t* data, uint32_t address, uint32_t count)
{
	for (uint32_t i = 0; i < count / 4; i++) {
		data[i] = (*(__IO uint32_t*)(address + (i * 4))); 
	}
}