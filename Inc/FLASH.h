#ifndef __FLASH_H
#define __FLASH_H

#include "stm32f1xx_hal.h"

void FLASH_page_erase(uint32_t pageAddress);
void FLASH_write(uint8_t* data, uint32_t address, uint32_t count);
void FLASH_read(uint32_t* data, uint32_t address, uint32_t size);


#endif /*__FLASH_H */