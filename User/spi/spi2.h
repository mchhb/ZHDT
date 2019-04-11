#ifndef __SPI2_H
#define __SPI2_H

#include "stm32f10x.h"
#define DEVICE_ID "W5500"
#define WIZ_SCS			    GPIO_Pin_12	// out
#define WIZ_SCLK				GPIO_Pin_13	// out
#define WIZ_MISO				GPIO_Pin_14	// in
#define WIZ_MOSI				GPIO_Pin_15	// out
void Reset_W5500(void);
void WIZ_SPI_Init(void);
void WIZ_CS(uint8_t val);
uint8_t SPI2_SendByte(uint8_t byte);
#endif

