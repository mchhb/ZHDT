#ifndef __DHT11_H
#define __DHT11_H

#include "stm32f10x.h"
#include "w5500_config.h"
#include "gpio.h"
//-----------------DHT11Òý½Å£¬PB5---------------------
#define	DHT11_DATA           GPIO_Pin_9
#define DHT11         (1<<1)
#define DHT11_SET(x)  GPIOC->ODR=(GPIOC->ODR&~DHT11)|(x ? DHT11:0)

#define rDHT11        GPIO_ReadInputDataBit(GPIOC ,GPIO_Pin_9)
//----------------------------------------------------
uint8 DHT11_ReadData(void);
uint8 DHT11_GetValue(uint8 *p);

#endif
