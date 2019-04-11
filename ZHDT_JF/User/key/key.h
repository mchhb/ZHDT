#ifndef __KEY_H
#define	__KEY_H

#include "stm32f10x.h"
 /*******
 *按键按下标置
 KEY_ON 0
 KEY_OFF 1
 ********/
#define KEY_ON	1
#define KEY_OFF	0

#define LED_ON  GPIO_SetBits(GPIOB, GPIO_Pin_9);
#define LED_OFF GPIO_ResetBits(GPIOB, GPIO_Pin_9);


void Key_GPIO_Config(void);
u8 Key_Scan(GPIO_TypeDef* GPIOx,u16 GPIO_Pin);


uint8_t din_get(void);

#endif /* __LED_H */

