#ifndef __GPIO_H_
#define __GPIO_H_

#include "stm32f10x.h"

#define WIZ_RESET		    GPIO_Pin_11	// out
#define WIZ_INT			    GPIO_Pin_8	// in

#define WORK_LED_NORMAL_ON  		GPIO_ResetBits(GPIOB, GPIO_Pin_3);
#define WORK_LED_NORMAL_OFF 		GPIO_SetBits(GPIOB, GPIO_Pin_3);
#define WORK_LED_ERROR_ON  			GPIO_ResetBits(GPIOC, GPIO_Pin_14);
#define WORK_LED_ERROR_OFF 			GPIO_SetBits(GPIOC, GPIO_Pin_14);

#define NETWORK_LED_SUCCESS_ON  GPIO_ResetBits(GPIOA, GPIO_Pin_12);
#define NETWORK_LED_SUCCESS_OFF GPIO_SetBits(GPIOA, GPIO_Pin_12);
#define NETWORK_LED_FAIL_ON  		GPIO_ResetBits(GPIOC, GPIO_Pin_15);
#define NETWORK_LED_FAIL_OFF 		GPIO_SetBits(GPIOC, GPIO_Pin_15);

void GPIO_Configuration(void);


#endif

