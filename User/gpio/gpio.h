#ifndef __GPIO_H_
#define __GPIO_H_

#include "stm32f10x.h"

#define WIZ_RESET		    GPIO_Pin_7	// out
#define WIZ_INT			    GPIO_Pin_6	// in

#define WORK_LED_NORMAL_ON  		GPIO_ResetBits(GPIOB, GPIO_Pin_8);
#define WORK_LED_NORMAL_OFF 		GPIO_SetBits(GPIOB, GPIO_Pin_8);
#define WORK_LED_ERROR_ON  			GPIO_ResetBits(GPIOB, GPIO_Pin_9);
#define WORK_LED_ERROR_OFF 			GPIO_SetBits(GPIOB, GPIO_Pin_9);

#define NETWORK_LED_SUCCESS_ON  GPIO_ResetBits(GPIOB, GPIO_Pin_4);
#define NETWORK_LED_SUCCESS_OFF GPIO_SetBits(GPIOB, GPIO_Pin_4);
#define NETWORK_LED_FAIL_ON  		GPIO_ResetBits(GPIOB, GPIO_Pin_5);
#define NETWORK_LED_FAIL_OFF 		GPIO_SetBits(GPIOB, GPIO_Pin_5);

#define POWER_LED_SUCCESS_ON  GPIO_ResetBits(GPIOC, GPIO_Pin_13);
#define POWER_LED_SUCCESS_OFF GPIO_SetBits(GPIOC, GPIO_Pin_13);
#define POWER_LED_FAIL_ON  		GPIO_ResetBits(GPIOC, GPIO_Pin_14);
#define POWER_LED_FAIL_OFF 		GPIO_SetBits(GPIOC, GPIO_Pin_14);

#define UART1_LED_SUCCESS_ON  GPIO_ResetBits(GPIOD, GPIO_Pin_2);
#define UART1_LED_SUCCESS_OFF GPIO_SetBits(GPIOD, GPIO_Pin_2);
#define UART1_LED_FAIL_ON  		GPIO_ResetBits(GPIOB, GPIO_Pin_3);
#define UART1_LED_FAIL_OFF 		GPIO_SetBits(GPIOB, GPIO_Pin_3);

#define UART3_LED_SUCCESS_ON  GPIO_ResetBits(GPIOC, GPIO_Pin_11);
#define UART3_LED_SUCCESS_OFF GPIO_SetBits(GPIOC, GPIO_Pin_11);
#define UART3_LED_FAIL_ON  		GPIO_ResetBits(GPIOC, GPIO_Pin_12);
#define UART3_LED_FAIL_OFF 		GPIO_SetBits(GPIOC, GPIO_Pin_12);

#define DOOR_OPEN_ON  				GPIO_ResetBits(GPIOC, GPIO_Pin_10);
#define DOOR_OPEN_OFF 				GPIO_SetBits(GPIOC, GPIO_Pin_10);
#define DOOR_CLOSE_ON  				GPIO_ResetBits(GPIOA, GPIO_Pin_15);
#define DOOR_CLOSE_OFF 				GPIO_SetBits(GPIOA, GPIO_Pin_15);


#define TOP_YES			  				GPIO_ResetBits(GPIOA, GPIO_Pin_11);
#define TOP_NO 								GPIO_SetBits(GPIOA, GPIO_Pin_11);
#define BUTTON_YES  					GPIO_ResetBits(GPIOA, GPIO_Pin_12);
#define BUTTON_NO 						GPIO_SetBits(GPIOA, GPIO_Pin_12);
void GPIO_Configuration(void);


#endif

