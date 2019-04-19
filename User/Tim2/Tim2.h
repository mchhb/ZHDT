#ifndef __TIM2__H
#define __TIM2__H

#include "stm32f10x.h"
 
#define T2_MS_ARRAY_NUM  		18
#define T2_COUNTER_MS_0			0
#define T2_COUNTER_MS_1			1
#define T2_COUNTER_MS_5			2
#define T2_COUNTER_MS_10		3
#define T2_COUNTER_MS_50		4
#define T2_COUNTER_MS_100		5
#define T2_COUNTER_MS_200		6
#define T2_COUNTER_MS_500		7
#define T2_COUNTER_MS_1000	8
#define T2_COUNTER_MS_2000	9
#define T2_COUNTER_MS_5000	10
#define T2_COUNTER_MS_400		11
#define T2_COUNTER_MS_3000		12
#define T2_COUNTER_MS_180000		13
#define T2_COUNTER_MS_10000		14
#define T2_COUNTER_MS_30000		15
#define T2_COUNTER_MS_6000		16
#define T2_COUNTER_MS_2500		17
// unit is 0.1ms
#define T2_TIMEOUT_MS_0				1			// 0
#define T2_TIMEOUT_MS_1				10    // 1
#define T2_TIMEOUT_MS_5				50		// 2
#define T2_TIMEOUT_MS_10			100		// 3
#define T2_TIMEOUT_MS_50			500		// 4
#define T2_TIMEOUT_MS_100			1000  // 5
#define T2_TIMEOUT_MS_200			2000	// 6
#define T2_TIMEOUT_MS_500			5000	// 7			
#define T2_TIMEOUT_MS_1000		10000	// 8
#define T2_TIMEOUT_MS_2000		20000 // 9
#define T2_TIMEOUT_MS_5000		50000	// 10
#define T2_TIMEOUT_MS_400			4000	// 11		
#define T2_TIMEOUT_MS_3000		30000 // 12
#define T2_TIMEOUT_MS_180000	1800000 // 13
#define T2_TIMEOUT_MS_10000		100000 // 14
#define T2_TIMEOUT_MS_30000		300000 // 15
#define T2_TIMEOUT_MS_6000	  60000	// 16
#define T2_TIMEOUT_MS_2500	  25000	// 17
extern uint32_t t2_count_ms[];

#define START_TIME2  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);TIM_Cmd(TIM2, ENABLE)
#define STOP_TIME2   TIM_Cmd(TIM2, DISABLE);RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE)

#define START_TIME3  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);TIM_Cmd(TIM3, ENABLE)
#define STOP_TIME3   TIM_Cmd(TIM3, DISABLE);RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , DISABLE)

void TIM2_NVIC_Configuration(void);
void TIM2_Configuration(void);

void TIM3_NVIC_Configuration(void);
void TIM3_Configuration(void);

#endif
