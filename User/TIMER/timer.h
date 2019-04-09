#ifndef __TIMER_H
#define __TIMER_H
#include "stm32f10x.h"

#define T2_MS_ARRAY_NUM  		10
#define T2_COUNTER_MS_0			0
#define T2_COUNTER_MS_1			1
#define T2_COUNTER_MS_2			2
#define T2_COUNTER_MS_3			3
#define T2_COUNTER_MS_4			4
#define T2_COUNTER_MS_5			5
#define T2_COUNTER_MS_6			6
#define T2_COUNTER_MS_7			7
#define T2_COUNTER_MS_8			8
#define T2_COUNTER_MS_9			9

// unit is 0.1ms
#define T2_TIMEOUT_MS_0			1
#define T2_TIMEOUT_MS_1			1000
#define T2_TIMEOUT_MS_2			5
#define T2_TIMEOUT_MS_3			1000
#define T2_TIMEOUT_MS_4			1000
#define T2_TIMEOUT_MS_5			100000
#define T2_TIMEOUT_MS_6			100				// for simple din
#define T2_TIMEOUT_MS_7			1000
#define T2_TIMEOUT_MS_8			100000
#define T2_TIMEOUT_MS_9			5000


extern uint32_t t2_count_ms[];

#define START_TIME2  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);TIM_Cmd(TIM2, ENABLE)
#define STOP_TIME2   TIM_Cmd(TIM2, DISABLE);RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE)


void TIM2_NVIC_Configuration(void);
void TIM2_Configuration(void);


#endif


