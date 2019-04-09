#ifndef __RTC_H
#define __RTC_H	    
	
#include "stm32f10x.h"	
	
void RTC_Init(void);
void dev_rtc_setAlarm(u32 AlarmValue);
uint8_t dev_rtc_isAlarm(void);
void dev_iwdg_feed(void);
void dev_iwdg_init(void);

void ctrl_standby(void);

#endif


