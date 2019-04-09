#include "nvic.h"

void NVIC_Config(void)
{
	   
		  //NVIC_InitTypeDef NVIC_InitStructure;  
  
	    #ifdef USING_RTCAlarm_IRQ
			/* Enable the RTCALR Interrupt */  
			NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;  
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
			NVIC_Init(&NVIC_InitStructure);
    #endif
     
	  #ifdef USING_RTC
			/* Enable the RTCSEC Interrupt */  
			NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);
	  #endif
	
	  #ifdef USING_TIM4_IRQ
			/*TIM4 Interrupt */ 
			NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4中断       // group_3 3bit pr+ 1bit order
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级0级
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级3级
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
			NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
		#endif
		
		#ifdef USING_EXTI9_5_ITQ
		/*EXTI9_5 Interrupt */ 
		NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//抢占优先级
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;					//子优先级
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
		NVIC_Init(&NVIC_InitStructure); 
		#endif
		
		#ifdef USING_EXTI4_ITQ
		/*EXTI4 Interrupt */ 
		NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;			
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//抢占优先级
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;					//子优先级
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
		NVIC_Init(&NVIC_InitStructure); 
		#endif
}
