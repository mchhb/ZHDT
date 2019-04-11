//**********************************************************************************************     
//  STM32F10x StopMode RTC Feed Dog 
//  compiler: Keil UV3     
//  2013-01-04 , By friehood     
//**********************************************************************************************  
#include "rtc.h"

u8 g_bRTCAlarm=0;
/*******************************************************************************
* Function Name  : RTC_Configuration
* Description    : Configures RTC clock source and prescaler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RTC_Init(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);   
	PWR_BackupAccessCmd(ENABLE);
	/* RTC clock source configuration ------------------------------------------*/
	/* Allow access to BKP Domain */
	PWR_BackupAccessCmd(ENABLE);

	/* Reset Backup Domain */
	BKP_DeInit();

	/* Enable the LSI OSC */
	RCC_LSICmd(ENABLE);

	/* Wait till LSI is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET){}
	
	/* Select the RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

	/* Enable the RTC Clock */
	RCC_RTCCLKCmd(ENABLE);

	/* RTC configuration -------------------------------------------------------*/
	/* Wait for RTC APB registers synchronisation */
	RTC_WaitForSynchro();

	/* Set RTC prescaler: set RTC period to 1sec */
	RTC_SetPrescaler(32767);
	
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();

	/* Enable the RTC Alarm interrupt */
	RTC_ITConfig(RTC_IT_ALR, ENABLE);
	
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();

	/* Configure EXTI Line17(RTC Alarm) to generate an interrupt on rising edge */
	EXTI_ClearITPendingBit(EXTI_Line17);
	EXTI_InitStructure.EXTI_Line = EXTI_Line17;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable the RTC Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = RTCAlarm_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

/*******************************************************************************
* Function Name  : RTCAlarm_IRQHandler
* Description    : This function handles RTC Alarm interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RTCAlarm_IRQHandler(void)
{
	if(RTC_GetITStatus(RTC_IT_ALR) != RESET)
	{
		/* Set the RTC alarm flag */
		g_bRTCAlarm = TRUE;

		/* Clear EXTI line17 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line17);

		/* Check if the Wake-Up flag is set */
		if(PWR_GetFlagStatus(PWR_FLAG_WU) != RESET)
		{
			/* Clear Wake Up flag */
			PWR_ClearFlag(PWR_FLAG_WU);
		}                                                                                                                                                
																																																		 
		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();   
		/* Clear RTC Alarm interrupt pending bit */
		RTC_ClearITPendingBit(RTC_IT_ALR);
		/* Wait until last write operation on RTC registers has finished */
		RTC_WaitForLastTask();
	}
}


/*******************************************************************************
* Function Name  : dev_rtc_setAlarm
* Description    : 设置RTC闹钟.
* Input          : 闹钟时间
* Output         : None
* Return         : None
*******************************************************************************/
void dev_rtc_setAlarm(u32 AlarmValue)
{
	/* Clear the RTC SEC flag */
	RTC_ClearFlag(RTC_FLAG_SEC);
	/* Wait clear RTC flag sccess */
	while(RTC_GetFlagStatus(RTC_FLAG_SEC) == RESET);
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask(); 

	/* Sets the RTC alarm value */
	RTC_SetAlarm(RTC_GetCounter() + AlarmValue);
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask(); 
}

// /*******************************************************************************
// * Function Name  : dev_rtc_isAlarm
// * Description    : RTC闹钟是否触发
// * Input          : None
// * Output         : None
// * Return         : TRUE:已触发，FALSE，未触发
// *******************************************************************************/
u8 dev_rtc_isAlarm(void)
{
	if(g_bRTCAlarm)
	{
		/* Clear the RTC alarm flag */
		g_bRTCAlarm = FALSE;
		return 1;
	}
	return 0;                        
}

void dev_iwdg_init(void)
{
	/* Enable write access to IWDG_PR and IWDG_RLR registers */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	/* IWDG counter clock: 40KHz(LSI) / 256 = 0.15625 KHz */
	IWDG_SetPrescaler(IWDG_Prescaler_256);
	/* Set counter reload value to 1562(10S) */
	IWDG_SetReload(625);        // 3s468
	/* Reload IWDG counter */
	IWDG_ReloadCounter();
	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	IWDG_Enable();
}

void dev_iwdg_feed(void)
{
	IWDG_ReloadCounter();
}

// /*******************************************************************************
// * Function Name  : dev_clk_restore
// * Description    : Restore system clock after wake-up from STOP: enable HSE, PLL
// *                  and select PLL as system clock source.
// * Input          : None
// * Output         : None
// * Return         : None
// *******************************************************************************/
ErrorStatus HSEStartUpStatus;
void dev_clk_restore(void)
{
  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  {
    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }
}

/*******************************************************************************
* Function Name  : EXTI_Configuration
* Description    : Configures EXTI Line3.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXIT_Configuration(void)
{
// 	EXTI_InitTypeDef EXTI_InitStructure;

// 	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource3);
// 	EXTI_ClearITPendingBit(EXTI_Line3);
// 	EXTI_InitStructure.EXTI_Line = EXTI_Line3;
// 	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;                                                                                  
// 	EXTI_InitStructure.EXTI_Trigger        = EXTI_Trigger_Falling;        
// 	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
// 	EXTI_Init(&EXTI_InitStructure);
}

void dev_exti_enable(uint8_t bEnable)
{
// 	NVIC_InitTypeDef NVIC_InitStructure;

// 	/* Clear the Key Button EXTI line pending bit */
// 	EXTI_ClearITPendingBit(EXTI_Line3);

// 	NVIC_ClearIRQChannelPendingBit(EXTI3_IRQChannel);
// 	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQChannel;
// 		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
// 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
// 	NVIC_InitStructure.NVIC_IRQChannelCmd = bEnable ? ENABLE : DISABLE;
// 	NVIC_Init(&NVIC_InitStructure);        
}


uint8_t m_bEnterStandByMode = 0;
void Sys_Standby(void);
void ctrl_standby(void)	{
	// 进入待机模式开关
	if(m_bEnterStandByMode)
	{        
		// 使能外部中断，GPIOB3，用以MCU从待机模式唤醒
//		dev_exti_enable(TRUE);
		Sys_Standby();
ENTERSTOPMODE:        
		// 设置RTC闹钟，5秒钟产生一次RTC闹钟中断*/
		dev_rtc_setAlarm(2);

		// 进入停止模式（低功耗），直至外部中断触发时被唤醒
		PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);

		// 是否是RTC闹钟中断唤醒
		if(dev_rtc_isAlarm())
		{
			// 喂狗
			dev_iwdg_feed();
			// 喂完狗继续进入停止模式
			goto ENTERSTOPMODE;        
		}
		// 禁止外部中断 
//		dev_exti_enable(FALSE);
		// 从停止模式唤醒后恢复系统时钟
		dev_clk_restore();
	}      
}



// int main(void)
// {
//   /* System Clocks Configuration */
//   RCC_Configuration();

//   /* NVIC configuration */
//   NVIC_Configuration();

//   /* Configure RTC clock source and prescaler */
//   RTC_Configuration();

//   /* Configure the SysTick to generate an interrupt each 1 millisecond */
//   SysTick_Configuration();

//   /* Configures EXTI Line3 */
//   EXIT_Configuration();

//   /* IWDG initialize*/
//   dev_iwdg_init();

//   while(1)
//   {
//     // 执行任务
//         Task1();
//         Task2();
//         // ..

//         // 喂狗
//         dev_iwdg_feed();

//         // 进入待机模式开关
//         if(m_bEnterStandByMode)
//         {        
//             // 使能外部中断，GPIOB3，用以MCU从待机模式唤醒
//             dev_exti_enable(TRUE);
// ENTERSTOPMODE:        
//                 // 设置RTC闹钟，5秒钟产生一次RTC闹钟中断*/
//                 dev_rtc_setAlarm(5);
//         
//                 // 进入停止模式（低功耗），直至外部中断触发时被唤醒
//                 PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
//         
//                 // 是否是RTC闹钟中断唤醒
//                 if(dev_rtc_isAlarm())
//                 {
//                         // 喂狗
//                         dev_iwdg_feed();
//                         // 喂完狗继续进入停止模式
//                         goto ENTERSTOPMODE;        
//                 }
//                 // 禁止外部中断 
//                 dev_exti_enable(FALSE);
//                 // 从停止模式唤醒后恢复系统时钟
//                 dev_clk_restore();
//         }                      
//   }
// }



