/******************** 鑫盛电子工作室 ********************
 * 文件名  ：pwm_output.c
 * 描述    ：         
 * 实验平台：MINI STM32开发板 基于STM32F103C8T6
 * 			
 * 库版本  ：ST3.0.0
 * 淘宝店：http://shop66177872.taobao.com
*********************************************************/
#include "pwm_output.h"
#include "misc.h"
#include "loop.h"
unsigned int adc;

void TIM1_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* 配置TIM1复用输出PWM时用到的I/O  */
static void TIM1_GPIO_Config(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOA and GPIOB clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin   =  GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

}

/*配置TIM1输出的PWM信号的模式，如周期、极性、占空比 */
static void TIM1_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* PWM信号电平跳变值 */
	u16 CCR1_Val = 0;        
	u16 CCR2_Val = 0;//5000;

/* TIM1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
/* -----------------------------------------------------------------------
    TIM1 Configuration: generate 2 PWM signals with 2 different duty cycles:
    TIM1CLK = 72 MHz, Prescaler = 0x0, TIM1 counter clock = 72 MHz
    TIM1 ARR Register = 50000 => TIM1 Frequency = TIM1 counter clock/(ARR + 1)
    TIM1 Frequency = 14400Hz.
    TIM1 Channel1 duty cycle = (TIM1_CCR1/ TIM1_ARR)* 100 
    TIM1 Channel2 duty cycle = (TIM1_CCR2/ TIM1_ARR)* 100 
    
  ----------------------------------------------------------------------- */

 /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 5000;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  TIM_ClearFlag(TIM1, TIM_FLAG_Update);					  /* 清除溢出中断标志 */
  TIM_Cmd(TIM1, DISABLE);													/* 先关闭时钟 */

	/* PWM1 Mode configuration: Channel1 */  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
//  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;//开反向通道
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
//  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;	  
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);	


/* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;//开反向通道
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
//  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;	  
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);	


	TIM_ARRPreloadConfig(TIM1, ENABLE);

	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);           //使用更新事件作为触发输出
	
  /* TIM1 enable counter */
  TIM_Cmd(TIM1, ENABLE);
  TIM_CtrlPWMOutputs(TIM1, ENABLE);	
}

/*配置和初始化TIM1 */
void TIM1_PWM_Init(void)
{
	TIM1_NVIC_Configuration();
	TIM1_GPIO_Config();
	TIM1_Mode_Config();	
}


extern uint32_t para[];
uint16_t  rt_pwm_value = 5;		//5000;
void pwm_duty_set(uint32_t value)	{
	TIM1->CCR2 = value;
	TIM1->CCR1 = value;
}
#if 0
void pwm_duty_dec (void)	{
	if(rt_pwm_value > 3500)	{
		rt_pwm_value -= para[PARA_IDX_DUTY_DECA]; //
	}
	else	{
		rt_pwm_value -= para[PARA_IDX_DUTY_DECB]; //
	}

	if((rt_pwm_value <= 10) || (rt_pwm_value > 5000)) {
		rt_pwm_value = 5;
	}
	TIM1->CCR2 = rt_pwm_value;
	TIM1->CCR1 = rt_pwm_value;
}

void pwm_duty_inc (void){ 
	if(rt_pwm_value < 3500)	{		
		rt_pwm_value += para[PARA_IDX_DUTY_INCA]; //
	}
  else	{
		rt_pwm_value += para[PARA_IDX_DUTY_INCB]; //		
  }	
	if(rt_pwm_value >= 4999)	{ 
		rt_pwm_value = 5000;
	}	
	
	TIM1->CCR2 = rt_pwm_value;
	TIM1->CCR1 = rt_pwm_value; 	

}


void pwm_duty_dec_min (void)	{
	rt_pwm_value -= 10; //

	if((rt_pwm_value <= 10) || (rt_pwm_value > 5000)) {
		rt_pwm_value = 5;
	}
	TIM1->CCR2 = rt_pwm_value;
	TIM1->CCR1 = rt_pwm_value;
}

void pwm_duty_inc_min (void){ 
	rt_pwm_value += 10; //
	if(rt_pwm_value >= 4999)	{ 
		rt_pwm_value = 5000;
	}	
	
	TIM1->CCR2 = rt_pwm_value;
	TIM1->CCR1 = rt_pwm_value; 	

}


#else
#define MAX_DPWM			10
uint16_t gp_dpwm[]     = {30,  40,   50,   60,   80,   100,  100,  200,  300,  400  };
uint16_t gp_time[]     = {500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000};
//uint16_t dec_gp_dpwm[] = {100, 100,  100,  100,  100,  100,  200,  300,  400,  500  };
uint16_t dec_gp_dpwm[] = {60,  60,  80,   100,   100,  100,  200,  300,  400,  500  };
uint16_t dec_gp_time[] = {500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000};

void pwm_duty_dec (void)	{
	int i;
	for(i = 0; i < MAX_DPWM; i++) {
		if(rt_pwm_value <= dec_gp_time[i]) {
			rt_pwm_value -= dec_gp_dpwm[i];
			break;
		}
	}
// 		rt_pwm_value -= 20;
	
// 	if(rt_pwm_value > 3500)	{
// 		rt_pwm_value -= para[PARA_IDX_DUTY_DECA]; //
// 	}
// 	else	{
// 		rt_pwm_value -= para[PARA_IDX_DUTY_DECB]; //
// 	}

	if((rt_pwm_value <= 10) || (rt_pwm_value > 5000)) {
		rt_pwm_value = 5;
	}
	TIM1->CCR2 = rt_pwm_value;
	TIM1->CCR1 = rt_pwm_value;
}

void pwm_duty_inc (void){ 
	int i;
	for(i = 0; i < MAX_DPWM; i++) {
		if(rt_pwm_value <= gp_time[i]) {
			rt_pwm_value += gp_dpwm[i];
			break;
		}
	}

// 	if(rt_pwm_value < 3500)	{		
// 		rt_pwm_value += para[PARA_IDX_DUTY_INCA]; //
// 	}
//   else	{
// 		rt_pwm_value += para[PARA_IDX_DUTY_INCB]; //		
//   }	
	if(rt_pwm_value >= 4999)	{ 
		rt_pwm_value = 5000;
	}	
	
	TIM1->CCR2 = rt_pwm_value;
	TIM1->CCR1 = rt_pwm_value; 	

}
void pwm_duty_dec_min (void)	{
	rt_pwm_value -= 10; //

	if((rt_pwm_value <= 10) || (rt_pwm_value > 5000)) {
		rt_pwm_value = 5;
	}
	TIM1->CCR2 = rt_pwm_value;
	TIM1->CCR1 = rt_pwm_value;
}

void pwm_duty_inc_min (void){ 
	rt_pwm_value += 10; //
	if(rt_pwm_value >= 4999)	{ 
		rt_pwm_value = 5000;
	}	
	
	TIM1->CCR2 = rt_pwm_value;
	TIM1->CCR1 = rt_pwm_value; 	

}


#endif
