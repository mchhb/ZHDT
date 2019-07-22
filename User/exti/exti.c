/********************************************************
 * 文件名  ：exti.c
*********************************************************/
#include "exti.h"
/*  配置 PA0 PA15 为线中断口，并设置中断优先级 */
void EXTI_PA0_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure; 
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE); //使能端口和复用IO时钟 
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);// 设置优先级分组
  
  /* 配置P[A|B|C|D|E]0为中断源 */
  NVIC_InitStructure.NVIC_IRQChannel 										= EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 0;	//先占优先级为0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority 				= 6;  //从优先级为0
  NVIC_InitStructure.NVIC_IRQChannelCmd 								= ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_0;       
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_IPD;	 					// 上拉输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0); //选择PA口的0号口为中断线路
  EXTI_InitStructure.EXTI_Line 		= EXTI_Line0; 							//外部中断线1
  EXTI_InitStructure.EXTI_Mode 		= EXTI_Mode_Interrupt; 			//设置为中断请求
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 		//下降沿中断
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure); 
}

void EXTI_PA1_Config(void)
{	
  NVIC_InitTypeDef NVIC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure; 
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE); //使能端口和复用IO时钟 
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);// 设置优先级分组
  
  /* 配置P[A|B|C|D|E]0为中断源 */
  NVIC_InitStructure.NVIC_IRQChannel 										= EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = 0;		//先占优先级为0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority 				= 7;  	//从优先级为0
  NVIC_InitStructure.NVIC_IRQChannelCmd 								= ENABLE;
  NVIC_Init(&NVIC_InitStructure);
			
  GPIO_InitStructure.GPIO_Pin 		= GPIO_Pin_1;       
  GPIO_InitStructure.GPIO_Mode 		= GPIO_Mode_IPD;	 						// 上拉输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);	//中断线0连接GPIOA.15
  EXTI_InitStructure.EXTI_Line    = EXTI_Line1;								//设置按键所有的外部线路
	EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;			//设外外部中断模式:EXTI线路为中断请求
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  		//下降沿触发
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);															// 初始化外部中断
}
void EXTI_ENABLE(void)
{
		EXTI_PA0_Config();
		EXTI_PA1_Config();
}
