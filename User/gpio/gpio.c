#include "gpio.h"

/**
  * @brief  Configure the TIM3 Ouput Channels.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
   
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
 	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;  
 	GPIO_Init(GPIOA, &GPIO_InitStructure);//w5500 int  	

 	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;  
 	GPIO_Init(GPIOA, &GPIO_InitStructure);//w5500 reset  	
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB,GPIO_Pin_5);//DHT11 DATA
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;  
 	GPIO_Init(GPIOB, &GPIO_InitStructure);//work normal led  
	GPIO_SetBits(GPIOB,GPIO_Pin_3);//
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;  
 	GPIO_Init(GPIOC, &GPIO_InitStructure);//work error led  	
	GPIO_SetBits(GPIOC,GPIO_Pin_14);//
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;  
 	GPIO_Init(GPIOA, &GPIO_InitStructure);//network success led 
 	GPIO_SetBits(GPIOA,GPIO_Pin_12);//
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;  
 	GPIO_Init(GPIOC, &GPIO_InitStructure);//network fail led
	GPIO_SetBits(GPIOC,GPIO_Pin_15);//
}

