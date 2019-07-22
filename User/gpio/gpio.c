#include "gpio.h"

/**
  * @brief  Configure the TIM3 Ouput Channels.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
   
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
 	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;  
 	GPIO_Init(GPIOC, &GPIO_InitStructure);//w5500 int  	

 	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;  
 	GPIO_Init(GPIOC, &GPIO_InitStructure);//w5500 reset  	
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_SetBits(GPIOC,GPIO_Pin_9);//DHT11 DATA
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;  
 	GPIO_Init(GPIOB, &GPIO_InitStructure);//work normal led  
	GPIO_SetBits(GPIOB,GPIO_Pin_8);//
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;  
 	GPIO_Init(GPIOB, &GPIO_InitStructure);//work error led  	
	GPIO_SetBits(GPIOB,GPIO_Pin_9);//
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;  
 	GPIO_Init(GPIOB, &GPIO_InitStructure);//network success led 
 	GPIO_SetBits(GPIOB,GPIO_Pin_4);//
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;  
 	GPIO_Init(GPIOB, &GPIO_InitStructure);//network fail led
	GPIO_SetBits(GPIOB,GPIO_Pin_5);//
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;  
 	GPIO_Init(GPIOC, &GPIO_InitStructure);//power success led 
 	GPIO_SetBits(GPIOC,GPIO_Pin_13);//
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;  
 	GPIO_Init(GPIOC, &GPIO_InitStructure);//power fail led
	GPIO_SetBits(GPIOC,GPIO_Pin_14);//
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;  
 	GPIO_Init(GPIOD, &GPIO_InitStructure);//sp32321 
 	GPIO_SetBits(GPIOD,GPIO_Pin_2);//
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;  
 	GPIO_Init(GPIOB, &GPIO_InitStructure);//sp32321
	GPIO_SetBits(GPIOB,GPIO_Pin_3);//
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;  
 	GPIO_Init(GPIOC, &GPIO_InitStructure);//sp32322 
 	GPIO_SetBits(GPIOC,GPIO_Pin_11);//
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;  
 	GPIO_Init(GPIOC, &GPIO_InitStructure);//sp32322
	GPIO_SetBits(GPIOC,GPIO_Pin_12);//
}

