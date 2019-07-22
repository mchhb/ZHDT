#include "input.h"
#include "Tim2.h"

uint8_t input1_time = 0;
uint8_t input2_time = 0;
uint8_t input3_time = 0;
uint8_t input4_time = 0;
uint8_t input5_time = 0;
uint8_t input6_time = 0;
uint8_t input1_value = 0;
uint8_t input2_value = 0;
uint8_t input3_value = 0;
uint8_t input4_value = 0;
uint8_t input5_value = 0;
uint8_t input6_value = 0;
void input_gpio(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
   
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE);
	
 	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;  
 	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;  
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;  
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;  
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;  
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;        
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
 	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;  
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
}


void input1_status(void)
{
	if(input1 == 0)
	{
		input1_time = 1;
		if( t4_count_ms[1] > T4_TIMEOUT_MS_1)
		{
			input1_value = 1;
			t4_count_ms[1] = 0;
		}			
	}
	else
	{
		input1_value = 0;
	}
}

void input2_status(void)
{
	if(input2 == 0)
	{
		input2_time = 1;
		if( t4_count_ms[2] > T4_TIMEOUT_MS_2)
		{
			input2_value = 1;
			t4_count_ms[2] = 0;
		}			
	}
	else
	{
		input2_value = 0;
	}
}

void input3_status(void)
{
	if(input3 == 0)
	{
		input3_time = 1;
		if( t4_count_ms[3] > T4_TIMEOUT_MS_3)
		{
			input3_value = 1;
			t4_count_ms[3] = 0;
		}			
	}
	else
	{
		input3_value = 0;
	}
}

void input4_status(void)
{
	if(input4 == 0)
	{
		input4_time = 1;
		if( t4_count_ms[4] > T4_TIMEOUT_MS_4)
		{
			input4_value = 1;
			t4_count_ms[4] = 0;
		}			
	}
	else
	{
		input4_value = 0;
	}
}

void input5_status(void)
{
	if(input5 == 0)
	{
		input5_time = 1;
		if( t4_count_ms[1] > T4_TIMEOUT_MS_5)
		{
			input5_value = 1;
			t4_count_ms[5] = 0;
		}			
	}
	else
	{
		input5_value = 0;
	}
}

void input6_status(void)
{
	if(input6 == 0)
	{
		input6_time = 1;
		if( t4_count_ms[6] > T4_TIMEOUT_MS_6)
		{
			input6_value = 1;
			t4_count_ms[6] = 0;
		}			
	}
	else
	{
		input6_value = 0;
	}
}
/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/

