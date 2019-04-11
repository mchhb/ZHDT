/******************** 鑫盛电子工作室 ********************
 * 文件名  ：key.c
 * 描述    ：按键应用函数库        
 * 实验平台：MINI STM32开发板 基于STM32F103C8T6
 * 硬件连接： PB12 - key1  PB13 - key12           
 * 库版本  ：ST3.0.0 
 * 淘宝店：http://shop66177872.taobao.com
**********************************************************************************/

#include "key.h" 
#include "delay.h"
 /*不精确的延时 */
 void Delay(__IO u32 nCount)
{
  for(; nCount != 0; nCount--);
} 

 /*	 配置按键用到的I/O口 */
void Key_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE); //开启按键端口PA的时钟
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP; //端口配置为上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);						//初始化端口

 	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD; 		//端口配置为上拉输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);						//初始化端口
 	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP; //端口配置为上拉输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);						//初始化端口
}

 /* 检测是否有按键按下 */
u8 Key_Scan(GPIO_TypeDef* GPIOx,u16 GPIO_Pin)
{			
		/*检测是否有按键按下 */
   	if(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == KEY_OFF ) 
	  {	   
	 	 /*延时消抖*/
	  	delay_ms(140);	
			
	   		if(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == KEY_OFF )  
					{	 
						/*等待按键释放 */
//						while(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == KEY_ON);   
			 			return 	KEY_OFF;	 
					}
			else
					return KEY_ON;
		}
	else
		return KEY_ON;
}


#define DIN_FILTER_TIMES			100							// 100 * 10ms = 1000ms
uint32_t flt_setting = DIN_FILTER_TIMES;			// value may to re-write by CAN
uint32_t flt_times  = 0;
uint8_t  last_value = 1;
uint8_t  real_value = 1;
uint8_t din_get(void)	{
	uint8_t rt_value;
	
	rt_value = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5);
	if(rt_value == last_value) {
		if(flt_times > flt_setting) {
			real_value = rt_value;
		}
		else {
			flt_times++;
		}
	}
	else {
		last_value = rt_value;
		flt_times  = 0;
	}
	return real_value;
}



