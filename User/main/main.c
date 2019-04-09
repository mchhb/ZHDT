/* 
  ******************************************************************************
  * @file    ZHDT
  * @author  June_hhb
  * @version 
  * @date    
  * @brief   
  *          
  *          
  *********************************************       *********************************
*/
#include "LOOP.h"
#include "delay.h"
int main(void)
{
	Systick_Init(72); //配置系统时钟为72M   
	
  Initialization_configuration();
	
	for(;;) 
	{
		
		application();
		
	}
}




