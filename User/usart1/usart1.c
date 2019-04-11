#include "usart1.h"

u8 USART1_RX_BUF[USART1_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u16 USART1_RX_STA=0;       //接收状态标记

volatile unsigned char rxd_buffer_locked1 =0;
volatile unsigned int usart_buf1[26];
volatile unsigned char usart_flg1=0;
volatile unsigned char ptr_rxd_buffer1=0;
unsigned char  start1=0;
unsigned char  crc1=0;
void USART1_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_USART1|RCC_APB2Periph_AFIO , ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  // Configure the USART1_Rx as input floating
  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IN_FLOATING ;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
	USART_Init(USART1, &USART_InitStructure);
	
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //Enable rx enable, 
  
  /* Configure the USARTx */ 
  /* Enable the USARTx */
  USART_Cmd(USART1, ENABLE);
	
	USART_ClearFlag(USART1, USART_FLAG_TC);     /* 清发送外城标志，Transmission Complete flag */
}
//int fputc(int ch, FILE *f)
//{
///* 将Printf内容发往串口 */
//  USART_SendData(USART1, (unsigned char) ch);
//  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); 
//  return (ch);
//}
void USART1_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;			// 抢占优先级设为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;						// 子优先级设为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								// IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);																// 根据指定的参数初始化NVIC寄存器
	//NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);  						// only app, no boot included
}
// 串口1中断服务程序
unsigned char usart_test = 0;
void USART1_IRQHandler(void)                	
{
	unsigned char c1 = 0;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{	
	//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , DISABLE);		/*先关闭等待使用*/ 	
		USART_ClearFlag(USART1,USART_IT_RXNE);
		// printf(" 进入中断 ！\r\n");
		 c1 = USART_ReceiveData(USART1);
		 //USART1_RX_BUF[usart_test++] = c1;
		 if(rxd_buffer_locked1)
		 {
			 return;
		 }
		 if((ptr_rxd_buffer1 == 0) &&(start1 == 0)&& (c1 == 0x66))  //receive 1byte
		 {
			 	 usart_buf1[ptr_rxd_buffer1++] = c1; 
         start1 = 1;
				 return;
		 }
		 if(ptr_rxd_buffer1<26&&start1==1) //receive 2-15byte
     {
		   if(ptr_rxd_buffer1<26)
		   {
	        	crc1 += c1; 
		   }
       usart_buf1[ptr_rxd_buffer1++] = c1; 
     }
		 if(ptr_rxd_buffer1 == 26)
     {
       if(usart_buf1[25]== 0x99)
       {
	        rxd_buffer_locked1 = 1;          // lock the rxd buffer
			    crc1 = 0;
				  usart_flg1 = 1;
				  ptr_rxd_buffer1 = 0;
       }
       ptr_rxd_buffer1 = 0; // reset rxd buffer pointer
       start1 = 0;
       crc1 = 0;
     }
	}
}
void USART1_Putc(unsigned char c)
{
    USART_SendData(USART1, c);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET );
}

void USART1_Puts(char * str)
{
    while(*str)
    {
        USART_SendData(USART1, *str++);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    }
}
