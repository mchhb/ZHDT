#ifndef _USART1_H_
#define _USART1_H_
#include "stm32f10x.h"
#include <stdio.h>
#include "types.h"

#define USART1_REC_LEN  		200  							// 定义串口接收缓存大小
#define EN_USART1_RX 			1									// 使能串口1接收，0为禁止
extern volatile unsigned char rxd_buffer_locked1;
extern volatile unsigned int usart_buf1[26];  	
extern u8  USART1_RX_BUF[USART1_REC_LEN]; 		// 声明串口接收缓存 
extern u16 USART1_RX_STA;         						// 声明串口接收状态标志位
void USART1_Config(void);
void USART1_Putc(unsigned char c);
void USART1_Puts(char * str);
void USART1_IRQHandler(void);
void USART1_NVIC_Configuration(void);
#endif 

