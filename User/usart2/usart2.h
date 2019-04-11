#ifndef __USART2_H
#define	__USART2_H
#include "stm32f10x.h"
#include <stdio.h>

extern volatile unsigned char rxd_buffer_locked;
extern volatile unsigned int usart_buf[16];
extern volatile unsigned char usart_flg;
extern volatile unsigned char ptr_rxd_buffer;
extern unsigned char  start;
extern unsigned char  crc;

void USART2_Config(void);
void UART2Test(void);
void print_msg(unsigned char *msg);
int fputc(int ch, FILE *f);
void USART2_NVIC_Configuration(void);
void USART2_IRQHandler(void);
void USART_SendData_CBuilder(USART_TypeDef* USARTx,unsigned char ch);
#endif /* __USART1_H */
