#ifndef __INPUT_H
#define	__INPUT_H

#include "stm32f10x.h"

void input_gpio(void);

#define input1	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_10)
#define input2	GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)
#define input3	GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10)
#define input4	GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)
#define input5	GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)
#define input6	GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5)

extern uint8_t input1_value;
extern uint8_t input2_value;
extern uint8_t input3_value;
extern uint8_t input4_value;
extern uint8_t input5_value;
extern uint8_t input6_value;

extern uint8_t input1_time;
extern uint8_t input2_time;
extern uint8_t input3_time;
extern uint8_t input4_time;
extern uint8_t input5_time;
extern uint8_t input6_time;

void input1_status(void);
void input2_status(void);
void input3_status(void);
void input4_status(void);
void input5_status(void);
void input6_status(void);
		
#endif /* __INPUT_H */



/*------------------------------------- END OF THIS FILE --------------------------------------------------------*/












