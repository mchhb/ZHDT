#ifndef __PWM_OUTPUT_H
#define	__PWM_OUTPUT_H

#include "stm32f10x.h"

void TIM1_PWM_Init(void);
void pwm_led_out (void);

void pwm_start (void);
void pwm_stop (void);


void pwm_duty_set (uint32_t value);
void pwm_duty_dec (void);
void pwm_duty_inc (void);	

void pwm_duty_dec_min (void);
void pwm_duty_inc_min (void);

#endif /* __PWM_OUTPUT_H */


