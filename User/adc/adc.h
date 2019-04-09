#ifndef __ADC_H
#define	__ADC_H

#include "stm32f10x_dma.h"
#include "stm32f10x_adc.h"
#include "stm32f10x.h"

void ADC1_Init(void);

uint16_t adc_filter(uint16_t dat);

#endif /* __ADC_H */



/*------------------------------------- END OF THIS FILE --------------------------------------------------------*/












