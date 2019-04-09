#ifndef __LOOP_H
#define	__LOOP_H

#include "stm32f10x.h"
#define MQTT_STRING 							
#define PARA_ADDR_FLASH						0x000800F400
#define PARA_ADDR_FLASH_A					0x000800F800
#define PARA_LENGTH								17				// unit is uint32_t

#define PARA_IDX_KEY1							0
#define PARA_IDX_KEY2							1
#define PARA_IDX_PROTECT_EN				2
#define PARA_IDX_ADC_VBAT_START		3
#define PARA_IDX_ADC_SGND_START		4
#define PARA_IDX_ADC_OVERLOAD			5
#define PARA_IDX_DUTY_VBAT_START	6
#define PARA_IDX_DUTY_SGND_START	7
#define PARA_IDX_DUTY_OVERLOAD		8
#define PARA_IDX_DUTY_INCA				9
#define PARA_IDX_DUTY_INCB				10
#define PARA_IDX_DUTY_DECA				11
#define PARA_IDX_DUTY_DECB				12
#define PARA_IDX_TIMES_VBAT_START	13
#define PARA_IDX_TIMES_SGND_START	14
#define PARA_IDX_TIMES_OVERLOAD		15
#define PARA_IDX_TIMES_STANDBY		16
extern uint8_t external_temp1,external_temp3;
extern uint8_t external_humi1,external_humi3;
extern uint8_t external_vibr1,external_vibr3;
extern uint8_t external_noise1,external_noise3;
extern uint8_t  network_status;
extern float temperature,humidity;
extern float 	AccelerationX;
extern float 	AccelerationY;
extern float 	AccelerationZ;
extern uint16_t 	noise;
extern uint8_t work_uart_status;
extern u32 CpuID[3];
extern uint8_t domain_ip[4];
extern uint8_t read_dht11;
extern uint8_t send_data;
extern uint32_t timePUB;
extern uint32_t timeSUB;
void Initialization_configuration(void);
void application(void);
void ENTER_SYS_STANDBY(void);
void SoftReset(void);
#endif



