#ifndef __24c16_H
#define __24c16_H

#include "stm32f10x.h"
#include "types.h"

#define I2C_SCK	        GPIO_Pin_6  //out
#define I2C_SDA	        GPIO_Pin_7
//------------------------------------目标IP
#define EE_ADDR_IP_ONE						0x00
#define EE_ADDR_IP_TWO						0x02
#define EE_ADDR_IP_THREE					0x04
#define EE_ADDR_IP_FOUR						0x06
//------------------------------------分配IP
#define EE_ADDR_DHCP_IP_ONE				0x10
#define EE_ADDR_DHCP_IP_TWO				0x12
#define EE_ADDR_DHCP_IP_THREE			0x14
#define EE_ADDR_DHCO_IP_FOUR			0x16
//------------------------------------MAC
#define EE_ADDR_MAC_ONE						0x20
#define EE_ADDR_MAC_TWO						0x22
#define EE_ADDR_MAC_THREE					0x24
#define EE_ADDR_MAC_FOUR					0x26
#define EE_ADDR_MAC_FIVE					0x28
#define EE_ADDR_MAC_SIX						0x2A
//------------------------------------子网掩码
#define EE_ADDR_SUBNET_ONE				0x30
#define EE_ADDR_SUBNET_TWO				0x32
#define EE_ADDR_SUBNET_THREE			0x34
#define EE_ADDR_SUBNET_FOUR				0x36
//------------------------------------网关
#define EE_ADDR_GATEWAY_ONE				0x40
#define EE_ADDR_GATEWAY_TWO				0x42
#define EE_ADDR_GATEWAY_THREE			0x44
#define EE_ADDR_GATEWAY_FOUR			0x46
//------------------------------------DNS
#define EE_ADDR_DNS_ONE						0x50
#define EE_ADDR_DNS_TWO						0x52
#define EE_ADDR_DNS_THREE					0x54
#define EE_ADDR_DNS_FOUR					0x56
//------------------------------------端口
#define EE_ADDR_PORT_1							0x60
#define EE_ADDR_PORT_2							0x62
//------------------------------------TEST
#define EE_ADDR_TEST							0X70
//------------------------------------w5500 config
#define EE_ADDR_W5500_CONFIG_DHCP_IP			0x100
#define EE_ADDR_W5500_CONFIG_MAC					0x102
#define EE_ADDR_W5500_CONFIG_PORT					0x104
#define EE_ADDR_W5500_CONFIG_IP						0x106
#define EE_ADDR_REBOOT										0x110
#define EE_ADDR_DOOR_CLOSE_TIMES					0x112
#define EE_ADDR_DOOR_CLOSE_TIMES_SUM			0x114
#define EE_ADDR_DOOR_CLOSE_TIMES_SUM1			0x116
#define EE_ADDR_DOOR_CLOSE_TIMES_SUM2			0x118

#define EE_ADDR_MQTT_SEND_TIME						0x200
#define EE_ADDR_ID_NAME1 									0x205
#define EE_ADDR_ID_NAME2									0x207
#define EE_ADDR_ID_NAME3 									0x209
#define EE_ADDR_ID_NAME4 									0x211
#define EE_ADDR_W5500_PC_CONFIG_SUCCESS     0x01
#define EE_ADDR_W5500_RAM_CONFIG_SUCCESS			0x02

void at24c16_init(void);
void at24c16_write(uint16 addr, unsigned char val);
unsigned char at24c16_read(uint16 addr);

void eep_block_write(uint16 eepAddr, uint8* dat, uint16 index, uint16 len);

void erase_eeprom(uint16 startAddr, uint16 len);

#endif /* __MAIN_H */




