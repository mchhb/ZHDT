#ifndef _CONFIG_H_
#define _CONFIG_H_

#include "stm32f10x.h"
#include "Types.h"
#define __GNUC__

#define SOCK_DHCP             0
#define SOCK_HTTP             1

#define FW_VER_HIGH   	1
#define FW_VER_LOW    	0

#define ON										1
#define OFF										0
#define HIGH									1
#define LOW										0
	
#define CONFIG_MSG_LEN        sizeof(CONFIG_MSG) - 4 // the 4 bytes OP will not save to EEPROM
#define MAX_BUF_SIZE					1460
#define KEEP_ALIVE_TIME	      30	// 30sec
// SRAM address range is 0x2000 0000 ~ 0x2000 BFFF (48KB)
#define SOCK_BUF_ADDR 				0x20000000
#define AppBackAddress        0x08020000 //from 128K
#define ConfigAddr						0x0800FC00
#define NORMAL_STATE          0
#define NEW_APP_IN_BACK       1 //there is new app in back address
#define CONFIGTOOL_FW_UP      2 //configtool update f/w in app


#endif


