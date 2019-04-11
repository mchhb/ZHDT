#include "w5500_config.h"
#include "Types.h"
#include "w5500.h"
#include "config.h"
#include "usart2.h"
#include <stdio.h> 
#include "w5500_config.h"
#include <string.h>
#include "dhcp.h"
#include "Tim2.h"
#include "24c16.h"
#include "LOOP.h"

CONFIG_MSG  ConfigMsg, RecvMsg;
EEPROM_MSG_STR EEPROM_MSG;	
void dhcp_config_w5500(void);
uint8 mac[6]={0x00,0x08,0xdc,0x22,0x22,0x22};
uint8 local_ip[4]={202,106,5,120};											/*定义W5500默认IP地址*/
uint8 subnet[4]={255,255,255,0};												/*定义W5500默认子网掩码*/
uint8 gateway[4]={202,106,5,1};													/*定义W5500默认网关*/
uint8 dns_server[4]={114,114,114,114};									/*定义W5500默认DNS*/
uint16 local_port=8080;	                    						/*定义本地端口*/
/*IP配置方法选择，请自行选择*/
uint8  ip_from=IP_FROM_DHCP;		

uint32 BD_TG_ping_time =0;  
uint8  dhcp_ok=0;																				/*dhcp成功获取IP*/
uint32 ms=0;																						/*毫秒计数*/
vu8    ntptimer = 0;																		/*NPT秒计数*/
uint8 txsize[MAX_SOCK_NUM] = {2,2,2,2,2,2,2,2};		// 选择8个Socket每个Socket发送缓存的大小，在w5500.c的void sysinit()有设置过程
uint8 rxsize[MAX_SOCK_NUM] = {2,2,2,2,2,2,2,2};		// 选择8个Socket每个Socket接收缓存的大小，在w5500.c的void sysinit()有设置过程
/**
*@brief		dhcp用到的定时器初始化
*@param		无
*@return	无
*/
void dhcp_timer_init(void)
{
  //timer2_init();	
  /*******TIM2 定时配置*********/
	TIM2_Configuration();
	/******* 中断优先级 **********/
	TIM2_NVIC_Configuration();
  /* TIM2 重新开时钟，开始计时 */	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
}

/**
*@brief		配置W5500的IP地址
*@param		无
*@return	无
*/
void set_w5500_ip(void)
{	
	/*使用DHCP获取IP参数，需调用DHCP子函数*/	
	//if(at24c16_read(EE_ADDR_W5500_CONFIG) == EE_ADDR_W5500_PC_CONFIG_SUCCESS)
	if(ip_from==IP_FROM_DEFINE)	
	{
		printf(" 使用定义的IP信息配置W5500:\r\n");
		memcpy(ConfigMsg.mac, mac, 6);
		memcpy(ConfigMsg.lip,local_ip,4);
		memcpy(ConfigMsg.sub,subnet,4);
		memcpy(ConfigMsg.gw,gateway,4);
		memcpy(ConfigMsg.dns,dns_server,4);
	}
		/*使用EEPROM存储的IP参数*/	
	if(at24c16_read(EE_ADDR_W5500_CONFIG_DHCP_IP) == EE_ADDR_W5500_PC_CONFIG_SUCCESS)
	{	
		/*如果读取EEPROM中MAC信息,如果已配置，则可使用*/		
			printf(" IP from EEPROM\r\n");
			/*复制EEPROM配置信息到配置的结构体变量*/
			EEPROM_MSG.lip[0] = at24c16_read(EE_ADDR_DHCP_IP_ONE);
			EEPROM_MSG.lip[1] = at24c16_read(EE_ADDR_DHCP_IP_TWO);
			EEPROM_MSG.lip[2] = at24c16_read(EE_ADDR_DHCP_IP_THREE);
			EEPROM_MSG.lip[3] = at24c16_read(EE_ADDR_DHCO_IP_FOUR);
			memcpy(ConfigMsg.lip,EEPROM_MSG.lip, 4);				
	}
	else
	{
		printf(" EEPROM未配置,使用DHCP的IP信息配置W5500,并写入EEPROM\r\n");
		//write_config_to_eeprom();	/*使用默认的IP信息，并初始化EEPROM中数据*/
		at24c16_write(EE_ADDR_DHCP_IP_ONE, DHCP_GET.lip[0]);
		at24c16_write(EE_ADDR_DHCP_IP_TWO, DHCP_GET.lip[1]);
		at24c16_write(EE_ADDR_DHCP_IP_THREE, DHCP_GET.lip[2]);
		at24c16_write(EE_ADDR_DHCO_IP_FOUR, DHCP_GET.lip[3]);		
		memcpy(ConfigMsg.lip,DHCP_GET.lip, 4);
	}	

	/*使用DHCP获取IP参数，需调用DHCP子函数*/		
	if(ip_from==IP_FROM_DHCP)								
	{
		/*复制DHCP获取的配置信息到配置结构体*/
		if(dhcp_ok==1)
		{
			printf(" IP from DHCP\r\n");		 
			memcpy(ConfigMsg.sub,DHCP_GET.sub, 4);
			memcpy(ConfigMsg.gw,DHCP_GET.gw, 4);
			memcpy(ConfigMsg.dns,DHCP_GET.dns,4);
		}
		else
		{
			printf(" DHCP子程序未运行,或者不成功\r\n");
			printf(" 使用定义的IP信息配置W5500\r\n");
		}
	}
		
	/*以下配置信息，根据需要选用*/	
	ConfigMsg.sw_ver[0]=FW_VER_HIGH;
	ConfigMsg.sw_ver[1]=FW_VER_LOW;	

	/*将IP配置信息写入W5500相应寄存器*/	
	setSUBR(ConfigMsg.sub);
	setGAR(ConfigMsg.gw);
	setSIPR(ConfigMsg.lip);
	
	getSIPR (local_ip);			
	printf(" W5500 IP地址   : %d.%d.%d.%d\r\n", local_ip[0],local_ip[1],local_ip[2],local_ip[3]);
	getSUBR(subnet);
	printf(" W5500 子网掩码 : %d.%d.%d.%d\r\n", subnet[0],subnet[1],subnet[2],subnet[3]);
	getGAR(gateway);
	printf(" W5500 网关     : %d.%d.%d.%d\r\n", gateway[0],gateway[1],gateway[2],gateway[3]);
}

void set_w5500_mac(void)
{
	if(at24c16_read(EE_ADDR_W5500_CONFIG_MAC) == EE_ADDR_W5500_PC_CONFIG_SUCCESS)
	{
		printf("Start read pc w5500 config ! \r\n");
		mac[0] = at24c16_read(EE_ADDR_MAC_ONE);
		mac[1] = at24c16_read(EE_ADDR_MAC_TWO);
		mac[2] = at24c16_read(EE_ADDR_MAC_THREE);
		mac[3] = at24c16_read(EE_ADDR_MAC_FOUR);
		mac[4] = at24c16_read(EE_ADDR_MAC_FIVE);
		mac[5] = at24c16_read(EE_ADDR_MAC_SIX);
		printf("Read pc w5500 config success ! \r\n");
	}
	else
	{
		printf(" First start ram write at24c16 ! \r\n");
		mac[2] = (CpuID[2]>>24)&0xff;
		mac[3] = (CpuID[2]>>16)&0xff;
		mac[4] = (CpuID[2]>>8)&0xff;
		mac[5] =  CpuID[2]&0xff;
		printf("CpuID[2] = %d,mac[2] = %d,mac[3] = %d,mac[4] = %d,mac[5] = %d.\r\n",CpuID[2],mac[2],mac[3],mac[4],mac[5]);
		at24c16_write(EE_ADDR_MAC_ONE, mac[0]);
		at24c16_write(EE_ADDR_MAC_TWO, mac[1]);
		at24c16_write(EE_ADDR_MAC_THREE, mac[2]);
		at24c16_write(EE_ADDR_MAC_FOUR, mac[3]);	//mcu id number  
		at24c16_write(EE_ADDR_MAC_FIVE, mac[4]);	//mcu id number 
		at24c16_write(EE_ADDR_MAC_SIX, mac[5]);		//mcu id number 
		printf(" First start ram write at24c16 ! \r\n");
	}
	memcpy(ConfigMsg.mac, mac, 6);
	setSHAR(ConfigMsg.mac);	/**/
	memcpy(DHCP_GET.mac, mac, 6);
}

void dhcp_config_w5500(void)
{
		printf(" First start ram write at24c16 ! \r\n");
		at24c16_write(EE_ADDR_SUBNET_ONE, subnet[0]);
		at24c16_write(EE_ADDR_SUBNET_TWO, subnet[1]);
		at24c16_write(EE_ADDR_SUBNET_THREE, subnet[2]);
		at24c16_write(EE_ADDR_SUBNET_FOUR, subnet[3]);	
		at24c16_write(EE_ADDR_GATEWAY_ONE, gateway[0]);
		at24c16_write(EE_ADDR_GATEWAY_TWO, gateway[1]);
		at24c16_write(EE_ADDR_GATEWAY_THREE, gateway[2]);
		at24c16_write(EE_ADDR_GATEWAY_FOUR, gateway[3]);
		at24c16_write(EE_ADDR_DNS_ONE, dns_server[0]);
		at24c16_write(EE_ADDR_DNS_TWO, dns_server[1]);
		at24c16_write(EE_ADDR_DNS_THREE, dns_server[2]);
		at24c16_write(EE_ADDR_DNS_FOUR, dns_server[3]);
		printf(" Ram writed w55000 config at24c16 success ! \r\n");
}




