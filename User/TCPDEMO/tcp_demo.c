/**
******************************************************************************
* @file    			tcp_demo.c                                               
* @version 			V1.2										  		
* @date    			2017-11-01								 
* @brief 			  TCP 演示函数     					        
*
* @company  		深圳炜世科技有限公司
* @information  WIZnet W5500官方代理商，全程技术支持，价格绝对优势！
* @website  		www.wisioe.com																							 	 				
* @forum        www.w5500.cn																								 		
* @qqGroup      383035001																										 
******************************************************************************
*/

#include <stdio.h>
#include <string.h>
#include "tcp_demo.h"
#include "W5500_config.h"
#include "w5500.h"
#include "socket.h"
#include "BD_TG.h"
#include "MqttKit.h"
#include "delay.h"
#include "LOOP.h"
#include "24c16.h"
#include "input.h"
uint8 buff[2048];				                             /*定义一个2KB的缓存*/
uint8  BD_TG_server_ip[4] = {172,16,40,176};//{202,106,5,12};        //BD_TG服务器IP地址
uint16 BD_TG_server_port  = 1883;                   //BD_TG服务器端口号


int MQTT_STATE = MQTT_PKT_CONNECT;   //连接
const char *topics[] = {"sensor-data","/alarm"}; //原来的主题名称stateUp
const char *topics1[] = {"subscibe"}; 
uint8 BD_TG_ping_pak[2] = {0xC0,0x00};

unsigned char HexToChar(unsigned char bChar);
unsigned char 	GetASCII(unsigned char  x);
uint16 hex1adecimal_to_decimal(u32 hex);
unsigned char *data_ptr = NULL;
extern unsigned char flag;
extern unsigned char TxBuffer2[200]; 
extern unsigned char RxBuffer2[200];
extern unsigned char TxCounter2;
extern unsigned char RxCounter2;
extern uint8_t w5500_connect_success;
extern uint8_t W5500_REBOOT;
extern uint8_t input5_printf;
extern uint8_t input6_printf;
extern int floor;
extern u32 id_name_all;
u32 door_close_times_mqtt = 0;
extern uint8_t door_open_over_time_sum;
extern uint8_t door_close_times;
extern uint8_t door_close_times_sum;
extern uint8_t door_close_times_sum1;
extern uint8_t door_close_times_sum2;
extern uint8_t door_not_close;
unsigned char ASII_hex=0;
extern uint8_t door_open_times;
char test[300];
extern uint8_t   pingceng;
extern uint8_t   power;
uint8_t tpye = 1;
/**
*@brief		TCP Client回环演示函数。
*@param		无
*@return	无
*/
void setKPALVTR(SOCKET i,uint8 val)
{
  IINCHIP_WRITE(Sn_KPALVTR(i),val);
}	
void do_tcp_client(void)
{	
	uint16 len=0;	
	switch(getSn_SR(SOCK_TCPC))								  				         /*获取socket的状态*/
	{
		case SOCK_CLOSED:											        		         /*socket处于关闭状态*/
			network_status = 1;
			setKPALVTR(SOCK_TCPC,1);//心跳保证，低端连接
			socket(SOCK_TCPC,Sn_MR_TCP,local_port++,Sn_MR_ND);
		break;

		case SOCK_INIT:													        	         /*socket处于初始化状态*/
			network_status = 1;
			connect(SOCK_TCPC,BD_TG_server_ip,BD_TG_server_port);                /*socket连接服务器*/ 
		break;

		case SOCK_ESTABLISHED: /*socket处于连接建立状态*/		
			w5500_connect_success = 1;			
			if(getSn_IR(SOCK_TCPC) & Sn_IR_CON)
			{
				setSn_IR(SOCK_TCPC, Sn_IR_CON); 							         /*清除接收中断标志位*/
			}

			len=getSn_RX_RSR(SOCK_TCPC); 								  	         /*定义len为已接收数据的长度*/
			if(len>0)
			{
				recv(SOCK_TCPC,buff,len); 							   		         /*接收来自Server的数据*/			
				data_ptr = buff;
				if(data_ptr != NULL)
				BD_TG_RevPro(data_ptr);
			}
			switch(MQTT_STATE)
			{
				/*MQTT协议连接BD_TG代理平台*/
				case MQTT_PKT_CONNECT:
					BD_TG_DevLink();
					MQTT_STATE = MQTT_PKT_SUBSCRIBE;
			   	
				break;
				/*订阅主题*/
				case MQTT_PKT_SUBSCRIBE:
					BD_TG_Subscribe(topics1,1);
					MQTT_STATE = MQTT_PKT_PINGREQ;
				break;	
				/*Qos2级别发布消息*/
				case MQTT_PKT_PUBLISH:
					printf("进入发布消息\r\n");
				  memset(test, 0, sizeof(test));
				  
#ifdef MQTT_STRING
				if(W5500_REBOOT == 1)
				{
						sprintf(test,"REBOOT is success");
						W5500_REBOOT = 0;
				}
				else if(door_open_times == 1)
				{
					sprintf(test,"The door opens over time : %d",++door_open_over_time_sum);
						door_open_times = 0;
				}
				else
				{
					door_close_times_mqtt = door_close_times+(door_close_times_sum*255)+(door_close_times_sum1*255*255)+(door_close_times_sum2*255*255*255);
				 sprintf(test,"{\"id\":%d,\"type\":%d,\"s1\":%d,\"s2\":%d,\"s3\":%d,\"s4\":%d,\"s5\":%d,\"s6\":%d,\"floor\":%d,\"t\":%d,\"h\":%d,\"vx\":%d,\"vy\":%d,\"vz\":%d,\"acdt\":%d}",id_name_all,tpye,door_not_close,input4_value,pingceng,input5_printf,input6_printf,power,floor,(int)temperature,(int)humidity,(int)AccelerationX,(int)AccelerationY,(int)AccelerationZ,door_close_times_mqtt);
				}
#else
				  for(int i=0; i<200;i++)
					{
						test[i] = i+0x30;
					}
#endif
					BD_TG_Publish(*topics,test,strlen(test)); //发布消息
                
					delay_ms(300);                                 //等待平台响应    
					/*接收平台发送的PubRec并回复PubRel响应*/		 
					len=getSn_RX_RSR(SOCK_TCPC); 
					recv(SOCK_TCPC,buff,len); 
					data_ptr = buff;
					if(data_ptr != NULL)
						BD_TG_RevPro(data_ptr);
					delay_ms(100);                                 //PubRel响应等待平台响应
					len=getSn_RX_RSR(SOCK_TCPC); 
					recv(SOCK_TCPC,buff,len); 
					data_ptr = buff;
					if(data_ptr != NULL)
						BD_TG_RevPro(data_ptr);
			    MQTT_STATE = MQTT_PKT_PINGREQ;
				/*120秒发送一次Ping响应保持长连接*/
				break;
				case MQTT_PKT_UNSUBSCRIBE:
					MQTT_UnSubscribe(topics,1);
					MQTT_STATE = MQTT_PKT_PINGREQ;
				break;
				case MQTT_PKT_PINGREQ:
				//	network_status = 0;
//					if(BD_TG_ping_time > 50)
//					{
//						send(SOCK_TCPC,BD_TG_ping_pak,2);
//						BD_TG_ping_time = 0;
//					}
				break;
				case MQTT_PKT_DISCONNECT:
						network_status = 1;
					break;
			}	
			
					
    break;

	case SOCK_CLOSE_WAIT: 											    	         /*socket处于等待关闭状态*/
		close(SOCK_TCPC);
		break;
	}
}

unsigned char HexToChar(unsigned char bChar)
{
if((bChar>=0x30)&&(bChar<=0x39))
{
bChar -= 0x30;
}
else if((bChar>=0x41)&&(bChar<=0x46)) // Capital
{
bChar -= 0x37;
}
else if((bChar>=0x61)&&(bChar<=0x66)) //littlecase
{
bChar -= 0x57;
}
else
{
bChar = 0xff;
}
return bChar;
}

unsigned char 	GetASCII(unsigned char  x)
{
	if(x>0x0f)
	{
		return 0;  
	}
	
	if(x>9)
	{
		return ('A'+x-10);
	}
	return ('0'+x);  
}	
uint16 hex1adecimal_to_decimal(u32 hex)
{
 uint16 ret; //
 unsigned char ret8,ret7,ret6,ret5,ret4,ret3,ret2,ret1; //局部变量
 ret8=hex/0x10000000; //取高位
 hex=hex<<4; //ret
 ret7=hex/0x10000000; //
 hex=hex<<4; 
 ret6=hex/0x10000000; //
 hex=hex<<4; 
 ret5=hex/0x10000000; // 
 hex=hex<<4;
 ret4=hex/0x10000000; //取高位
 hex=hex<<4; //ret
 ret3=hex/0x10000000; //
 hex=hex<<4; 
 ret2=hex/0x10000000; //
 hex=hex<<4; 
 ret1=hex/0x10000000; // 
 ret=ret1+ret2*10+ret3*100+ret4*1000+ret5*10000+ret6*100000+ret7*1000000+ret8*10000000;//十进制
 return(ret);
}

void set_object_ip(void)
{
		printf("目的IP是否被设置 ?\r\n");
		if(at24c16_read(EE_ADDR_W5500_CONFIG_IP) == EE_ADDR_W5500_PC_CONFIG_SUCCESS)
		{
				printf("目的IP被设置，开始读取 !\r\n");
				BD_TG_server_ip[0] = at24c16_read(EE_ADDR_IP_ONE);
				BD_TG_server_ip[1] = at24c16_read(EE_ADDR_IP_TWO);
				BD_TG_server_ip[2] = at24c16_read(EE_ADDR_IP_THREE);
			  BD_TG_server_ip[3] = at24c16_read(EE_ADDR_IP_FOUR);
			  printf("目的IP:%d.%d.%d.%d.\r\n",BD_TG_server_ip[0],BD_TG_server_ip[1],BD_TG_server_ip[2],BD_TG_server_ip[3]);
		}
		else
		{
			printf("目的IP未被设置 !\r\n");
			at24c16_write(EE_ADDR_IP_ONE, BD_TG_server_ip[0]);
			at24c16_write(EE_ADDR_IP_TWO, BD_TG_server_ip[1]);
			at24c16_write(EE_ADDR_IP_THREE, BD_TG_server_ip[2]);
			at24c16_write(EE_ADDR_IP_FOUR, BD_TG_server_ip[3]);	
		}
}
