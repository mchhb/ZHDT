#include "stm32f10x.h"
#include "usart2.h"
#include "LOOP.h"
#include "delay.h"
#include "Tim2.h"
#include "spi2.h"
#include "w5500_config.h"
#include "gpio.h"
#include "dht11.h"
#include "w5500.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "iwdg.h"
#include "dhcp.h"
#include "usart1.h"
#include "usart3.h"
#include "ADXL345.h"
#include "24c16.h"
#include "adc.h"
#include "exti.h"
#include "MqttKit.h"
#include "tcp_demo.h"
#include "socket.h"
#include "BD_TG.h"
//#include "MQTTConnectClient.h"
extern __IO uint16_t ADC_ConvertedValue;// ADC1转换的电压值通过MDA方式传到SRAM
extern __IO uint16_t ADC_Result[1];
extern uint16 local_port;
extern uint8 BD_TG_ping_pak[2];
unsigned char RxBuffer2[200];
extern uint8 buff[2048];	
extern unsigned char w5500_buf[128];   //之前是128
extern volatile unsigned int  tim3fquence;	//定时中断次数
extern int tim3i;
unsigned char result[20] = {0x66,0x04,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
														0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x99};
//extern char *DEVID;
void GetLockCode(void);
void USART_SendDatacmd(USART_TypeDef* USARTx,unsigned char ch);
void Power_check(void);
char *rand_str(char *str);
//------------------------------
u32 Lock_Code;
u32 CpuID[3];
//------------------------------
unsigned char  flag;
uint8_t   ee_test = 0;
uint8_t 	read_dht11 = 0;
uint8_t 	send_data = 0;
uint8 Disconnect_Request[2] = {0xe0,0x00};
uint8_t 	domain_ip[4]={198,41,30,241};
uint32_t 	timePUB = 0;
uint32_t 	timeSUB = 0;
uint8_t 	timePING = 0;
uint8_t 	temp_rh[2];
uint8_t   uart2_send_read = 0;
char *mimi;
uint8_t   uart2_send_write = 0;
uint16_t 	noise;
uint8_t publish_crc = 0;
uint8_t 	noise_status = 0;
uint8_t   uart_config = 0;
uint32_t 	w5500_connect = 0;
uint8_t   uart_w5500_ip_config = 0;
uint8_t   uart_w5500_mac_config = 0;
uint8_t   uart_w5500_subnet_config = 0;
uint8_t   uart_w5500_gateway_config = 0;
uint8_t   uart_w5500_dns_config = 0;
uint8_t   uart_w5500_port_config = 0;
float 	AccelerationX;
float 	AccelerationY;
float 	AccelerationZ;
float 	adxl_val[3];
//------------------------
uint8_t mqtt_send_time = 0;
u32 id_name_all;
unsigned char id_name[4];

uint8_t work_ACCadd = 0;
uint8_t work_dht11_status;
uint8_t status_code1 = 0;
uint8_t status_code2 = 0;
uint8_t status_code3 = 0;
uint8_t work_noise_status;
uint8_t work_uart_status = 0;
uint8_t work_Acceleration_status;
uint16 disconnet=0;
uint16 overtime=0;
uint8_t overtime_reset = 0;
uint8_t  test_while = 0;
uint8_t  work_status;
uint8_t  network_status = 1;
uint8_t  led_set_work;
uint8_t  led_set_network;
#define WORK_NORMAL				0
#define WORK_ERROR				1
#define WORK_USART				2
#define NETWORK_SUCCESS		0
#define NETWORK_FAIL			1
#define NETWORK_USART			2
//------------------------
MQTT_PACKET_STRUCTURE mqttPacket;		
unsigned char sum=0;
float temperature,humidity;
uint8_t external_temp1,external_temp3;
uint8_t external_humi1,external_humi3;
uint8_t external_vibx1,external_vibx3;
uint8_t external_viby1,external_viby3;
uint8_t external_vibz1,external_vibz3;
uint8_t external_noise1,external_noise3;
uint8_t external_driver = 0;
uint8_t external_led1 = 0;
uint8_t external_led2 = 0;
uint8_t external_led3 = 0;
uint8_t external_led4 = 0;
uint8_t external_led_open = 0;
int rc=0;	
int test_time = 0;
int test_time1 = 0;
uint8_t power_status;
uint8_t battery_status = 9;
uint8_t power_time = 0;
uint8_t battery_time = 0;
uint8 txBuffer[40];
uint8_t w5500_connect_success = 0;
uint8_t domain_name[]="www.embed-net.com";
uint8_t W5500_REBOOT = 0;
void read_config(void);


void Initialization_configuration(void) {

	delay_ms(1000);
	
	IWDG_Init(4,3750);//((prer)*rlr)/40; 
	
	TIM2_Configuration();

	TIM2_NVIC_Configuration();
	
	TIM3_Configuration();

	TIM3_NVIC_Configuration();
	
	GPIO_Configuration();

	WIZ_SPI_Init();
	
	at24c16_init();

	ADXL345_init();
	
	ADC1_Init();
	
	EXTI_ENABLE();
	
	USART2_Config();		

	USART1_Config();	
	
	USART3_Config();	
	
	USART1_NVIC_Configuration();
	
	USART3_NVIC_Configuration();
	
	USART2_NVIC_Configuration();
	
	Power_check();

	GetLockCode();
	
	Reset_W5500();

  w5500_check();	

	set_w5500_mac();
	
	set_object_ip();
	
	read_config();

	sysinit(txsize, rxsize); 													// 初始化8个socket

  printf("W5500 Init Complete!\r\n");
	
	START_TIME2;
}
void application(void)
{
	IWDG_Feed();
	w5500_check();
	auto_dhcp();
	delay_ms(100);
	if(dhcp_ok==1)
	{	
			do_tcp_client();                    /*TCP_Client 数据回环测试程序*/
			delay_ms(100);
	}
	if(disconnet==1)
	{
		disconnet=0;
		w5500_connect_success = 0;
		//send(SOCK_TCPC,Restore_factory,10);//发送心跳
		Reset_W5500();												/*硬复位W5500*/
		w5500_check();													/*检查网线是否接入*/   
		set_w5500_mac();											/*配置MAC地址*/
		set_w5500_ip();
		dhcp_config_w5500();
		sysinit(txsize, rxsize); 													// 初始化8个socket
		MQTT_STATE =MQTT_PKT_CONNECT;			
	}
	if(t2_count_ms[T2_COUNTER_MS_2500] >= T2_TIMEOUT_MS_2500)  //2.5s
	{
		t2_count_ms[T2_COUNTER_MS_2500] = 0;	 
		if(external_driver == 0)
		{
			for(int k = 0; k < 20; k++)
			{
				USART_SendDatacmd(USART1,result[k]);
			}
			external_led1 = 1;
			external_driver = 1;
			if(rxd_buffer_locked1 == 1)
			{
				rxd_buffer_locked1 = 0;
				external_led2 = 1; 
				external_temp1 = usart_buf1[14];
				external_humi1 = usart_buf1[15];
				external_vibx1 = usart_buf1[16];
				external_noise1 = usart_buf1[17];
			}
			for(int k = 0; k < 20; k++)
			{
				USART_SendDatacmd(USART1,0x11);
				delay_us(100);
			}
		}
		else
		{
			for(int m = 0; m < 20; m++)
			{
				USART_SendDatacmd(USART3,result[m]);
			}
			external_driver = 0;
			external_led3 = 1;
			if(rxd_buffer_locked3 == 1)
			{
				rxd_buffer_locked3 = 0;
				external_led4 = 1;
				external_temp3 = usart_buf3[14];
				external_humi3 = usart_buf3[15];
				external_vibx3 = usart_buf3[16];
				external_noise3 = usart_buf3[17];
			for(int k = 0; k < 20; k++)
			{
				USART_SendDatacmd(USART1,0x11);
			}
			}
		}
	}
	if(t2_count_ms[T2_COUNTER_MS_10000] >= T2_TIMEOUT_MS_10000)  //10s
	{
		t2_count_ms[T2_COUNTER_MS_10000] = 0;	 
		test_time++;
		send(SOCK_TCPC,BD_TG_ping_pak,2);//发送心跳      						
		delay_ms(100);
		if(buff[0]==0xd0)
		{		 	 	
			MQTT_STATE = MQTT_PKT_PINGREQ;
			overtime = 0;
			overtime_reset = 0;
			memset(buff,0,sizeof(buff));
			flag =1;	
			printf("收到心跳\r\n");
			network_status = 0;
		}	
		else 
		{
			overtime++;
			overtime_reset++;
			MQTT_STATE = MQTT_PKT_DISCONNECT;
			printf("未收到心跳次数： %d\r\n",overtime);
			if(overtime>2)
			{
				printf("心跳超时，重新连接！\r\n");
				overtime=0;
				disconnet=1;
				send(SOCK_TCPC,Disconnect_Request,2);//发送断网
				flag=0;					
				memset(buff,0,sizeof(buff));
				memset(w5500_buf,0,sizeof(w5500_buf));
			}
		}			
	}	
//	if(t2_count_ms[T2_COUNTER_MS_180000] >= T2_TIMEOUT_MS_180000)  //180s
//	{
//		t2_count_ms[T2_COUNTER_MS_180000] = 0;
	if(t2_count_ms[20] >= (mqtt_send_time*10000))  //30s
	{
		t2_count_ms[20] = 0;
		test_time1++;
		if((flag == 1)&&(overtime<1))//发布
		{	   
				printf("发布消息\r\n");
				MQTT_STATE = MQTT_PKT_PUBLISH;		
		}
	}
	if(t2_count_ms[T2_COUNTER_MS_0] >= T2_TIMEOUT_MS_0)  //100us
	{
		t2_count_ms[T2_COUNTER_MS_0] = 0;
		if(work_uart_status == 1)
		{  
			work_status = 2;
			network_status = 2;
		}
		else if((work_dht11_status == 1) || (work_noise_status == 1) || (work_Acceleration_status == 1))
		{
			work_status = 1;
		}
		else
		{
			work_status = 0;
		}
	}		
	if(t2_count_ms[T2_COUNTER_MS_400] >= T2_TIMEOUT_MS_400)  //400s
	{
		t2_count_ms[T2_COUNTER_MS_400] = 0;
		test_while++;
		switch(work_status)
		{
			case WORK_NORMAL:
				if(led_set_work == 0) 		{ WORK_LED_ERROR_OFF;	WORK_LED_NORMAL_ON; 	led_set_work = 1; }
				else 											{ WORK_LED_ERROR_OFF;	WORK_LED_NORMAL_OFF;	led_set_work = 0; }
			break;
			case WORK_ERROR:
				if(led_set_work == 1) 		{ WORK_LED_NORMAL_OFF; WORK_LED_ERROR_ON; 	led_set_work = 0; }
				else 											{ WORK_LED_NORMAL_OFF; WORK_LED_ERROR_OFF;	led_set_work = 1; }
		  break;
			case WORK_USART:
					WORK_LED_ERROR_OFF;	WORK_LED_NORMAL_ON;
					NETWORK_LED_FAIL_OFF; NETWORK_LED_SUCCESS_ON; 
			break;
		}
	}
	if(t2_count_ms[T2_COUNTER_MS_2000] >= T2_TIMEOUT_MS_2000)  //2s
	{
		t2_count_ms[T2_COUNTER_MS_2000] = 0;
		external_led_open++;

		DHT11_GetValue(temp_rh);
		temperature = temp_rh[1];
		humidity = temp_rh[0];
		if((temperature > 50) || (humidity > 100) ||((temperature == 0)&&(humidity == 0)))
		{
			printf("The DHT11 is demaged !\r\n");
			work_dht11_status = 1;
		}
		else
		{
			//printf("temperature  is %f\r\n",temperature);
			//printf("humidity     is %f\r\n",humidity);	
			work_dht11_status = 0;
		}		
		noise = ADC_Result[0];
		if(noise == 0)
		{
			noise_status++;
			if(noise_status > 5)
			{
				printf("The noise	is bad !\r\n");
				work_noise_status = 1;
			}
		}
		else if(noise > 1000)
		{
			//printf(" It's too noisy \r\n");
		//	printf("noise		     is %d\r\n",noise);
			work_noise_status = 1;
		}
		else
		{
			//printf("noise		     is %d\r\n",noise);
			work_noise_status = 0;
			noise_status = 0;
		}
		if((work_dht11_status == 1)||(work_noise_status == 1)||(work_Acceleration_status == 1))
		{
			if(work_dht11_status == 1)
			{
				status_code1 = 1;
			}
			if(work_noise_status == 1)
			{
				status_code1 = 2;
			}
			if(work_Acceleration_status == 1)
			{
				status_code1 = 3;
			}
			if((work_dht11_status == 1)&&(work_noise_status == 1))
			{
				status_code1 = 4;
			}
			if((work_dht11_status == 1)&&(work_Acceleration_status == 1))
			{
				status_code1 = 5;
			}
			if((work_noise_status == 1)&&(work_Acceleration_status == 1))
			{
				status_code1 = 6;
			}
			if((work_noise_status == 1)&&(work_Acceleration_status == 1)&&(work_dht11_status == 1))
			{
				status_code1 = 7;
			}
		}
		else
		{
			status_code1 = 0;
		}
	}
	if(t2_count_ms[T2_COUNTER_MS_200] >= T2_TIMEOUT_MS_200)  //0.2s
	{
		t2_count_ms[T2_COUNTER_MS_200] = 0;
		switch(network_status)
		{
			case NETWORK_SUCCESS:
				if(led_set_network == 0) 	{ NETWORK_LED_FAIL_OFF;	NETWORK_LED_SUCCESS_ON; led_set_network = 1; }
				else 											{ NETWORK_LED_FAIL_OFF;	NETWORK_LED_SUCCESS_OFF;	led_set_network = 0; }
			break;
			case NETWORK_FAIL:
				if(led_set_network == 1) 	{ NETWORK_LED_SUCCESS_OFF; NETWORK_LED_FAIL_ON;  led_set_network = 0; }
				else 											{ NETWORK_LED_SUCCESS_OFF; NETWORK_LED_FAIL_OFF; led_set_network = 1; }
			break;
			case NETWORK_USART:
				WORK_LED_ERROR_OFF;	WORK_LED_NORMAL_ON;
			  NETWORK_LED_FAIL_OFF; NETWORK_LED_SUCCESS_ON; 
			  STOP_TIME2;
			break;	
		}
	}
	if((power_status == 2)&&(power_time == 0))
	{
		if(t2_count_ms[T2_COUNTER_MS_300] >= T2_TIMEOUT_MS_300)  //300ms
		{
			t2_count_ms[T2_COUNTER_MS_300] = 0;
			if(power_status == 2)
			{
				POWER_LED_SUCCESS_ON;
				POWER_LED_FAIL_OFF;
				power_time++;
				printf("acc is closed !\r\n");
			}
		}
	}
	if((power_status == 0)&&(power_time != 0))
	{
		if(t2_count_ms[T2_COUNTER_MS_300] >= T2_TIMEOUT_MS_300)  //300ms
		{
			t2_count_ms[T2_COUNTER_MS_300] = 0;
			if(power_status == 0)
			{
				POWER_LED_SUCCESS_OFF;
				POWER_LED_FAIL_ON;
				power_time = 0;
				printf("acc is opened !\r\n");
			}
		}
	}
	if((battery_status == 1)&&(battery_time != 0))
	{
		if(t2_count_ms[T2_COUNTER_MS_250] >= T2_TIMEOUT_MS_250)  //300ms
		{
			t2_count_ms[T2_COUNTER_MS_250] = 0;
			if(battery_status == 1)
			{
				battery_time = 0;
				printf("battery is high !\r\n");
			}
		}
	}
	if((battery_status == 0)&&(battery_time == 0))
	{
		if(t2_count_ms[T2_COUNTER_MS_250] >= T2_TIMEOUT_MS_250)  //300ms
		{
			t2_count_ms[T2_COUNTER_MS_250] = 0;
			if(battery_status == 0)
			{
				battery_time++;
				printf("battery is low !\r\n");
			}
		}
	}
	if(t2_count_ms[T2_COUNTER_MS_500] >= T2_TIMEOUT_MS_500)  //0.5s
	{
		t2_count_ms[T2_COUNTER_MS_500] = 0;
		ADXL345_ReadXYZ(adxl_val);
		AccelerationX = adxl_val[0];
		AccelerationY = adxl_val[1];
		AccelerationZ = adxl_val[2];
		printf("x = %f,y = %f,z = %f\r\n",AccelerationX,AccelerationY,AccelerationZ);
		if((AccelerationX == 0)&&(AccelerationY == 0)&&(AccelerationZ == 0))
		{
			work_ACCadd++;
			if(work_ACCadd > 5)
			{
				work_Acceleration_status = 1;
				printf(" Acceleration is demaged ! \r\n");
			}
		}
		else
		{
			work_ACCadd = 0;
			work_Acceleration_status = 0;
		}
	}	
	if(overtime_reset > 5)
	{
		SoftReset();		
	}
	if((publish_buf[1] == 0x88)&&(publish_buf[3] == 0x99))
	{
		//publish_crc = (publish_buf[1]+publish_buf[3]+publish_buf[4]+publish_buf[5]+publish_buf[6]+publish_buf[7]+publish_buf[8]+publish_buf[9]);
		//if(publish_buf[10] == publish_crc)
		//{
			if((publish_buf[4] == id_name[0])&&(publish_buf[5] == id_name[1])&&(publish_buf[6] == id_name[2])&&(publish_buf[7] == id_name[3]))
			{
					if(publish_buf[8] == 0x01)
					{
							printf("订阅收到重启指令!\r\n");
							at24c16_write(EE_ADDR_REBOOT, 0x01);
							delay_ms(1000);
							SoftReset();
					}
					else if(publish_buf[8] == 0x02)
					{
						  mqtt_send_time = publish_buf[9];
							printf("订阅收到发布间隔指令!间隔变更为：%d ！\r\n",mqtt_send_time);
						  at24c16_write(EE_ADDR_MQTT_SEND_TIME, mqtt_send_time);
					}
					memset(publish_buf, 0, sizeof(publish_buf));
			}
		//}
	}
	else
	{
		memset(publish_buf, 0, sizeof(publish_buf));
	}
	if((external_led_open%2 == 0) &&((external_led1 == 1)))
	{
			UART1_LED_SUCCESS_ON;
			UART1_LED_FAIL_OFF;
			external_led1 = 0;
	}
	else if((external_led_open%2 == 0) &&((external_led2 == 1)))
	{
			UART1_LED_SUCCESS_OFF;
			UART1_LED_FAIL_ON;
			external_led2 = 0;
	}
	else
	{
			UART1_LED_SUCCESS_OFF;
			UART1_LED_FAIL_OFF;
	}
	if((external_led_open%2 == 0) &&((external_led3 == 1)))
	{
			UART3_LED_SUCCESS_ON;
			UART3_LED_FAIL_OFF;
			external_led3 = 0;
	}
	else if((external_led_open%2 == 0) &&((external_led4 == 1)))
	{
			UART3_LED_SUCCESS_OFF;
			UART3_LED_FAIL_ON;
			external_led4 = 0;
	}
	else
	{
			UART3_LED_SUCCESS_OFF;
			UART3_LED_FAIL_OFF;
	}
	if(rxd_buffer_locked == 1)
	{
		tim3i = 0;
		tim3fquence=0;
		rxd_buffer_locked = 0;
		if(usart_buf[1] == 0x01)
		{
			if(usart_buf[3] == 0x01)	//目的IP
			{
				uart2_send_read = 1;
				txBuffer[0] = 0x24;
				txBuffer[1] = 0x01;		
				txBuffer[2] = 0x06;	
				txBuffer[3] = at24c16_read(EE_ADDR_IP_ONE);	
				txBuffer[4] = at24c16_read(EE_ADDR_IP_TWO);	
				txBuffer[5] = at24c16_read(EE_ADDR_IP_THREE);	
				txBuffer[6] = at24c16_read(EE_ADDR_IP_FOUR);	
				txBuffer[7] = 0x00;		
				txBuffer[8] = 0x00;			
			}
			if(usart_buf[3] == 0x02)	//分配IP
			{
				uart2_send_read = 1;
				txBuffer[0] = 0x24;
				txBuffer[1] = 0x02;
				txBuffer[2] = 0x06;	
				txBuffer[3] = at24c16_read(EE_ADDR_DHCP_IP_ONE);	
				txBuffer[4] = at24c16_read(EE_ADDR_DHCP_IP_TWO);	
				txBuffer[5] = at24c16_read(EE_ADDR_DHCP_IP_THREE);	
				txBuffer[6] = at24c16_read(EE_ADDR_DHCO_IP_FOUR);	
				txBuffer[7] = 0x00;		
				txBuffer[8] = 0x00;	
			}
			if(usart_buf[3] == 0x03)	//MAC
			{
				uart2_send_read = 1;
				txBuffer[0] = 0x24;
				txBuffer[1] = 0x03;
				txBuffer[2] = 0x06;	
				txBuffer[3] = at24c16_read(EE_ADDR_MAC_ONE);	
				txBuffer[4] = at24c16_read(EE_ADDR_MAC_TWO);	
				txBuffer[5] = at24c16_read(EE_ADDR_MAC_THREE);	
				txBuffer[6] = at24c16_read(EE_ADDR_MAC_FOUR);	
				txBuffer[7] = at24c16_read(EE_ADDR_MAC_FIVE);	
				txBuffer[8] = at24c16_read(EE_ADDR_MAC_SIX);	
				
			}
			if(usart_buf[3] == 0x04)	//子网掩码
			{
				uart2_send_read = 1;
				txBuffer[0] = 0x24;
				txBuffer[1] = 0x04;
				txBuffer[2] = 0x06;	
				txBuffer[3] = at24c16_read(EE_ADDR_SUBNET_ONE);	
				txBuffer[4] = at24c16_read(EE_ADDR_SUBNET_TWO);	
				txBuffer[5] = at24c16_read(EE_ADDR_SUBNET_THREE);	
				txBuffer[6] = at24c16_read(EE_ADDR_SUBNET_FOUR);	
				txBuffer[7] = 0x00;	
				txBuffer[8] = 0x00;					
			}
			if(usart_buf[3] == 0x05)	//网关
			{
				uart2_send_read = 1;
				txBuffer[0] = 0x24;
				txBuffer[1] = 0x05;
				txBuffer[2] = 0x06;	
				txBuffer[3] = at24c16_read(EE_ADDR_ID_NAME1);	
				txBuffer[4] = at24c16_read(EE_ADDR_ID_NAME2);	
				txBuffer[5] = at24c16_read(EE_ADDR_ID_NAME3);	
				txBuffer[6] = at24c16_read(EE_ADDR_ID_NAME4);	
				txBuffer[7] = 0x00;
				txBuffer[8] = 0x00;				
			}
			if(usart_buf[3] == 0x06)	//DNS
			{
				uart2_send_read = 1;
				txBuffer[0] = 0x24;
				txBuffer[1] = 0x06;
				txBuffer[2] = 0x06;	
				txBuffer[3] = at24c16_read(EE_ADDR_DNS_ONE);	
				txBuffer[4] = at24c16_read(EE_ADDR_DNS_TWO);	
				txBuffer[5] = at24c16_read(EE_ADDR_DNS_THREE);	
				txBuffer[6] = at24c16_read(EE_ADDR_DNS_FOUR);	
				txBuffer[7] = 0x00;
				txBuffer[8] = 0x00;				
			}
			if(usart_buf[3] == 0x07)	//端口
			{
				uart2_send_read = 1;
				txBuffer[0] = 0x24;
				txBuffer[1] = 0x07;
				txBuffer[2] = 0x06;	
				txBuffer[3] = at24c16_read(EE_ADDR_PORT_1);	
				txBuffer[4] = at24c16_read(EE_ADDR_PORT_2);	
				txBuffer[5] = 0x00;	
				txBuffer[6] = 0x00;	
				txBuffer[7] = 0x00;	
				txBuffer[8] = 0x00;		
			}
			if(usart_buf[3] == 0x08)	//温度湿度加速度
			{
				ADXL345_ReadXYZ(adxl_val);
				AccelerationX = adxl_val[0];
				AccelerationY = adxl_val[1];
				AccelerationZ = adxl_val[2];
				DHT11_GetValue(temp_rh);
				temperature = temp_rh[1];
				humidity = temp_rh[0];
				uart2_send_read = 1;
				txBuffer[0] = 0x24;
				txBuffer[1] = 0x08;
				txBuffer[2] = 0x06;	
				txBuffer[3] = (int)temperature;	
				txBuffer[4] = (int)humidity;	
				txBuffer[5] = AccelerationX;	
				txBuffer[6] = AccelerationY;	
				txBuffer[7] = AccelerationZ;	
				txBuffer[8] = 0x00;		
			}
			if(usart_buf[3] == 0x09)	//擦除EEprom
			{
				uart2_send_read = 1;
				txBuffer[0] = 0x24;
				txBuffer[1] = 0x09;
				txBuffer[2] = 0x06;	
				txBuffer[3] = 0x00;	
				txBuffer[4] = 0x00;	
				txBuffer[5] = 0x00;	
				txBuffer[6] = 0x00;	
				txBuffer[7] = 0x00;	
				txBuffer[8] = 0x00;
				erase_eeprom(0x00,0x830);				
			}
			if(usart_buf[3] == 0x10)	//CPU复位
			{
				delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				delay_ms(1000);
				SoftReset();				
			}
//			if(usart_buf[3] == 0x11)	//置PC标志位
//			{
//				uart2_send_read = 1;
//				txBuffer[0] = 0x24;
//				txBuffer[1] = 0x11;
//				txBuffer[2] = 0x06;	
//				txBuffer[3] = 0;	
//				txBuffer[4] = 0x00;	
//				txBuffer[5] = 0x00;	
//				txBuffer[6] = 0x00;	
//				txBuffer[7] = 0x00;	
//				txBuffer[8] = 0x00;		
//				at24c16_write(EE_ADDR_W5500_CONFIG, EE_ADDR_W5500_PC_CONFIG_SUCCESS);
//			}
			if(usart_buf[3] == 0x12)	//一键读取
			{
				txBuffer[0] = 0x24;
				txBuffer[1] = 0x12;		
				txBuffer[2] = 0x20;	
				txBuffer[3] = at24c16_read(EE_ADDR_IP_ONE);	
				txBuffer[4] = at24c16_read(EE_ADDR_IP_TWO);	
				txBuffer[5] = at24c16_read(EE_ADDR_IP_THREE);	
				txBuffer[6] = at24c16_read(EE_ADDR_IP_FOUR);	
				txBuffer[7] = at24c16_read(EE_ADDR_DHCP_IP_ONE);	
				txBuffer[8] = at24c16_read(EE_ADDR_DHCP_IP_TWO);	
				txBuffer[9] = at24c16_read(EE_ADDR_DHCP_IP_THREE);	
				txBuffer[10] = at24c16_read(EE_ADDR_DHCO_IP_FOUR);	
				txBuffer[11] = at24c16_read(EE_ADDR_MAC_ONE);	
				txBuffer[12] = at24c16_read(EE_ADDR_MAC_TWO);	
				txBuffer[13] = at24c16_read(EE_ADDR_MAC_THREE);	
				txBuffer[14] = at24c16_read(EE_ADDR_MAC_FOUR);	
				txBuffer[15] = at24c16_read(EE_ADDR_MAC_FIVE);	
				txBuffer[16] = at24c16_read(EE_ADDR_MAC_SIX);
				txBuffer[17] = at24c16_read(EE_ADDR_SUBNET_ONE);	
				txBuffer[18] = at24c16_read(EE_ADDR_SUBNET_TWO);	
				txBuffer[19] = at24c16_read(EE_ADDR_SUBNET_THREE);	
				txBuffer[20] = at24c16_read(EE_ADDR_SUBNET_FOUR);				
				txBuffer[21] = at24c16_read(EE_ADDR_GATEWAY_ONE);	
				txBuffer[22] = at24c16_read(EE_ADDR_GATEWAY_TWO);	
				txBuffer[23] = at24c16_read(EE_ADDR_GATEWAY_THREE);	
				txBuffer[24] = at24c16_read(EE_ADDR_GATEWAY_FOUR);	
				txBuffer[25] = at24c16_read(EE_ADDR_DNS_ONE);	
				txBuffer[26] = at24c16_read(EE_ADDR_DNS_TWO);	
				txBuffer[27] = at24c16_read(EE_ADDR_DNS_THREE);	
				txBuffer[28] = at24c16_read(EE_ADDR_DNS_FOUR);	
				txBuffer[29] = at24c16_read(EE_ADDR_PORT_1);
				txBuffer[30] = at24c16_read(EE_ADDR_PORT_2);
				ADXL345_ReadXYZ(adxl_val);
				AccelerationX = adxl_val[0];
				AccelerationY = adxl_val[1];
				AccelerationZ = adxl_val[2];
				DHT11_GetValue(temp_rh);
				temperature = temp_rh[1];
				humidity = temp_rh[0];
				txBuffer[31] = (int)temperature;	
				txBuffer[32] = (int)humidity;	
				txBuffer[33] = AccelerationX;	
				txBuffer[34] = AccelerationY;	
				txBuffer[35] = AccelerationZ;	
				sum = 0;
				for(int k = 0; k < 36; k++)
				{
						sum+=txBuffer[k];	
				}
				txBuffer[36]=sum;
				for(int k=0; k < 37; k++)
				{
						USART_SendData(USART2,txBuffer[k]);
				}
			}
		}
		if(usart_buf[1] == 0x02)
		{
			if(usart_buf[3] == 0x01)	//目的IP
			{
				uart2_send_write = 1;
				at24c16_write(EE_ADDR_IP_ONE, usart_buf[4]);
				at24c16_write(EE_ADDR_IP_TWO, usart_buf[5]);
				at24c16_write(EE_ADDR_IP_THREE, usart_buf[6]);
				at24c16_write(EE_ADDR_IP_FOUR, usart_buf[7]);	
				at24c16_write(EE_ADDR_W5500_CONFIG_IP,EE_ADDR_W5500_PC_CONFIG_SUCCESS);	
				
			}
			if(usart_buf[3] == 0x02)	//分配IP
			{
				uart2_send_write = 1;
				at24c16_write(EE_ADDR_DHCP_IP_ONE, usart_buf[4]);
				at24c16_write(EE_ADDR_DHCP_IP_TWO, usart_buf[5]);
				at24c16_write(EE_ADDR_DHCP_IP_THREE, usart_buf[6]);
				at24c16_write(EE_ADDR_DHCO_IP_FOUR, usart_buf[7]);
				at24c16_write(EE_ADDR_W5500_CONFIG_DHCP_IP,EE_ADDR_W5500_PC_CONFIG_SUCCESS);
			}
			if(usart_buf[3] == 0x03)	//MAC
			{
				uart2_send_write = 1;
				at24c16_write(EE_ADDR_MAC_ONE, usart_buf[4]);
				at24c16_write(EE_ADDR_MAC_TWO, usart_buf[5]);
				at24c16_write(EE_ADDR_MAC_THREE, usart_buf[6]);
				at24c16_write(EE_ADDR_MAC_FOUR, usart_buf[7]);
				uart_w5500_mac_config = 1;
			}
			if(usart_buf[3] == 0x04)	//MAC
			{
				uart2_send_write = 1;
				at24c16_write(EE_ADDR_MAC_FIVE, usart_buf[4]);
				at24c16_write(EE_ADDR_MAC_SIX, usart_buf[5]);
				if(uart_w5500_mac_config == 1)
				{
					uart_w5500_mac_config = 0;
					at24c16_write(EE_ADDR_W5500_CONFIG_MAC,EE_ADDR_W5500_PC_CONFIG_SUCCESS);
				}
			}
//			if(usart_buf[3] == 0x05)	//子网掩码
//			{
//				uart2_send_write = 1;
//				at24c16_write(EE_ADDR_SUBNET_ONE, usart_buf[4]);
//				at24c16_write(EE_ADDR_SUBNET_TWO, usart_buf[5]);
//				at24c16_write(EE_ADDR_SUBNET_THREE, usart_buf[6]);
//				at24c16_write(EE_ADDR_SUBNET_FOUR, usart_buf[7]);	
//				uart_w5500_subnet_config = 1;
//			}
//			if(usart_buf[3] == 0x06)	//网关
//			{
//				uart2_send_write = 1;
//			  at24c16_write(EE_ADDR_GATEWAY_ONE, usart_buf[4]);
//				at24c16_write(EE_ADDR_GATEWAY_TWO, usart_buf[5]);
//				at24c16_write(EE_ADDR_GATEWAY_THREE, usart_buf[6]);
//				at24c16_write(EE_ADDR_GATEWAY_FOUR, usart_buf[7]);
//				uart_w5500_gateway_config = 1;
//			}
//			if(usart_buf[3] == 0x07)	//DNS
//			{
//				uart2_send_write = 1;
//		    at24c16_write(EE_ADDR_DNS_ONE, usart_buf[4]);
//				at24c16_write(EE_ADDR_DNS_TWO, usart_buf[5]);
//				at24c16_write(EE_ADDR_DNS_THREE, usart_buf[6]);
//				at24c16_write(EE_ADDR_DNS_FOUR, usart_buf[7]);
//				uart_w5500_dns_config = 1;
//			}
			if(usart_buf[3] == 0x08)	//端口
			{
				uart2_send_write = 1;
				at24c16_write(EE_ADDR_PORT_1, usart_buf[4]);
				at24c16_write(EE_ADDR_PORT_2, usart_buf[5]);
				local_port = usart_buf[4]<<8|usart_buf[5];
				uart_w5500_port_config = 1;
			}
			if(usart_buf[3] == 0x10)
			{
				uart2_send_write = 1;
				at24c16_write(EE_ADDR_ID_NAME1, usart_buf[4]);
				at24c16_write(EE_ADDR_ID_NAME2, usart_buf[5]);
				at24c16_write(EE_ADDR_ID_NAME3, usart_buf[6]);
				at24c16_write(EE_ADDR_ID_NAME4, usart_buf[7]);
			}
//			if(( uart_w5500_ip_config == 1) && ( uart_w5500_mac_config == 1) && ( uart_w5500_subnet_config == 1) && \
//				(uart_w5500_gateway_config == 1)	&&	(uart_w5500_dns_config == 1) && (uart_w5500_port_config))
//			{
//					uart_w5500_ip_config = 0;
//					uart_w5500_mac_config = 0;
//					uart_w5500_subnet_config = 0;
//					uart_w5500_gateway_config = 0;
//					uart_w5500_dns_config = 0;
//					uart_w5500_port_config = 0;
//					at24c16_write(EE_ADDR_W5500_CONFIG, EE_ADDR_W5500_PC_CONFIG_SUCCESS);
//					printf(" The w5500 config is success ! \r\n");
//			}
		}
		if(uart2_send_read == 1)				
		{
			uart2_send_read = 0;
			sum = 0;
			for(int k = 0; k < 9; k++)
			{
					sum+=txBuffer[k];	
			}
			txBuffer[9]=sum;
			for(int k=0; k < 10; k++)
			{
					USART_SendData(USART2,txBuffer[k]);
				  delay_us(100);
			}
		}
		if(uart2_send_write == 1)
		{
			sum = 0;
			uart2_send_write = 0;
				txBuffer[0] = 0x24;
				txBuffer[1] = 0x10+usart_buf[2];
				txBuffer[2] = 0x06;	
				txBuffer[3] = 0x00;	
				txBuffer[4] = 0x00;	
				txBuffer[5] = 0x00;	
				txBuffer[6] = 0x00;	
				txBuffer[7] = 0x00;		
				txBuffer[8] = 0x00;	
			for(int k = 0; k < 9; k++)
			{
					sum+=txBuffer[k];	
			}
			txBuffer[9] = sum;
			for(int k=0; k < 10; k++)
			{
					USART_SendData(USART2,txBuffer[k]);
					delay_us(100);
			}
		}
	}	
}


void GetLockCode(void)
{
	CpuID[0]=*(vu32*)(0x1ffff7e8);
	CpuID[1]=*(vu32*)(0x1ffff7ec);
	CpuID[2]=*(vu32*)(0x1ffff7f0);

	Lock_Code=(CpuID[0]>>1)+(CpuID[1]>>2)+(CpuID[2]>>3);
	//DEVID = (char *)malloc(200*sizeof(char));
	//sprintf(DEVID,"%s",Lock_Code);
	printf("唯一id：%d\r\n",Lock_Code);
}

void SoftReset(void)
{
	__set_FAULTMASK(1);      						
	NVIC_SystemReset();									
}

void USART_SendDatacmd(USART_TypeDef* USARTx,unsigned char ch)
{
	while((USARTx->SR&USART_SR_TXE)==0);
	USARTx->DR=ch;
}

void Power_check(void)
{
	if(((GPIOA->IDR & 0x0002) == 2) || ((GPIOA->IDR & 0x0002) == 1))
	{
		POWER_LED_SUCCESS_ON;
		POWER_LED_FAIL_OFF;
	}
	else
	{    
		POWER_LED_SUCCESS_OFF;
		POWER_LED_FAIL_ON;
	}	
}
void read_config(void)
{
	W5500_REBOOT = at24c16_read(EE_ADDR_REBOOT);
	printf("W5500_REBOOT = %d\r\n",W5500_REBOOT);
	if(at24c16_read(EE_ADDR_REBOOT) == 0x01)
	{
		  at24c16_write(EE_ADDR_REBOOT, 0x00);
			W5500_REBOOT = 1;
	}		
	if(at24c16_read(EE_ADDR_MQTT_SEND_TIME) == 0xFF)
	{
		mqtt_send_time = 30;
	}
	else
	{
		mqtt_send_time = at24c16_read(EE_ADDR_MQTT_SEND_TIME);
	}
	id_name[0] = at24c16_read(EE_ADDR_ID_NAME1);
	id_name[1] = at24c16_read(EE_ADDR_ID_NAME2);
	id_name[2] = at24c16_read(EE_ADDR_ID_NAME3);
	id_name[3] = at24c16_read(EE_ADDR_ID_NAME4);
	id_name_all = id_name[0] << 24 | id_name[1] << 16 | id_name[2] << 8 | id_name[3];
	
}



