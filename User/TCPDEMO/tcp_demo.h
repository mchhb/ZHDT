#ifndef __TCP_DEMO_H
#define __TCP_DEMO_H
#include "types.h"
#include "stm32f10x.h"
#define SOCK_TCPC             4
void set_object_ip(void);
extern uint16 W5500_tcp_server_port;
extern int MQTT_STATE;
void do_tcp_server(void);//TCP Server回环演示函数
void do_tcp_client(void);//TCP Clinet回环演示函数
uint16 hex1adecimal_to_decimal(u32 hex);
#endif 
