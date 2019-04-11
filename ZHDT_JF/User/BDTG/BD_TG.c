#include "stm32f10x.h"

#include "BD_TG.h"
#include "MqttKit.h"
#include "socket.h"
#include "w5500_config.h"
#include "w5500.h"
#include "tcp_demo.h"

#include "delay.h"
#include <string.h>
#include <stdio.h>


#define PROID		"utfuett/device_001"							// Éè±¸È«³Æ//Ö®Ç°ÓÃ»§ÃûÊÇ£º96ppxbd/s001

#define AUTH_INFO	"j9NiGuEaIuKPVexp/MUwWAlZLDox1YQ5+eqX6DuuP/k="	// APIÃÜÔ¿  //Ö®Ç°ÃÜÂëÊÇc8opPYlA7ejO7RxAmMjynNBBf015UaGf04UxAIowQIU=

#define DEVID		"DeviceClient005"																								// ÈÙÒ«ÌìµØÉè±¸ID 002ºÅ
//uint8 Disconnect_Request[2] = {0xe0,0x00};
unsigned char w5500_buf[128];   //Ö®Ç°ÊÇ128
unsigned char publish_buf[64]; 
extern unsigned char Restore_factory[10];
extern unsigned char  flag;
extern unsigned char  BD_TGflag;
//==========================================================
//	º¯ÊýÃû³Æ£º	BD_TG_DevLink

//	º¯Êý¹¦ÄÜ£º	Óë°Ù¶ÈÌì¹¤´´½¨Á¬½Ó

//	Èë¿Ú²ÎÊý£º	ÎÞ

//	·µ»Ø²ÎÊý£º	1-³É¹¦	0-Ê§°Ü

//	ËµÃ÷£º		Óë°Ù¶ÈÌì¹¤Æ½Ì¨½¨Á¢Á¬½Ó
//==========================================================
//const char *topics1[] = {"stateup"};

void BD_TG_DevLink(void)
{
	
	int len;
	static MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};					//Ð­Òé°ü

	unsigned char *dataPtr;
//	unsigned char mqtt=0;

	printf( "°Ù¶ÈÌì¹¤_DevLink\r\nPROID: %s,	AUIF: %s,	DEVID:%s\r\n",PROID, AUTH_INFO, DEVID);
	
	if(MQTT_PacketConnect(PROID, AUTH_INFO, DEVID, 60, 0, MQTT_QOS_LEVEL0, NULL, NULL, 0, &mqttPacket) == 0)
	{
			
		
		send(SOCK_TCPC,mqttPacket._data,mqttPacket._len);	
		delay_ms(2000);  //µÈ´ýÆ½Ì¨ÏìÓ¦
      //send(SOCK_TCPC,test,sizeof(test));/////////µ÷ÊÔ´úÂë//////////
			//BD_TG_Publish(*topics1, "123"); //·¢²¼ÏûÏ¢
//		while( w5500_buf[0] != 0x20)
		while( w5500_buf[0] != 0x20)
		{	
			len=getSn_RX_RSR(SOCK_TCPC);		
			recv(SOCK_TCPC,w5500_buf,len); 
			//send(SOCK_TCPC,Disconnect_Request, 2);	
			//send(SOCK_TCPC,mqttPacket._data,mqttPacket._len);
            delay_ms(1);
					  break;	
 		}
		if(w5500_buf[0] == 0x20)
	  {
		  flag=1;
		}	
//		else
//		{	
//			len=getSn_RX_RSR(SOCK_TCPC);		
//			recv(SOCK_TCPC,w5500_buf,len); 
//			//send(SOCK_TCPC,Disconnect_Request, 2);	
//			//send(SOCK_TCPC,mqttPacket._data,mqttPacket._len);
////            delay_ms(100);
//			
//		//BD_TGflag=1;	
// 		}
	
		dataPtr = w5500_buf;	
		if(dataPtr != NULL)
		{
			if(MQTT_UnPacketRecv(dataPtr) == MQTT_PKT_CONNACK)
			{
				switch(MQTT_UnPacketConnectAck(dataPtr))
				{
					case 0:printf( "Tips:	Á¬½Ó³É¹¦\r\n");break;			
					case 1:printf("WARN:	Á¬½ÓÊ§°Ü£ºÐ­Òé´íÎó\r\n");break;
					case 2:printf("WARN:	Á¬½ÓÊ§°Ü£º·Ç·¨µÄclientid\r\n");break;
					case 3:printf("WARN:	Á¬½ÓÊ§°Ü£º·þÎñÆ÷Ê§°Ü\r\n");break;
					case 4:printf("WARN:	Á¬½ÓÊ§°Ü£ºÓÃ»§Ãû»òÃÜÂë´íÎó\r\n");break;
					case 5:printf("WARN:	Á¬½ÓÊ§°Ü£º·Ç·¨Á´½Ó(±ÈÈçtoken·Ç·¨)\r\n");break;		
					default:printf("ERR:	Á¬½ÓÊ§°Ü£ºÎ´Öª´íÎó\r\n");break;
				}
			}
		}
		
		MQTT_DeleteBuffer(&mqttPacket);								//É¾°ü
	}
	else
	printf( "WARN:	MQTT_PacketConnect Failed\r\n");
	

}


void MQTT_UnSubscribe(const char *topics[], uint8 topics_cnt)
{

	int len;
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};					//Ð­Òé°ü

	if(MQTT_PacketUnSubscribe(10, topics,topics_cnt, &mqttPacket) == 0)
	{
	   send(SOCK_TCPC,mqttPacket._data,mqttPacket._len);
	   delay_ms(100);  //µÈ´ýÆ½Ì¨ÏìÓ¦
		 len=getSn_RX_RSR(SOCK_TCPC);
     recv(SOCK_TCPC,w5500_buf,len);
		 if ((w5500_buf[0] >> 4) == MQTT_PKT_UNSUBACK)
		    printf("³É¹¦È¡Ïû¶©ÔÄ\r\n");					
	}	

	

}
//==========================================================
//	º¯ÊýÃû³Æ£º	BD_TG_Subscribe

//	º¯Êý¹¦ÄÜ£º	¶©ÔÄ

//	Èë¿Ú²ÎÊý£º	topics£º¶©ÔÄµÄtopic
//				topic_cnt£ºtopic¸öÊý

//	·µ»Ø²ÎÊý£º	SEND_TYPE_OK-³É¹¦	SEND_TYPE_SUBSCRIBE-ÐèÒªÖØ·¢

//	ËµÃ÷£º		
//==========================================================
void BD_TG_Subscribe(const char *topics[], unsigned char topic_cnt)
{
	
	unsigned char i = 0;
	uint16  len;
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};							//Ð­Òé°ü
	
	for(; i < topic_cnt; i++)
		printf( "Subscribe Topic: %s\r\n", topics[i]);
	
	if(MQTT_PacketSubscribe(MQTT_SUBSCRIBE_ID, MQTT_QOS_LEVEL0, topics, topic_cnt, &mqttPacket) == 0)
	{
		send(SOCK_TCPC,mqttPacket._data, mqttPacket._len);					//ÏòÆ½Ì¨·¢ËÍ¶©ÔÄÇëÇó
		
		MQTT_DeleteBuffer(&mqttPacket);											//É¾°ü
	}
	
	delay_ms(100);  //µÈ´ýÆ½Ì¨ÏìÓ¦
	len=getSn_RX_RSR(SOCK_TCPC);
	recv(SOCK_TCPC,w5500_buf,len); 
	switch(w5500_buf[4])
	{
			case 0x00:
				printf("¶©ÔÄÈ·ÈÏ£¬ QoSµÈ¼¶0\r\n");
			  break;
			case 0x01:
				printf("¶©ÔÄÈ·ÈÏ£¬ QoSµÈ¼¶1\r\n");
			  break;
		  case 0x02:
				printf("¶©ÔÄÈ·ÈÏ£¬ QoSµÈ¼¶2 \r\n");
			  break;
			case 0x80:
				printf("¶©ÔÄÊ§°Ü \r\n");
			  break;
		}
	 

}

//==========================================================
//	º¯ÊýÃû³Æ£º	BD_TG_Publish
//
//	º¯Êý¹¦ÄÜ£º	·¢²¼ÏûÏ¢
//
//	Èë¿Ú²ÎÊý£º	topic£º·¢²¼µÄÖ÷Ìâ
//				msg£ºÏûÏ¢ÄÚÈÝ
//
//	·µ»Ø²ÎÊý£º	SEND_TYPE_OK-³É¹¦	SEND_TYPE_PUBLISH-ÐèÒªÖØËÍ
//
//	ËµÃ÷£º		
//==========================================================
char test1[200]={1,2,3,4,5,0,6,7,8,9,0,12};
unsigned char ttt[100]={1,2,3,4,5,0,6,7,8,9,0,12};;
extern MQTT_PACKET_STRUCTURE mqttPacket;							//Ð­Òé°ü
    
void BD_TG_Publish(const char *topic,char *msg,unsigned short Length)  //º¯ÊýÔ­ÐÍvoid BD_TG_Publish(const char *topic, const char *msg)
{
//	char i;
     
	if(MQTT_PacketPublish(MQTT_PUBLISH_ID, topic, msg, Length, MQTT_QOS_LEVEL0, 0, 1, &mqttPacket) == 0)
	{
		send(SOCK_TCPC,mqttPacket._data, mqttPacket._len);					////mqttPacket._lenÏòÆ½Ì¨·¢ËÍ¶©ÔÄÇëÇ
	  printf("·¢ËÍ³É¹¦\r\n"); 
		MQTT_DeleteBuffer(&mqttPacket);
		delay_ms(100);
	}

}

//==========================================================
//	º¯ÊýÃû³Æ£º	BD_TG_RevPro
//
//	º¯Êý¹¦ÄÜ£º	Æ½Ì¨·µ»ØÊý¾Ý¼ì²â
//
//	Èë¿Ú²ÎÊý£º	dataPtr£ºÆ½Ì¨·µ»ØµÄÊý¾Ý
//
//	·µ»Ø²ÎÊý£º	ÎÞ
//
//	ËµÃ÷£º		
//==========================================================
void BD_TG_RevPro(unsigned char *cmd)
{
	
	MQTT_PACKET_STRUCTURE mqttPacket = {NULL, 0, 0, 0};								//Ð­Òé°ü
	
	char *req_payload = NULL;
	char *cmdid_topic = NULL;
	unsigned char type = 0;
	unsigned char qos = 0;
	static unsigned short pkt_id = 0;
	
	short result = 0;

	char *dataPtr = NULL;
	char numBuf[10];
	int num = 0;
	
	type = MQTT_UnPacketRecv(cmd);
	switch(type)
	{
		case MQTT_PKT_CMD:															//ÃüÁîÏÂ·¢
			
			result = MQTT_UnPacketCmd(cmd, &cmdid_topic, &req_payload);				//½â³ötopicºÍÏûÏ¢Ìå
			if(result == 0)
			{
				printf("cmdid: %s, req: %s\r\n", cmdid_topic, req_payload);
				
				if(MQTT_PacketCmdResp(cmdid_topic, req_payload, &mqttPacket) == 0)	//ÃüÁî»Ø¸´×é°ü
				{
				 printf( "Tips:	Send CmdResp\r\n");
					
					send(SOCK_TCPC,mqttPacket._data, mqttPacket._len);			//»Ø¸´ÃüÁî
					MQTT_DeleteBuffer(&mqttPacket);									//É¾°ü
				}
			}
		
		break;
			
		case MQTT_PKT_PUBLISH:														//½ÓÊÕµÄPublishÏûÏ¢
		
			result = MQTT_UnPacketPublish(cmd, &cmdid_topic, &req_payload, &qos, &pkt_id);
			if(result == 0)
			{
				printf("topic: %s\r\npayload: %s\r\n", cmdid_topic, req_payload);
				memcpy(publish_buf,req_payload,strlen(req_payload));
				switch(qos)
				{
					case 1:													//ÊÕµ½publishµÄqosÎª1£¬Éè±¸ÐèÒª»Ø¸´Ack
					
						if(MQTT_PacketPublishAck(pkt_id, &mqttPacket) == 0)
						{
							printf( "Tips:	Send PublishAck\r\n");
							send(SOCK_TCPC,mqttPacket._data, mqttPacket._len);
							MQTT_DeleteBuffer(&mqttPacket);
						}
					
					break;
					
					case 2:													//ÊÕµ½publishµÄqosÎª2£¬Éè±¸ÏÈ»Ø¸´Rec
																					//Æ½Ì¨»Ø¸´Rel£¬Éè±¸ÔÙ»Ø¸´Comp
						if(MQTT_PacketPublishRec(pkt_id, &mqttPacket) == 0)
						{
							printf("Tips:	Send PublishRec\r\n");
							send(SOCK_TCPC,mqttPacket._data, mqttPacket._len);
							MQTT_DeleteBuffer(&mqttPacket);
						}
					
					break;
					
					default:
						break;
				}
			}
		
		break;
			
		case MQTT_PKT_PUBACK:														//·¢ËÍPublishÏûÏ¢£¬Æ½Ì¨»Ø¸´µÄAck
		
			if(MQTT_UnPacketPublishAck(cmd) == 0)
				printf("Tips:	MQTT Publish Send OK\r\n");
			
		break;
			
		case MQTT_PKT_PUBREC:														//·¢ËÍPublishÏûÏ¢£¬Æ½Ì¨»Ø¸´µÄRec£¬Éè±¸Ðè»Ø¸´RelÏûÏ¢
		
			if(MQTT_UnPacketPublishRec(cmd) == 0)
			{
				printf("Tips:	Rev PublishRec\r\n");
				if(MQTT_PacketPublishRel(MQTT_PUBLISH_ID, &mqttPacket) == 0)
				{
					 printf ("Tips:	Send PublishRel\r\n");
					send(SOCK_TCPC,mqttPacket._data, mqttPacket._len);
					MQTT_DeleteBuffer(&mqttPacket);
				}
			}
		
		break;
			
		case MQTT_PKT_PUBREL:														//ÊÕµ½PublishÏûÏ¢£¬Éè±¸»Ø¸´Recºó£¬Æ½Ì¨»Ø¸´µÄRel£¬Éè±¸ÐèÔÙ»Ø¸´Comp
			
			if(MQTT_UnPacketPublishRel(cmd, pkt_id) == 0)
			{
				printf("Tips:	Rev PublishRel\r\n");
				if(MQTT_PacketPublishComp(MQTT_PUBLISH_ID, &mqttPacket) == 0)
				{
					printf( "Tips:	Send PublishComp\r\n");
					send(SOCK_TCPC,mqttPacket._data, mqttPacket._len);
					MQTT_DeleteBuffer(&mqttPacket);
				}
			}
		
		break;
		
		case MQTT_PKT_PUBCOMP:														//·¢ËÍPublishÏûÏ¢£¬Æ½Ì¨·µ»ØRec£¬Éè±¸»Ø¸´Rel£¬Æ½Ì¨ÔÙ·µ»ØµÄComp
		
			if(MQTT_UnPacketPublishComp(cmd) == 0)
			{
			 printf("Tips:	Rev PublishComp\r\n");
			}
		
		break;
			
		case MQTT_PKT_SUBACK:														//·¢ËÍSubscribeÏûÏ¢µÄAck
		
			if(MQTT_UnPacketSubscribe(cmd) == 0)
				printf("Tips:	MQTT Subscribe OK\r\n");
			else
				printf("Tips:	MQTT Subscribe Err\r\n");
		
		break;
			
		case MQTT_PKT_UNSUBACK:														//·¢ËÍUnSubscribeÏûÏ¢µÄAck
		
			if(MQTT_UnPacketUnSubscribe(cmd) == 0)
				printf( "Tips:	MQTT UnSubscribe OK\r\n");
			else
				printf("Tips:	MQTT UnSubscribe Err\r\n");
		
		break;
		
		default:
			result = -1;
		break;
	}
	
	//ESP8266_Clear();									//Çå¿Õ»º´æ
	
	if(result == -1)
		return;
	
	dataPtr = strchr(req_payload, '}');					//ËÑË÷'}'

	if(dataPtr != NULL && result != -1)					//Èç¹ûÕÒµ½ÁË
	{
		dataPtr++;
		
		while(*dataPtr >= '0' && *dataPtr <= '9')		//ÅÐ¶ÏÊÇ·ñÊÇÏÂ·¢µÄÃüÁî¿ØÖÆÊý¾Ý
		{
			numBuf[num++] = *dataPtr++;
		}
		
		num = atoi((const char *)numBuf);				//×ªÎªÊýÖµÐÎÊ½
		
	}

	if(type == MQTT_PKT_CMD || type == MQTT_PKT_PUBLISH)
	{
		MQTT_FreeBuffer(cmdid_topic);
		MQTT_FreeBuffer(req_payload);
	}
}
