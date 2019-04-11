#include "can.h" 
#include <stdio.h>
#include <string.h>

can_queue_t rx_queue;

can_err_t can_queue_init(can_queue_t *q)
{
	q->getp = 0;
	q->putp = 0;
	memset(q->buffer, 0, sizeof(q->buffer));
	return CAN_ERR_OK;
}

can_err_t can_queue_putMessage(can_queue_t *q, can_frame_t *msg)
{
	uint16_t next_putp;
	void *put_ptr;

	next_putp = q->putp;
	
	if(next_putp == (CAN_QUEUE_SIZE-1) ){
		next_putp = 0;
        }
	else{
		next_putp++;
        }
        
	if(next_putp == q->getp) 
		return CAN_ERR_QUEUE_FULL;
	
	put_ptr = &(q->buffer[q->putp]);
	
	memcpy(put_ptr, msg, sizeof(can_frame_t));
	q->putp = next_putp;
	
	return CAN_ERR_OK;
}

can_err_t can_queue_getMessage(can_queue_t *q, can_frame_t *msg)
{
	uint16_t next_getp;
	void *get_ptr;
	
	if(q->getp == q->putp) 
		return CAN_ERR_QUEUE_EMPTY;
	
	get_ptr = &(q->buffer[q->getp]);
	
	memcpy(msg, get_ptr, sizeof(can_frame_t));
	
	next_getp = q->getp;
	
	if(next_getp == (CAN_QUEUE_SIZE-1) ){
		next_getp = 0;
        }
	else{
		next_getp++;
	}
	q->getp = next_getp;

	return CAN_ERR_OK;
	
}

can_bool_t can_queue_isFull(can_queue_t *q)
{

	uint16_t next_putp;

	next_putp = q->putp;
	
	if(next_putp == (CAN_QUEUE_SIZE-1) ){
		next_putp = 0;
        }
	else{
		next_putp++;
        }
	return (next_putp == q->getp); 
}

can_bool_t can_queue_isEmpty(can_queue_t *q)
{
	return ( q->getp == q->putp);
}





void CAN1_Config_init(uint32_t baud)
{
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/* CAN register init */
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);

	/* CAN cell init */ //36MHz 500Kbps
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
      
  CAN_InitStructure.CAN_Mode= CAN_Mode_Normal; 
	switch(baud) {
		case BPS_250K:				// 36M/(1+12+3)/9= 250kbps
			CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1 = CAN_BS1_12tq;
			CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
			CAN_InitStructure.CAN_Prescaler = 9;
			break;
		case BPS_500K:				//36M/(1+5+2)/9 = 500kbps
			CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1 = CAN_BS1_5tq;
			CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
			CAN_InitStructure.CAN_Prescaler = 9;
			break;
		case BPS_1M:					//36M(1+2+1)/9 = 1M
			CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1 = CAN_BS1_2tq;
			CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
			CAN_InitStructure.CAN_Prescaler = 9;
			break;
		default:							//36M/(1+12+3)/9= 250kbps
			CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
			CAN_InitStructure.CAN_BS1 = CAN_BS1_12tq;
			CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
			CAN_InitStructure.CAN_Prescaler = 9;
			break;
	}
  CAN_Init(CAN1,&CAN_InitStructure); 
                                         
	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber 					= 0;  
	CAN_FilterInitStructure.CAN_FilterMode 						= CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale 				 	= CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh 				 	= 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow 				 	= 0x0000;;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh 		 	= 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow      	= 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment 	= CAN_FIFO0;;
	CAN_FilterInitStructure.CAN_FilterActivation     	= ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
 
}
 
// ***************************************************************
//       BaudRate = 1 / NominalBitTime
//       NominalBitTime = 1tq + tBS1 + tBS2
//       tq = (BRP[9:0] + 1) x tPCLK
//       tPCLK = CAN's clock = APB1's clock
//   1Mbps ???,???????6tq???,BS1=5, BS2=2
//   500kbps ???,???????8tq???,BS1=7, BS2=3
//   250kbps ???,???????14tq???,BS1=13, BS2=2
//   125k, 100k, 50k, 20k, 10k ??????? 250K ??
// ****************************************************************
void  can_init(uint32_t bps)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;

	can_queue_init(&rx_queue);
	
	/* Enable CAN RX0 interrupt IRQ channel */
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = CAN1_SCE_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	 
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	//Can Rx
	GPIO_InitStruct.GPIO_Pin  =  GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode =  GPIO_Mode_IPU;
	GPIO_Init(GPIOA,&GPIO_InitStruct);

	//Can Tx
	GPIO_InitStruct.GPIO_Pin  =  GPIO_Pin_12;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Mode =  GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	 
	CAN1_Config_init(bps);
	CAN_ITConfig(CAN1,CAN_IT_FMP0 | CAN_IT_FOV0, ENABLE);  
	CAN_ITConfig(CAN1,CAN_IT_FMP1 | CAN_IT_FOV1, ENABLE);  
	CAN_ITConfig(CAN1,CAN_IT_BOF  | CAN_IT_LEC , ENABLE);

// 	CAN_ITConfig(CAN1,CAN_IT_FMP0 | CAN_IT_FF0 | CAN_IT_FOV0, ENABLE);  
// 	CAN_ITConfig(CAN1,CAN_IT_FMP1 | CAN_IT_FF1 | CAN_IT_FOV1, ENABLE);  
// 	CAN_ITConfig(CAN1,CAN_IT_TME, DISABLE);              
// 	CAN_ITConfig(CAN1,CAN_IT_EWG | CAN_IT_EPV | CAN_IT_BOF | CAN_IT_LEC | CAN_IT_ERR | CAN_IT_WKU | CAN_IT_SLK, ENABLE);  
}
 
void USB_LP_CAN1_RX0_IRQHandler(void)			// used for STM32F103
{
	uint32_t 		i;
	CanRxMsg 		canMessage;
	can_frame_t canFrame;

	if(CAN_GetITStatus(CAN1, CAN_IT_FMP0) == SET )	{
		CAN_Receive(CAN1, CAN_FIFO0, &canMessage);
		canFrame.can_id = canMessage.StdId;
		if(canMessage.IDE) {
			canFrame.can_id |= CAN_EFF_FLAG;
			canFrame.can_id |= canMessage.ExtId;
		}
		else{
			canFrame.can_id |= canMessage.StdId;
		}
		if(canMessage.RTR) {
			canFrame.can_id |= CAN_RTR_FLAG;
		}
		
		canFrame.can_dlc = (CAN_FIFO0<<4)|(canMessage.DLC);

		for(i = 0; i < (canFrame.can_dlc&0x0f); i++) {
			canFrame.data[i] = canMessage.Data[i];
		}
// 	build up timestamp				
		canFrame.ts[0] = (CAN1->sFIFOMailBox[CAN_FIFO0].RDTR&0xFFFF0000)>>16;
//		canFrame.ts[0] = (uint32_t)(RTC->TR & 0x007F7F7F);
//		canFrame.ts[1] = RTC->SSR;
			
		if(can_queue_putMessage(&rx_queue, &canFrame) != CAN_ERR_OK ) {

		}
	}	
}

void CAN1_RX1_IRQHandler(void)
{
	uint32_t 		i;
	CanRxMsg 		canMessage;
	can_frame_t canFrame;

	if(CAN_GetITStatus(CAN1, CAN_IT_FMP1) == SET )	{
		CAN_Receive(CAN1, CAN_FIFO1, &canMessage);
		canFrame.can_id = canMessage.StdId;
		if(canMessage.IDE) {
			canFrame.can_id |= CAN_EFF_FLAG;
			canFrame.can_id |= canMessage.ExtId;
		}
		else{
			canFrame.can_id |= canMessage.StdId;
		}
		if(canMessage.RTR) {
			canFrame.can_id |= CAN_RTR_FLAG;
		}
		
		canFrame.can_dlc = (CAN_FIFO0<<4)|(canMessage.DLC);

		for(i = 0; i < (canFrame.can_dlc&0x0f); i++) {
			canFrame.data[i] = canMessage.Data[i];
		}
// 	build up timestamp				
		canFrame.ts[0] = (CAN1->sFIFOMailBox[CAN_FIFO0].RDTR&0xFFFF0000)>>16;
//		canFrame.ts[0] = (uint32_t)(RTC->TR & 0x007F7F7F);
//		canFrame.ts[1] = RTC->SSR;
			
		if(can_queue_putMessage(&rx_queue, &canFrame) != CAN_ERR_OK ) {

		}
	}	
}


void CAN1_SCE_IRQHandler(void)
{
	can_frame_t canFrame;
	
	if( CAN_GetITStatus(CAN1, CAN_IT_ERR) == SET ) {
		canFrame.can_id |= CAN_ERR_FLAG;
		canFrame.can_dlc = 8;
		canFrame.data[0] = (CAN1->ESR  & 0x000000ff); 				//LEC(bit6-4), BOFF(bit2),EPVF(bit1),EWGF(bit0)
		canFrame.data[1] = (CAN1->ESR  & 0xff000000) >> 24; 	//REC
		canFrame.data[2] = (CAN1->ESR  & 0x00ff0000) >> 16;		//TEC
		canFrame.data[3] = (CAN1->RF0R & 0x000000ff); 				//RFORM0(bit5) FOVR0(bit4) FULL0(bit3)
		canFrame.data[4] = (CAN1->RF1R & 0x000000ff); 				//RFORM1(bit5) FOVR1(bit4) FULL1(bit3)
		canFrame.data[5] = (CAN1->TSR  & 0x000000ff);
		canFrame.data[6] = (CAN1->TSR  & 0x0000ff00) >> 8;
		canFrame.data[7] = (CAN1->TSR  & 0x00ff0000) >> 16;
		
		if( CAN_GetITStatus(CAN1, CAN_IT_BOF)==SET ){
			CAN_ClearITPendingBit(CAN1, CAN_IT_BOF);
//			can_device->busoff_count++;
			canFrame.can_id |= CAN_IT_BOF;
		}
		if( CAN_GetITStatus(CAN1, CAN_IT_FOV0)==SET ){
			CAN_ClearITPendingBit(CAN1, CAN_IT_FOV0);
//			can_device->rx_hw_fifo_over_count++;
			canFrame.can_id |= CAN_IT_FOV0;
		}
		if( CAN_GetITStatus(CAN1, CAN_IT_FOV1)==SET ){
			CAN_ClearITPendingBit(CAN1, CAN_IT_FOV1);
//			can_device->rx_hw_fifo_over_count++;
			canFrame.can_id |= CAN_IT_FOV1;
		}
		if( CAN_GetITStatus(CAN1, CAN_IT_LEC)==SET ){
			CAN_ClearITPendingBit(CAN1, CAN_IT_LEC);
			CAN_ClearFlag(CAN1, CAN_FLAG_LEC);
//			can_device->rx_err_frame_count++;
			canFrame.can_id |= CAN_IT_LEC;
		}

		CAN_ClearITPendingBit(CAN1, CAN_IT_ERR);
	}
}

can_err_t can_recv(can_frame_t *message)
{
	if( can_queue_getMessage(&rx_queue, message) == CAN_ERR_OK) {

		return CAN_ERR_OK;
	}
	return CAN_ERR_RX_BUF_EMPTY;
}



// for TX 



can_err_t can_send(can_frame_t *message, uint32_t loop)
{
	CanTxMsg TxMessage;
	uint8_t i;
	uint8_t TransmitMailbox;
	
	TxMessage.IDE = CAN_Id_Standard;
	TxMessage.RTR = CAN_RTR_Data;

	if(message->can_id&CAN_EFF_FLAG){ 
		TxMessage.IDE 	= CAN_Id_Extended;
		TxMessage.ExtId = (message->can_id&CAN_EFF_MASK);
	}
	else {
		TxMessage.IDE   = CAN_Id_Standard;
		TxMessage.StdId = (message->can_id&CAN_SFF_MASK);
	}
	
	TxMessage.RTR = (message->can_id&CAN_RTR_FLAG)? CAN_RTR_REMOTE : CAN_RTR_DATA;
	TxMessage.DLC = message->can_dlc;
	for(i=0; i<TxMessage.DLC; i++)
		TxMessage.Data[i] = message->data[i];

	TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);

	if(TransmitMailbox == CAN_TxStatus_NoMailBox)	{
//		can->tx_hw_fifo_over_count++;
		return CAN_ERR_TX_BUF_FULL;
	}
	while(loop--){
		if( CAN_TransmitStatus(CAN1, TransmitMailbox) == CAN_TxStatus_Ok ){
//			can->tx_count++;
			return CAN_ERR_OK;
		}
	}
	
	return CAN_ERR_SEND_FAIL;
}





