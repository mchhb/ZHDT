#ifndef __CAN_H
#define	__CAN_H

#include "stm32f10x.h"

typedef int   can_bool_t;      /**< boolean type */
typedef long 	can_err_t;
#define CAN_TX_TIMEOUT_DEFAULT	3000

#define CAN_ERR_OK							0
#define CAN_ERR_NODEV						-1
#define CAN_ERR_SEND_FAIL				-2
#define CAN_ERR_RX_BUF_EMPTY		-3
#define CAN_ERR_RX_BUF_FULL			-4
#define CAN_ERR_TX_BUF_FULL			-5
#define CAN_ERR_PARAM_ERROR			-6
#define CAN_ERR_INIT_FAIL				-7
#define CAN_ERR_QUEUE_FULL			-8
#define CAN_ERR_QUEUE_EMPTY			-9
#define CAN_ERR_SETBAUD_FAIL		-10


/* special address description flags for the CAN_ID */
#define CAN_EFF_FLAG 0x80000000U /* EFF/SFF is set in the MSB */
#define CAN_RTR_FLAG 0x40000000U /* remote transmission request */
#define CAN_ERR_FLAG 0x20000000U /* error frame */

/* valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK 0x000007FFU /* standard frame format (SFF) */
#define CAN_EFF_MASK 0x1FFFFFFFU /* extended frame format (EFF) */
#define CAN_ERR_MASK 0x1FFFFFFFU /* omit EFF, RTR, ERR flags */

#define BPS_250K      0
#define BPS_500K      1
#define BPS_1M      	2


/**
 * struct can_frame - basic CAN frame structure
 * @can_id:  the CAN ID of the frame and CAN_*_FLAG flags, see above.
 * @can_dlc: the data length field of the CAN frame
 * @data:    the CAN frame payload.
 */
struct can_frame {
	uint32_t	can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
	uint8_t		can_dlc; /* data length code: 0 .. 8 */
	uint8_t		data[8];
	uint32_t	ts[2];
};
typedef struct can_frame	can_frame_t;

#define CAN_QUEUE_SIZE		128

struct can_queue
{
	uint16_t getp;
	uint16_t putp;
	can_frame_t buffer[CAN_QUEUE_SIZE];
};

typedef struct can_queue can_queue_t;


void can_init(uint32_t bps);
can_err_t can_recv(can_frame_t *message);
can_err_t can_send(can_frame_t *message, uint32_t loop);


#endif

/*------------------------------------- END OF THIS FILE --------------------------------------------------------*/












