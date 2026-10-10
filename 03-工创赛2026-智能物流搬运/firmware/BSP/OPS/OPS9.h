#ifndef __OPS9_H
#define __OPS9_H

#include "sys.h"

#define OPS9_RX_DATA_LEN	14      
#define OPS9_TX_DATA_LEN	2

typedef __packed struct
{
	uint8_t header;
	float x;
	float y;
	float z;
	uint8_t check_sum;
}OPS9_Rx_Data_t;

typedef struct
{
	OPS9_Rx_Data_t OPS9_Rx_Data;
}ops9_para_t;

void OPS9_init(void);
void usart1_Init(void);
void OPS9_Unpack(uint8_t ops9_receive_data[OPS9_RX_DATA_LEN]);
ops9_para_t *get_ops9_data_point(void);
void OPS9_stop(void);

#endif
