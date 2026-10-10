#ifndef __HCAN_H__
#define __HCAN_H__

#include "stm32f4xx_hal.h"
#include <string.h> 
#include "can.h"

#define CAN_NUM &hcan1

extern uint8_t G6220_flag;

void CAN_Start(CAN_HandleTypeDef *hcan);
void Can_SendCmd(uint16_t ID,uint8_t *cmd, uint8_t len);
void CAN_SendEXData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len);
void CAN_SendData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t Len);
void CAN_Rx_Callback(CAN_HandleTypeDef *hcan);
void Response_Handle(uint32_t ExtId,uint32_t StdId, uint8_t *rxBuf);


#endif
