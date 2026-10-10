#include "OPS9.h"
#include "usart.h"
#include "crc8_crc16.h"

ops9_para_t ops9_para;

uint8_t ops9_transmit_data[OPS9_TX_DATA_LEN];
uint8_t ops9_receive_data[OPS9_RX_DATA_LEN];
/*-----------------------------------------------------------------*/
/*-----------------------------------------------------------------*/

void OPS9_init(void)
{
	HAL_Delay(500);
	ops9_transmit_data[0] = 0xC5;
	ops9_transmit_data[1] = 0x22;
	HAL_UART_Transmit(&huart6, ops9_transmit_data, OPS9_TX_DATA_LEN, 50);
	HAL_Delay(20);
	ops9_transmit_data[0] = 0xC5;
	ops9_transmit_data[1] = 0x30;
	HAL_UART_Transmit(&huart6, ops9_transmit_data, OPS9_TX_DATA_LEN, 50);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6, ops9_receive_data, OPS9_RX_DATA_LEN);
	__HAL_UART_DISABLE_IT(&huart6, UART_IT_ERR);  // 关闭错误中断（溢出中断）
}

void usart1_Init(void)
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Emm_V5_rxCmd,Emm_V5_FIFO_SIZE);
}

void OPS9_stop(void)
{
    // 中止USART6的DMA空闲接收，仅停止OPS9的数据解析
    HAL_UART_AbortReceive(&huart6);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == USART6)
	{
		if(verify_CRC8_check_sum(ops9_receive_data, OPS9_RX_DATA_LEN))
		{
			memcpy(&ops9_para.OPS9_Rx_Data, &ops9_receive_data, OPS9_RX_DATA_LEN);
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, ops9_receive_data, OPS9_RX_DATA_LEN);
	}
	else  if (huart->Instance == USART2)
    {
        // 标记一帧数据接收完成
        HWT101_rxFrameFlag = true;
        
        // 重启空闲中断DMA接收，准备下一帧
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)HWT101_rxCmd, HWT101_FIFO_SIZE);
    }
}

void OPS9_Unpack(uint8_t ops9_receive_data[OPS9_RX_DATA_LEN])
{
	if(verify_CRC8_check_sum(ops9_receive_data, OPS9_RX_DATA_LEN))
	{
		memcpy(&ops9_para.OPS9_Rx_Data,&ops9_receive_data, OPS9_RX_DATA_LEN);
	}
}

ops9_para_t *get_ops9_data_point(void)
{
	return &ops9_para;
}
