#include "TJC.h"
#include "Chassis_task.h"

/* ========== 基础发送 ========== */

/**
 * 发送单字节
 */
void USART5_SendByte(uint8_t data)
{
    HAL_UART_Transmit(&huart5, &data, 1, 10);
}

/**
 * 发送多字节数据
 */
void USART5_SendData(uint8_t *data, uint16_t len)
{
    if (len > 0)
    {
        HAL_UART_Transmit(&huart5, data, len, HAL_MAX_DELAY);
    }
}

/* ========== 底盘通信 ========== */

/**
 * 初始化UART5接收中断
 * 在 main.c 中 MX_UART5_Init() 之后调用
 */
void chassis_uart_init(void)
{
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);    // 使能UART5接收中断

    /* NVIC配置（CubeMX可能未生成UART5的中断配置） */
    HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
}

/**
 * 由 UART5 中断服务函数调用
 * 收到 0xB0=到位 或 0xB1=超时 → 通知 Main_task 状态机
 */
void chassis_uart_rx_byte(uint8_t data)
{
    if (data == 0xB0 || data == 0xB1)
    {
        chassis_rsp_set(data);  // 写入应答缓冲区
    }
}
