#ifndef __TJC_H
#define __TJC_H

#include "sys.h"
#include "usart.h"

/* ========== 基础收发 ========== */
void USART5_SendByte(uint8_t data);
void USART5_SendData(uint8_t *data, uint16_t len);

/* ========== 底盘指令接口 ========== */
void chassis_uart_init(void);            // 初始化UART5接收
void chassis_uart_rx_byte(uint8_t data); // 由UART5 RX中断调用

#endif
