#ifndef __SERIAL_H
#define __SERIAL_H

#include "sys.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

/* 串口端口号宏定义：只需改这里即可切换 USART1/2/3 --------------------*/
#define SERIAL_USART                USART3
#define SERIAL_USART_CLK            RCC_APB1Periph_USART3
#define SERIAL_GPIO_CLK             RCC_AHB1Periph_GPIOC   // 注意：F4 系列 GPIO 时钟在 AHB1
#define SERIAL_GPIO_PORT            GPIOC
#define SERIAL_TX_PIN               GPIO_Pin_10
#define SERIAL_RX_PIN               GPIO_Pin_11
#define SERIAL_IRQn                 USART3_IRQn
#define SERIAL_IRQHandler           USART3_IRQHandler
/*--------------------------------------------------------------------*/
#define RXD_LEN      2       //接收数据的个数（功能码1: dx/dy）
#define TYPE_LEN     2       //接收数据的个数（功能码4: 人质类型）
#define Tx_length    4       //发送数据的个数

/* 视觉协议参数*/
#define FRAME_HEADER      0xABCD  // 帧头
#define FRAME_TAIL        0xEF12  // 帧尾
#define FRAME_LEN         14      // 固定帧长

/* 对外接口（与原来保持一致） */
void Serial_Init(void);
void Serial_Vision_SendByte(uint8_t Byte);
void Serial_Vision_SendArray(uint8_t *Array, uint16_t Length);
void Serial_Vision_SendString(char *String);
void Serial_Vision_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Vision_Printf(char *format, ...);
void Serial_Vision_SendPacket(void);

/* 解包函数：由任务侧调用，将最近一帧视觉数据解出 dx/dy 存入 RXD_I */
void Serial_Vision_Unpack(void);

/* 接收缓冲区与标志 */
extern uint8_t  Serial_TxPacket[4];
extern uint8_t  Serial_RxFlag;
extern int RXD_I[RXD_LEN];
extern int TYPE_I[TYPE_LEN];
extern uint8_t Func_Code;



#endif
