#ifndef __HWT101_H
#define __HWT101_H

#include "sys.h"
#include <stdbool.h>

#define 	HWT101_FIFO_SIZE   128

extern __IO bool HWT101_rxFrameFlag;
extern __IO uint8_t HWT101_rxCmd[HWT101_FIFO_SIZE];
extern __IO uint8_t HWT101_rxCount;

void HWT101_nvic_init(void);
void HWT101_clock_init(void);
void HWT101_usart_init(void);
void HWT101_board_init(void);//初始化函数
void MPUData_processing(void);//数据处理函数
void HWT101(void);
extern uint8_t Serial_RxFla_Data_processing;			//定义接收边框坐标数据包标志位
#endif

