#ifndef __BOARD_H
#define __BOARD_H

#include "sys.h"

/**********************************************************
***	Emm_V5.0步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/

#define 	Emm_V5_FIFO_SIZE   128

extern __IO bool Emm_V5_rxFrameFlag;
extern __IO uint8_t Emm_V5_rxCmd[Emm_V5_FIFO_SIZE];
extern __IO uint8_t Emm_V5_rxCount;

void nvic_init(void);
void clock_init(void);
void usart_init(void);
void board_init(void);

#endif
