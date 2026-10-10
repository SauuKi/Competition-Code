#ifndef __USART_H
#define __USART_H

#include "sys.h"

/**********************************************************
***	Emm_V5.0步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/


extern int32_t clk_corret;
void usart_SendCmd(__IO uint8_t *cmd, uint8_t len);
void usart_SendByte(uint16_t data);
void Serial_SendArray(uint8_t *Array, uint8_t Length);
extern unsigned int Serial_RxPacket_1[10];
#endif
