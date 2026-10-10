#include "usart.h"

/**********************************************************
***	Emm_V5.0步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/

/**
	* @brief   USART1中断函数
	* @param   无
	* @retval  无
	*/
int RxState_1;
int Serial_RxFlag_1;
int pRxPacket_1;

unsigned int Serial_RxPacket_1[10];
int32_t clk_corret=0;

void USART1_IRQHandler(void)
{
	// 错误处理
	if (USART_GetFlagStatus(USART1, USART_FLAG_PE) != RESET) {
			USART_ReceiveData(USART1);
			USART_ClearFlag(USART1, USART_FLAG_PE);
	}
	
	if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET) {
			USART_ReceiveData(USART1);
			USART_ClearFlag(USART1, USART_FLAG_ORE);
	}
	
	if (USART_GetFlagStatus(USART1, USART_FLAG_FE) != RESET) {
			USART_ReceiveData(USART1);
			USART_ClearFlag(USART1, USART_FLAG_FE);
	}

/**********************************************************
***	串口空闲中断
**********************************************************/
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		// 先读SR再读DR，清除IDLE中断
		volatile uint16_t temp;
		temp = USART1->SR; 
		temp = USART1->DR;
		(void)temp;  // 避免编译器警告
		
		// 关闭DMA，防止继续接收
		DMA_Cmd(DMA2_Stream5, DISABLE);
		// 等待DMA完全关闭（可选，但更可靠）
    while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE);
		
		// 计算接收到的数据长度
		Emm_V5_rxCount = Emm_V5_FIFO_SIZE - DMA_GetCurrDataCounter(DMA2_Stream5);
		Emm_V5_rxFrameFlag = true;  // 设置帧接收完成标志
		
		// 清除DMA所有标志
		DMA_ClearFlag(DMA2_Stream5, 
								 DMA_FLAG_TCIF5  // 传输完成标志
							 | DMA_FLAG_HTIF5  // 半传输标志
							 | DMA_FLAG_TEIF5  // 传输错误标志
							 | DMA_FLAG_DMEIF5 // 直接模式错误标志
							 | DMA_FLAG_FEIF5); // FIFO错误标志
		// 重新设置内存地址和传输计数器
		DMA2_Stream5->M0AR = (uint32_t)Emm_V5_rxCmd;
		DMA_SetCurrDataCounter(DMA2_Stream5, Emm_V5_FIFO_SIZE);
		
		// 重新使能DMA
		DMA_Cmd(DMA2_Stream5, ENABLE);
	}
}
void Serial_SendByte_1(uint16_t Byte);
/**
	* @brief   USART发送多个字节
	* @param   无
	* @retval  无
	*/
void usart_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
	__IO uint8_t i = 0;
	
	for(i=0; i < len; i++) { Serial_SendByte_1(cmd[i]); }
}


/**
	* @brief   USART发送一个字节
	* @param   无
	* @retval  无
	*/
void usart_SendByte(uint16_t data)
{
	__IO uint16_t t0 = 0;
	
	USART1->DR = (data & (uint16_t)0x01FF);

	while(!(USART1->SR & USART_FLAG_TXE))
	{
		++t0; if(t0 > 8000)	{	return; }
	}
}

/**
  * 函    数：串口发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial_SendByte_1(uint16_t Byte)
{
	USART_SendData(USART1, Byte);		//将字节数据写入数据寄存器，写入后USART自动生成时序波形
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	//等待发送完成
	/*下次写入数据寄存器会自动清除发送完成标志位，故此循环后，无需清除标志位*/
}



/**
  * 函    数：串口发送一个数组
  * 参    数：Array 要发送数组的首地址
  * 参    数：Length 要发送数组的长度
  * 返 回 值：无
  */
void Serial_SendArray(uint8_t *Array, uint8_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)		//遍历数组
	{
		Serial_SendByte_1(Array[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}
