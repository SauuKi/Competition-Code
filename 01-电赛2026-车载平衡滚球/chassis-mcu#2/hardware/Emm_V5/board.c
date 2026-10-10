#include "board.h"

/**********************************************************
***	Emm_V5.0步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/

__IO bool Emm_V5_rxFrameFlag = false;
__IO uint8_t Emm_V5_rxCmd[Emm_V5_FIFO_SIZE] = {0};// DMA接收缓冲区
__IO uint8_t Emm_V5_rxCount = 0;
/**
	* @brief   配置NVIC控制器
	* @param   无
	* @retval  无
	*/
void nvic_init(void)
{	
	// 4bit抢占优先级位
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}

/**
	*	@brief		外设时钟初始化
	*	@param		无
	*	@retval		无
	*/
void clock_init(void)
{
	// 使能GPIOA、AFIO外设时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	//使能GPIOA时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); 

	// 使能USART1外设时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	// 使能DMA2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	

}


/**
	* @brief   初始化USART
	* @param   无
	* @retval  无
	*/
void usart_init(void)
{
	
	clock_init();
	
/**********************************************************
***	初始化USART1引脚
**********************************************************/
	GPIO_InitTypeDef  GPIO_InitStructure;
	// PA9 - USART1_TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// PA10 - USART1_RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;					/* 浮空输入 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

/**********************************************************
***	初始化USART1
**********************************************************/
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 921600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART1, &USART_InitStructure);

/**********************************************************
	***	初始化DMA (USART1_RX使用DMA2_Stream5_Channel4)
	**********************************************************/
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA2_Stream5);
	
	// 等待DMA流关闭
	while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE);
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;  // USART1_RX对应DMA通道4
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;  // 外设地址(数据寄存器)
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Emm_V5_rxCmd;  // 内存地址(接收缓冲区)
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;  // 外设到内存方向
	DMA_InitStructure.DMA_BufferSize = Emm_V5_FIFO_SIZE;  // 缓冲区大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  // 外设地址不递增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  // 内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  // 字节传输
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;  // 字节传输
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  // 非循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;  // 高优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream5, &DMA_InitStructure);
	
/**********************************************************
***	配置中断
**********************************************************/	
	nvic_init();
	
/**********************************************************
***	清除USART1中断
**********************************************************/
	USART1->SR; 
	USART1->DR;
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);

/**********************************************************
	***	使能DMA接收和UART中断
	**********************************************************/
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);  // 使能DMA接收
	DMA_Cmd(DMA2_Stream5, ENABLE);  // 使能DMA流
	
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);  // 使能空闲中断

/**********************************************************
***	使能USART1
**********************************************************/
	USART_Cmd(USART1, ENABLE);
}

/**
	*	@brief		板载初始化
	*	@param		无
	*	@retval		无
	*/
void board_init(void)
{
	usart_init();
}














