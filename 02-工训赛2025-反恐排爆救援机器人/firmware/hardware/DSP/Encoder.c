#include "Encoder.h"                  // Device header

/* TIM3编码器接口配置（PA6/PA7） */
void Encoder_TIM3_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_ICInitTypeDef TIM_ICInitStruct;

    // 1. 开启时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);    // GPIOA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);     // TIM3时钟

    // 2. 配置GPIO引脚（主用PA6/PA7，备用PB0/PB1注释）
    // PA6: TIM3_CH1, PA7: TIM3_CH2
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;        // 复用模式
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;        // 上拉防止浮空
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 3. 引脚复用映射
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);

    // 4. 定时器基础配置
    TIM_TimeBaseInitStruct.TIM_Prescaler = 1-1;
    TIM_TimeBaseInitStruct.TIM_Period = 65536 - 1;       // 16位计数器最大值
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);

    // 5. 编码器接口配置
    TIM_EncoderInterfaceConfig(TIM3, 
                              TIM_EncoderMode_TI12,    // 双通道模式
                              TIM_ICPolarity_Rising,   // 通道1极性
                              TIM_ICPolarity_Rising);  // 通道2极性

    // 6. 输入捕获滤波器配置
		TIM_ICStructInit(&TIM_ICInitStruct);//初始化输入捕获
		TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;				//选择配置定时器通道1
		TIM_ICInitStruct.TIM_ICFilter = 0xF;							//输入滤波器参数，可以过滤信号抖动
		TIM_ICInit(TIM3, &TIM_ICInitStruct);							//将结构体变量交给TIM_ICInit，配置TIM3的输入捕获通道
		TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;				//选择配置定时器通道2
		TIM_ICInitStruct.TIM_ICFilter = 0xF;							//输入滤波器参数，可以过滤信号抖动
		TIM_ICInit(TIM3, &TIM_ICInitStruct);							//将结构体变量交给TIM_ICInit，配置TIM3的输入捕获通道

    // 8. 启动编码器
    TIM_SetCounter(TIM3, 0);
    TIM_Cmd(TIM3, ENABLE);
}

/* TIM4编码器接口配置（PB6/PB7） */
void Encoder_TIM4_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_ICInitTypeDef TIM_ICInitStruct;

    // 1. 开启时钟
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);     // TIM4时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);    // GPIOB时钟

    // 2. 配置GPIO引脚（主用PB6/PB7，备用PB8/PB9注释）
    // PB6: TIM4_CH1, PB7: TIM4_CH2
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 3. 引脚复用映射
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);

    // 4. 定时器基础配置
    TIM_TimeBaseInitStruct.TIM_Prescaler = 1-1;
    TIM_TimeBaseInitStruct.TIM_Period = 65536-1;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);

    // 5. 编码器接口配置
    TIM_EncoderInterfaceConfig(TIM4,
                              TIM_EncoderMode_TI12,
                              TIM_ICPolarity_Rising,
                              TIM_ICPolarity_Rising);

    // 6. 输入捕获滤波器
		TIM_ICStructInit(&TIM_ICInitStruct);//初始化输入捕获
    TIM_ICStructInit(&TIM_ICInitStruct);//初始化输入捕获
		TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;				//选择配置定时器通道1
		TIM_ICInitStruct.TIM_ICFilter = 0xF;							//输入滤波器参数，可以过滤信号抖动
		TIM_ICInit(TIM4, &TIM_ICInitStruct);							//将结构体变量交给TIM_ICInit，配置TIM3的输入捕获通道
		TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;				//选择配置定时器通道2
		TIM_ICInitStruct.TIM_ICFilter = 0xF;							//输入滤波器参数，可以过滤信号抖动
		TIM_ICInit(TIM4, &TIM_ICInitStruct);							//将结构体变量交给TIM_ICInit，配置TIM3的输入捕获通道

    // 8. 启动编码器
    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);
}

/* TIM2编码器接口配置（PA15/PB3） */
void Encoder_TIM2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_ICInitTypeDef TIM_ICInitStruct;
	
    // 1. 开启时钟（使用位操作，避免宏定义问题）
    RCC->APB1ENR |= (1 << 0);  // 第0位：TIM2时钟使能
    RCC->AHB1ENR |= (1 << 0) | (1 << 1);  // 第0位：GPIOA，第1位：GPIOB时钟使能
    RCC->APB2ENR |= (1 << 14); // 第14位：SYSCFG时钟使能
	
    // 2. 释放JTAG引脚（PA15和PB3默认用于JTAG，需重映射为普通IO）
    RCC->APB2ENR |= (1 << 22);  // 使能DBGMCU时钟
    DBGMCU->CR = (DBGMCU->CR & ~0x0F) | 0x02;  // 仅保留SWD调试，释放JTAG引脚
	
    // 3. 配置GPIO引脚（PA15=TIM2_CH1，PB3=TIM2_CH2）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
	
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 4. 引脚复用映射到TIM2
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);  // PA15 -> TIM2_CH1
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);   // PB3 -> TIM2_CH2

    // 5. 定时器基础配置
    TIM_TimeBaseInitStruct.TIM_Prescaler = 0;                 // 不分频
    TIM_TimeBaseInitStruct.TIM_Period = 65535;                // 最大计数范围
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

    // 6. 编码器接口配置
    TIM_EncoderInterfaceConfig(TIM2,
                              TIM_EncoderMode_TI12,
                              TIM_ICPolarity_Rising,
                              TIM_ICPolarity_Rising);

    // 7. 输入捕获滤波器配置
    TIM_ICStructInit(&TIM_ICInitStruct);  // 初始化结构体
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStruct.TIM_ICFilter = 0xF;
    TIM_ICInit(TIM2, &TIM_ICInitStruct);
	
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStruct.TIM_ICFilter = 0xF;
    TIM_ICInit(TIM2, &TIM_ICInitStruct);

    // 8. 启动编码器
    TIM_SetCounter(TIM2, 0);
    TIM_Cmd(TIM2, ENABLE);
}
/* TIM5编码器接口配置（PA0/PA1） */
void Encoder_TIM5_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_ICInitTypeDef TIM_ICInitStruct;

    // 1. 开启时钟
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);     // TIM4时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);    // GPIOB时钟

    // 2. 配置GPIO引脚（主用PB6/PB7，备用PB8/PB9注释）
    // PB6: TIM4_CH1, PB7: TIM4_CH2
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 3. 引脚复用映射
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);

    // 4. 定时器基础配置
    TIM_TimeBaseInitStruct.TIM_Prescaler = 1-1;
    TIM_TimeBaseInitStruct.TIM_Period = 65536-1;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStruct);

    // 5. 编码器接口配置
    TIM_EncoderInterfaceConfig(TIM5,
                              TIM_EncoderMode_TI12,
                              TIM_ICPolarity_Rising,
                              TIM_ICPolarity_Rising);

    // 6. 输入捕获滤波器
		TIM_ICStructInit(&TIM_ICInitStruct);//初始化输入捕获
    TIM_ICStructInit(&TIM_ICInitStruct);//初始化输入捕获
		TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;				//选择配置定时器通道1
		TIM_ICInitStruct.TIM_ICFilter = 0xF;							//输入滤波器参数，可以过滤信号抖动
		TIM_ICInit(TIM5, &TIM_ICInitStruct);							//将结构体变量交给TIM_ICInit，配置TIM3的输入捕获通道
		TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;				//选择配置定时器通道2
		TIM_ICInitStruct.TIM_ICFilter = 0xF;							//输入滤波器参数，可以过滤信号抖动
		TIM_ICInit(TIM5, &TIM_ICInitStruct);							//将结构体变量交给TIM_ICInit，配置TIM3的输入捕获通道

    // 8. 启动编码器
    TIM_SetCounter(TIM5, 0);
    TIM_Cmd(TIM5, ENABLE);
}


/**
  * @brief  定时器6中断初始化
  * @param  无
  * @retval 无
  */
void Timer6_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    // 1. 使能时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);  // TIM6挂载在APB1总线

    // 2. 配置时基单元
    // 假设系统时钟168MHz，TIM6时钟=APB1时钟*2=84MHz
    TIM_TimeBaseInitStruct.TIM_Prescaler = 8400 - 1;      // 预分频值 8400分频
    TIM_TimeBaseInitStruct.TIM_Period = 50 - 1;           // 自动重装载值（生成5ms中断）
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStruct);

    // 3. 中断配置
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);             // 使能更新中断
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);            // 清除中断标志

    // 4. NVIC配置
    NVIC_InitStruct.NVIC_IRQChannel = TIM6_DAC_IRQn;      // TIM6与DAC共享中断通道
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1; // 抢占优先级
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;       // 响应优先级
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    // 5. 启动定时器
    TIM_Cmd(TIM6, ENABLE);
}

/**
  * @brief  定时器6中断服务函数
  */
//void TIM6_DAC_IRQHandler(void)
//{
//    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
//    {
//        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
//        
//        // 用户中断处理代码
//        // GPIO_ToggleBits(GPIOD, GPIO_Pin_12); // 示例：翻转LED
//    }
//}




/**********************
编码器
速度读取函数
入口参数：定时器
**********************/
int value_2=0;
int value_3=0;
int encoder_right=0;
int encoder_left=0;
int Read_Speed(int TIMx)
{
	int value_1;
	switch(TIMx)
	{
		case 3:
			value_1=(short)TIM_GetCounter(TIM3);
//			encoder_right=encoder_right+value_1;
			TIM_SetCounter(TIM3,0);

		break;//IF是定时器2，1.采集编码器的计数值并保存。2.将定时器的计数值清零。
		case 4:
			value_1=-(short)TIM_GetCounter(TIM4);
//			encoder_left=encoder_left+value_1;
			TIM_SetCounter(TIM4,0);

		break;
		case 5:
			value_1=-(short)TIM_GetCounter(TIM5);
//			encoder_right=encoder_right+value_1;
			TIM_SetCounter(TIM5,0);

		break;//IF是定时器2，1.采集编码器的计数值并保存。2.将定时器的计数值清零。
		case 2:
			value_1=(short)TIM_GetCounter(TIM2);
//			encoder_left=encoder_left+value_1;
			TIM_SetCounter(TIM2,0);

		break;
		default:
			value_1=0;
	}
	return value_1;
}



