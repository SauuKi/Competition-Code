#include "PWM.h"


void PWM_Init_TIM8(u16 arr, u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    /**********************************************************
    * 1. 时钟使能（无错误，保留）
    **********************************************************/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);  // TIM8（APB2）时钟使能
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); // GPIOC时钟使能

    /**********************************************************
    * 2. GPIO初始化（修复：添加上拉配置，避免浮空）
    **********************************************************/
    // GPIOC8(TIM8_CH3)、GPIOC9(TIM8_CH4)    
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        // 复用模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      // 推挽输出（PWM需要）
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;  // 高速
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        // 上拉（避免干扰，关键修复）

    // 其他参数与上面一致，无需重复赋值
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /**********************************************************
    * 3. 引脚复用映射（无错误，保留）
    **********************************************************/
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);  // PC8→TIM8_CH3
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);  // PC9→TIM8_CH4

    /**********************************************************
    * 4. 时间基数初始化（修复：使用函数参数psc，删除硬编码）
    **********************************************************/
    TIM_TimeBaseStructure.TIM_Period = arr - 1;                 // 自动重装载值（用户传入）
    TIM_TimeBaseStructure.TIM_Prescaler = psc-1;                  // 预分频器（使用用户传入的psc，关键修复）
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     // 时钟分频（普通PWM用DIV1即可）
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    /**********************************************************
    * 5. PWM输出配置（删除冗余的OC预装载使能）
    **********************************************************/
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             // PWM模式1（高电平有效）
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 主输出使能
    TIM_OCInitStructure.TIM_Pulse = 0;                            // 初始占空比0（用户后续可修改）
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     // 输出极性：高电平有效

    // 高级定时器必须使能主输出（MOE位），无错误
    TIM_CtrlPWMOutputs(TIM8, ENABLE);

    // 通道3配置
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

    // 通道4配置
    TIM_OC4Init(TIM8, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);

    /**********************************************************
    * 6. 使能定时器（无错误，保留）
    **********************************************************/
    TIM_ARRPreloadConfig(TIM8, ENABLE);  // 使能自动重装载预装载
    TIM_Cmd(TIM8, ENABLE);               // 使能TIM8计数器
}

void PWM_Init_TIM9(u16 arr,u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_2 | GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // 复用推挽输出
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure); // 初始化GPIO
	
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM9);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM9);

    // 初始化TIM8
    TIM_TimeBaseStructure.TIM_Period = arr - 1;                 // 设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = psc - 1;              // 设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     // 死区时间
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // TIM向上计数模式
    TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);             // 根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    /*--------------------输出比较结构体初始化-------------------*/
    // 占空比配置

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             // 配置为PWM模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 输出使能
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 输出通道电平极性配置
    TIM_CtrlPWMOutputs(TIM9, ENABLE);                         // MOE 主输出使能

    // 输出比较通道 1       
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC1Init(TIM9, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);

    // 输出比较通道 2      
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC2Init(TIM9, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);

    // 使能计数器
    TIM_ARRPreloadConfig(TIM9, ENABLE);               // 使能TIMx在ARR上的预装载寄存器
		TIM_OC1PreloadConfig(TIM9,TIM_OCPreload_Enable);/*输出比较预装载使能*/
		TIM_OC2PreloadConfig(TIM9,TIM_OCPreload_Enable);
    TIM_CtrlPWMOutputs(TIM9, ENABLE);
    TIM_Cmd(TIM9, ENABLE);
}

void PWM_Init_TIM12(u16 arr,u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_14 | GPIO_Pin_15;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // 复用推挽输出
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure); // 初始化GPIO
	
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_TIM12);
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_TIM12);

    // 初始化TIM12
    TIM_TimeBaseStructure.TIM_Period = arr - 1;                 // 设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = psc - 1;              // 设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     // 死区时间
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // TIM向上计数模式
    TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);             // 根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    /*--------------------输出比较结构体初始化-------------------*/
    // 占空比配置

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             // 配置为PWM模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 输出使能
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 输出通道电平极性配置
    TIM_CtrlPWMOutputs(TIM12, ENABLE);                         // MOE 主输出使能

    // 输出比较通道 1       
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC1Init(TIM12, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);

    // 输出比较通道 2      
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC2Init(TIM12, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM12, TIM_OCPreload_Enable);

    // 使能计数器
    TIM_ARRPreloadConfig(TIM12, ENABLE);               // 使能TIMx在ARR上的预装载寄存器
		TIM_OC1PreloadConfig(TIM12,TIM_OCPreload_Enable);/*输出比较预装载使能*/
		TIM_OC2PreloadConfig(TIM12,TIM_OCPreload_Enable);
    TIM_CtrlPWMOutputs(TIM12, ENABLE);
    TIM_Cmd(TIM12, ENABLE);
}
void PWM_Init_TIM10(u16 arr, u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // 1. 时钟使能（正确，保留）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);  // TIM10在APB2总线
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  // GPIOB时钟

    // 2. GPIO初始化（修复：添加上拉配置，避免浮空）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;              // TIM10_CH1对应PB8
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           // 复用模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         // 推挽输出（PWM需要）
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;     // 高速
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;           // 上拉（关键修复：避免浮空）
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 3. 复用映射（确认AF3，TIM10的正确复用功能）
    // 注：STM32F4中GPIO_AF_TIM10应定义为0x03（AF3），若宏错误需手动替换为0x03
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM10);

    // 4. 定时器时基初始化（修复：使用传入的psc，之前的问题说明忽略此点但建议修复）
    TIM_TimeBaseStructure.TIM_Period = arr - 1;            // 自动重装载值
    TIM_TimeBaseStructure.TIM_Prescaler = psc-1;             // 预分频器（建议使用参数）
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;// 时钟分频（无用，默认即可）
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);

    // 5. PWM输出配置（删除高级定时器的冗余函数）
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;      // PWM模式1（高电平有效）
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 输出使能
    TIM_OCInitStructure.TIM_Pulse = 0;                     // 初始占空比0（后续需修改）
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 输出极性高

    // 配置通道1（TIM10只有1个通道）
    TIM_OC1Init(TIM10, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable);     // 使能输出比较预装载

    // 6. 使能定时器（删除冗余的TIM_CtrlPWMOutputs）
    TIM_ARRPreloadConfig(TIM10, ENABLE);                   // 使能自动重装载预装载
    TIM_Cmd(TIM10, ENABLE);                                // 启动定时器
}

void PWM_Init_TIM11(u16 arr, u16 psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // 1. 时钟使能（正确，保留）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);  // TIM10在APB2总线
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  // GPIOB时钟

    // 2. GPIO初始化（修复：添加上拉配置，避免浮空）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;              // TIM10_CH1对应PB8
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           // 复用模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         // 推挽输出（PWM需要）
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;     // 高速
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;           // 上拉（关键修复：避免浮空）
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 3. 复用映射（确认AF3，TIM10的正确复用功能）
    // 注：STM32F4中GPIO_AF_TIM10应定义为0x03（AF3），若宏错误需手动替换为0x03
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM11);

    // 4. 定时器时基初始化（修复：使用传入的psc，之前的问题说明忽略此点但建议修复）
    TIM_TimeBaseStructure.TIM_Period = arr - 1;            // 自动重装载值
    TIM_TimeBaseStructure.TIM_Prescaler = psc-1;             // 预分频器（建议使用参数）
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;// 时钟分频（无用，默认即可）
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInit(TIM11, &TIM_TimeBaseStructure);

    // 5. PWM输出配置（删除高级定时器的冗余函数）
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;      // PWM模式1（高电平有效）
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 输出使能
    TIM_OCInitStructure.TIM_Pulse = 0;                     // 初始占空比0（后续需修改）
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // 输出极性高

    // 配置通道1（TIM10只有1个通道）
    TIM_OC1Init(TIM11, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM11, TIM_OCPreload_Enable);     // 使能输出比较预装载

    // 6. 使能定时器（删除冗余的TIM_CtrlPWMOutputs）
    TIM_ARRPreloadConfig(TIM11, ENABLE);                   // 使能自动重装载预装载
    TIM_Cmd(TIM11, ENABLE);                                // 启动定时器
}

/**
  * 函    数：PWM设置CCR
  * 参    数：Compare 要写入的CCR的值，范围：0~100
  * 返 回 值：无
  * 注意事项：CCR和ARR共同决定占空比，此函数仅设置CCR的值，并不直接是占空比
  *           占空比Duty = CCR / (ARR + 1)
  */
void PWM_SetCompare1(uint16_t Compare)
{
	TIM_SetCompare3(TIM8, Compare);		//设置CCR2的值
}

void PWM_SetCompare2(uint16_t Compare)
{
	TIM_SetCompare4(TIM8, Compare);		//设置CCR2的值
}

void PWM_SetCompare3(uint16_t Compare)
{
	TIM_SetCompare1(TIM9, Compare);		//设置CCR2的值
}

void PWM_SetCompare4(uint16_t Compare)
{
	TIM_SetCompare2(TIM9, Compare);		//设置CCR2的值
}

void PWM_SetCompare5(uint16_t Compare)
{
	TIM_SetCompare1(TIM10, Compare);		//设置CCR2的值
}

void PWM_SetCompare6(uint16_t Compare)
{
	TIM_SetCompare1(TIM11, Compare);		//设置CCR2的值
}

void PWM_SetCompare7(uint16_t Compare)
{
	TIM_SetCompare1(TIM12, Compare);		//设置CCR2的值
}

void PWM_SetCompare8(uint16_t Compare)
{
	TIM_SetCompare2(TIM12, Compare);		//设置CCR2的值
}
