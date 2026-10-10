#include "Motor.h"

void PWM_Init_TIM1(u16 Psc, u16 Per) {
	
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;

    // 1. 开启时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); // GPIOE时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);  // TIM1时钟

    // 2. 配置GPIO引脚为复用功能（CH3: PE13, CH4: PE14）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStruct);

    // 3. 设置引脚复用为TIM1
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1); // PE13: TIM1_CH3
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1); // PE14: TIM1_CH4

    // 4. 定时器时基配置
    TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
    TIM_TimeBaseInitStruct.TIM_Period = 8400 - 1;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0; // 高级定时器特有
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);

    // 5. 输出比较配置（CH3/CH4）
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;

    // 初始化CH3/CH4
    TIM_OCInitStruct.TIM_Pulse = 0; // 初始占空比为0
    TIM_OC3Init(TIM1, &TIM_OCInitStruct); // CH3
    TIM_OC4Init(TIM1, &TIM_OCInitStruct); // CH4

    // 6. 使能主输出（MOE）
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    // 7. 使能预装载
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    // 8. 启动定时器
    TIM_Cmd(TIM1, ENABLE);
}

/*电机初始化函数*/
void Motor_Init(void)
{
	PWM_Init_TIM1(0, 7199);

	GPIO_InitTypeDef GPIO_InitStruct;

	// 1. 开启GPIO时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);

	// 2. 配置PC0/PC1为左后轮方向控制引脚
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	// 3. 配置PD3/PD4为右后轮方向控制引脚
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	// 4. 初始化默认电平（安全状态）
	GPIO_ResetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_1);
	GPIO_ResetBits(GPIOD, GPIO_Pin_3 | GPIO_Pin_4);
}

/*限幅函数*/
void Limit(int *motoA, int *motoB)
{
	if (*motoA > PWM_MAX) *motoA = PWM_MAX;
	if (*motoA < PWM_MIN) *motoA = PWM_MIN;

	if (*motoB > PWM_MAX) *motoB = PWM_MAX;
	if (*motoB < PWM_MIN) *motoB = PWM_MIN;
}

/*绝对值函数*/
int GFP_abs(int p)
{
	int q;
	q = p > 0 ? p : (-p);
	return q;
}

/*赋值函数 —— 左后轮（PC0/PC1, TIM1 CH3）*/
void Load_left(int moto)
{
	// 1. 方向控制
	if (moto > 0)
		PCout(0) = 0, PCout(1) = 1;  // 正转
	else if (moto < 0)
		PCout(0) = 1, PCout(1) = 0;  // 反转
	else
		PCout(0) = 0, PCout(1) = 0;  // 停止

	// 2. PWM限幅
	if (moto > PWM_MAX) moto = PWM_MAX;
	else if (moto < PWM_MIN) moto = PWM_MIN;

	TIM_SetCompare3(TIM1, 8400 - GFP_abs(moto));
}

/*赋值函数 —— 右后轮（PD3/PD4, TIM1 CH4）*/
void Load_right(int moto)
{
	// 1. 方向控制
	if (moto > 0)
		PDout(3) = 1, PDout(4) = 0;  // 正转
	else if (moto < 0)
		PDout(3) = 0, PDout(4) = 1;  // 反转
	else
		PDout(3) = 0, PDout(4) = 0;  // 停止

	// 2. PWM限幅
	if (moto > PWM_MAX) moto = PWM_MAX;
	else if (moto < PWM_MIN) moto = PWM_MIN;

	TIM_SetCompare4(TIM1, 8400 - GFP_abs(moto));
}

/*统一赋值函数（左后 + 右后）*/
void Load(int moto_left, int moto_right)
{
	Load_left(moto_left);
	Load_right(moto_right);
}
