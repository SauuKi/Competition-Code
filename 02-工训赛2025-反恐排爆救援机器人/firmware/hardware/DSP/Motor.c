#include "Motor.h"

void PWM_Init_TIM1(u16 Psc, u16 Per) {
	
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;

    // 1. 开启时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); // GPIOE时钟
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // GPIOE时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);  // TIM1时钟

    // 2. 配置GPIO引脚为复用功能
    GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStruct);

		// 2. 配置GPIO引脚为复用功能
    GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_8;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
	
    // 3. 设置引脚复用为TIM1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);  // PA8: TIM1_CH1
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1); // PE11: TIM1_CH2
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1); // PE13: TIM1_CH3
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1); // PE14: TIM1_CH4

    // 4. 定时器时基配置
    TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
    TIM_TimeBaseInitStruct.TIM_Period = 8400-1;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0; // 高级定时器特有
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);

    // 5. 输出比较配置（四个通道）
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
//    TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Enable; // 互补通道（可选）
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
//    TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;     // 互补极性（可选）
//    TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;    // 空闲状态
//    TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    // 初始化所有四个通道
    TIM_OCInitStruct.TIM_Pulse = 0; // 初始占空比为0
    TIM_OC1Init(TIM1, &TIM_OCInitStruct); // CH1
    TIM_OC2Init(TIM1, &TIM_OCInitStruct); // CH2
    TIM_OC3Init(TIM1, &TIM_OCInitStruct); // CH3
    TIM_OC4Init(TIM1, &TIM_OCInitStruct); // CH4

		// 7. 使能主输出（MOE）
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    // 6. 使能预装载
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    // 8. 启动定时器
    TIM_Cmd(TIM1, ENABLE);
}

/*电机初始化函数*/
/* 电机初始化函数（STM32F407VET6适用）*/
void Motor_Init(void)
{
	PWM_Init_TIM1(0,7199);
	
	GPIO_InitTypeDef GPIO_InitStruct ;
	
	// 1. 开启GPIO时钟（F4使用AHB1总线）
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);

	// 2. 配置PC0-3为电机控制引脚
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;        // 通用输出模式
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;       // 推挽输出
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;   // F4支持更高速度
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;       // 禁用上下拉
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	// 3. 配置PD3-6为电机控制引脚（注意PD3可能用于SWD调试）
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3  | GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;        // 通用输出模式
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;       // 推挽输出
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;   // F4支持更高速度
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;       // 禁用上下拉
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	// 4. 初始化默认电平（安全状态）
	GPIO_ResetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3| GPIO_Pin_4 | GPIO_Pin_5);
	GPIO_ResetBits(GPIOD, GPIO_Pin_3 | GPIO_Pin_4);

}

/*限幅函数*/
void Limit(int *motoA,int *motoB)
{
	if(*motoA>PWM_MAX)*motoA=PWM_MAX;
	if(*motoA<PWM_MIN)*motoA=PWM_MIN;
	
	if(*motoB>PWM_MAX)*motoB=PWM_MAX;
	if(*motoB<PWM_MIN)*motoB=PWM_MIN;
}

/*绝对值函数*/
int GFP_abs(int p)
{
	int q;
	q=p>0?p:(-p);
	return q;
}

/*问题：	（1）占空比小转速高，占空比小转速低，相反
				（2）占空比达到4200以上才有力度，因此底层减去4200
*/

/*赋值函数*/
/*入口参数：PID运算完成后的最终PWM值*/
void Load_right(int moto1,int moto2)//moto1=-200：反转200个脉冲
{
	//1.研究正负号，对应正反转
	if(moto1>0)	PCout(4)=1,PCout(5)=0;//正转
	else if(moto1<0) PCout(4)=0,PCout(5)=1;//反转
	else if(moto1==0) PCout(4)=0,PCout(5)=0;//反转
	//2.研究PWM值
	if(moto1>PWM_MAX)moto1=PWM_MAX;
	else if(moto1<PWM_MIN)moto1=PWM_MIN;
	TIM_SetCompare2(TIM1,8400-GFP_abs(moto1));
	
	if(moto2>0)	PDout(3)=1,PDout(4)=0;
	else if(moto2<0) PDout(3)=0,PDout(4)=1;	
	else if(moto2==0) PDout(3)=0,PDout(4)=0;
	
	if(moto2>PWM_MAX)moto2=PWM_MAX;
	else if(moto2<PWM_MIN)moto2=PWM_MIN;
	
	TIM_SetCompare4(TIM1,8400-GFP_abs(moto2));
	
	if(moto1==0&&moto2==0)
	{
		PCout(4)=0,PCout(5)=0;//正转
		PDout(3)=0,PDout(4)=0;
	}
}

/*赋值函数*/
/*入口参数：PID运算完成后的最终PWM值*/
void Load_left(int moto1,int moto2)//moto1=-200：反转200个脉冲
{
	//1.研究正负号，对应正反转
	if(moto1>0)	PCout(2)=0,PCout(3)=1;//正转
	else if(moto1<0) PCout(2)=1,PCout(3)=0;//反转
	else if(moto1==0) PCout(2)=0,PCout(3)=0;//反转
	//2.研究PWM值
	if(moto1>PWM_MAX)moto1=PWM_MAX;
	else if(moto1<PWM_MIN)moto1=PWM_MIN;
	TIM_SetCompare1(TIM1,8400-GFP_abs(moto1));
	
	if(moto2>0)	PCout(0)=0,PCout(1)=1;
	else if(moto2<0)	PCout(0)=1,PCout(1)=0;
	else if(moto2==0)	PCout(0)=0,PCout(1)=0;
	if(moto2>PWM_MAX)moto2=PWM_MAX;
	else if(moto2<PWM_MIN)moto2=PWM_MIN;
	TIM_SetCompare3(TIM1,8400-GFP_abs(moto2));
	
	if(moto1==0&&moto2==0)
	{
		PCout(2)=0,PCout(3)=0;//正转
		PCout(0)=0,PCout(1)=0;
	}
}

void Load(int moto_1,int moto_2,int moto_3,int moto_4)
{
	Load_left(moto_1,moto_2);
	Load_right(moto_3,moto_4);
}
