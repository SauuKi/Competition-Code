#include "Servo.h"

/* 
 * TIM8->168MHz->168-1->1 MHz->1us 
 */
#define US_TO_CNT(us)  ((us))

static void GPIO_Config(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(SERVO_GPIOC_CLK, ENABLE);

    /* PC6 PC7 PC8 -> AF3 (TIM8_CH1 CH2 CH3) */
    gpio.GPIO_Pin = SERVO_CH1_PIN | SERVO_CH2_PIN | SERVO_CH3_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(SERVO_GPIOC_PORT, &gpio);
    GPIO_PinAFConfig(SERVO_GPIOC_PORT, GPIO_PinSource6, GPIO_AF_TIM8);
    GPIO_PinAFConfig(SERVO_GPIOC_PORT, GPIO_PinSource7, GPIO_AF_TIM8);
    GPIO_PinAFConfig(SERVO_GPIOC_PORT, GPIO_PinSource8, GPIO_AF_TIM8);
}

static void TIM_Config(void)
{
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;

    RCC_APB2PeriphClockCmd(SERVO_TIM8_CLK, ENABLE);

    /* 统一定时器1 MHz 周期20 000 对应 20 ms 周期 */
    tim.TIM_Prescaler = 168 - 1;         /* 168 MHz / 168 = 1 MHz */
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 20000 - 1;          /* 20 ms */
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0;       /* TIM8需要设置重复计数器 */

    TIM_TimeBaseInit(TIM8, &tim);

    /* 统一 PWM1 模式 */
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 0;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;  /* TIM8需要设置互补极性 */
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC1Init(TIM8, &oc);   /* PC6 */
    TIM_OC2Init(TIM8, &oc);   /* PC7 */
    TIM_OC3Init(TIM8, &oc);   /* PC8 */

    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM8, ENABLE);

    /* TIM8需要启动主输出 */
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    TIM_Cmd(TIM8, ENABLE);
}

/* -------------- 初始化 -------------- */
void SERVO_Init(void)
{
    GPIO_Config();
    TIM_Config();
}

void Servo_SetPWM(uint8_t ch, uint32_t pulse)
{
    if (pulse < MinPulse) pulse = MinPulse;
    if (pulse > MaxPulse) pulse = MaxPulse;

    switch (ch)
    {
        case 1: TIM_SetCompare1(TIM8, US_TO_CNT(pulse)); break;
        case 2: TIM_SetCompare2(TIM8, US_TO_CNT(pulse)); break;
        case 3: TIM_SetCompare3(TIM8, US_TO_CNT(pulse)); break;
    }
}

void Servo_SetAngle(uint8_t ch, float angle)
{
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 270.0f) angle = 270.0f;

    uint32_t pulse = (uint32_t)(500.0f + angle * 2000.0f / 270.0f); /* 500~2500 ��s */
    Servo_SetPWM(ch, pulse);
}
