#include "Encoder.h"                  // Device header

/* TIM2 Encoder: PA15/PB3 — 左后轮 */
void Encoder_TIM2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_ICInitTypeDef TIM_ICInitStruct;

    // 1. Clock
    RCC->APB1ENR |= (1 << 0);  // TIM2
    RCC->AHB1ENR |= (1 << 0) | (1 << 1);  // GPIOA + GPIOB
    RCC->APB2ENR |= (1 << 14); // SYSCFG

    // 2. Release JTAG (keep SWD only)
    RCC->APB2ENR |= (1 << 22);  // DBGMCU
    DBGMCU->CR = (DBGMCU->CR & ~0x0F) | 0x02;

    // 3. GPIO: PA15=TIM2_CH1, PB3=TIM2_CH2
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 4. AF mapping
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);

    // 5. Timer base
    TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
    TIM_TimeBaseInitStruct.TIM_Period = 65535;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

    // 6. Encoder interface
    TIM_EncoderInterfaceConfig(TIM2,
                              TIM_EncoderMode_TI12,
                              TIM_ICPolarity_Rising,
                              TIM_ICPolarity_Rising);

    // 7. Input capture filter
    TIM_ICStructInit(&TIM_ICInitStruct);
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStruct.TIM_ICFilter = 0xF;
    TIM_ICInit(TIM2, &TIM_ICInitStruct);
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStruct.TIM_ICFilter = 0xF;
    TIM_ICInit(TIM2, &TIM_ICInitStruct);

    // 8. Start
    TIM_SetCounter(TIM2, 0);
    TIM_Cmd(TIM2, ENABLE);
}

/* TIM5 Encoder: PA0/PA1 — 右后轮 */
void Encoder_TIM5_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_ICInitTypeDef TIM_ICInitStruct;

    // 1. Clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // 2. GPIO: PA0=TIM5_CH1, PA1=TIM5_CH2
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 3. AF mapping
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);

    // 4. Timer base
    TIM_TimeBaseInitStruct.TIM_Prescaler = 1 - 1;
    TIM_TimeBaseInitStruct.TIM_Period = 65536 - 1;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStruct);

    // 5. Encoder interface
    TIM_EncoderInterfaceConfig(TIM5,
                              TIM_EncoderMode_TI12,
                              TIM_ICPolarity_Rising,
                              TIM_ICPolarity_Rising);

    // 6. Input capture filter
    TIM_ICStructInit(&TIM_ICInitStruct);
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStruct.TIM_ICFilter = 0xF;
    TIM_ICInit(TIM5, &TIM_ICInitStruct);
    TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStruct.TIM_ICFilter = 0xF;
    TIM_ICInit(TIM5, &TIM_ICInitStruct);

    // 7. Start
    TIM_SetCounter(TIM5, 0);
    TIM_Cmd(TIM5, ENABLE);
}

/* Timer6: 5ms control loop interrupt */
void Timer6_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    // 84MHz / 8400 / 50 = 200Hz = 5ms
    TIM_TimeBaseInitStruct.TIM_Prescaler = 8400 - 1;
    TIM_TimeBaseInitStruct.TIM_Period = 50 - 1;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStruct);

    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);

    NVIC_InitStruct.NVIC_IRQChannel = TIM6_DAC_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    TIM_Cmd(TIM6, ENABLE);
}

/**********************
 Speed read function
 Reads encoder delta over 5ms, then resets counter
**********************/
int Read_Speed(int TIMx)
{
    int value_1;
    switch (TIMx)
    {
        case 2:
            value_1 = (short)TIM_GetCounter(TIM2);
            TIM_SetCounter(TIM2, 0);
            break;
        case 5:
            value_1 = -(short)TIM_GetCounter(TIM5);
            TIM_SetCounter(TIM5, 0);
            break;
        default:
            value_1 = 0;
    }
    return value_1;
}
