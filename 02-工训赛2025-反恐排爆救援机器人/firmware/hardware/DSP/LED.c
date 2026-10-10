#include "LED.h"

void LED_Init(void)
{
	RCC_AHB1PeriphClockCmd(LED_RCC,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = LED_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;     //配置输出模式
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	
    GPIO_Init(LED_PORT, &GPIO_InitStruct);

    GPIO_ResetBits(LED_PORT, LED_PIN);
}

void LED_TURN(void)
{
	GPIO_ToggleBits(LED_PORT,LED_PIN);
}

/**
 * @brief   设置LED为高电平
 * @param   无
 * @retval  无
 */
void LED_ON(void)
{
	GPIO_SetBits(LED_PORT, LED_PIN);
}

/**
 * @brief   设置LED为低电平
 * @param   无
 * @retval  无
 */
void LED_OFF(void)
{
	GPIO_ResetBits(LED_PORT, LED_PIN);
}
