#include "Key.h"

void Key_Init(void)
{
	// 使能按键端口时钟
	RCC_AHB1PeriphClockCmd(KEY1_RCC | KEY2_RCC | KEY3_RCC | KEY4_RCC | KEY5_RCC, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct;
	// 配置所有按键引脚为输入模式，上拉
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;         // 输入模式
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;         // 上拉
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;       
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;   
	
	// 初始化各个按键引脚
	GPIO_InitStruct.GPIO_Pin = KEY1_PIN;
	GPIO_Init(KEY1_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = KEY2_PIN;
	GPIO_Init(KEY2_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = KEY3_PIN;
	GPIO_Init(KEY3_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = KEY4_PIN;
	GPIO_Init(KEY4_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = KEY5_PIN;
	GPIO_Init(KEY5_PORT, &GPIO_InitStruct);
}
	
uint8_t Key_GetNum(void)
{
	uint8_t KeyNum = 0;
	
	// 检测按键4 (PG0) - 对应键码1
	if(GPIO_ReadInputDataBit(KEY4_PORT, KEY4_PIN) == 0)
	{
		delay_ms(KEY_DELAY_MS);  // 去抖动
		while(GPIO_ReadInputDataBit(KEY4_PORT, KEY4_PIN) == 0);  // 等待按键释放
		delay_ms(KEY_DELAY_MS);  // 去抖动
		KeyNum = KEY4_VALUE;
	}
	// 检测按键5 (PG1) - 对应键码2
	else if(GPIO_ReadInputDataBit(KEY5_PORT, KEY5_PIN) == 0)
	{
		delay_ms(KEY_DELAY_MS);
		while(GPIO_ReadInputDataBit(KEY5_PORT, KEY5_PIN) == 0);
		delay_ms(KEY_DELAY_MS);
		KeyNum = KEY5_VALUE;
	}
	// 检测按键1 (PE7) - 对应键码3
	else if(GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN) == 0)
	{
		delay_ms(KEY_DELAY_MS);
		while(GPIO_ReadInputDataBit(KEY1_PORT, KEY1_PIN) == 0);
		delay_ms(KEY_DELAY_MS);
		KeyNum = KEY1_VALUE;
	}
	// 检测按键2 (PE8) - 对应键码4
	else if(GPIO_ReadInputDataBit(KEY2_PORT, KEY2_PIN) == 0)
	{
		delay_ms(KEY_DELAY_MS);
		while(GPIO_ReadInputDataBit(KEY2_PORT, KEY2_PIN) == 0);
		delay_ms(KEY_DELAY_MS);
		KeyNum = KEY2_VALUE;
	}
	// 检测按键3 (PE9) - 对应键码5
	else if(GPIO_ReadInputDataBit(KEY3_PORT, KEY3_PIN) == 0)
	{
		delay_ms(KEY_DELAY_MS);
		while(GPIO_ReadInputDataBit(KEY3_PORT, KEY3_PIN) == 0);
		delay_ms(KEY_DELAY_MS);
		KeyNum = KEY3_VALUE;
	}
	
	return KeyNum;
}



