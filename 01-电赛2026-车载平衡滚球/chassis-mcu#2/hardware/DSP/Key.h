#ifndef __KEY_H
#define __KEY_H

#include "sys.h"

// 按键引脚宏定义 - 便于移植和修改
#define KEY1_PORT           GPIOE
#define KEY1_PIN            GPIO_Pin_7
#define KEY1_RCC            RCC_AHB1Periph_GPIOE

#define KEY2_PORT           GPIOE
#define KEY2_PIN            GPIO_Pin_8
#define KEY2_RCC            RCC_AHB1Periph_GPIOE

#define KEY3_PORT           GPIOE
#define KEY3_PIN            GPIO_Pin_9
#define KEY3_RCC            RCC_AHB1Periph_GPIOE

#define KEY4_PORT           GPIOG
#define KEY4_PIN            GPIO_Pin_0
#define KEY4_RCC            RCC_AHB1Periph_GPIOG

#define KEY5_PORT           GPIOG
#define KEY5_PIN            GPIO_Pin_1
#define KEY5_RCC            RCC_AHB1Periph_GPIOG

// 按键值定义
#define KEY1_VALUE          3  // PE7 -> 键码3
#define KEY2_VALUE          4  // PE8 -> 键码4
#define KEY3_VALUE          5  // PE9 -> 键码5
#define KEY4_VALUE          1  // PG0 -> 键码1
#define KEY5_VALUE          2  // PG1 -> 键码2

// 按键去抖动延时
#define KEY_DELAY_MS        20

void Key_Init(void);
uint8_t Key_GetNum(void);

#endif

