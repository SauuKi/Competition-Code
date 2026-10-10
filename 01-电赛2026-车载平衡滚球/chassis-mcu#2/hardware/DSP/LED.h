#ifndef __LED_H
#define __LED_H
#include "sys.h"

// LED引脚宏定义 - 便于移植和修改
#define LED_PORT            GPIOC
#define LED_PIN             GPIO_Pin_12
#define LED_RCC             RCC_AHB1Periph_GPIOC

void LED_Init(void);
void LED_TURN(void);
void LED_ON(void);      // 设置LED为高电平
void LED_OFF(void);     // 设置LED为低电平

#endif
