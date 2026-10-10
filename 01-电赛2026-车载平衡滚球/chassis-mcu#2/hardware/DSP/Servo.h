#ifndef __SERVO_H
#define __SERVO_H

#include "sys.h"

/* ================ 舵机相关宏定义 ================= */
#define SERVO_TIM8_CLK            RCC_APB2Periph_TIM8

#define SERVO_GPIOC_CLK           RCC_AHB1Periph_GPIOC

#define SERVO_GPIOC_PORT          GPIOC

#define SERVO_CH1_PIN             GPIO_Pin_6     /* PC6  TIM8_CH1 */
#define SERVO_CH2_PIN             GPIO_Pin_7     /* PC7  TIM8_CH2 */
#define SERVO_CH3_PIN             GPIO_Pin_8     /* PC8  TIM8_CH3 */

#define MinPulse                  500
#define MaxPulse                  2500
/* ====================================================== */

void SERVO_Init(void);
void Servo_SetAngle(uint8_t ch, float angle);   /* 0~270�� */
void Servo_SetPWM(uint8_t ch, uint32_t pulse); /* 500~2500 ��s */

#endif
