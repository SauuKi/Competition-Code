#ifndef __GRAY_H
#define __GRAY_H

#include "sys.h"

#define SENSOR_NUM  12

/* ================================================================
 * 循迹传感器 GPIO 引脚定义（12路红外光电管, PG2~PG13）
 *   低电平有效：检测到黑线 = 读到0
 * ================================================================ */

/* 传感器 1 ~ 12 : GPIOG Pin 2 ~ Pin 13 */
#define GPIO_GRAY_PIN_0_PORT    GPIOG
#define GPIO_GRAY_PIN_0_PIN     GPIO_Pin_2

#define GPIO_GRAY_PIN_1_PORT    GPIOG
#define GPIO_GRAY_PIN_1_PIN     GPIO_Pin_3

#define GPIO_GRAY_PIN_2_PORT    GPIOG
#define GPIO_GRAY_PIN_2_PIN     GPIO_Pin_4

#define GPIO_GRAY_PIN_3_PORT    GPIOG
#define GPIO_GRAY_PIN_3_PIN     GPIO_Pin_5

#define GPIO_GRAY_PIN_4_PORT    GPIOG
#define GPIO_GRAY_PIN_4_PIN     GPIO_Pin_6

#define GPIO_GRAY_PIN_5_PORT    GPIOG
#define GPIO_GRAY_PIN_5_PIN     GPIO_Pin_7

#define GPIO_GRAY_PIN_6_PORT    GPIOG
#define GPIO_GRAY_PIN_6_PIN     GPIO_Pin_8

#define GPIO_GRAY_PIN_7_PORT    GPIOG
#define GPIO_GRAY_PIN_7_PIN     GPIO_Pin_9

#define GPIO_GRAY_PIN_8_PORT    GPIOG
#define GPIO_GRAY_PIN_8_PIN     GPIO_Pin_10

#define GPIO_GRAY_PIN_9_PORT    GPIOG
#define GPIO_GRAY_PIN_9_PIN     GPIO_Pin_11

#define GPIO_GRAY_PIN_10_PORT   GPIOG
#define GPIO_GRAY_PIN_10_PIN    GPIO_Pin_12

#define GPIO_GRAY_PIN_11_PORT   GPIOG
#define GPIO_GRAY_PIN_11_PIN    GPIO_Pin_13


void Gray_Init(void);
void GetDigtal(uint8_t *Digtal);
uint8_t Active_Count(uint8_t *Digtal);
float Line_Position_Calc(uint8_t *Digtal);

#endif
