/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 一、引用标准输入输出库 */
#include <stdio.h>
#include "FreeRTOS.h"  
#include "queue.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* 二、定义串口数据缓冲区配置 */
#define UART2_BUFFER_SIZE        (32)    //串口2缓冲区字节
#define UART6_BUFFER_SIZE        (14)    //串口6缓冲区字节
#define UART1_BUFFER_SIZE        (128)    //串口1缓冲区字节
#define UART3_BUFFER_SIZE        (128)    //串口3缓冲区字节
#define UART_BUFFER_QUANTITY     (5)

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* 三、定义串口数据结构体，DMA传输的数据将保存在这里 */
typedef struct
{
    uint8_t buffer[UART2_BUFFER_SIZE];  /* 存放数据的空间 */
    uint16_t size;                      /* 已存放数据的大小 */
}UART2_RX_TypeDef;

typedef struct
{
    uint8_t buffer[UART6_BUFFER_SIZE];  /* 存放数据的空间 */
    uint16_t size;                      /* 已存放数据的大小 */
}UART6_RX_TypeDef;

typedef struct
{
    uint8_t buffer[UART1_BUFFER_SIZE];  /* 存放数据的空间 */
    uint16_t size;                      /* 已存放数据的大小 */
}UART1_RX_TypeDef;

typedef struct
{
    uint8_t buffer[UART3_BUFFER_SIZE];  /* 存放数据的空间 */
    uint16_t size;                      /* 已存放数据的大小 */
}UART3_RX_TypeDef;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY1_Pin GPIO_PIN_0
#define KEY1_GPIO_Port GPIOG
#define KEY2_Pin GPIO_PIN_1
#define KEY2_GPIO_Port GPIOG
#define KEY3_Pin GPIO_PIN_7
#define KEY3_GPIO_Port GPIOE
#define KEY4_Pin GPIO_PIN_8
#define KEY4_GPIO_Port GPIOE
#define KEY5_Pin GPIO_PIN_9
#define KEY5_GPIO_Port GPIOE
#define OLED_I2C_SCL_Pin GPIO_PIN_10
#define OLED_I2C_SCL_GPIO_Port GPIOB
#define OLED_I2C_SDA_Pin GPIO_PIN_11
#define OLED_I2C_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
