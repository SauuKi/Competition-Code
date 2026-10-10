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

/* 云台舵机参数：行程按实际机械行程修改，测试用 180，换 360 舵机时改成 360 */
#define GIMBAL_SERVO_CH           1
#define GIMBAL_SERVO_TRAVEL_DEG   360.0f
#define GIMBAL_SERVO_SPEED_DPS    100.0f
#define GIMBAL_SERVO_MAX_DPS      300.0f
#define SERVO_ROTATE_PERIOD_MS    10

//三个舵机的最大角度，根据实际使用修改
#define Servo1_Max_Angle GIMBAL_SERVO_TRAVEL_DEG
#define Servo2_Max_Angle 180.0f
#define Servo3_Max_Angle 270.0f

/* ====================================================== */

/* 舵机运动状态结构体 */
typedef struct
{
    uint8_t ch;             // 舵机通道号
    uint8_t is_moving;      // 是否正在运动
    float current_angle;    // 当前角度
    float target_angle;     // 目标角度
    float step;             // 每次更新的步长
    TickType_t last_update; // 上次更新时间
} Servo_Motion_t;

extern Servo_Motion_t gimbal_servo;     // 用于云台舵机控制

void SERVO_Init(void);
void Servo_SetAngle(uint8_t ch, float angle);   /* 0~270�� */
void Servo_SetPWM(uint8_t ch, uint32_t pulse); /* 500~2500 ��s */
void Servo_StartRotate(Servo_Motion_t *servo, uint8_t ch, float target_angle, float speed_dps);
uint8_t Servo_UpdateRotate(Servo_Motion_t *servo);
void Servo_SetCurrentAngle(uint8_t ch, float angle);

#endif
