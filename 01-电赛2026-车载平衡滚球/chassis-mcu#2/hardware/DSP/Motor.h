#ifndef  _MOTOR_H
#define  _MOTOR_H

#include "sys.h" 

/* 左后轮方向引脚（PC0/PC1） */
#define Left_in1   PCout(0)
#define Left_in2   PCout(1)
/* 右后轮方向引脚（PD3/PD4） */
#define Right_in1  PDout(3)
#define Right_in2  PDout(4)

#define PWM_MAX  8400
#define PWM_MIN -8400

void PWM_Init_TIM1(u16 Psc, u16 Per);
void Motor_Init(void);

void Limit(int *motoA, int *motoB);
int GFP_abs(int p);
void Load_right(int moto);
void Load_left(int moto);
void Load(int moto_left, int moto_right);

#endif
