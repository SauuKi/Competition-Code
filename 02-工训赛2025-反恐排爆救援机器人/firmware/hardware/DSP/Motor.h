#ifndef  _MOTOR_H
#define  _MOTOR_H

#include "sys.h" 

#define Right_in1  PCout(2)
#define Right_in2  PCout(3)
#define Right_in3  PCout(4)
#define Right_in4  PCout(5)

#define Left_in1  PCout(0)/*【4】*/
#define Left_in2  PCout(1)
#define Left_in3  PDout(3)/*【4】*/
#define Left_in4  PDout(4)

#define PWM_MAX 8400
#define PWM_MIN -8400

void PWM_Init_TIM1(u16 Psc,u16 Per);
void Motor_Init(void);/*【5】*/

void Limit(int *motoA,int *motoB);
int GFP_abs(int p);
void Load_right(int moto1,int moto2);
void Load_left(int moto1,int moto2);
void Load(int moto_1,int moto_2,int moto_3,int moto_4);

#endif



