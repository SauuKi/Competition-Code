#ifndef  _ENCODER_H
#define  _ENCODER_H

#include "sys.h"

void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
int Read_Speed(int TIMx);
void Encoder_TIM3_Init(void);
void Encoder_TIM4_Init(void);
void Encoder_TIM2_Init(void);
void Encoder_TIM5_Init(void);
void Timer6_Init(void);

extern int encoder_right;
extern int encoder_left;

#endif


