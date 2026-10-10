#ifndef  _PWM_H
#define  _PWM_H

#include "sys.h" 

void PWM_Init_TIM8(u16 arr,u16 psc);
void PWM_Init_TIM9(u16 arr,u16 psc);
void PWM_Init_TIM12(u16 arr,u16 psc);
void PWM_Init_TIM10(u16 arr,u16 psc);
void PWM_Init_TIM11(u16 arr, u16 psc);

void PWM_SetCompare1(uint16_t Compare);
void PWM_SetCompare2(uint16_t Compare);
void PWM_SetCompare3(uint16_t Compare);
void PWM_SetCompare4(uint16_t Compare);
void PWM_SetCompare5(uint16_t Compare);
void PWM_SetCompare6(uint16_t Compare);
void PWM_SetCompare7(uint16_t Compare);
void PWM_SetCompare8(uint16_t Compare);


#endif
