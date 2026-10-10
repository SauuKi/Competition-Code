#ifndef __G6220_H
#define __G6220_H

#include "hcan.h"
#include "sys.h"

#define G6220_Enable 1
#define G6220_Disable 2
#define G6220_Save_Zero_Position 3
#define G6220_Clear_Error 4
#define G6220_MIT 0
#define G6220_POZ 0x100
#define G6220_SPEED 0x200
#define G6220_Modo G6220_POZ
#define	G6220_CAN_ID 0X01
#define P_MAX 12.5f
#define V_MAX 45.f
#define T_MAX 10.f

void G6220_Init(void);
void G6220_Command(uint8_t CMD);
void G6220_Position(int32_t position,uint16_t speed);
void G6220_Speed(uint16_t speed);

#endif
