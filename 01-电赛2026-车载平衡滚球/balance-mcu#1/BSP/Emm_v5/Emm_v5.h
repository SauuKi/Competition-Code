#ifndef __EMM_V5_H
#define __EMM_V5_H

#include "usart.h"
#include "sys.h"
#include <stdbool.h> 

#define 	Emm_V5_FIFO_SIZE   128             

extern bool Emm_V5_rxFrameFlag;
extern uint8_t Emm_V5_rxCmd[Emm_V5_FIFO_SIZE];
extern uint8_t Emm_V5_rxCount;

typedef enum {
	S_VER   = 0,			/* 读取固件版本和对应的硬件版本 */
	S_RL    = 1,			/* 读取读取相电阻和相电感 */
	S_PID   = 2,			/* 读取PID参数 */
	S_VBUS  = 3,			/* 读取总线电压 */
	S_CPHA  = 5,			/* 读取相电流 */
	S_ENCL  = 7,			/* 读取经过线性化校准后的编码器值 */
	S_TPOS  = 8,			/* 读取电机目标位置角度 */
	S_VEL   = 9,			/* 读取电机实时转速 */
	S_CPOS  = 10,			/* 读取电机实时位置角度 */
	S_PERR  = 11,			/* 读取电机位置误差角度 */
	S_FLAG  = 13,			/* 读取使能/到位/堵转状态标志位 */
	S_Conf  = 14,			/* 读取驱动参数 */
	S_State = 15,			/* 读取系统状态参数 */
	S_ORG   = 16,     /* 读取正在回零/回零失败状态标志位 */
}SysParams_t;

void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);
void ReadClk_Z(void);
void ReadClk_Y(void);
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);
void Emm_vel_control(int MotorSpeed1 ,int MotorSpeed2 ,int MotorSpeed3 ,int MotorSpeed4 );
void SetVel(int MotorSpeed );
void Emm_V5_Stop_Now(uint8_t addr, bool snF);
void Emm_V5_Synchronous_motion(uint8_t addr);
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr);//将当前位置清零
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);

#endif
