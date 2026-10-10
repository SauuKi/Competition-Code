#ifndef __CONTROLS_H
#define __CONTROLS_H

#include "sys.h"
#include <math.h>

#define PI		3.1415926535897932384626433832795f

/* ========== 轴方向定义 ========== */
#define Z_UP   0
#define Z_DOWN 1
#define Y_SHEN 1
#define Y_SHUO 0

/* ========== 丝杆/齿条参数 ========== */
#define PULSE_PER_CM       8000.0f   // 每厘米对应8000个脉冲
#define CLAW_SPEED         1000
#define CLAW_ACC           0
#define MAX_HEIGHT         20.32f    // 最高高度 8inch=20.32cm
#define PULSE_PER_MM       67.9055f  // 齿条移动1mm所需脉冲数
#define RACK_MAX_EXTEND    180.0f    // 齿条最大前伸距离(mm)
#define RACK_MIN_RETRACT   0.0f      // 齿条最小回收距离(mm)
#define MAX_PULSE          (MAX_HEIGHT * PULSE_PER_CM)

/* ========== 摆杆参数 ========== */
#define BEAM_LENGTH_MM     250.0f    // 摆杆长度(mm)
#define SCREW_LEAD_MM      8.0f      // 丝杆导程(mm/圈)
#define PULSE_PER_REV      6400      // 步进一圈脉冲数
#define PULSE_PER_MM_Z     800.0f    // Z轴每mm对应脉冲 (PULSE_PER_REV / SCREW_LEAD_MM)
#define BEAM_ANGLE_LIMIT   5.0f      // 摆杆角度限幅(度)
#define BEAM_SPEED          300       // 固定速度(RPM)
#define BEAM_ACC            0         // 摆杆运动加速度

/* 动态速度接口（BEAM_SPEED_DYNAMIC=1时启用） */
#define BEAM_SPEED_DYNAMIC  1         // 0=固定速度, 1=动态速度
#define BEAM_SPEED_MIN      100       //  动态: 最小RPM
#define BEAM_SPEED_MAX      500       //  动态: 最大RPM
#define BEAM_SPEED_K        0.030f    //  动态: 比例系数 (RPM/pulse)

/* ========== 步进位置变量(外部可用) ========== */
extern float Motor_Cur_Pos_robot_arm_Z;    // Z轴(丝杆升降, 摆杆角度控制)
extern float Motor_Cur_Pos_robot_arm_Y;    // Y轴(齿条伸缩)
extern volatile int32_t Emm_revolutions[7]; // 脉冲累计

/* ========== 工具函数 ========== */
float ConvertToAngle(float z);
float radians_to_degrees(float radians);
float degrees_to_radians(float degrees);
float Absolute_value(float yaw);

/* ========== 步进数据解包 ========== */
void Emm_Data_Unpack(uint8_t Emm_V5_rxCount, uint8_t Emm_V5_rxCmd[128]);

/* ========== Z轴控制(丝杆升降, 摆杆驱动) ========== */
void Set_Pos_Z(uint8_t dir, int32_t clk, uint16_t v, uint8_t a);
void Set_Clk_Z(int32_t clk, uint16_t v, uint8_t a);
void Set_Claw_Height(float target_height);
void Set_Claw_Height_Relative(float target_height, uint16_t v, uint8_t a);
void Back_Zero(int Arr, uint16_t v, uint8_t a);

/* ========== Y轴控制(齿条伸缩) ========== */
void Set_Pos_Y(uint8_t dir, int32_t clk, uint16_t v, uint8_t a);
void Set_Clk_Y(int32_t clk, uint16_t v, uint8_t a);
float Get_Rack_Position_Y(void);
int8_t Move_Rack_Absolute_Y(float target_mm, uint16_t v, uint8_t a);
int8_t Move_Rack_Absolute_Y_One(float target_mm, uint16_t v, uint8_t a);
int8_t Move_Rack_Relative_Y(float delta_mm, uint16_t v, uint8_t a);

/* ========== 摆杆控制(球平衡) ========== */
int32_t beam_angle_to_pulse(float angle_deg);    // 角度 → 脉冲换算
void beam_set_angle(float angle_deg);             // 直接设定摆杆角度

#endif
