#ifndef __SERVO_H
#define __SERVO_H

#include <math.h>  // 用于roundf四舍五入
#include "sys.h"
#include "tim.h"

/*限值*/
#define MinPulse                  500
#define MaxPulse                  2500
#define Gimbal_Servo_ID           1
#define Claw_Servo_ID             3

#define MAX_SERVO_CHANNELS 4

// 舵机参数宏定义（方便后续校准修改）
// CH1: 360度位置型舵机
#define SERVO_CH1_MIN_ANGLE  0.0f
#define SERVO_CH1_MAX_ANGLE  360.0f
#define SERVO_CH1_MIN_PULSE  500    // 0度对应脉冲
#define SERVO_CH1_MAX_PULSE  2500   // 360度对应脉冲

// CH2: 180度位置型舵机
#define SERVO_CH2_MIN_ANGLE  0.0f
#define SERVO_CH2_MAX_ANGLE  180.0f
#define SERVO_CH2_MIN_PULSE  500    // 0度对应脉冲
#define SERVO_CH2_MAX_PULSE  2500   // 180度对应脉冲

// 舵机运动状态结构体
typedef struct {
    uint8_t ch;             // 舵机通道号
    bool is_moving;         // 是否正在运动
    float current_angle;    // 当前角度
    float target_angle;     // 目标角度
    float speed;            // 转动速度（度/秒）
    float step;             // 每次更新的步长
    TickType_t last_update; // 上次更新时间
} Servo_Motion_t;

extern Servo_Motion_t gimbal_servo;//用于底盘舵机控制

void Servo_Init(void);
void Servo_setpwm(uint8_t ch, uint32_t pulse);
void Set_Angle(uint8_t ch,float Angle);
void Servo_Init_Angle(uint8_t ch, float init_angle);
float Servo_Get_Current_Angle(uint8_t ch);
void rotate_angle_slow(uint8_t ch, float target_angle, float speed_deg_per_sec);
void servo_start_rotate(Servo_Motion_t *servo, uint8_t ch, float target_angle, float speed_deg_per_sec);
bool servo_update_rotate(Servo_Motion_t *servo);

#endif
