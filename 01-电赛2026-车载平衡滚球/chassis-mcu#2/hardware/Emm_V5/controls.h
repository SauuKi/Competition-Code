#ifndef __CONTROLS_H
#define __CONTROLS_H

#include "sys.h"

extern uint8_t time_flag;
extern uint32_t  timekeeping;

// 步骤类型枚举（和之前一致）
typedef enum {
    STEP_NONE,					//无
    STEP_FIXED_DIST,		//整体走直线，固定距离，固定速度
    STEP_DIFF_TURN,			//差速转弯，到目标角度
    STEP_SINGLE_SPEED		//4轮分别速度控制，运行到目标距离
} Motor_control_StepType;

// 各类型参数结构体（和之前一致）
typedef struct {
    float tar_pos;      // 目标距离
    float tar_speed;    // 目标速度
    float tar_angle;    // 目标角度
} Motor_control_FixedDistParams;

typedef struct {
    float left_speed;   // 左电机速度
    float right_speed;  // 右电机速度
    float tar_angle;    // 目标角度
} Motor_control_DiffTurnParams;

typedef struct {
    float left_speed;    // 左轮速度
    float right_speed;   // 右轮速度
    float tar_pos;       // 目标距离
} Motor_control_SingleSpeedParams;

// 参数联合体
typedef union {
    Motor_control_FixedDistParams fixed_dist;
    Motor_control_DiffTurnParams diff_turn;
    Motor_control_SingleSpeedParams single_speed;
} Motor_control_StepParams;

// 函数指针类型（业务函数）
typedef void (*StepActionFunc)(uint8_t step_index);

// 4. 定义步骤结构体（一个步骤包含：类型、参数、是否需要等待完成）
typedef struct {
	Motor_control_StepType type;			// 步骤类型
	Motor_control_StepParams params;	// 步骤参数
	uint8_t need_wait;								// 1=需要等待完成，0=不需要
	StepActionFunc action;						// 额外业务代码（NULL表示无操作）
	
	uint8_t skip_motor;          // 是否跳过电机动作（1=跳过，只执行业务）
	int16_t action_repeat;       // 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）
	volatile uint8_t action_exit_flag; // 业务循环退出标志（用于action_repeat=-1时）
	
} Motor_control_MotorStep;

// 声明全局变量（供外部访问）
extern Motor_control_MotorStep motion_steps[];  // 运动步骤数组（旧比赛，保留兼容）
extern uint8_t current_step;      // 当前步骤索引
extern uint8_t total_steps;       // 总步骤数
extern float global_speed_scale;  // 速度缩放系数
extern uint8_t need_skip_next_step;  // 跳过步骤标志

extern const uint8_t TOTAL_STEPS;

/* ========== 多路径支持 ========== */
// 路径注册结构体：一个步骤数组 + 它的长度
typedef struct {
    Motor_control_MotorStep *steps;  // 指向步骤数组
    uint8_t total_steps;             // 该路径的总步骤数
} MotionPath;

// 路径注册表（按 mode 索引）
extern MotionPath motion_paths[];

// mode 枚举
#define MODE_IDLE        0  // 待机
#define MODE_CIRCLE      1  // 快速一圈（要求2）
#define MODE_AB          2  // A→B 直线段（要求4）
#define MODE_CIRCLE_BALL 3  // 慢速一圈 + 球控制（要求5/6）

#define MOTION_PATH_COUNT 4  // mode 0~3

#endif
