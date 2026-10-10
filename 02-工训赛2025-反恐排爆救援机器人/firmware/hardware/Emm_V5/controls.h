#ifndef __CONTROLS_H
#define __CONTROLS_H

#include "sys.h"

#define robot_arm_right 1		//机械臂右
#define robot_arm_left  0		//机械臂左

#define holder_right 		1		//云台右
#define holder_left  		0		//云台左

#define robot_arm_1_shen  1		//机械臂关节一伸
#define robot_arm_1_shou  0		//机械臂关节一收

#define robot_arm_2_shen  1		//机械臂关节二右
#define robot_arm_2_jian  0		//机械臂关节二左

/* 丝杆高度参数 */
#define PULSE_PER_CM    8000.0f   //每厘米对应脉冲数
#define MAX_HEIGHT      15.0f     //丝杆最大高度，暂定15cm
#define Z_UP            robot_arm_2_shen
#define Z_DOWN          robot_arm_2_jian

//云台舵机角度封装
#define Rotate_Start_Angle 66.0f   //底盘云台初始角度
#define Rotate_Catch_Angle 203.0f  //底盘云台夹取物体角度
#define Rotate_Set_Angle   21.0f   //底盘云台放置小球角度
#define Rotate_Watch_Angle 153.0f  //底盘云台看第二个人质的角度

//夹爪舵机角度封装
#define Claw_Ball_Angle      69.0f     //夹取小球的舵机角度
#define Claw_YuanTai_Angle   69.0f     //夹取圆台的舵机角度
#define Claw_YuanZhu_Angle   75.0f     //夹取圆柱的舵机角度
#define Claw_YaoGu_Angle     74.0f     //夹取腰鼓的舵机角度
#define Claw_Release_Angle   180.0f    //释放球体的舵机角度

//夹爪夹取延时
#define Claw_Catch_Delay 1000

//激光舵机角度封装
#define Pitch_Start_Angle 140.0f
#define Pitch_Aim_Angle   141.5f

#define Fix_Angle -89.25f//修正角度
#define Fix_Angle_2 -178.5f//修正角度

#define After_Aim_distance_1  217500
#define After_Aim_distance_2  147500
#define After_Aim_distance_3  77500

#define After_Aim_Y_Clk 3000

void control(void);
void Readclk_robot_arm(void);
void Readclk_holder(void);
void Readclk_robot_arm_1(void);//3
void Readclk_robot_arm_2(void);//4
int Read_robot_arm(void);//1
int Read_holder(void);//2
int Read_Z(void);//5
int Read_Y(void);//6

void rotate_angle_slow(float Current_Angle,float Angle);

void Back_Zero(int Arr);//Z轴步进和Y轴步进回到初始位置函数

void rotate_angle(float Angle);
void claw_angle(float Angle);
void pitch_angle(float Angle);

void Control_robot_arm(uint8_t dir,int32_t clk,uint16_t v,uint8_t a);//1
void Control_holder(uint8_t dir,int32_t clk,uint16_t v,uint8_t a);//2
void Control_robot_arm_1(uint8_t dir,int32_t clk,uint16_t v,uint8_t a);//3
void Control_robot_arm_2(uint8_t dir,int32_t clk,uint16_t v,uint8_t a);//4
void Control_robot_arm_Y(uint8_t dir,int32_t clk,uint16_t v,uint8_t a);//6
void Control_robot_arm_Z(uint8_t dir,int32_t clk,uint16_t v,uint8_t a);//5
void Set_Claw_Height_Relative(float target_height, uint16_t v, uint8_t a);

extern float pos_robot_arm; 
extern float Motor_Cur_Pos_robot_arm;//右后
extern float pos_holder;
extern float Motor_Cur_Pos_holder;//左后
extern float pos_robot_arm_1;
extern float Motor_Cur_Pos_robot_arm_1;
extern float pos_robot_arm_2;
extern float Motor_Cur_Pos_robot_arm_2;

extern float Current_Claw_Height_Relative;

extern uint8_t time_flag;
extern uint32_t  timekeeping;

extern uint8_t In_Servo;//舵机到位标志

extern int QR_CODE;     //二维码内容

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
    float lf_speed;     // 左前速度
    float lb_speed;     // 左后速度
    float rf_speed;     // 右前速度
    float rb_speed;     // 右后速度
    float tar_pos;      // 目标距离
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
extern Motor_control_MotorStep motion_steps[];  // 运动步骤数组
extern uint8_t current_step;      // 当前步骤索引
extern uint8_t total_steps;       // 总步骤数
extern float global_speed_scale;  // 速度缩放系数
extern uint8_t need_skip_next_step;  // 跳过步骤标志

extern const uint8_t TOTAL_STEPS;
volatile extern int32_t wheel_revolutions[5];				//脉冲累计

void step0_extra_action(uint8_t step_index);
void step4_extra_action(uint8_t step_index);
void step5_extra_action(uint8_t step_index);
void step6_extra_action(uint8_t step_index);
void step8_extra_action(uint8_t step_index);
void step9_extra_action(uint8_t step_index);
void step10_extra_action(uint8_t step_index);
void step13_extra_action(uint8_t step_index);
void step14_extra_action(uint8_t step_index);
void step15_extra_action(uint8_t step_index);


#endif
