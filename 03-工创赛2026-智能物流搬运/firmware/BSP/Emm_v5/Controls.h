#ifndef __CONTROLS_H
#define __CONTROLS_H

#include "sys.h"
#include "Output_task.h"
#include "Tower_task.h"
#include <math.h>

#define PI		3.1415926535897932384626433832795f
#define WHEEL_RADIUS        0.05f       // 麦轮半径，单位m
#define PULSE_PER_ROUND     6400        // 步进电机一圈脉冲数
#define DEAD_ZONE_M         0.001f      // 死区，小于1mm不移动，避免浮点数误差
#define TOTAL_CHASSIS_NUM 4
#define Z_UP 1
#define Z_DOWN 0
#define Y_SHEN 1
#define Y_SHUO 0
#define Move_Speed 200
#define Move_a     150  
//#define Move_Speed 150
//#define Move_a     100  
// 丝杆参数定义
#define PULSE_PER_CM    8000.0f   // 每厘米对应8000个脉冲
#define CLAW_SPEED      1000
#define CLAW_ACC        0
#define MAX_HEIGHT      20.32f    // 最高高度8inch=20.32cm
// 传动参数定义
#define PULSE_PER_MM        67.9055f                 // 齿条移动1mm所需脉冲数
#define RACK_MAX_EXTEND     169.0f                   // 齿条最大前伸距离(mm)，根据实际机械修改
#define RACK_MIN_RETRACT    0.0f                     // 齿条最小回收距离(mm)，根据实际机械修改
#define MAX_PULSE       (MAX_HEIGHT * PULSE_PER_CM)  // 最大累计脉冲(地面)

// 控制过程参数
#define Claw_Angle 54                                //夹取角度
#define Release_Angle 97                            //松开角度
#define rotate_angle_set 256                         //底盘舵机放置角度
#define rotate_angle_correct 95.0f                   //舵机矫正角度
#define rotate_angle_1  112                          //物料1号位角度(左上角)
#define rotate_angle_3  78                           //物料3号位角度(右上角)                   
#define Turntable_Hight 14.4f                        //转盘高度
#define Set_Hight       18.32f                       //放置物料到车上转盘所需高度
#define Material_Hight  6.1f
#define Material_Hight_2 13.0f  
//物料区
#define delay_time_turntable_car 1100                //1.1s
#define Correct_Middle_LEN 50.0f
#define Middle_LEN 49.0f                              //物料区中心靶需要伸的长度
#define Left_LEN   105.0f                              //物料区左边靶需要伸的长度
#define Right_LEN  95.0f                              //物料区右边靶需要伸的长度
#define rotate_angle_red 57.0f                        //物料区右边靶需要的角度
#define rotate_angle_green 96.0f                      //物料区中心靶需要的角度
#define rotate_angle_bule 134.0f                      //物料区左边靶需要的角度

#define zero_pos_y 0                                 //齿条初始位置
#define middle_pos_y 115
#define turntable_kp  (0.01f / 27.0f)             //转盘比例
#define Material_Area_x_kp  (0.01f / 16.0f)       //物料区x的比例
#define Material_Area_y_kp  (0.01f / 14.0f)       //物料区y的比例
#define Material_Area2_x_kp  (0.01f / 18.0f)       //物料区x的比例
#define Material_Area2_y_kp  (0.01f / 16.0f)       //物料区y的比例

#define Object1_Angle 15.0f                          //第一位放入转盘的物料角度
#define Object2_Angle 135.0f                          //第二位放入转盘的物料角度
#define Object3_Angle 255.0f                          //第三位放入转盘的物料角度

#define fast_z_speed 2000                         //长距离
#define slow_z_speed 1000                         //短距离
#define z_acc 0                                   //丝杆加速度统一为0

#define fast_y_speed 300
#define fast_y_acc 230

#define slow_y_acc 230



typedef struct
{
    int32_t clk_set[4];
    uint16_t vel_set[4];
    uint8_t acc_set[4];
}Motor_control_para_t;


extern Motor_control_para_t motor_control_para;
extern float Motor_Cur_Pos_robot_arm_Z ;
extern float Motor_Cur_Pos_robot_arm_Y ;
extern float Motor_Cur_Pos_robot_arm_T ;
extern float Motor_Cur_Pos_wheel[4];
extern volatile int32_t Emm_revolutions [7];
extern uint8_t clockwise_flag;

float ConvertToAngle(float z);
void Emm_Data_Unpack(uint8_t Emm_V5_rxCount,uint8_t Emm_V5_rxCmd [128]);//数据解包函数
void Reset_CurPos_To_Zero(void);
void Back_Zero(int Arr,uint16_t v,uint8_t a);
void Set_Pos_Z(uint8_t dir,int32_t clk,uint16_t v,uint8_t a);
void Set_Claw_Height(float target_height);
void Set_Claw_Height_Relative(float target_height,uint16_t v, uint8_t a);
int8_t Move_Rack_Absolute_Y_One(float target_mm, uint16_t v, uint8_t a);
int8_t Move_Rack_Absolute_Y(float target_mm, uint16_t v, uint8_t a);
int8_t Move_Rack_Relative_Y(float delta_mm, uint16_t v, uint8_t a);
void Set_Pos_Y(uint8_t dir,int32_t clk,uint16_t v,uint8_t a);
void Set_Clk_Y(int32_t clk,uint16_t v,uint8_t a);
void Set_Turntable_Angle(float Tar_Angle);
void chassis_pos_send(int32_t clk[TOTAL_CHASSIS_NUM],uint16_t v[TOTAL_CHASSIS_NUM],uint8_t a[TOTAL_CHASSIS_NUM]);
void motor_set_total_pos(void);
uint8_t In_place_inspection(void);
void Move_Diff_Turn(float left_clk,float right_clk,uint16_t v,uint8_t a);
void Move_Straight(float tar_clk,uint16_t v,uint8_t a);
void MOVE_SLANT(float Right_Slant_clk,float Left_Slant_clk,uint8_t v,uint8_t a);
void calculate_wheel_revolutions(int16_t angle,uint16_t v,uint8_t a);
void calculate_wheel_cali_lateral_revolution(float delta_lateral,uint16_t v,uint8_t a);
void calculate_wheel_cali_straight_revolution(float delta_straight, uint16_t v, uint8_t a);
void correct_angle(float Target_angle);
void correct_Delicate_angle(float Target_angle);
void cali_z(float Target_angle);
void correct_x(float tar_x);
void cali_x(float tar_x);
void correct_y(float tar_y);
void cali_y(float tar_y);
void cali_pos(float tar_x,float tar_y,float target_angle);
void cali_Straight(float distance_x,float distance_y,float target_angle,float correct_angain);
void vision_cali(float x_kp,float y_kp);
void Move_xandy(float tar_x,float tar_y,uint8_t v,uint8_t a);

typedef enum{
	STEP_NONE,
	STEP_STRAIGHT,   //固定距离固定速度
	STEP_DIFF_TURN,  //差速转弯
	STEP_COORDINATE, //运动到目标坐标和角度
	STEP_Slant,      //斜走
}Task_StepType;

typedef struct{
	float distance_x;       //运动距离
	float distance_y;
	float tar_angle;      //期望偏航角
	float correct_angain; //直走后是否进行角度矫正（0否1是）
} Move_Straight_Param;

typedef struct{
	float tar_left_clk;        //左电机脉冲
	float tar_right_clk;       //右电机脉冲
	float tar_speed;           //目标速度
    float tar_Acceleration;    //目标加速度
} Move_DiffTurn_Param;

typedef struct{
	float tar_x;               //目标的x坐标
	float tar_y;               //目标的y坐标
	float tar_angle;           //目标偏航角
} Move_Coordinate_Param; 

typedef struct{
	float Right_Slant_clk;               
	float Left_Slant_clk;               
	uint8_t v;   
    uint8_t a;	
} Move_Slant_Param; 

typedef union{
	Move_Straight_Param straight;
	Move_DiffTurn_Param diffturn;
	Move_Coordinate_Param coordinate;
	Move_Slant_Param slant;
} Move_StepParams;

typedef void (*StepActionFunc)(uint8_t step_index);

typedef struct {
    Task_StepType type;		// 步骤类型
	Move_StepParams params;	// 步骤参数
	uint8_t need_wait;                  // 1=需要等待完成，0=不需要
	StepActionFunc action;				// 额外业务代码（NULL表示无操作）
	
	uint8_t skip_motor;                 // 是否跳过电机动作（1=跳过，只执行业务）
	int16_t action_repeat;              // 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）
	volatile uint8_t action_exit_flag;  // 业务循环退出标志（用于action_repeat=-1时）	
} Move_Control_Step;

extern Move_Control_Step motion_steps[];  // 运动步骤数组
extern uint8_t current_step;                    // 当前步骤索引
extern uint8_t need_skip_next_step;             // 跳过步骤标志
extern const uint8_t TOTAL_STEPS;               // 总步骤数
extern uint8_t Angle_verification;
extern int jiaodu_num;
extern uint8_t In_Serial;

#endif
