#ifndef  _PID_H
#define  _PID_H

#include "sys.h" 

extern float Motor_tar_pos;
extern float Motor_tar_speed;
extern float Motor_tar_angle;
extern float Motor_tar_speed_left;
extern float Motor_tar_speed_right;

extern int motor_control_mode;

// 定义各模式的参数结构体（按需传入，避免参数冗余）
typedef struct {
    float tar_pos;       // 目标距离（所有轮子共用）
    float tar_speed;     // 目标速度（所有轮子共用）
    float tar_angle;     // 目标角度（方向微调）
} FixedDistParams;

typedef struct {
    float tar_speed_left;  // 左轮目标速度（0/1轮）
    float tar_speed_right; // 右轮目标速度（2/3轮）
    float tar_angle;       // 目标角度（差速转弯用）
} DiffTurnParams;

typedef struct {
    float tar_speed_single[4];  // 4个轮子的独立目标速度
    uint32_t tar_total_pos;     // 目标总路程
    float tar_angle;            // 目标角度（到位后微调）
} SingleSpeedParams;

// 声明全局结构体变量（供其他文件访问）
extern FixedDistParams fixed_dist_params;
extern DiffTurnParams diff_turn_params;
extern SingleSpeedParams single_speed_params;

void pid_init(void);

float cascade_pid_control(int wheel_idx, float now_pos, 
	int16_t now_speed, float tar_pos, float tar_speed, float tar_angle);

uint8_t check_wheel_reached(int wheel_idx);

uint8_t check_all_reached(void);
uint8_t check_all_reached_filter(void);

void Motor_control_fixed_distance(float now_pos, int16_t *now_speed, void *params);
void Motor_control_diff_turn(int16_t *now_speed,void *params);
void Motor_control_single_speed(float now_pos,int16_t *now_speed, void *params);
void Motor_control_fixed( void *params);

//整体走直线，固定距离，固定速度
void motor_fixed_distance_params(float tar_pos, float tar_speed, float tar_angle);

//差速转弯，到目标角度
void motor_diff_turn_params( float tar_speed_left, float tar_speed_right, float tar_angle);

//4轮分别速度控制，运行到目标距离
void motor_single_speed_params( float tar_speed_left_qian, float tar_speed_left_hou,float tar_speed_right_qian,float tar_speed_right_hou, float tar_pos);

//整体走直线,固定角度
void motor_fixed_params( float tar_speed, float tar_angle);

void motor_clear_and_init_params(void);


 float myabsf(float val);
 int32_t myabs(int32_t val);

#endif
