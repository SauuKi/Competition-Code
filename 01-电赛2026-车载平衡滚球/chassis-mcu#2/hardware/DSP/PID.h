#ifndef  _PID_H
#define  _PID_H

#include "sys.h" 
#include "Gray.h"

extern float Motor_tar_pos;
extern float Motor_tar_speed;
extern float Motor_tar_angle;
extern float Motor_tar_speed_left;
extern float Motor_tar_speed_right;

extern int motor_control_mode;

/* 各模式参数结构体 */
typedef struct {
    float tar_pos;       // 目标距离
    float tar_speed;     // 目标速度
    float tar_angle;     // 目标角度
} FixedDistParams;

typedef struct {
    float tar_speed_left;  // 左轮目标速度
    float tar_speed_right; // 右轮目标速度
    float tar_angle;       // 目标角度
} DiffTurnParams;

typedef struct {
    float left_speed;    // 左轮速度
    float right_speed;   // 右轮速度
    float tar_pos;       // 目标距离
} SingleSpeedParams;

typedef struct {
    float base_speed;           // 基础速度（IRQ中可能会被动态修改）
    float base_speed_orig;      // 原始基础速度（用于加减速计算参考）
    float steer_gain;           // 转向增益（IRQ中可能会被动态修改）
    float steer_gain_orig;      // 原始转向增益（随 base_speed 等比缩放）
    int32_t stop_hard;          // 硬上限停止脉冲
    int32_t detect_start;       // 停止线检测起始脉冲（到 stop_hard 之间检测中间4路全黑）
    uint8_t accel_enable;       // 加减速使能：0=关闭（快速循迹），1=开启（慢速循迹）
    float accel_min_speed;      // 加减速最低速度
    int32_t accel_distance;     // 加速段距离（脉冲数）
    int32_t decel_distance;     // 减速段距离（从 stop_hard 往前算，脉冲数）
    uint8_t yaw_stop_enable;    // Yaw角提前停止使能：1=检测区间内Yaw<33°则提前停止
} LineTrackParams;

/* 全局结构体变量 */
extern FixedDistParams fixed_dist_params;
extern DiffTurnParams diff_turn_params;
extern SingleSpeedParams single_speed_params;
extern LineTrackParams line_track_params;

void pid_init(void);

float cascade_pid_control(int wheel_idx, float now_pos, 
	int16_t now_speed, float tar_pos, float tar_speed, float tar_angle);

uint8_t check_wheel_reached(int wheel_idx);
uint8_t check_all_reached(void);
uint8_t check_all_reached_filter(void);

void Motor_control_fixed_distance(float now_pos, int16_t *now_speed, void *params);
void Motor_control_diff_turn(int16_t *now_speed, void *params);
void Motor_control_single_speed(float now_pos, int16_t *now_speed, void *params);
void Motor_control_fixed(void *params);
void Motor_control_line_track(int16_t *now_speed, void *params);

/* 参数设置函数 */
void motor_fixed_distance_params(float tar_pos, float tar_speed, float tar_angle);
void motor_diff_turn_params(float tar_speed_left, float tar_speed_right, float tar_angle);
void motor_single_speed_params(float left_speed, float right_speed, float tar_pos);
void motor_fixed_params(float tar_speed, float tar_angle);
void motor_line_track_params(float base_speed, float steer_gain, int32_t stop_hard, int32_t detect_start,
                             uint8_t accel_enable, float accel_min_speed, int32_t accel_distance, int32_t decel_distance,
                             uint8_t yaw_stop_enable);
void motor_clear_and_init_params(void);

float myabsf(float val);
int32_t myabs(int32_t val);

#endif
