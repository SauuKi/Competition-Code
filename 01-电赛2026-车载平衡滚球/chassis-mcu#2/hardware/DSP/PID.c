#include "PID.h"
#include <stdint.h>
#include "Motor.h"

// 1. 向前声明
typedef struct CascadePID CascadePID;

// 2. 函数指针类型
typedef float (*CascadePID_posFunc)(CascadePID *pid, float now_position, float tar_position);
typedef float (*CascadePID_speedFunc)(CascadePID *pid, int16_t now_speed, float tar_speed, float now_position, float tar_position);
typedef float (*CascadePID_angleFunc)(CascadePID *pid, float now_angle, float tar_angle);

float Motor_tar_pos = 0;
float Motor_tar_speed = 0;
float Motor_tar_speed_left = 0;
float Motor_tar_speed_right = 0;
float Motor_tar_angle = 0;

int motor_control_mode = 0;

/* 各模式参数 */
FixedDistParams fixed_dist_params;
DiffTurnParams diff_turn_params;
SingleSpeedParams single_speed_params;
LineTrackParams line_track_params;

// 3. 完整结构体
struct CascadePID {
    // 角度环（最外环）
    struct {
        float Err;
        float last_err;
        float Integral;
        float output;
        float p, i, d;
    } angle;

    // 位置环
    struct {
        float Err;
        float last_err;
        float Integral;
        float pwm;
        float p, i, d;
        float max_output;
    } pos;

    // 速度环
    struct {
        float Err;
        float last_err;
        float next_err;
        float pwm;
        float add;
        float p, i, d;
    } speed;

    // 梯形加减速
    struct {
        float total_distance;
        float current_speed;
        float max_speed;
        float acceleration;
        float deceleration;
        float accel_phase_dist;
        float decel_phase_dist;
        float cruise_dist;
        float start_pos;
        uint8_t is_initialized;
    } trapz;

    uint8_t is_reached;

    float tar_pos;
    float tar_speed;
    float tar_angle;

    // 函数指针
    CascadePID_angleFunc PID_angle;
    CascadePID_posFunc PID_pos;
    CascadePID_speedFunc PID_speed;
};

/* 工具函数 */
float myabsf(float val) { return val < 0 ? -val : val; }
int32_t myabs(int32_t val) { return val < 0 ? -val : val; }

/* PWM限幅 */
static float pwm_control(float pwm) {
    const float MAX_PWM = 8399.0f;
    const float MIN_PWM = -8399.0f;
    return (pwm > MAX_PWM) ? MAX_PWM : (pwm < MIN_PWM ? MIN_PWM : pwm);
}

/* 梯形加减速规划 */
static float trapezoidal_profile(CascadePID *pid, float now_position, float tar_position, float tar)
{
    float direction = (tar > 0) ? 1.0f : -1.0f;
    float abs_tar_speed = myabsf(tar);
    float signed_tar_speed = abs_tar_speed * direction;
    float remaining = tar_position - now_position;
    float abs_remaining = myabsf(remaining);

    // 减速阶段
    if (abs_remaining <= 50000) {
        pid->trapz.current_speed -= pid->trapz.deceleration * direction;
        if (direction > 0) {
            if (pid->trapz.current_speed <= 300)
                pid->trapz.current_speed = 300;
        } else {
            if (pid->trapz.current_speed >= -300)
                pid->trapz.current_speed = -300;
        }
    }
    // 加速阶段
    else if (myabsf(now_position) <= 50000) {
        pid->trapz.current_speed += pid->trapz.acceleration * direction;
        if (direction > 0) {
            if (pid->trapz.current_speed >= signed_tar_speed)
                pid->trapz.current_speed = signed_tar_speed;
        } else {
            if (pid->trapz.current_speed <= signed_tar_speed)
                pid->trapz.current_speed = signed_tar_speed;
        }
    }
    // 匀速阶段
    else {
        pid->trapz.current_speed = signed_tar_speed;
    }

    return pid->trapz.current_speed;
}

/* 增量式PID速度控制 */
float Speed_pid(CascadePID *pid, int16_t now_speed, float tar_speed, float now_position, float tar_position)
{
    pid->speed.Err = tar_speed - now_speed;
    if (pid->speed.Err < 5)
        pid->speed.i = 0.1;
    else
        pid->speed.i = 0.1;
    pid->speed.add = pid->speed.p * (pid->speed.Err - pid->speed.last_err) +
                     pid->speed.i * (pid->speed.Err) +
                     pid->speed.d * (pid->speed.Err + pid->speed.next_err - 2 * pid->speed.last_err);
    pid->speed.pwm += pid->speed.add;
    pid->speed.pwm = pwm_control(pid->speed.pwm);
    pid->speed.next_err = pid->speed.last_err;
    pid->speed.last_err = pid->speed.Err;
    return pid->speed.pwm;
}

/* 位置式PID位置控制 */
float pos_pid(CascadePID *pid, float now_position, float tar_position)
{
    pid->pos.Err = tar_position - now_position;
    if (myabs(pid->pos.Err) < 250)
        pid->pos.p = 0.01;
    else
        pid->pos.p = 1;
    pid->pos.pwm = pid->pos.p * pid->pos.Err +
                   pid->pos.i * pid->pos.Integral +
                   pid->pos.d * (pid->pos.Err - pid->pos.last_err);
    pid->pos.last_err = pid->pos.Err;
    return pid->pos.pwm;
}

/* 角度环 */
float angle_pid(CascadePID *pid, float now_angle, float tar_angle)
{
    if (tar_angle == 180) {
        if (now_angle >= 0)
            pid->angle.Err = tar_angle - now_angle;
        else
            pid->angle.Err = -(tar_angle + now_angle);
    } else {
        pid->angle.Err = tar_angle - now_angle;
    }

    pid->angle.Integral += pid->angle.Err;
    if (pid->angle.Integral >= 10000)
        pid->angle.Integral = 10000;
    if (pid->angle.Integral <= -10000)
        pid->angle.Integral = -10000;

    pid->angle.output = pid->angle.p * pid->angle.Err
                      + pid->angle.i * pid->angle.Integral
                      + pid->angle.d * (pid->angle.Err - pid->angle.last_err);

    pid->angle.last_err = pid->angle.Err;

    return -pid->angle.output;
}

/* ---------- 2轮PID控制器 ---------- */
CascadePID wheel_pid[2];

void pid_init(void)
{
    memset(wheel_pid, 0, sizeof(wheel_pid));

    // 左轮
    wheel_pid[0].angle.p = 50.0f;
    wheel_pid[0].angle.i = 0.0f;
    wheel_pid[0].angle.d = 3.0f;

    wheel_pid[0].pos.p = 2.0f;
    wheel_pid[0].pos.i = 0.03f;
    wheel_pid[0].pos.d = 1.5f;

    wheel_pid[0].speed.p = 3.0f;
    wheel_pid[0].speed.i = 0.3f;
    wheel_pid[0].speed.d = 1.0f;

    wheel_pid[0].trapz.acceleration = 50.0f;
    wheel_pid[0].trapz.deceleration = 20.0f;
    wheel_pid[0].trapz.is_initialized = 0;

    wheel_pid[0].PID_pos = pos_pid;
    wheel_pid[0].PID_speed = Speed_pid;
    wheel_pid[0].PID_angle = angle_pid;

    wheel_pid[0].is_reached = 0;
    wheel_pid[0].tar_pos = 0;
    wheel_pid[0].tar_speed = 0;
    wheel_pid[0].tar_angle = 0;

    // 右轮 = 左轮参数
    wheel_pid[1] = wheel_pid[0];
}

float limit(float now, float tar)
{
    if (now > tar) now = tar;
    if (now < -tar) now = -tar;
    return now;
}

/* ========== 各控制模式（2轮） ========== */

/* 模式1：整体走直线，固定距离，固定速度 */
float Mode_fixed_distance(int wheel_idx, float now_pos, int16_t now_speed,
                          float tar_pos, float tar_speed, float tar_angle)
{
    if (wheel_idx < 0 || wheel_idx >= 2) return 0.0f;

    CascadePID *pid = &wheel_pid[wheel_idx];
    float pwm = 0;

    float trapz_speed = trapezoidal_profile(pid, now_pos, tar_pos, tar_speed);

    pwm = pid->PID_pos(pid, now_pos, tar_pos);
    pwm = limit(pwm, trapz_speed);

    // 角度修正：左轮加、右轮减（差速纠偏）
    if (wheel_idx == 0)
        pwm = pwm + pid->PID_angle(pid, Yaw, tar_angle);
    else
        pwm = pwm - pid->PID_angle(pid, Yaw, tar_angle);

    if (myabs(tar_pos - now_pos) < 250) {
        pwm = 0;
        pid->is_reached = 1;
    } else {
        pwm = pid->PID_speed(pid, now_speed, pwm, now_pos, tar_pos);
        pid->is_reached = 0;
    }

    return pwm;
}

/* 模式2：差速转弯，到目标角度 */
float Mode_diff_turn(int wheel_idx, int16_t now_speed, float tar_speed, float tar_angle)
{
    if (wheel_idx < 0 || wheel_idx >= 2) return 0.0f;

    CascadePID *pid = &wheel_pid[wheel_idx];
    float pwm = 0;
    float angle_error = 0;
    float actual_tar_speed = tar_speed;

    angle_error = tar_angle - Yaw;
    if (angle_error > 180.0f)
        angle_error -= 360.0f;
    else if (angle_error < -180.0f)
        angle_error += 360.0f;

    if (myabsf(angle_error) < 1.0f) {
        actual_tar_speed = 0.0f;
        pid->is_reached = 1;
    } else {
        int main_direction = (tar_speed > 0) ? 1 : -1;
        int error_direction = (angle_error > 0) ? -1 : 1;
        if (error_direction != main_direction)
            actual_tar_speed = -tar_speed;
        pid->is_reached = 0;
    }

    // 2轮差速：左轮同向，右轮反向
    if (wheel_idx == 0)
        pwm = pid->PID_speed(pid, now_speed, actual_tar_speed, 0, 0);
    else
        pwm = pid->PID_speed(pid, now_speed, -actual_tar_speed, 0, 0);

    return pwm;
}

/* 模式3：2轮分别速度控制，运行到目标距离 */
float Mode_single_speed(int wheel_idx, float now_pos, int16_t now_speed,
                        float tar_pos, float tar_speed)
{
    if (wheel_idx < 0 || wheel_idx >= 2) return 0.0f;

    CascadePID *pid = &wheel_pid[wheel_idx];
    float pwm = 0;

    pwm = pid->PID_pos(pid, now_pos, tar_pos);
    pwm = pwm > 0 ? tar_speed : (-tar_speed);

    if (myabs(tar_pos - now_pos) < 250) {
        pwm = 0;
        pid->is_reached = 1;
    } else {
        pwm = pid->PID_speed(pid, now_speed, pwm, 0, 0);
        pid->is_reached = 0;
    }

    return pwm;
}

/* 模式4：整体走直线，固定角度 */
float Mode_fixed(int wheel_idx, float tar_speed, float tar_angle)
{
    if (wheel_idx < 0 || wheel_idx >= 2) return 0.0f;

    CascadePID *pid = &wheel_pid[wheel_idx];
    float pwm = 0;

    if (wheel_idx == 0)
        pwm = tar_speed + pid->PID_angle(pid, Yaw, tar_angle);
    else
        pwm = tar_speed - pid->PID_angle(pid, Yaw, tar_angle);

    pid->is_reached = 0;
    return pwm;
}

/* 模式5：循迹控制 —— 2轮差速驱动 */
float Mode_line_track(int wheel_idx, int16_t now_speed, float base_speed,
                      float line_error, float steer_gain)
{
    if (wheel_idx < 0 || wheel_idx >= 2) return 0.0f;

    CascadePID *pid = &wheel_pid[wheel_idx];
    float tar_speed;

    // 左轮(0)：偏差左偏→减速；右轮(1)：偏差左偏→加速
    if (wheel_idx == 0)
        tar_speed = base_speed - steer_gain * line_error;
    else
        tar_speed = base_speed + steer_gain * line_error;

    pid->is_reached = 0;
    return pid->PID_speed(pid, now_speed, tar_speed, 0, 0);
}

/* ========== 到位检测 ========== */

uint8_t check_wheel_reached(int wheel_idx) {
    if (wheel_idx < 0 || wheel_idx >= 2) return 0;
    return wheel_pid[wheel_idx].is_reached;
}

uint8_t check_all_reached(void) {
    if (motor_control_mode == 1) {
        return (wheel_pid[0].is_reached && wheel_pid[1].is_reached) &&
               (Motor_tar_pos <= Speed_sum + 250) &&
               (Motor_tar_pos >= Speed_sum - 250) ? 1 : 0;
    } else if (motor_control_mode == 2) {
        return (wheel_pid[0].is_reached && wheel_pid[1].is_reached) ? 1 : 0;
    } else if (motor_control_mode == 3) {
        return (wheel_pid[0].is_reached && wheel_pid[1].is_reached) &&
               (Motor_tar_pos <= Speed_sum + 250) &&
               (Motor_tar_pos >= Speed_sum - 250) ? 1 : 0;
    } else if (motor_control_mode == 5) {
        return 0;  // 循迹模式永不自动到位
    } else {
        return 1;
    }
}

uint8_t check_all_reached_filter(void)
{
    if (check_all_reached() == 1) {
        vTaskDelay(20);
        if (check_all_reached() == 1)
            return 1;
    }
    return 0;
}

/* ========== Motor_control 统一入口（2轮） ========== */

int pwm_output[2] = {0};

void Motor_control_fixed_distance(float now_pos, int16_t *now_speed, void *params)
{
    FixedDistParams *p = (FixedDistParams *)params;
    pwm_output[0] = (int)Mode_fixed_distance(0, now_pos, now_speed[0],
                                              p->tar_pos, p->tar_speed, p->tar_angle);
    pwm_output[1] = (int)Mode_fixed_distance(1, now_pos, now_speed[1],
                                              p->tar_pos, p->tar_speed, p->tar_angle);
    Load(pwm_output[0], pwm_output[1]);
}

void Motor_control_diff_turn(int16_t *now_speed, void *params)
{
    DiffTurnParams *p = (DiffTurnParams *)params;
    pwm_output[0] = (int)Mode_diff_turn(0, now_speed[0], p->tar_speed_left, p->tar_angle);
    pwm_output[1] = (int)Mode_diff_turn(1, now_speed[1], p->tar_speed_right, p->tar_angle);
    Load(pwm_output[0], pwm_output[1]);
}

void Motor_control_single_speed(float now_pos, int16_t *now_speed, void *params)
{
    SingleSpeedParams *p = (SingleSpeedParams *)params;
    pwm_output[0] = (int)Mode_single_speed(0, now_pos, now_speed[0],
                                            Motor_tar_pos, p->left_speed);
    pwm_output[1] = (int)Mode_single_speed(1, now_pos, now_speed[1],
                                            Motor_tar_pos, p->right_speed);
    Load(pwm_output[0], pwm_output[1]);
}

void Motor_control_fixed(void *params)
{
    FixedDistParams *p = (FixedDistParams *)params;
    pwm_output[0] = (int)Mode_fixed(0, p->tar_speed, p->tar_angle);
    pwm_output[1] = (int)Mode_fixed(1, p->tar_speed, p->tar_angle);
    Load(pwm_output[0], pwm_output[1]);
}

void Motor_control_line_track(int16_t *now_speed, void *params)
{
    LineTrackParams *p = (LineTrackParams *)params;
    uint8_t Digtal[SENSOR_NUM];
    float line_error;

    GetDigtal(Digtal);
    line_error = Line_Position_Calc(Digtal);

    pwm_output[0] = (int)Mode_line_track(0, now_speed[0], p->base_speed, line_error, p->steer_gain);
    pwm_output[1] = (int)Mode_line_track(1, now_speed[1], p->base_speed, line_error, p->steer_gain);

    Load(pwm_output[0], pwm_output[1]);
}

/* ========== 参数设置函数 ========== */

void motor_fixed_distance_params(float tar_pos, float tar_speed, float tar_angle)
{
    taskENTER_CRITICAL();
    motor_control_mode = 1;
    fixed_dist_params.tar_pos = tar_pos;
    fixed_dist_params.tar_speed = tar_speed;
    fixed_dist_params.tar_angle = tar_angle;
    Motor_tar_pos = tar_pos;
    taskEXIT_CRITICAL();
}

void motor_diff_turn_params(float tar_speed_left, float tar_speed_right, float tar_angle)
{
    taskENTER_CRITICAL();
    motor_control_mode = 2;
    diff_turn_params.tar_speed_left = tar_speed_left;
    diff_turn_params.tar_speed_right = tar_speed_right;
    diff_turn_params.tar_angle = tar_angle;
    taskEXIT_CRITICAL();
}

void motor_single_speed_params(float left_speed, float right_speed, float tar_pos)
{
    taskENTER_CRITICAL();
    motor_control_mode = 3;
    single_speed_params.left_speed = left_speed;
    single_speed_params.right_speed = right_speed;
    single_speed_params.tar_pos = tar_pos;
    Motor_tar_pos = tar_pos;
    taskEXIT_CRITICAL();
}

void motor_fixed_params(float tar_speed, float tar_angle)
{
    taskENTER_CRITICAL();
    motor_control_mode = 4;
    fixed_dist_params.tar_pos = 0;
    fixed_dist_params.tar_speed = tar_speed;
    fixed_dist_params.tar_angle = tar_angle;
    Motor_tar_pos = 0;
    taskEXIT_CRITICAL();
}

void motor_line_track_params(float base_speed, float steer_gain, int32_t stop_hard, int32_t detect_start,
                             uint8_t accel_enable, float accel_min_speed, int32_t accel_distance, int32_t decel_distance,
                             uint8_t yaw_stop_enable)
{
	taskENTER_CRITICAL();
	motor_control_mode = 5;
	line_track_params.base_speed     = base_speed;
	line_track_params.base_speed_orig = base_speed;
	line_track_params.steer_gain     = steer_gain;
	line_track_params.steer_gain_orig = steer_gain;
	line_track_params.stop_hard      = stop_hard;
	line_track_params.detect_start   = detect_start;
	line_track_params.accel_enable   = accel_enable;
	line_track_params.accel_min_speed = accel_min_speed;
	line_track_params.accel_distance = accel_distance;
	line_track_params.decel_distance = decel_distance;
	line_track_params.yaw_stop_enable = yaw_stop_enable;
	taskEXIT_CRITICAL();
}

void motor_clear_and_init_params(void)
{
    taskENTER_CRITICAL();
    pid_init();
    motor_control_mode = 0;
    Load(0, 0);
    Speed_sum = 0;
    taskEXIT_CRITICAL();
}
