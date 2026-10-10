#include "Tower_task.h"
#include "Main_task.h"

/* 摄像头坐标换算: 1cm ≈ 23.5坐标, 换成mm */
#define CAM_PIXELS_PER_CM  20.0f
#define CAM_COORD_TO_MM(c)  ((float)(c) / CAM_PIXELS_PER_CM * 10.0f)

/* ================================================================
 * 球位置PID —— 增量式，带积分分离+输出限幅
 *
 * 输入: error = 目标位置 - 实际位置 (mm)
 * 输出: 摆杆角度 (度), 经 beam_set_angle() 执行
 *
 * 调参建议:
 *   先调Kp(0.1~1.0)让球缓慢收敛
 *   再调Kd(0.5~5.0)压制震荡
 *   Ki尽量小(0.01~0.05)消除静差，注意积分分离阈值
 * ================================================================ */

/* PID参数结构体 */
typedef struct {
    float kp;        // 比例系数 (度/mm)
    float ki;        // 积分系数
    float kd;        // 微分系数（阻尼）
    float dead;      // 死区(mm)
    float i_limit;   // 积分项限幅(度)
    float out_max;   // 输出角度上限(度)
} BallPIDParams;

/* 默认PID参数（当前值不变） */
static const BallPIDParams PID_DEFAULT = {
    0.03f,  // kp
    0.01f,  // ki
    0.25f,  // kd
    1.0f,   // dead
    2.0f,   // i_limit
    5.0f,   // out_max
};

/* offset=+5 专用PID参数（独立调节，请根据实测填入） */
static const BallPIDParams PID_OFFSET_P5 = {
    0.03f,  // kp  ← 待调
    0.01f,  // ki  ← 待调
    0.25f,  // kd  ← 待调
    1.0f,   // dead ← 待调
    4.0f,   // i_limit ← 待调
    5.0f,   // out_max ← 待调
};

/* MODE_TASK3 专用PID参数（独立调节，请根据实测填入） */
static const BallPIDParams PID_TASK3 = {
    0.02f,  // kp  ← 待调
    0.01f,  // ki  ← 待调
    0.4f,  // kd  ← 待调
    1.0f,   // dead ← 待调
    2.0f,   // i_limit ← 待调
    3.0f,   // out_max ← 待调
};


/* 低通滤波器参数 */
#define POS_FILTER_ALPHA  0.7f   // 球位置低通系数 (0~1, 越小越平滑)
#define D_FILTER_ALPHA     0.5f   // D项低通系数

/* PID状态 */
static float ball_last_err = 0.0f;   // 上一次误差
static float ball_integral = 0.0f;   // 积分累计
static float ball_out_last = 0.0f;   // 上次输出(用于死区比较)

/**
 * 球位置PID
 * @param  error  位置误差(mm), 正值=球偏外(需要升高)
 * @return        目标摆杆角度(度), 正值=升高末端(球向内滚)
 */
static float ball_pid_update(float error)
{
    const BallPIDParams *p;
    float p_term, i_term, d_term, output;

    /* 根据 selected_mode / offset_value 选择PID参数集 */
    if (selected_mode == MODE_TASK3)
        p = &PID_TASK3;
    else if (offset_select_mode && offset_value == 5)
        p = &PID_OFFSET_P5;
    else
        p = &PID_DEFAULT;

    /* 死区 */
    if (fabsf(error) < p->dead)
    return 0;

    /* 积分分离: 误差大时不积分, 防止windup */
    if (fabsf(error) < 20.0f)  // 误差小于20mm才积分
    {
        ball_integral += error;
        if (ball_integral >  p->i_limit) ball_integral =  p->i_limit;
        if (ball_integral < -p->i_limit) ball_integral = -p->i_limit;
    }
    else
    {
        ball_integral = 0.0f;
    }

    /* PID计算 */
    p_term = p->kp * error;
    i_term = p->ki * ball_integral;

    /* D项单独低通: 抑制摄像头噪声放大 */
    {
        static float d_input_filtered = 0.0f;
        float d_raw = error - ball_last_err;
        d_input_filtered = d_input_filtered * (1.0f - D_FILTER_ALPHA) + d_raw * D_FILTER_ALPHA;
        d_term = p->kd * d_input_filtered;
    }

    output = p_term + i_term + d_term;

    /* 输出限幅 */
    if (output >  p->out_max) output =  p->out_max;
    if (output < -p->out_max) output = -p->out_max;

    /* 输出死区: 角度太小不动作，避免丝杆高频往复抖动 */
    if (fabsf(output) < 0.05f)
        output = 0.0f;

    /* 保存状态 */
    ball_last_err = error;
    ball_out_last = output;

    return output;
}

/* ========== 摄像头数据获取 ========== */
float camera_get_ball_position(void)
{
    if (parsed_data.is_valid)
        return CAM_COORD_TO_MM(parsed_data.dx);  // 坐标 → mm
    return 0.0f;
}

/*
 * Tower_task - 球控制任务（PID闭环）
 * 优先级：高, 周期：50ms（匹配摄像头 ~20fps）
 */
void Tower_task(void)
{
    float ball_pos = 0.0f;
    float error = 0.0f;
    float target_angle = 0.0f;

    /* 球位置一阶低通状态 */
    static float ball_pos_filtered = 0.0f;
    static bool  pos_filter_init  = false;
    static bool  last_enable      = false;

    while (1)
    {
        /* ball_control_enable 翻转时重置滤波器 */
        if (ball_control_enable != last_enable)
        {
            last_enable = ball_control_enable;
            if (ball_control_enable)
                pos_filter_init = false;   // 新使能 → 重新初始化
        }

        if (ball_control_enable)
        {
            /* 1. 获取摄像头球位置 → 一阶低通滤波 */
            ball_pos = camera_get_ball_position();
            if (!pos_filter_init)
            {
                ball_pos_filtered = ball_pos;   // 初始化直接用第一帧
                pos_filter_init = true;
            }
            else
            {
                ball_pos_filtered = ball_pos_filtered * (1.0f - POS_FILTER_ALPHA)
                                  + ball_pos * POS_FILTER_ALPHA;
            }
            error = target_ball_pos - ball_pos_filtered;

            /* 2. PID计算目标倾角 → 执行摆杆控制 */
            target_angle = ball_pid_update(error);
            dbg_target_angle = target_angle;          // 送OLED显示
            if (fabsf(target_angle) > 0.01f)         // 死区内不调丝杆，保持当前倾角
                beam_set_angle(target_angle);

            /* MODE_TASK3 稳定检测: 连续15帧误差 < 5mm → 关闭球控 */
            if (selected_mode == MODE_TASK3)
            {
                static uint8_t stable_cnt = 0;
                if (fabsf(error) < 5.0f)
                {
                    stable_cnt++;
                    if (stable_cnt >= 15)
                    {
                        ball_control_enable = false;
                        Serial_SendString("THREE");
                        stable_cnt = 0;
                    }
                }
                else
                {
                    stable_cnt = 0;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(30));  // ~30fps, 与摄像头帧率同步
    }
}
