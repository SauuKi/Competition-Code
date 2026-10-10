#include "Controls.h"

// ========== 步进位置变量 ==========
float Motor_Cur_Pos_robot_arm_Z = 0.0f;    // Z轴(丝杆升降, 摆杆驱动)
float Motor_Cur_Pos_robot_arm_Y = 0.0f;    // Y轴(齿条伸缩)
volatile int32_t Emm_revolutions[7];        // 脉冲累计

// ========== 夹爪/摆杆高度状态 ==========
float Current_Claw_Height = MAX_HEIGHT;
float Current_Claw_Height_Relative = MAX_HEIGHT;

/* ================================================================
 * 工具函数
 * ================================================================ */

/*
 * 公式: 角度 = Z值 × (180 / π)
 */
float ConvertToAngle(float z)
{
    return (float)z * (180.0f / PI);
}

/**
 * 弧度转角度
 */
float radians_to_degrees(float radians)
{
    return radians * (180.0 / PI);
}

/**
 * 角度转弧度
 */
float degrees_to_radians(float degrees)
{
    return degrees * (PI / 180.0);
}

/**
 * 浮点数绝对值
 */
float Absolute_value(float yaw)
{
    if (yaw < 0) yaw = -yaw;
    return yaw;
}

/* ================================================================
 * 步进数据解包
 * ================================================================ */

/**
 * 串口数据解包函数：处理电机位置(0x36)
 * 仅处理 Z轴(addr=5) 和 Y轴(addr=6) 的位置数据
 */
void Emm_Data_Unpack(uint8_t Emm_V5_rxCount, uint8_t Emm_V5_rxCmd[128])
{
    uint8_t addr = Emm_V5_rxCmd[0];  // 电机地址

    // 处理位置数据 0x36
    if (Emm_V5_rxCmd[1] == 0x36)
    {
        // 数据长度校验 + 仅处理addr 5/6
        if (Emm_V5_rxCount != 8 || addr < 5 || addr > 6)
            return;

        // 拼接32位位置原始值
        uint32_t pos = (uint32_t)(
                ((uint32_t)Emm_V5_rxCmd[3] << 24) |
                ((uint32_t)Emm_V5_rxCmd[4] << 16) |
                ((uint32_t)Emm_V5_rxCmd[5] << 8)  |
                ((uint32_t)Emm_V5_rxCmd[6] << 0)
        );

        // 位置单位转换（pos → pulse）
        int32_t pulse = (float)pos * 360.0f / 65536.0f;
        pulse = (int32_t)(6400 * (pulse / 360.0f));

        // 符号判断 + 写入对应位置变量
        if (addr == 5)
        {
            Motor_Cur_Pos_robot_arm_Z = (Emm_V5_rxCmd[2] == 0) ? -pulse : pulse;
        }
        else if (addr == 6)
        {
            Motor_Cur_Pos_robot_arm_Y = (Emm_V5_rxCmd[2] == 0) ? -pulse : pulse;
        }
    }
}

/* ================================================================
 * Z轴控制 (丝杆升降, 摆杆角度驱动)
 * ================================================================ */

/**
 * Z轴相对模式：指定方向走指定脉冲
 */
void Set_Pos_Z(uint8_t dir, int32_t clk, uint16_t v, uint8_t a)
{
    taskENTER_CRITICAL();

    if (dir == Z_UP)
    {
        Emm_V5_Pos_Control(5, Z_UP, v, a, clk, 1, 0);
        Emm_revolutions[4] = clk;
    }
    else if (dir == Z_DOWN)
    {
        Emm_V5_Pos_Control(5, Z_DOWN, v, a, clk, 1, 0);
        Emm_revolutions[4] = -clk;
    }
    else
    {
        Emm_V5_Stop_Now(5, 0);
    }

    taskEXIT_CRITICAL();
    osDelay(1);
}

/**
 * Z轴绝对模式：正值为升，负值为降
 */
void Set_Clk_Z(int32_t clk, uint16_t v, uint8_t a)
{
    taskENTER_CRITICAL();
    if (clk > 0) {
        Emm_V5_Pos_Control(5, Z_UP, v, a, clk, 0, 0);
        vTaskDelay(1);
        Emm_revolutions[4] = Emm_revolutions[4] + clk;
    }
    else if (clk < 0) {
        Emm_V5_Pos_Control(5, Z_DOWN, v, a, -clk, 0, 0);
        vTaskDelay(1);
        Emm_revolutions[4] = Emm_revolutions[4] + clk;
    }
    else {
        Emm_V5_Stop_Now(5, 0);
        vTaskDelay(1);
    }
    taskEXIT_CRITICAL();
    osDelay(1);
}

/**
 * 控制摆杆到指定高度(厘米)
 * target_height: 0.0 ~ MAX_HEIGHT
 */
void Set_Claw_Height(float target_height)
{
    if (target_height < 0.0f) target_height = 0.0f;
    if (target_height > MAX_HEIGHT) target_height = MAX_HEIGHT;

    float pulse_val = (MAX_HEIGHT - target_height) * PULSE_PER_CM;
    int32_t fixed_pulse = (int32_t)(pulse_val + 0.5f);

    if (target_height == Current_Claw_Height) return;

    if (target_height > Current_Claw_Height)
    {
        Set_Pos_Z(Z_UP, fixed_pulse, CLAW_SPEED, CLAW_ACC);
    }
    else
    {
        Set_Pos_Z(Z_DOWN, fixed_pulse, CLAW_SPEED, CLAW_ACC);
    }

    Current_Claw_Height_Relative = target_height;
}

/**
 * 控制摆杆到指定高度(厘米) - 相对脉冲版本
 */
void Set_Claw_Height_Relative(float target_height, uint16_t v, uint8_t a)
{
    if (target_height < 0.0f) target_height = 0.0f;
    if (target_height > MAX_HEIGHT) target_height = MAX_HEIGHT;

    if (fabsf(target_height - Current_Claw_Height_Relative) < 0.01f) return;

    float height_diff = target_height - Current_Claw_Height_Relative;
    float pulse_val = height_diff * PULSE_PER_CM;
    int32_t fixed_pulse = (int32_t)(pulse_val + 0.5f);

    Set_Clk_Z(fixed_pulse, v, a);

    Current_Claw_Height_Relative = target_height;
}

/**
 * 回零: Arr=5 回Z轴零位, Arr=6 回Y轴零位
 */
void Back_Zero(int Arr, uint16_t v, uint8_t a)
{
    if (Arr == 5)
    {
        Emm_V5_Pos_Control(5, 1, v, a, 0, 1, 0);
        Emm_revolutions[4] = 0;
        Current_Claw_Height_Relative = MAX_HEIGHT;
        vTaskDelay(1);
    }
    else if (Arr == 6)
    {
        Emm_V5_Pos_Control(6, 0, 50, 0, 0, 1, 0);
        Emm_revolutions[5] = 0;
        vTaskDelay(1);
    }
}

/* ================================================================
 * Y轴控制 (齿条伸缩)
 * ================================================================ */

/**
 * Y轴相对模式：指定方向走指定脉冲
 */
void Set_Pos_Y(uint8_t dir, int32_t clk, uint16_t v, uint8_t a)
{
    taskENTER_CRITICAL();

    if (dir == Y_SHEN)
    {
        Emm_V5_Pos_Control(6, Y_SHEN, v, a, clk, 1, 0);
        Emm_revolutions[5] = clk;
    }
    else if (dir == Y_SHUO)
    {
        Emm_V5_Pos_Control(6, Y_SHUO, v, a, clk, 1, 0);
        Emm_revolutions[5] = -clk;
    }
    else
    {
        Emm_V5_Stop_Now(6, 0);
    }

    taskEXIT_CRITICAL();
    osDelay(1);
}

/**
 * Y轴绝对模式：正值为伸，负值为缩
 */
void Set_Clk_Y(int32_t clk, uint16_t v, uint8_t a)
{
    taskENTER_CRITICAL();
    if (clk > 0) {
        Emm_V5_Pos_Control(6, Y_SHEN, v, a, clk, 0, 0);
        vTaskDelay(1);
        Emm_revolutions[5] = Emm_revolutions[5] + clk;
    }
    else if (clk < 0) {
        Emm_V5_Pos_Control(6, Y_SHUO, v, a, -clk, 0, 0);
        vTaskDelay(1);
        Emm_revolutions[5] = Emm_revolutions[5] + clk;
    }
    else {
        Emm_V5_Stop_Now(6, 0);
        vTaskDelay(1);
    }
    taskEXIT_CRITICAL();
    osDelay(1);
}

/**
 * 获取齿条当前位置(mm)
 */
float Get_Rack_Position_Y(void)
{
    return (float)Emm_revolutions[5] / PULSE_PER_MM;
}

/**
 * 齿条绝对位置移动(mm)
 */
int8_t Move_Rack_Absolute_Y(float target_mm, uint16_t v, uint8_t a)
{
    if (target_mm > RACK_MAX_EXTEND) {
        target_mm = RACK_MAX_EXTEND;
        return -1;
    }
    if (target_mm < RACK_MIN_RETRACT) {
        target_mm = RACK_MIN_RETRACT;
        return -1;
    }

    float current_mm = Get_Rack_Position_Y();
    float delta_mm = target_mm - current_mm;
    int32_t delta_pulse = (int32_t)(delta_mm * PULSE_PER_MM + 0.5f);

    if (abs(delta_pulse) < 2) {
        return 0;
    }

    Set_Clk_Y(delta_pulse, v, a);
    return 0;
}

/**
 * 齿条绝对位置移动(直接指定目标mm值)
 */
int8_t Move_Rack_Absolute_Y_One(float target_mm, uint16_t v, uint8_t a)
{
    if (target_mm > RACK_MAX_EXTEND) {
        target_mm = RACK_MAX_EXTEND;
        return -1;
    }
    if (target_mm < RACK_MIN_RETRACT) {
        target_mm = RACK_MIN_RETRACT;
        return -1;
    }

    float delta_mm = target_mm;
    int32_t delta_pulse = (int32_t)(delta_mm * PULSE_PER_MM + 0.5f);

    if (abs(delta_pulse) < 2) {
        return 0;
    }

    uint8_t dir;
    if (delta_pulse > 0) dir = 1;
    else dir = 0;

    Set_Pos_Y(dir, delta_pulse, v, a);
    return 0;
}

/**
 * 齿条相对位置移动(mm): 正值前伸，负值回收
 */
int8_t Move_Rack_Relative_Y(float delta_mm, uint16_t v, uint8_t a)
{
    float target_mm = Get_Rack_Position_Y() + delta_mm;
    return Move_Rack_Absolute_Y(target_mm, v, a);
}

/* ================================================================
 * 摆杆控制 (球平衡)
 * ================================================================ */

/**
 * 角度 → Z轴脉冲换算
 *   摆杆长250mm, 丝杆导程8mm
 *   脉冲 = 250 × tan(θ°) × (6400 / 8)
 */
int32_t beam_angle_to_pulse(float angle_deg)
{
    /* 限幅 */
    if (angle_deg >  BEAM_ANGLE_LIMIT) angle_deg =  BEAM_ANGLE_LIMIT;
    if (angle_deg < -BEAM_ANGLE_LIMIT) angle_deg = -BEAM_ANGLE_LIMIT;

    float angle_rad = angle_deg * (PI / 180.0f);
    float delta_mm  = BEAM_LENGTH_MM * tanf(angle_rad);
    int32_t pulse   = (int32_t)(delta_mm * PULSE_PER_MM_Z + 0.5f);  // 四舍五入

    return pulse;
}

/**
 * 设定摆杆角度（停止当前运动 → 发新指令）
 *   正值=摆杆末端升高(球向内滚), 负值=降低(球向外滚)
 */
void beam_set_angle(float angle_deg)
{
    int32_t delta_pulse;
    uint16_t speed;

    /* 角度接近0 → 直接回零 */
    if (fabsf(angle_deg) < 0.05f)
    {
        Back_Zero(5, BEAM_SPEED, BEAM_ACC);
        return;
    }

    /* 计算脉冲 → 相对运动 */
    delta_pulse = beam_angle_to_pulse(angle_deg);
    if (delta_pulse != 0)
    {
        uint8_t dir = (delta_pulse > 0) ? Z_UP : Z_DOWN;
        uint32_t pulse_abs = (delta_pulse > 0) ? delta_pulse : -delta_pulse;

#if BEAM_SPEED_DYNAMIC
        /* 动态速度: 脉冲越多速度越快, 限幅在 [MIN, MAX] */
        speed = BEAM_SPEED_MIN + (uint16_t)((float)pulse_abs * BEAM_SPEED_K);
        if (speed > BEAM_SPEED_MAX) speed = BEAM_SPEED_MAX;
#else
        /* 固定速度 */
        speed = BEAM_SPEED;
#endif

        Set_Pos_Z(dir, pulse_abs, speed, BEAM_ACC);
    }
}
