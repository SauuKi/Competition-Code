#include "Servo.h"

void Servo_Init(void)
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
}

/*
1为底盘舵机(360)，3为夹爪舵机(180)
*/
void Servo_setpwm(uint8_t ch, uint32_t pulse)
{
	if (pulse < MinPulse) pulse = MinPulse;
    if (pulse > MaxPulse) pulse = MaxPulse;
	
	switch(ch)
	{
		case 1:__HAL_TIM_SET_COMPARE (&htim1,TIM_CHANNEL_1 ,pulse);break;
		case 3:__HAL_TIM_SET_COMPARE (&htim8,TIM_CHANNEL_3 ,pulse);break;
	}
}

void Set_Angle(uint8_t ch, float Angle)
{
    float min_angle, max_angle;
    uint16_t min_pulse, max_pulse;
    
    // 1. 通道合法性检查
    if (ch != 1 && ch != 3) {
        return;  // 无效通道直接返回
    }
    
    // 2. 根据通道选择对应参数
    if (ch == 1) {
        min_angle = SERVO_CH1_MIN_ANGLE;
        max_angle = SERVO_CH1_MAX_ANGLE;
        min_pulse = SERVO_CH1_MIN_PULSE;
        max_pulse = SERVO_CH1_MAX_PULSE;
    } 
	else if (ch == 3) {
        min_angle = SERVO_CH2_MIN_ANGLE;
        max_angle = SERVO_CH2_MAX_ANGLE;
        min_pulse = SERVO_CH2_MIN_PULSE;
        max_pulse = SERVO_CH2_MAX_PULSE;
    }
    
    // 3. 角度边界限幅
    if (Angle < min_angle) {
        Angle = min_angle;
    } else if (Angle > max_angle) {
        Angle = max_angle;
    }
    
    // 4. 精确脉冲计算
    float pulse_float = min_pulse + (Angle - min_angle) * (max_pulse - min_pulse) / (max_angle - min_angle);
    uint16_t pulse = (uint16_t)roundf(pulse_float);
    
    // 5. 最终脉冲限幅
    if (pulse < min_pulse) {
        pulse = min_pulse;
    } else if (pulse > max_pulse) {
        pulse = max_pulse;
    }
    
    // 6. 设置PWM输出
    Servo_setpwm(ch, pulse);
}

//void rotate_angle_slow(float Current_Angle,float Angle)
//{
//	if(Current_Angle<Angle)
//	{
//		float now_angle=Current_Angle;
//		for(int i=0;i<(Angle-Current_Angle);i++)
//		{
//			now_angle+=1;
//			Set_Angle(Gimbal_Servo_ID,now_angle);
//			vTaskDelay(10);
//		}
//	}
//	else if(Current_Angle>Angle)
//	{
//		float now_angle=Current_Angle;
//		for(int i=0;i<(Current_Angle-Angle);i++)
//		{
//			now_angle-=1;
//			Set_Angle(Gimbal_Servo_ID,now_angle);
//			vTaskDelay(10);
//		}
//	}
//}

// 静态数组，自动维护每个舵机的当前角度（封装在函数内部，不暴露全局）
static float servo_current_angle[MAX_SERVO_CHANNELS] = {0.0f};

/**
 * @brief  初始化舵机角度（系统启动时调用一次）
 * @param  ch: 舵机通道号
 * @param  init_angle: 初始角度
 * @retval 无
 */
void Servo_Init_Angle(uint8_t ch, float init_angle)
{
    if (ch >= MAX_SERVO_CHANNELS) return;
    
    servo_current_angle[ch] = init_angle;
    Set_Angle(ch, init_angle);
}

/**
 * @brief  获取舵机当前角度
 * @param  ch: 舵机通道号
 * @retval 当前角度
 */
float Servo_Get_Current_Angle(uint8_t ch)
{
    if (ch >= MAX_SERVO_CHANNELS) return 0.0f;
    
    return servo_current_angle[ch];
}

/**
 * @brief  舵机平滑转动函数（自动维护当前角度，阻塞式）
 * @param  ch: 舵机通道号
 * @param  target_angle: 目标角度
 * @param  speed_deg_per_sec: 转动速度（度/秒），推荐范围：10-200
 * @retval 无
 */
void rotate_angle_slow(uint8_t ch, float target_angle, float speed_deg_per_sec)
{
    // 1. 参数合法性检查
    if (ch >= MAX_SERVO_CHANNELS) return;
    if (speed_deg_per_sec <= 0.0f) speed_deg_per_sec = 50.0f; // 默认50度/秒
    if (speed_deg_per_sec > 300.0f) speed_deg_per_sec = 300.0f; // 最大速度限制
    
    float current_angle = servo_current_angle[ch];
    if (fabsf(target_angle - current_angle) < 0.1f) return;   // 角度差太小，直接返回
    
    // 2. 计算运动参数
    const float angle_diff = target_angle - current_angle;
    const int8_t direction = (angle_diff > 0.0f) ? 1 : -1;
    
    // 3. 动态计算步长和延迟（保证速度恒定，同时兼顾平滑性）
    const TickType_t min_delay_ms = 10; // 最小延迟10ms（FreeRTOS调度最小粒度）
    const float step_per_delay = speed_deg_per_sec * min_delay_ms / 1000.0f;
    const float step = (step_per_delay < 0.1f) ? 0.1f : step_per_delay; // 最小步长0.1度
    
    // 4. 平滑运动循环
    float now_angle = current_angle;
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (fabsf(target_angle - now_angle) > step)
    {
        now_angle += direction * step;
        Set_Angle(ch, now_angle);
        servo_current_angle[ch] = now_angle; // 更新当前角度
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(min_delay_ms));
    }
    
    // 5. 最后一步直接跳到目标角度（消除累计误差）
    Set_Angle(ch, target_angle);
    servo_current_angle[ch] = target_angle; // 最终更新当前角度
}

Servo_Motion_t gimbal_servo = {0};

/**
 * @brief  启动舵机平滑转动（非阻塞式）
 * @param  servo: 舵机运动状态结构体指针
 * @param  ch: 舵机通道号
 * @param  target_angle: 目标角度
 * @param  speed_deg_per_sec: 转动速度（度/秒）
 * @retval 无
 */
void servo_start_rotate(Servo_Motion_t *servo, uint8_t ch, float target_angle, float speed_deg_per_sec)
{
    if (servo == NULL || speed_deg_per_sec <= 0.0f) return;
    
    servo->ch = ch;
    servo->target_angle = target_angle;
    servo->speed = speed_deg_per_sec;
    servo->step = speed_deg_per_sec * 10 / 1000.0f; // 10ms更新一次
    if (servo->step < 0.1f) servo->step = 0.1f;
    
    servo->is_moving = true;
    servo->last_update = xTaskGetTickCount();
}

/**
 * @brief  舵机运动更新函数（需要在任务中定期调用）
 * @param  servo: 舵机运动状态结构体指针
 * @retval true: 已到达目标角度，false: 正在运动中
 */
bool servo_update_rotate(Servo_Motion_t *servo)
{
    if (servo == NULL || !servo->is_moving) return true;
    
    TickType_t now = xTaskGetTickCount();
    if (now - servo->last_update >= pdMS_TO_TICKS(10))
    {
        servo->last_update = now;
        
        float angle_diff = servo->target_angle - servo->current_angle;
        if (fabsf(angle_diff) < servo->step)
        {
            // 到达目标角度
            servo->current_angle = servo->target_angle;
            Set_Angle(servo->ch, servo->current_angle);
            servo->is_moving = false;
            return true;
        }
        
        // 更新角度
        int8_t direction = (angle_diff > 0.0f) ? 1 : -1;
        servo->current_angle += direction * servo->step;
        Set_Angle(servo->ch, servo->current_angle);
    }
    
    return false;
}

