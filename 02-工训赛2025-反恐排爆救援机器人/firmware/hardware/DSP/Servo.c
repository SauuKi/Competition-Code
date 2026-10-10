#include "Servo.h"
/*
使用舵机函数时，需要确定好不同舵机的最大角度，然后在头文件中进行修改
*/


/* 
 * TIM8->168MHz->168-1->1 MHz->1us 
 */
#define US_TO_CNT(us)  ((us))

static void GPIO_Config(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(SERVO_GPIOC_CLK, ENABLE);

    /* PC6 PC7 PC8 -> AF3 (TIM8_CH1 CH2 CH3) */
    gpio.GPIO_Pin = SERVO_CH1_PIN | SERVO_CH2_PIN | SERVO_CH3_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(SERVO_GPIOC_PORT, &gpio);
    GPIO_PinAFConfig(SERVO_GPIOC_PORT, GPIO_PinSource6, GPIO_AF_TIM8);
    GPIO_PinAFConfig(SERVO_GPIOC_PORT, GPIO_PinSource7, GPIO_AF_TIM8);
    GPIO_PinAFConfig(SERVO_GPIOC_PORT, GPIO_PinSource8, GPIO_AF_TIM8);
}

static void TIM_Config(void)
{
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;

    RCC_APB2PeriphClockCmd(SERVO_TIM8_CLK, ENABLE);

    /* 统一定时器1 MHz 周期20 000 对应 20 ms 周期 */
    tim.TIM_Prescaler = 168 - 1;         /* 168 MHz / 168 = 1 MHz */
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 20000 - 1;          /* 20 ms */
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0;       /* TIM8需要设置重复计数器 */

    TIM_TimeBaseInit(TIM8, &tim);

    /* 统一 PWM1 模式 */
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 0;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;  /* TIM8需要设置互补极性 */
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC1Init(TIM8, &oc);   /* PC6 */
    TIM_OC2Init(TIM8, &oc);   /* PC7 */
    TIM_OC3Init(TIM8, &oc);   /* PC8 */

    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM8, ENABLE);

    /* TIM8需要启动主输出 */
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    TIM_Cmd(TIM8, ENABLE);
}

/* -------------- 初始化 -------------- */
void SERVO_Init(void)
{
    GPIO_Config();
    TIM_Config();
}

void Servo_SetPWM(uint8_t ch, uint32_t pulse)
{
    if (pulse < MinPulse) pulse = MinPulse;
    if (pulse > MaxPulse) pulse = MaxPulse;

    switch (ch)
    {
        case 1: TIM_SetCompare1(TIM8, US_TO_CNT(pulse)); break;
        case 2: TIM_SetCompare2(TIM8, US_TO_CNT(pulse)); break;
        case 3: TIM_SetCompare3(TIM8, US_TO_CNT(pulse)); break;
    }
}

void Servo_SetAngle(uint8_t ch, float angle)
{
    float max_angle = 0.0f;
    //限幅下限
    if (angle < 0.0f)
        angle = 0.0f;

    switch(ch)
    {
        case 1: max_angle = Servo1_Max_Angle; break;
        case 2: max_angle = Servo2_Max_Angle; break;
        case 3: max_angle = Servo3_Max_Angle; break;
        default: return;  //非法通道直接退出，不输出PWM
    }
    //限幅上限
    if (angle > max_angle)
        angle = max_angle;

    //公式：500us ~ 2500us 对应0~max_angle，+0.5f实现四舍五入
    uint32_t pulse = (uint32_t)(500.0f + angle * 2000.0f / max_angle + 0.5f);
    Servo_SetPWM(ch, pulse);
}

Servo_Motion_t gimbal_servo = {0};

/* 启动舵机非阻塞转动：只设置目标和速度，实际角度在 Servo_UpdateRotate 中推进 */
void Servo_StartRotate(Servo_Motion_t *servo, uint8_t ch, float target_angle, float speed_dps)
{
    if (ch != GIMBAL_SERVO_CH) return;

    if (speed_dps <= 0.0f) speed_dps = GIMBAL_SERVO_SPEED_DPS;
    if (speed_dps > GIMBAL_SERVO_MAX_DPS) speed_dps = GIMBAL_SERVO_MAX_DPS;

    if (target_angle < 0.0f) target_angle = 0.0f;
    if (target_angle > GIMBAL_SERVO_TRAVEL_DEG) target_angle = GIMBAL_SERVO_TRAVEL_DEG;

    servo->ch = ch;
    servo->target_angle = target_angle;
    servo->step = speed_dps * (float)SERVO_ROTATE_PERIOD_MS / 1000.0f;
    if (servo->step < 0.1f) servo->step = 0.1f;

    if (servo->target_angle >= servo->current_angle - servo->step &&
        servo->target_angle <= servo->current_angle + servo->step)
    {
        servo->current_angle = servo->target_angle;
        servo->is_moving = 0;
        Servo_SetAngle(servo->ch, servo->current_angle);
        return;
    }

    servo->is_moving = 1;
    servo->last_update = xTaskGetTickCount();
}

/* 非阻塞更新舵机角度：到达目标返回1，正在运动返回0 */
uint8_t Servo_UpdateRotate(Servo_Motion_t *servo)
{
    if (servo->is_moving == 0) return 1;

    TickType_t now = xTaskGetTickCount();
    if ((now - servo->last_update) < pdMS_TO_TICKS(SERVO_ROTATE_PERIOD_MS)) return 0;
    servo->last_update = now;

    float angle_diff = servo->target_angle - servo->current_angle;
    float abs_diff = (angle_diff >= 0.0f) ? angle_diff : -angle_diff;
    if (abs_diff <= servo->step)
    {
        servo->current_angle = servo->target_angle;
        servo->is_moving = 0;
        Servo_SetAngle(servo->ch, servo->current_angle);
        return 1;
    }

    if (angle_diff > 0.0f) servo->current_angle += servo->step;
    else                   servo->current_angle -= servo->step;

    Servo_SetAngle(servo->ch, servo->current_angle);
    return 0;
}

/* 直接设定云台角度：同步状态并取消正在进行的慢转 */
void Servo_SetCurrentAngle(uint8_t ch, float angle)
{
    if (ch == GIMBAL_SERVO_CH)
    {
        if (angle < 0.0f) angle = 0.0f;
        if (angle > GIMBAL_SERVO_TRAVEL_DEG) angle = GIMBAL_SERVO_TRAVEL_DEG;

        gimbal_servo.ch = ch;
        gimbal_servo.is_moving = 0;
        gimbal_servo.current_angle = angle;
        gimbal_servo.target_angle = angle;
    }

    Servo_SetAngle(ch, angle);
}

