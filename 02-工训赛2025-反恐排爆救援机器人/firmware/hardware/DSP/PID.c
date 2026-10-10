#include "PID.h"
#include <stdint.h>

// 1. 先声明结构体（向前引用）
typedef struct CascadePID CascadePID;

// 2. 声明函数指针类型（使用已声明的结构体）
typedef float (*CascadePID_posFunc)(CascadePID *pid, float now_position,float tar_position);
typedef float (*CascadePID_speedFunc)(CascadePID *pid, int16_t now_speed,float tar_speed, float now_position,float tar_position);
typedef float (*CascadePID_angleFunc)(CascadePID *pid, float now_angle, float tar_angle);

float Motor_tar_pos=0;
float Motor_tar_speed=0;
float Motor_tar_angle=0;

float Motor_tar_speed_left=0;
float Motor_tar_speed_right=0;

int motor_control_mode=0;

// 模式1：固定距离参数
FixedDistParams fixed_dist_params;  
// 模式2：差速转弯参数
DiffTurnParams diff_turn_params;    
// 模式3：四轮分别速度+总路程参数
SingleSpeedParams single_speed_params;


// 3. 完整定义结构体（包含函数指针）
struct CascadePID {
		// 角度环（最外环，位置式PID）
    struct {
        float Err;         // 当前误差
        float last_err;    // 上一次误差
        float Integral;    // 积分累计
        float output;      // 输出（目标位置）
        float p;           // Kp
        float i;           // Ki
        float d;           // Kd
    } angle;

    // 位置环（核心：增强接近目标时的收敛性）
    struct {
        float Err;         // 当前误差（目标位置-当前位置）
        float last_err;    // 上一次误差
        float Integral;    // 积分累计（带分离逻辑）
        float pwm;      // 输出（目标速度）
        float p;           // Kp（动态减小）
        float i;           // Ki（误差小时禁用）
        float d;           // Kd（增强阻尼）
        float max_output;  // 最大目标速度（动态限制）
    } pos;

    // 速度环（核心：动态加减速参数）
    struct {
        float Err;         // 当前误差
        float last_err;    // 上一次误差
        float next_err;    // 上上次误差
        float pwm;         // 最终PWM
        float add;         // 增量值
        float p;           // Kp（误差小时减小）
        float i;           // Ki（误差小时禁用）
        float d;           // Kd（增强阻尼）
    } speed;

    // 梯形加减速参数
    struct {
        float total_distance;  // 总距离（目标-起点）
        float current_speed;   // 当前规划速度
        float max_speed;       // 最大允许速度
        float acceleration;    // 加速度
        float deceleration;    // 减速度
        float accel_phase_dist;// 加速阶段所需距离
        float decel_phase_dist;// 减速阶段所需距离
        float cruise_dist;     // 匀速阶段距离
        float start_pos;       // 记录运动起点位置
        uint8_t is_initialized;// 初始化标志
    } trapz;
		// 到位状态标志（新增）
			uint8_t is_reached;  // 1: 已到位，0: 未到位
		
			float tar_pos;
			float tar_speed; 
			float tar_angle;
		
		

    // 函数指针（使用已定义的函数指针类型）
		CascadePID_angleFunc PID_angle;
    CascadePID_posFunc PID_pos;
		CascadePID_speedFunc PID_speed;
};
// 绝对值函数
 float myabsf(float val) { return val < 0 ? -val : val; }
//static int16_t myabs16(int16_t val) { return val < 0 ? -val : val; }
// 绝对值函数（共用）
 int32_t myabs(int32_t val) {
    return val < 0 ? -val : val;
}


// PWM限幅函数（可根据硬件调整）
static float pwm_control(float pwm) {
    const float MAX_PWM = 8399.0f;  // 最大PWM
    const float MIN_PWM = -8399.0f; // 最小PWM
    return (pwm > MAX_PWM) ? MAX_PWM : (pwm < MIN_PWM ? MIN_PWM : pwm);
}
// 梯形加减速规划
static float trapezoidal_profile(CascadePID *pid, float now_position, float tar_position,float tar) 
{
	// 1. 计算运动方向（1:正向, -1:反向）
    float direction = (tar > 0) ? 1.0f : -1.0f;
	// 目标速度取绝对值后乘以方向，确保方向正确
    float abs_tar_speed = myabsf(tar);
    float signed_tar_speed = abs_tar_speed * direction;

    // 2. 计算当前位置到目标的剩余距离（带方向）
    float remaining = tar_position - now_position;
    // 剩余距离的绝对值（用于判断加速/减速阶段）
    float abs_remaining = myabsf(remaining);
	
	
	// 4. 减速阶段（最后10000单位距离）
		if (abs_remaining <= 50000) {
        // 按减速度降低速度（带方向，减速时加速度与方向相反）
        pid->trapz.current_speed -= pid->trapz.deceleration * direction;
        
        // 限制最小速度（正向不低于500，反向不高于-500）
        if (direction > 0) {
            if (pid->trapz.current_speed <= 300) {
                pid->trapz.current_speed = 300;
            }
        }
				else {
            if (pid->trapz.current_speed >= -300) {
                pid->trapz.current_speed = -300;
            }
        }
    }
		// 3. 加速阶段（前10000单位距离）
   else if (myabsf(now_position) <= 50000) {
        // 按加速度增加速度（带方向）
        pid->trapz.current_speed += pid->trapz.acceleration * direction;
        
        // 限制最大速度（不超过目标速度）
        if (direction > 0) {
            // 正向：不超过正向目标速度
            if (pid->trapz.current_speed >= signed_tar_speed) {
                pid->trapz.current_speed = signed_tar_speed;
            }
        } 
				else {
            // 反向：不超过反向目标速度（更负）
            if (pid->trapz.current_speed <= signed_tar_speed) {
                pid->trapz.current_speed = signed_tar_speed;
            }
        }
    }
    // 5. 匀速阶段（中间部分）
    else {
        pid->trapz.current_speed = signed_tar_speed;
    }

    return pid->trapz.current_speed;
}
/*****************************************************************************************************/
//增量式pid速度控制
float Speed_pid(CascadePID *pid, int16_t now_speed,float tar_speed, float now_position,float tar_position)
{
//    now_speed=myabs(now_speed);
    pid->speed.Err=tar_speed-now_speed;
	  if(pid->speed.Err<5)
			pid->speed.i=0.1;
		else
			pid->speed.i=0.1;
    pid->speed.add=pid->speed.p*(pid->speed.Err-pid->speed.last_err)+
									 pid->speed.i*(pid->speed.Err)+
									 pid->speed.d*(pid->speed.Err+pid->speed.next_err-2*pid->speed.last_err);
    pid->speed.pwm+=pid->speed.add;
    pid->speed.pwm=pwm_control(pid->speed.pwm);
//		pid->speed.pwm=pwm_accel(pid->speed.pwm , now_position ,tar_position);
    pid->speed.next_err=pid->speed.last_err;
    pid->speed.last_err=pid->speed.Err;
    return pid->speed.pwm;
}

/***********************************************************************************************************/
//位置式pid位置控制
float pos_pid(CascadePID *pid, float now_position,float tar_position)
{
//    now_position=myabs(now_position);
    pid->pos.Err=tar_position-now_position;
    if(myabs(pid->pos.Err)<250)
        pid->pos.p=0.01;
    else 
        pid->pos.p=1;
    pid->pos.pwm=pid->pos.p * pid->pos.Err +pid->pos.i * pid->pos.Integral +pid->pos.d * (pid->pos.Err - pid->pos.last_err);
    pid->pos.last_err=pid->pos.Err;
    return pid->pos.pwm;
}
/*****************************************************************************************************/
// 角度环（最外环，位置式PID）
float angle_pid(CascadePID *pid, float now_angle, float tar_angle) 
{
	if(tar_angle==180)
	{
		if(now_angle>=0)
		{
			pid->angle.Err=tar_angle-now_angle;
		}
		else
		{
			pid->angle.Err=-(tar_angle+now_angle);
		}
	}
	else
	{
		pid->angle.Err=tar_angle-now_angle;
	}
	
	 // 积分项累加（带限幅）
	pid->angle.Integral += pid->angle.Err;
	if(pid->angle.Integral >= 10000)        //积分限幅
	{
		pid->angle.Integral =  10000;
	}
	if(pid->angle.Integral <= -10000)
	{
		pid->angle.Integral = -10000;
	}
	
    pid->angle.output = pid->angle.p * pid->angle.Err
                      + pid->angle.i * pid->angle.Integral
                      + pid->angle.d * (pid->angle.Err - pid->angle.last_err);
    
    pid->angle.last_err = pid->angle.Err;//储存上次误差
	
    return -pid->angle.output;  // 输出作为位置环目标
}

// 全局声明，在函数外部定义
CascadePID wheel_pid[4];  // 全局数组，生命周期贯穿整个程序
// 初始化函数
void pid_init(void) {
    // 先清零所有参数，避免随机值干扰
    memset(wheel_pid, 0, sizeof(wheel_pid));
    
	// 左前轮参数（其他轮子同理）
    // 角度环
    wheel_pid[0].angle.p = 200.0f;
    wheel_pid[0].angle.i = 0.5f;
    wheel_pid[0].angle.d = 1.0f;  // 增强微分阻尼
    
    // 位置环
    wheel_pid[0].pos.p = 2.0f;
    wheel_pid[0].pos.i = 0.03f;
    wheel_pid[0].pos.d = 1.5f;    // 增强微分，抑制震荡
    
    // 速度环
    wheel_pid[0].speed.p = 3.0f;
    wheel_pid[0].speed.i = 0.3f;
    wheel_pid[0].speed.d = 1.0f;  // 增强微分，快速减速
   
	// 梯形加减速参数（核心参数，根据设备调整）
    wheel_pid[0].trapz.acceleration = 50.0f;  // 加速度（小一点更平稳）
    wheel_pid[0].trapz.deceleration = 100.0f;  // 减速度（稍大防止过冲）
    wheel_pid[0].trapz.is_initialized = 0;    // 未初始化
		
    wheel_pid[0].PID_pos = pos_pid;
    wheel_pid[0].PID_speed = Speed_pid;
		wheel_pid[0].PID_angle=angle_pid;
		
		wheel_pid[0].is_reached = 0;
		wheel_pid[0].tar_pos=0;
		wheel_pid[0].tar_speed=0;
		wheel_pid[0].tar_angle=0;
		
    // 初始化右前轮（复制左前轮参数，可单独修改）
    wheel_pid[1] = wheel_pid[0];
    
    // 初始化左后轮
    wheel_pid[2] = wheel_pid[0];
    
    // 初始化右后轮
    wheel_pid[3] = wheel_pid[0];
		
		 // 初始化到位标志（新增）
  
}
float limit(float now,float tar)
{
    if(now>tar)
        now=tar;
    if(now<-tar)
        now=-tar;
    return now;
}

//整体走直线，固定距离，固定速度
float Mode_fixed_distance (int wheel_idx, float now_pos, int16_t now_speed, float tar_pos, float tar_speed, float tar_angle) 
{
    if (wheel_idx < 0 || wheel_idx >= 4) return 0.0f;  // 边界检查
    
    CascadePID *pid = &wheel_pid[wheel_idx];
    float pwm=0;
	
	 // 1. 计算梯形加减速的目标速度
    float trapz_speed = trapezoidal_profile(pid, now_pos, tar_pos,tar_speed);
	
    // 1. 位置环计算（输出作为速度环的目标速度）
    pwm = pid->PID_pos(pid, now_pos, tar_pos);
		pwm=limit(pwm,trapz_speed);
		if(wheel_idx==0||wheel_idx==1)
		{
			pwm=pwm+pid->PID_angle(pid,Yaw,tar_angle);
		}
		else if(wheel_idx==2||wheel_idx==3)
		{
			pwm=pwm-pid->PID_angle(pid,Yaw,tar_angle);
		}
		if(myabs(tar_pos-now_pos)<250)
		{
			pwm=0;
			pid->is_reached = 1;  // 置位到位标志
		}	
		else
		{
			// 2. 速度环计算（使用位置环的输出作为目标速度）
			pwm=pid->PID_speed(pid, now_speed, pwm, now_pos, tar_pos);
			pid->is_reached = 0;  // 清零到位标志
		}
		
    return pwm;
}

//差速转弯，到目标角度
float Mode_diff_turn(int wheel_idx, int16_t now_speed, float tar_speed, float tar_angle) 
{
    if (wheel_idx < 0 || wheel_idx >= 4) return 0.0f;  // 边界检查
    
    CascadePID *pid = &wheel_pid[wheel_idx];
    float pwm=0;
	float angle_error=0;
	float actual_tar_speed = tar_speed;  // 实际使用的目标速度（可能被修正）
	
		// 1. 标准化角度误差（核心：确保误差在[-180°, 180°]，避免多转一圈）
    angle_error = tar_angle - Yaw;
    // 当误差超过180°或小于-180°时，取最短路径
    if (angle_error > 180.0f) {
        angle_error -= 360.0f;
    } else if (angle_error < -180.0f) {
        angle_error += 360.0f;
    }
		
		// 2. 根据角度误差计算目标速度（速度环主导控制）
    if (myabsf(angle_error) < 1.0f) {
        // 误差很小，停止转动
        actual_tar_speed = 0.0f;
        pid->is_reached = 1;  // 置位到位标志
    }
	 else {
			// 提取外部指定的主方向（tar_speed的正负）
			int main_direction = (tar_speed > 0) ? 1 : -1;  // 1=顺时针，-1=逆时针
			
			// 误差方向（1=需要顺时针转，-1=需要逆时针转）
			int error_direction = (angle_error > 0) ? -1 : 1;
			
			// 如果误差方向与主方向相反（超调），临时反向修正
			if (error_direction != main_direction) {
					actual_tar_speed = -tar_speed;  // 反转速度方向，修正超调
			}
			// 否则，遵循外部指定的主方向（actual_tar_speed = tar_speed）
			pid->is_reached = 0;  // 清零到位标志
    }
		
		// 3. 根据轮子索引调整方向（差速逻辑：左右轮反向转动实现转弯）
    // 假设0/1是左轮，2/3是右轮（根据实际硬件调整）
    if (wheel_idx == 0 || wheel_idx == 1) {
        // 左轮：与目标速度同向
        pwm = pid->PID_speed(pid, now_speed, actual_tar_speed, 0, 0);
    } else {
        // 右轮：与目标速度反向（差速转弯核心）
        pwm = pid->PID_speed(pid, now_speed, -actual_tar_speed, 0, 0);
    }
		
    return pwm;
}

////4轮分别速度控制，运行到目标距离

//float Mode_single_speed(int wheel_idx, float now_pos, int16_t now_speed, float tar_pos, float tar_speed) 
//{	
//    if (wheel_idx < 0 || wheel_idx >= 4) return 0.0f;  // 边界检查
//    
//    CascadePID *pid = &wheel_pid[wheel_idx];
//    float pwm = 0;
//    
//    // 1. 计算梯形加减速的目标速度（参考第一个函数）
//    float trapz_speed = trapezoidal_profile(pid, now_pos, tar_pos, tar_speed);
//    
//    // 2. 位置环计算（输出作为速度环的目标速度）
//    pwm = pid->PID_pos(pid, now_pos, tar_pos);
//    
//    // 3. 限制位置环输出在梯形速度范围内（参考第一个函数）
//    pwm = limit(pwm, trapz_speed);
//    
//    if(myabs(tar_pos - now_pos) < 250)  // 到位判断
//    {
//        pwm = 0;
//        pid->is_reached = 1;  // 置位到位标志
//    }	
//    else
//    {
//        // 4. 速度环计算（使用限制后的pwm作为目标速度）
//        pwm = pid->PID_speed(pid, now_speed, pwm, now_pos, tar_pos);
//        pid->is_reached = 0;  // 清零到位标志
//    }
//    
//    return pwm;
//}
float Mode_single_speed(int wheel_idx	,	float now_pos	, int16_t now_speed	, float tar_pos	, float tar_speed) 
{	
    if (wheel_idx < 0 || wheel_idx >= 4) return 0.0f;  // 边界检查
    
    CascadePID *pid = &wheel_pid[wheel_idx];
    float pwm=0;
    // 1. 位置环计算（输出作为速度环的目标速度）
    pwm = pid->PID_pos(pid, now_pos, tar_pos);

		pwm = pwm > 0 ? tar_speed : (-tar_speed);
		
		if(myabs(tar_pos-now_pos)<250)//未到位
		{
			pwm=0;
			pid->is_reached = 1;  // 置位到位标志
		}	
		else
		{
			// 2. 速度环计算（使用位置环的输出作为目标速度）
			pwm=pid->PID_speed(pid, now_speed, pwm, 0, 0);
			pid->is_reached = 0;  // 清零到位标志
		}
    return pwm;
}
//整体走直线,固定角度
float Mode_fixed (int wheel_idx, float tar_speed, float tar_angle) 
{
    if (wheel_idx < 0 || wheel_idx >= 4) return 0.0f;  // 边界检查
    
    CascadePID *pid = &wheel_pid[wheel_idx];
    float pwm=0;
	
		if(wheel_idx==0||wheel_idx==1)
		{
			pwm=tar_speed+pid->PID_angle(pid,Yaw,tar_angle);
		}
		else if(wheel_idx==2||wheel_idx==3)
		{
			pwm=tar_speed-pid->PID_angle(pid,Yaw,tar_angle);
		}
   
		pid->is_reached = 0;  // 清零到位标志
			
    return pwm;
}
// 查询单个轮子是否到位（返回1:到位，0:未到位）
uint8_t check_wheel_reached(int wheel_idx) {
    if (wheel_idx < 0 || wheel_idx >= 4) return 0;
    return wheel_pid[wheel_idx].is_reached;
}

// 查询所有轮子是否都到位（返回1:全到位，0:未全到位）
uint8_t check_all_reached(void) {
	// 所有轮子的到位标志都为1时，返回1
	if(motor_control_mode==1)
	{
		return (wheel_pid[0].is_reached && 
					wheel_pid[1].is_reached && 
					wheel_pid[2].is_reached && 
					wheel_pid[3].is_reached)&&
					(Motor_tar_pos<=Speed_sum+250)&&
					(Motor_tar_pos>=Speed_sum-250)	? 1 : 0;
	}
	else if(motor_control_mode==2)
	{
		return (wheel_pid[0].is_reached && 
					wheel_pid[1].is_reached && 
					wheel_pid[2].is_reached && 
					wheel_pid[3].is_reached) ? 1 : 0;
	}
	else if(motor_control_mode==3)
	{
		return (wheel_pid[0].is_reached && 
					wheel_pid[1].is_reached && 
					wheel_pid[2].is_reached && 
					wheel_pid[3].is_reached)&&
					(Motor_tar_pos<=Speed_sum+250)&&
					(Motor_tar_pos>=Speed_sum-250)	? 1 : 0;
	}
	else return 1;
	
}

// 到位检测滤波函数（模仿按键消抖逻辑）
// 功能：只有连续两次检测到位（中间加延时过滤波动），才认为真正到位
// 返回值：1-稳定到位，0-未到位或波动
uint8_t check_all_reached_filter(void) 
{
    // 第一次检测是否到位
    if (check_all_reached() == 1) {
        // 延时一段时间（过滤短暂波动，类似按键消抖）
        vTaskDelay(20);  // 延时20ms，可根据实际波动情况调整
        
        // 第二次检测，如果仍然到位，说明状态稳定
        if (check_all_reached() == 1) {
            return 1;  // 稳定到位
        }
    }
    // 任意一次检测不到位，都认为未稳定
    return 0;
}

int pwm_output[4] = {0};              // 四个轮子的输出PWM
void Motor_control_fixed_distance(float now_pos, int16_t *now_speed, void *params)
{
	FixedDistParams *p = (FixedDistParams*)params;
	// 计算每个轮子的控制量//位置//速度
	pwm_output[0] = (int)Mode_fixed_distance(0, now_pos, now_speed[0]	, p->tar_pos, p->tar_speed, p->tar_angle);
	pwm_output[1] = (int)Mode_fixed_distance(1, now_pos, now_speed[1]	, p->tar_pos, p->tar_speed, p->tar_angle);
	pwm_output[2] = (int)Mode_fixed_distance(2, now_pos, now_speed[2]	, p->tar_pos, p->tar_speed, p->tar_angle);
	pwm_output[3] = (int)Mode_fixed_distance(3, now_pos, now_speed[3]	, p->tar_pos, p->tar_speed, p->tar_angle);

	Load(pwm_output[0],pwm_output[1],pwm_output[2],pwm_output[3]);
} 

void Motor_control_diff_turn(int16_t *now_speed,void *params)
{
	DiffTurnParams *p = (DiffTurnParams*)params;
	// 计算每个轮子的控制量//位置//速度
	pwm_output[0] = (int)Mode_diff_turn(0, now_speed[0] , p->tar_speed_left, p->tar_angle);
	pwm_output[1] = (int)Mode_diff_turn(1, now_speed[1] , p->tar_speed_left, p->tar_angle);
	pwm_output[2] = (int)Mode_diff_turn(2, now_speed[2] , p->tar_speed_right, p->tar_angle);
	pwm_output[3] = (int)Mode_diff_turn(3, now_speed[3] , p->tar_speed_right, p->tar_angle);

	Load(pwm_output[0],pwm_output[1],pwm_output[2],pwm_output[3]);
}

void Motor_control_single_speed(float now_pos,int16_t *now_speed, void *params)
{
	SingleSpeedParams *p = (SingleSpeedParams*)params;
	// 计算每个轮子的控制量//位置//速度
	pwm_output[0] = (int)Mode_single_speed(0, now_pos, now_speed[0], Motor_tar_pos, p->tar_speed_single[0]);
	pwm_output[1] = (int)Mode_single_speed(1, now_pos, now_speed[1], Motor_tar_pos, p->tar_speed_single[1]);
	pwm_output[2] = (int)Mode_single_speed(2, now_pos, now_speed[2], Motor_tar_pos, p->tar_speed_single[2]);
	pwm_output[3] = (int)Mode_single_speed(3, now_pos, now_speed[3], Motor_tar_pos, p->tar_speed_single[3]);

	Load(pwm_output[0],pwm_output[1],pwm_output[2],pwm_output[3]);
}
void Motor_control_fixed( void *params)
{
	FixedDistParams *p = (FixedDistParams*)params;
	// 计算每个轮子的控制量//位置//速度
	pwm_output[0] = (int)Mode_fixed(0	, p->tar_speed, p->tar_angle);
	pwm_output[1] = (int)Mode_fixed(1	, p->tar_speed, p->tar_angle);
	pwm_output[2] = (int)Mode_fixed(2	, p->tar_speed, p->tar_angle);
	pwm_output[3] = (int)Mode_fixed(3	, p->tar_speed, p->tar_angle);

	Load(pwm_output[0],pwm_output[1],pwm_output[2],pwm_output[3]);
} 
//整体走直线，固定距离，固定速度
void motor_fixed_distance_params(float tar_pos, float tar_speed, float tar_angle)
{
	taskENTER_CRITICAL(); // 进入临界区
	motor_control_mode=1;
	fixed_dist_params.tar_pos=tar_pos;
	fixed_dist_params.tar_speed=tar_speed;
	fixed_dist_params.tar_angle=tar_angle;
	Motor_tar_pos=tar_pos;
	taskEXIT_CRITICAL(); // 退出临界区
}

//差速转弯，到目标角度
void motor_diff_turn_params( 	float tar_speed_left, float tar_speed_right, 
															float tar_angle)
{
	taskENTER_CRITICAL(); // 进入临界区
	motor_control_mode=2;
	diff_turn_params.tar_speed_left=tar_speed_left;
	diff_turn_params.tar_speed_right=tar_speed_right;
	diff_turn_params.tar_angle=tar_angle;
	taskEXIT_CRITICAL(); // 退出临界区
}

//4轮分别速度控制，运行到目标距离
void motor_single_speed_params( float tar_speed_left_qian, float tar_speed_left_hou,
																float tar_speed_right_qian,float tar_speed_right_hou, 
																float tar_pos)
{
	taskENTER_CRITICAL(); // 进入临界区
	motor_control_mode=3;
	single_speed_params.tar_speed_single[0]=tar_speed_left_qian;
	single_speed_params.tar_speed_single[1]=tar_speed_left_hou;
	single_speed_params.tar_speed_single[2]=tar_speed_right_qian;
	single_speed_params.tar_speed_single[3]=tar_speed_right_hou;
	single_speed_params.tar_total_pos=tar_pos;
	Motor_tar_pos=tar_pos;
	taskEXIT_CRITICAL(); // 退出临界区
}

//整体走直线,固定角度
void motor_fixed_params( float tar_speed, float tar_angle)
{
	taskENTER_CRITICAL(); // 进入临界区
	motor_control_mode=4;
	fixed_dist_params.tar_pos=0;
	fixed_dist_params.tar_speed=tar_speed;
	fixed_dist_params.tar_angle=tar_angle;
	Motor_tar_pos=0;
	taskEXIT_CRITICAL(); // 退出临界区
}
//清零函数
void motor_clear_and_init_params(void)
{
	taskENTER_CRITICAL(); // 进入临界区
	pid_init();
	motor_control_mode=0;
	Load(0,0,0,0);
	Speed_sum=0;
	taskEXIT_CRITICAL(); // 退出临界区
}
