#include "Controls.h"

float Motor_Cur_Pos_wheel[4]={0.0f,0.0f,0.0f,0.0f};//底盘四个电机的位置
float Motor_Cur_Pos_robot_arm_Z = 0.0f;           
float Motor_Cur_Pos_robot_arm_Y = 0.0f;
float Motor_Cur_Pos_robot_arm_T = 0.0f;//转盘
volatile int32_t Emm_revolutions [7];

/*
公式:角度 = Z值 × (180 / π)
*/
float ConvertToAngle(float z)
{
    return (float)z * (180.0f /PI);
}

/*--------------------步进数据解包--------------------*/
// 串口数据解包函数：处理电机位置(0x36)和速度(0x35)数据
void Emm_Data_Unpack(uint8_t Emm_V5_rxCount, uint8_t Emm_V5_rxCmd[128])
{
    // 位置映射：电机地址1~6 → 对应存储变量
    static float * const motor_pos[8] = {
        [1] = &Motor_Cur_Pos_wheel[0],
        [2] = &Motor_Cur_Pos_wheel[1],
        [3] = &Motor_Cur_Pos_wheel[2],
        [4] = &Motor_Cur_Pos_wheel[3],
        [5] = &Motor_Cur_Pos_robot_arm_Z,
        [6] = &Motor_Cur_Pos_robot_arm_Y,
		[7] = &Motor_Cur_Pos_robot_arm_T,
    };
    
    uint8_t addr = Emm_V5_rxCmd[0];  // 电机地址(1~7)

    // ==============================================
    // 处理位置数据  0x36
    // ==============================================
    if (Emm_V5_rxCmd[1] == 0x36)
    {
        // 通用数据长度校验 + 地址范围限制
        if (Emm_V5_rxCount != 8 || addr < 1 || addr > 7)
            return;
        
        // 拼接32位位置原始值
        uint32_t pos = (uint32_t)(
                ((uint32_t)Emm_V5_rxCmd[3] << 24) |
                ((uint32_t)Emm_V5_rxCmd[4] << 16) |
                ((uint32_t)Emm_V5_rxCmd[5] << 8)  |
                ((uint32_t)Emm_V5_rxCmd[6] << 0)
        );

            // 原有位置单位转换逻辑（pos → pulse）
            int32_t pulse = (float)pos * 360.0f / 65536.0f;
            pulse = (int32_t)(6400 * (pulse / 360.0f));
            
            // 符号判断 + 写入对应位置变量
			if(addr==2||addr==3){
			 *motor_pos[addr] = (Emm_V5_rxCmd[2] == 0) ? pulse : -pulse;
			}
		    else{
            *motor_pos[addr] = (Emm_V5_rxCmd[2] == 0) ? -pulse : pulse;
            }
            return;
     }
}

//底盘步进清零函数，重新累计脉冲避免误差累计
void Reset_CurPos_To_Zero(void)
{
	for (int i=0;i<4;i++)
	{
		Emm_V5_Reset_CurPos_To_Zero(i+1);//将对应地址的电机脉冲清零
		vTaskDelay(5);
		Motor_Cur_Pos_wheel[i]=0;
		Emm_revolutions[i]=0;	
	}
}

/*相对模式*/
void Set_Pos_Z(uint8_t dir,int32_t clk,uint16_t v,uint8_t a)
{
    // 进入临界区（只保护 发指令 + 修改变量）
    taskENTER_CRITICAL();

    if(dir == Z_UP)
    {
        Emm_V5_Pos_Control(5, Z_UP, v, a, clk, 1, 0);
        Emm_revolutions[4] = clk;
    }
    else if(dir == Z_DOWN)
    {
        Emm_V5_Pos_Control(5, Z_DOWN, v, a, clk, 1, 0);
        Emm_revolutions[4] = -clk;
    }
    else
    {
        Emm_V5_Stop_Now(5, 0);
    }

    // 退出临界区
    taskEXIT_CRITICAL();

    osDelay(1);   
}

/*绝对模式*/
void Set_Clk_Z(int32_t clk,uint16_t v,uint8_t a)
{
	taskENTER_CRITICAL(); // 进入临界区
	if(clk>0){
		Emm_V5_Pos_Control(5,Z_UP,v,a,clk,0,0);
		vTaskDelay(1);
		Emm_revolutions[4]=Emm_revolutions[4]+clk;
	}
	else if(clk<0)
	{
		Emm_V5_Pos_Control(5,Z_DOWN,v,a,-clk,0,0);
		vTaskDelay(1);
		Emm_revolutions[4]=Emm_revolutions[4]+clk;
	}
	else 
	{
		Emm_V5_Stop_Now(5,0);
		vTaskDelay(1);
	}
	taskEXIT_CRITICAL();
	osDelay(1); 
}

/*相对模式*/
void Set_Pos_Y(uint8_t dir,int32_t clk,uint16_t v,uint8_t a)
{
    // 进入临界区（只保护 发指令 + 修改变量）
    taskENTER_CRITICAL();

    if(dir == Y_SHEN)
    {
        Emm_V5_Pos_Control(6, Y_SHEN, v, a, clk, 1, 0);
        Emm_revolutions[5] = clk;
    }
    else if(dir == Y_SHUO)
    {
        Emm_V5_Pos_Control(6, Y_SHUO, v, a, clk, 1, 0);
        Emm_revolutions[5] = -clk;
    }
    else
    {
        Emm_V5_Stop_Now(6, 0);
    }

    // 退出临界区
    taskEXIT_CRITICAL();

    osDelay(1);   
}

/*绝对模式*/
void Set_Clk_Y(int32_t clk,uint16_t v,uint8_t a)
{
	taskENTER_CRITICAL(); // 进入临界区
	if(clk>0){
		Emm_V5_Pos_Control(6,Y_SHEN,v,a,clk,0,0);
		vTaskDelay(1);
		Emm_revolutions[5]=Emm_revolutions[5]+clk;
	}
	else if(clk<0)
	{
		Emm_V5_Pos_Control(6,Y_SHUO,v,a,-clk,0,0);
		vTaskDelay(1);
		Emm_revolutions[5]=Emm_revolutions[5]+clk;
	}
	else 
	{
		Emm_V5_Stop_Now(6,0);
		vTaskDelay(1);
	}
	taskEXIT_CRITICAL();
	osDelay(1); 
}

/**
 * @brief  获取齿条当前位置(mm)
 * @retval 当前位置，0为原点，正值为前伸
 */
float Get_Rack_Position_Y(void)
{
    return (float)Emm_revolutions[5] / PULSE_PER_MM;
}

/**
 * @brief  齿条绝对位置移动
 * @param  target_mm: 目标位置(mm)，0为原点，正值为前伸
 * @param  v: 运动速度(与原函数参数一致)
 * @param  a: 运动加速度(与原函数参数一致)
 * @retval 0: 成功，-1: 目标位置超出限位
 */
int8_t Move_Rack_Absolute_Y(float target_mm, uint16_t v, uint8_t a)
{
    // 软限位检查与限幅
    if(target_mm > RACK_MAX_EXTEND) {
        target_mm = RACK_MAX_EXTEND;
        return -1;
    }
    if(target_mm < RACK_MIN_RETRACT) {
        target_mm = RACK_MIN_RETRACT;
        return -1;
    }
    
    // 计算需要移动的脉冲数
    float current_mm = Get_Rack_Position_Y();
    float delta_mm = target_mm - current_mm;
    int32_t delta_pulse = (int32_t)(delta_mm * PULSE_PER_MM + 0.5f); // 四舍五入
    
    // 移动距离过小，不执行
    if(abs(delta_pulse) < 2) {
        return 0;
    }
    
    // 执行运动
    Set_Clk_Y(delta_pulse, v, a);
    
    return 0;
}

int8_t Move_Rack_Absolute_Y_One(float target_mm, uint16_t v, uint8_t a)
{
    // 软限位检查与限幅
    if(target_mm > RACK_MAX_EXTEND) {
        target_mm = RACK_MAX_EXTEND;
        return -1;
    }
    if(target_mm < RACK_MIN_RETRACT) {
        target_mm = RACK_MIN_RETRACT;
        return -1;
    }
    
    // 计算需要移动的脉冲数
    float delta_mm = target_mm;
    int32_t delta_pulse = (int32_t)(delta_mm * PULSE_PER_MM + 0.5f); // 四舍五入
    
    // 移动距离过小，不执行
    if(abs(delta_pulse) < 2) {
        return 0;
    }
	
	uint8_t dir;
	
	if (delta_pulse>0)dir=1;
	else dir=0;
    
    // 执行运动
    Set_Pos_Y(dir,delta_pulse, v, a);
    
    return 0;
}

/**
 * @brief  齿条相对位置移动
 * @param  delta_mm: 相对移动距离(mm)，正值前伸，负值回收
 * @param  v: 运动速度(与原函数参数一致)
 * @param  a: 运动加速度(与原函数参数一致)
 * @retval 0: 成功，-1: 目标位置超出限位
 */
int8_t Move_Rack_Relative_Y(float delta_mm, uint16_t v, uint8_t a)
{
    float target_mm = Get_Rack_Position_Y() + delta_mm;
    return Move_Rack_Absolute_Y(target_mm, v, a);
}



// 全局变量
float Current_Claw_Height = MAX_HEIGHT;  // 当前高度(初始最高)
float Current_Claw_Height_Relative = MAX_HEIGHT;  // 当前高度(初始最高)

/**
 * @brief 控制夹爪到指定高度（单位：厘米）
 * @param target_height 目标高度 0.0 ~ 20.32cm
 */
void Set_Claw_Height(float target_height)
{
    // 安全限制
    if(target_height < 0.0f) target_height = 0.0f;
    if(target_height > MAX_HEIGHT) target_height = MAX_HEIGHT;

    // ===================== 核心：高度 → 固定脉冲映射 =====================
    // 每个高度对应唯一固定脉冲（最高点=0，高度越低脉冲越大）
    float pulse_val = (MAX_HEIGHT - target_height) * PULSE_PER_CM;
    int32_t fixed_pulse = (int32_t)(pulse_val + 0.5f);

    // 无需移动
    if(target_height == Current_Claw_Height) return;

    // 判断方向，发送【固定脉冲】
    if(target_height > Current_Claw_Height)
    {
        // 升高：向上 + 目标高度固定脉冲
        Set_Pos_Z(Z_UP, fixed_pulse, CLAW_SPEED, CLAW_ACC);
    }
    else
    {
        // 降低：向下 + 目标高度固定脉冲
        Set_Pos_Z(Z_DOWN, fixed_pulse, CLAW_SPEED, CLAW_ACC);
    }

    // 更新当前高度
    Current_Claw_Height_Relative = target_height;
}

/**
 * @brief 控制夹爪到指定高度（单位：厘米）
 * @param target_height 目标高度 0.0 ~ 20.32cm
 * @note 已修改为使用相对脉冲控制
 */
void Set_Claw_Height_Relative(float target_height,uint16_t v, uint8_t a)
{
    // 安全限制（完全不变）
    if(target_height < 0.0f) target_height = 0.0f;
    if(target_height > MAX_HEIGHT) target_height = MAX_HEIGHT;
	
    // 唯一强制修复：浮点数正确比较方式（原直接==会导致随机不执行）
    // 10微米误差认为相等，可根据你的机械精度调整
    if(fabsf(target_height - Current_Claw_Height_Relative) < 0.01f) return;

    // ========== 核心修改：相对脉冲计算 ==========
    // 相对脉冲只关心移动的距离，不关心方向
    float height_diff = target_height - Current_Claw_Height_Relative;
    float pulse_val = height_diff * PULSE_PER_CM;
    int32_t fixed_pulse = (int32_t)(pulse_val + 0.5f); // 四舍五入保持精度

	Set_Clk_Z(fixed_pulse, v, a);
	
    // 更新当前高度（保持你原来的写法不变）
    Current_Claw_Height_Relative = target_height;
}

//回归0点
void Back_Zero(int Arr,uint16_t v,uint8_t a)
{
	if(Arr==5)
	{
		Emm_V5_Pos_Control(5,1,v,a,0,1,0);
		Emm_revolutions[4]=0;
		Current_Claw_Height_Relative=MAX_HEIGHT;
		vTaskDelay(1);
	}
	else if(Arr==6)
	{
		Emm_V5_Pos_Control(6,0,50,0,0,1,0);
		Emm_revolutions[5]=0;
		vTaskDelay(1);
	}
}

float Current_Turntable_Angle = 0.0f; // 转盘当前角度（始终归一化在 0~360°）
const float PULSE_PER_DEGREE = 6400.0f / 360.0f; // 每度对应的脉冲数（宏定义，更清晰）

// 设定转盘角度（自动处理360°↔0°环形旋转，最短路径转动）
void Set_Turntable_Angle(float Tar_Angle)
{
    // ===================== 步骤1：目标角度归一化到 0~360° =====================
    // 处理任意输入角度（负数、超过360°都自动修正）
    Tar_Angle = fmodf(Tar_Angle, 360.0f);
    if (Tar_Angle < 0.0f)
    {
        Tar_Angle += 360.0f;
    }

    // ===================== 步骤2：计算角度差值（核心！相对转动角度） =====================
    float delta_angle = Tar_Angle - Current_Turntable_Angle;

    // 【关键】环形角度优化：强制走最短路径（差值限制在 -180° ~ +180°）
    // 解决 350°→10° 不反转340°，而是正转20°；360°→0° 差值归0
    if (delta_angle > 180.0f)
    {
        delta_angle -= 360.0f;
    }
    else if (delta_angle < -180.0f)
    {
        delta_angle += 360.0f;
    }

    // ===================== 步骤3：计算实际需要转动的脉冲数（绝对值） =====================
    float pulse_float = fabsf(delta_angle) * PULSE_PER_DEGREE;
    int32_t move_pulse = (int32_t)(pulse_float + 0.5f); // 四舍五入

    // ===================== 步骤4：根据差值正负判断方向，控制电机 =====================
    if (delta_angle > 0.5f) // 正转（微小死区，避免浮点误差抖动）
    {
        Emm_V5_Pos_Control(7, 1, 50, 25, move_pulse, 0, 0);
        vTaskDelay(1);
        Emm_revolutions[6] += move_pulse; // 累加实际转动脉冲
    }
    else if (delta_angle < -0.5f) // 反转
    {
        Emm_V5_Pos_Control(7, 0, 50, 25, move_pulse, 0, 0);
        vTaskDelay(1);
        Emm_revolutions[6] -= move_pulse; // 减去实际转动脉冲
    }
    else // 角度差极小，无需转动
    {
        Emm_V5_Stop_Now(7, 0);
        vTaskDelay(1);
    }

    // ===================== 步骤5：更新当前角度（自动归一化，360°→0°） =====================
    Current_Turntable_Angle += delta_angle;
    Current_Turntable_Angle = fmodf(Current_Turntable_Angle, 360.0f);
    if (Current_Turntable_Angle < 0)
    {
        Current_Turntable_Angle += 360.0f;
    }
}

/*----------------------------------------------------*/
Motor_control_para_t motor_control_para;

//底盘步进控制
void motor_pos_control(uint8_t addr,int32_t clk,uint16_t v,uint8_t a)
{
    // 进入临界区（只保护 发指令 + 修改变量）
    taskENTER_CRITICAL();
    if(clk>0){
        Emm_V5_Pos_Control(addr, 1, v, a, clk, 0, 1);
    }
    else {
        Emm_V5_Pos_Control(addr, 0, v, a, -clk, 0, 1);
    }
    // 退出临界区
    taskEXIT_CRITICAL();
}

//底盘位置提交函数
void chassis_pos_send(int32_t clk[TOTAL_CHASSIS_NUM],uint16_t v[TOTAL_CHASSIS_NUM],uint8_t a[TOTAL_CHASSIS_NUM])
{
	taskENTER_CRITICAL();
    for(uint8_t i=0;i<TOTAL_CHASSIS_NUM;i++)
    {
       motor_control_para.clk_set[i]=clk[i];
       motor_control_para.vel_set[i]=v[i];
       motor_control_para.acc_set[i]=a[i];
       /*统计电机脉冲值*/
       Emm_revolutions[i]=Emm_revolutions[i]+clk[i];
    }
    taskEXIT_CRITICAL();
}

//位置结算函数
void motor_set_total_pos(void)
{
    for(uint8_t i=0;i<TOTAL_CHASSIS_NUM;i++)
    {
		motor_pos_control(i+1,motor_control_para.clk_set[i],motor_control_para.vel_set[i],motor_control_para.acc_set[i]);
		vTaskDelay(1);
	}
	Emm_V5_Synchronous_motion(0);
	vTaskDelay(1);
}

/**
  * 函数功能：弧度转换为角度
  * 参数说明：radians - 弧度值
  * 返回值：对应的角度值
  */
float radians_to_degrees(float radians) 
{
	return radians * (180.0 / PI);
}

/**
  * 函数功能：角度转换为弧度
  * 参数说明：degrees - 角度值
  * 返回值：对应的弧度值
  */
float degrees_to_radians(float degrees) 
{
	return degrees * (PI / 180.0);
}

/**
  * 函数功能：计算浮点数的绝对值
  * 参数说明：yaw - 输入的浮点数
  * 返回值：yaw的绝对值
  */
float Absolute_value(float yaw)
{
	if(yaw<0)
	{
		yaw=-yaw;
	}
	else
	{
		yaw=yaw;
	}
	return yaw;
}

// 车轮旋转运动计算临时数组
int32_t wheel[4]={0,0,0,0};
int correct_tag=0;                  // 角度控制辅助计数

uint16_t correct_step=0;
uint8_t Angle_verification=0;
int jiaodu_num=0;

void Move_Diff_Turn(float left_clk,float right_clk,uint16_t v,uint8_t a)
{
	int32_t clk[4] = {
        right_clk,
        left_clk,
        left_clk,
		right_clk
    };
	
	uint16_t temporary_v[4]={v,v,v,v};
	uint8_t temporary_a[4]={a,a,a,a};	
	
	chassis_pos_send(clk,temporary_v,temporary_a);
	
	motor_set_total_pos();	
}

void Move_Straight(float tar_clk,uint16_t v,uint8_t a)
{
	int32_t clk[4] = {
        tar_clk,
        tar_clk,
        tar_clk,
		tar_clk
    };
	
	uint16_t temporary_v[4]={v,v,v,v};
	uint8_t temporary_a[4]={a,a,a,a};	
	
	chassis_pos_send(clk,temporary_v,temporary_a);
	motor_set_total_pos();	
}

//旋转角度函数
void calculate_wheel_revolutions(int16_t angle,uint16_t v,uint8_t a)
{
	for (int i = 0; i < 4; i++) {
		wheel[i] = 0;
	}
	float rotation_angle =degrees_to_radians(-angle*4);
	float revs =rotation_angle/(2*PI);
	wheel[0] = wheel[1] = wheel[2] = wheel[3] = revs * 6650;
	//车轮2和车轮3的转动方向需要相反
	wheel[1]*=-1;
	wheel[2]*=-1;
	
	uint16_t temporary_v[4]={v,v,v,v};
	uint8_t temporary_a[4]={a,a,a,a};	
	
	Angle_verification=1;
	
	chassis_pos_send(wheel,temporary_v,temporary_a);
	
	motor_set_total_pos();
}

void calculate_wheel_revolutions_2(int16_t angle,uint16_t v,uint8_t a)
{
	for (int i = 0; i < 4; i++) {
		wheel[i] = 0;
	}
	float rotation_angle =degrees_to_radians(-angle*4);
	float revs =rotation_angle/(2*PI);
	wheel[0] = wheel[1] = wheel[2] = wheel[3] = revs * 1000;
	//车轮2和车轮3的转动方向需要相反
	wheel[1]*=-1;
	wheel[2]*=-1;
	
	uint16_t temporary_v[4]={v,v,v,v};
	uint8_t temporary_a[4]={a,a,a,a};	
	
	Angle_verification=1;
	
	chassis_pos_send(wheel,temporary_v,temporary_a);
	
	motor_set_total_pos();
}

/*横移校准基础代码*/
void calculate_wheel_cali_lateral_revolution(float delta_lateral,uint16_t v,uint8_t a)
{
	// 1. 死区判断：小于1mm直接不移动，避免浮点数噪声误触发
	if(fabsf(delta_lateral) < DEAD_ZONE_M)
    {
        return;
    }
	// 2. 计算轮子周长：C = 2πr
	float wheel_circumference = 2.0f * PI * WHEEL_RADIUS; // 单位m/圈
	// 3. 计算总脉冲数（核心换算）
    float pulse_float = (delta_lateral / wheel_circumference) * (PULSE_PER_ROUND+450);
    int32_t pulse_total = (int32_t)roundf(pulse_float); // 四舍五入，保证精度

    // 4. 直接用系数乘pulse_total
    int32_t clk[4] = {
        -pulse_total,
         pulse_total,
        -pulse_total,
		 pulse_total
    };
	
	// 5. 准备速度和加速度数组（四个电机速度/加速度大小一致）
    uint16_t temporary_v[4] = {v, v, v, v};
    uint8_t temporary_a[4] = {a, a, a, a};
	
	chassis_pos_send(clk,temporary_v,temporary_a);
	
	motor_set_total_pos();
}


/*直走校准基础代码*/
void calculate_wheel_cali_straight_revolution(float delta_straight, uint16_t v, uint8_t a)
{
    // 1. 死区判断：小于1mm直接不移动
    if(fabsf(delta_straight) < DEAD_ZONE_M) return;

    // 2. 计算总脉冲数（和X轴完全一样，轮子半径相同，delta_y天生带正负）
    float wheel_circumference = 2.0f * PI * WHEEL_RADIUS;
    float pulse_float = (delta_straight / wheel_circumference) * (PULSE_PER_ROUND+90);
    int32_t pulse_total = (int32_t)roundf(pulse_float);

    // 3. Y轴固定方向系数：四个电机全部一致（向前为正）

    // 4. 直接用系数乘pulse_total，一行搞定
    int32_t clk[4] = {
        pulse_total,
        pulse_total,
        pulse_total,
		pulse_total
    };

    // 5. 准备速度加速度数组
    uint16_t temporary_v[4] = {v, v, v, v};
    uint8_t temporary_a[4] = {a, a, a, a};

    // 6. 调用你现有的函数发送指令
    chassis_pos_send(clk, temporary_v, temporary_a);
    motor_set_total_pos();
}

/*底盘到位检测(阻塞)*/
uint8_t In_place_inspection(void)
{
	uint8_t motor_id=1;//电机id号，用于累加轮询
	do{
		osDelay(5);
		Emm_V5_Read_Sys_Params(motor_id,S_CPOS);
		motor_id++;
		if(motor_id>4)motor_id=1;
	  }while(!((Emm_revolutions[0]-200)<=(Motor_Cur_Pos_wheel[0])
			&&(Motor_Cur_Pos_wheel[0])<=(Emm_revolutions[0]+200)
			&&(Emm_revolutions[1]-200)<=(Motor_Cur_Pos_wheel[1])
			&&(Motor_Cur_Pos_wheel[1])<=(Emm_revolutions[1]+200)
			&&(Emm_revolutions[2]-200)<=(Motor_Cur_Pos_wheel[2])
			&&(Motor_Cur_Pos_wheel[2])<=(Emm_revolutions[2]+200)
			&&(Emm_revolutions[3]-200)<=(Motor_Cur_Pos_wheel[3])
			&&(Motor_Cur_Pos_wheel[3])<=(Emm_revolutions[3]+200)
			));
	osDelay(100);
	return 1;
}

void MOVE_SLANT(float Right_Slant_clk,float Left_Slant_clk,uint8_t v,uint8_t a)
{
	 int32_t clk[4] = {
        Right_Slant_clk,
        Left_Slant_clk,
        Right_Slant_clk,
		Left_Slant_clk
    };

    // 5. 准备速度加速度数组
    uint16_t temporary_v[4] = {v, v, v, v};
    uint8_t temporary_a[4] = {a, a, a, a};

    // 6. 调用你现有的函数发送指令
    chassis_pos_send(clk, temporary_v, temporary_a);
    motor_set_total_pos();
}

// 角度校正函数包含到位检测
void correct_angle(float Target_angle)
{
    /*获得当前的航向角*/
	ops9_para_t *ops9_Data=get_ops9_data_point();
	float Yaw=ConvertToAngle(ops9_Data->OPS9_Rx_Data.z);
	if (Target_angle==180)
	{	
		if(Yaw>0&&(180-Absolute_value(Yaw))>0.5)
		{
			correct_tag=0;
			calculate_wheel_revolutions(-(Target_angle-Absolute_value(Yaw)),Move_Speed,Move_a);
			correct_tag=In_place_inspection();	
		}
		else if(Yaw<0&&(180-Absolute_value(Yaw))>0.5)
		{
			correct_tag=0;
			calculate_wheel_revolutions((Target_angle-Absolute_value(Yaw)),Move_Speed,Move_a);
			correct_tag=In_place_inspection();	
		}
	}
	else
	{
		if(Absolute_value(Target_angle-Yaw)>0.5)
        {
		   correct_tag=0;
		   calculate_wheel_revolutions(-(Target_angle-Yaw),Move_Speed,Move_a);
		   correct_tag=In_place_inspection();
		}
	}	
}

// 计算归一化角度差，返回范围 [-180, 180]
// 正值：目标在当前角度的逆时针方向（角度需增加）
// 负值：目标在当前角度的顺时针方向（角度需减小）
static float angle_diff(float target, float current)
{
    float diff = target - current;
    while (diff > 180.0f)  diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

//void correct_Delicate_angle(float Target_angle)
//{
//	float Current_Yaw = Yaw;
//    // 新增：计算归一化最短路径角度差，范围[-180, 180]，解决跨180°转圈问题
//    float diff = angle_diff(Target_angle, Current_Yaw);

//	if (Target_angle == 180)
//	{	
//		if(correct_step == 0)
//		{
//            // 原逻辑：误差>1°时大角度调整，仅替换判断条件与角度参数，方向逻辑完全兼容
//			if(fabsf(diff) > 1.0f)
//			{
//				calculate_wheel_revolutions(-diff, Move_Speed, 150);
//				correct_step = In_place_inspection();	
//			}
//            // 原逻辑：误差0~1°时步进微调，仅替换方向判断条件
//			else if(fabsf(diff) > 0.0f && fabsf(diff) <= 1.0f)
//			{
//				if(diff > 0)
//				{
//					calculate_wheel_revolutions_2(-1, 150, 0);
//				}
//				else
//				{
//					calculate_wheel_revolutions_2(1, 150, 0);
//				}
//				correct_step = 1;
//				osDelay(250);
//			}
//	    }

//        // 原到位判断逻辑不变，仅替换为归一化差值判断，避免跨180°判据失效
//		if( (fabsf(diff) <= 0.5f) ||
//			((jiaodu_num == 5) && (fabsf(diff) <= 1.0f)) )
//		{
//			Angle_verification = 0;
//			correct_step = 0;
//			jiaodu_num = 0;
//		}
//		else
//		{
//			correct_step++;
//			if(correct_step > 5)
//			{
//				correct_step = 0;
//				jiaodu_num++;
//			}
//		}
//	}
//	else
//	{
//		if(correct_step == 0)
//		{
//            // 原逻辑：误差>1°时大角度调整，仅替换角度差计算
//			if(fabsf(diff) > 1.0f)
//			{
//			   calculate_wheel_revolutions(-diff, Move_Speed, Move_a);
//			   correct_step = In_place_inspection();
//			}
//			else
//			{
//                // 原微调逻辑不变，仅替换方向判断条件
//				if(diff > 0)
//				{
//					calculate_wheel_revolutions_2(-1, 150, 0);
//					correct_step = 1;
//					osDelay(250);
//				}
//				else if(diff < 0)
//				{
//					calculate_wheel_revolutions_2(1, 150, 0);
//					correct_step = 1;
//					osDelay(250);
//				}
//			}
//	    }

//        // 原到位判断逻辑不变，仅替换为归一化差值判断
//		if( (fabsf(diff) <= 0.3f) ||
//            ((jiaodu_num == 5) && (fabsf(diff) <= 1.0f)) )		
//		{
//			Angle_verification = 0;
//			correct_step = 0;
//			jiaodu_num = 0;
//		}
//		else
//		{
//			correct_step++;
//			if(correct_step > 5)
//			{
//				correct_step = 0;
//				jiaodu_num++;
//			}
//		}
//	}	
//}

void correct_Delicate_angle(float Target_angle)
{
    float Current_Yaw = Yaw;
    float diff = angle_diff(Target_angle, Current_Yaw);
    float err_threshold;

    // 修改点1：严格复刻原代码 jiaodu_num==5 才放宽的逻辑
    if (Target_angle == 180.0f)
        err_threshold = (jiaodu_num == 5) ? 1.0f : 0.5f;
    else
        err_threshold = (jiaodu_num == 5) ? 1.0f : 0.3f;

    if (correct_step == 0)
    {
        if (fabsf(diff) > 1.0f)
        {
            uint16_t move_acc = (Target_angle == 180.0f) ? 150 : Move_a;
            calculate_wheel_revolutions(-diff, Move_Speed, move_acc);
            correct_step = In_place_inspection();
        }
        else if(fabsf(diff) > 0.0f) // 修改点2：误差为0时不进微调，不执行delay
        {
            if (diff > 0)
            {
                calculate_wheel_revolutions_2(-1, 150, 0);
            }
            else if (diff < 0)
            {
                calculate_wheel_revolutions_2(1, 150, 0);
            }
            correct_step = 1;
            osDelay(250);
        }
    }

    if (fabsf(diff) <= err_threshold)
    {
        Angle_verification = 0;
        correct_step = 0;
        jiaodu_num = 0;
    }
    else
    {
        correct_step++;
        if (correct_step > 5)
        {
            correct_step = 0;
            jiaodu_num++;
        }
    }
}

//

/*偏航角二次校准(首选)*/
void cali_z(float Target_angle)
{
	correct_angle(Target_angle);
	if(correct_tag)
	{
		correct_angle(Target_angle);
	}
}

/*根据ops9的数据校准底盘数据，校准x轴*/
void correct_x(float tar_x)
{
	/*获取当前x的坐标*/
	ops9_para_t *ops9_Data=get_ops9_data_point();
	float cur_x=ops9_Data->OPS9_Rx_Data.x;
	float cur_angle=ConvertToAngle(ops9_Data->OPS9_Rx_Data.z);
	float delta_x= tar_x-cur_x;

	if(cur_angle>-5.0f&&cur_angle<5.0f)
	{
		correct_tag=0;
		calculate_wheel_cali_lateral_revolution(delta_x,Move_Speed,Move_a);
		correct_tag=In_place_inspection();
	}
	else if(cur_angle>175.0f&&cur_angle<185.0f)
	{
		correct_tag=0;
		calculate_wheel_cali_lateral_revolution(-delta_x,Move_Speed,Move_a);
		correct_tag=In_place_inspection();
	}	
	else if(cur_angle<-175.0f&&cur_angle>-185.0f)
	{
		correct_tag=0;
		calculate_wheel_cali_lateral_revolution(-delta_x,Move_Speed,Move_a);
		correct_tag=In_place_inspection();
	}
	else if(cur_angle>85.0f&&cur_angle<95.0f)
	{
		correct_tag=0;
		calculate_wheel_cali_straight_revolution(-delta_x,Move_Speed,Move_a);
		correct_tag=In_place_inspection();
	}
	else if(cur_angle>-95&&cur_angle<-85)
	{
	    correct_tag=0;
		calculate_wheel_cali_straight_revolution(delta_x,Move_Speed,Move_a);
		correct_tag=In_place_inspection();	
	}
}

/*x轴坐标二次校准*/
void cali_x(float tar_x)
{
	correct_x(tar_x);
	if(correct_tag)
	{
		correct_x(tar_x);
	}
}

void correct_y(float tar_y)
{
	/*获取当前y的坐标*/
	ops9_para_t *ops9_Data=get_ops9_data_point();
	float cur_y=ops9_Data->OPS9_Rx_Data.y;
	float cur_angle=ConvertToAngle(ops9_Data->OPS9_Rx_Data.z);
	float delta_y= tar_y-cur_y;
	if(cur_angle>-5.0f&&cur_angle<5.0f)
	{
		correct_tag=0;
		calculate_wheel_cali_straight_revolution(delta_y,Move_Speed,Move_a);
		correct_tag=In_place_inspection();
	}
	else if(cur_angle>175.0f&&cur_angle<185.0f)
	{
		correct_tag=0;
		calculate_wheel_cali_straight_revolution(-delta_y,Move_Speed,Move_a);
		correct_tag=In_place_inspection();
	}	
	else if(cur_angle<-175.0f&&cur_angle>-185.0f)
	{
		correct_tag=0;
		calculate_wheel_cali_straight_revolution(-delta_y,Move_Speed,Move_a);
		correct_tag=In_place_inspection();
	}
	else if(cur_angle>85.0f&&cur_angle<95.0f)
	{
		correct_tag=0;
		calculate_wheel_cali_lateral_revolution(delta_y,Move_Speed,Move_a);
		correct_tag=In_place_inspection();
	}
	else if(cur_angle>-95&&cur_angle<-85)
	{
	    correct_tag=0;
		calculate_wheel_cali_lateral_revolution(-delta_y,Move_Speed,Move_a);
		correct_tag=In_place_inspection();	
	}
}

void cali_y(float tar_y)
{
	correct_y(tar_y);
	if(correct_tag)
	{
		correct_y(tar_y);
	}
}

void correct_xandy(float tar_x,float tar_y,uint8_t v,uint8_t a)
{
	ops9_para_t *ops9_Data=get_ops9_data_point();
	float cur_x=ops9_Data->OPS9_Rx_Data.x;
	float cur_y=ops9_Data->OPS9_Rx_Data.y;
	float cur_angle=ConvertToAngle(ops9_Data->OPS9_Rx_Data.z);
	
	float delta_x= tar_x-cur_x;
	float delta_y= tar_y-cur_y;
	
	float wheel_circumference = 2.0f * PI * WHEEL_RADIUS; // 单位m/圈
	int32_t pulse_x=0;
	int32_t pulse_y=0;
	int32_t clk[4];
	if(cur_angle>-5.0f&&cur_angle<5.0f)
	{
		float pulse_float_x = (delta_x / wheel_circumference) * (PULSE_PER_ROUND+450);
		float pulse_float_y = (delta_y / wheel_circumference) * (PULSE_PER_ROUND+90);
        pulse_x = (int32_t)roundf(pulse_float_x);
		pulse_y = (int32_t)roundf(pulse_float_y); // 四舍五入，保证精度
		clk[0] =  -pulse_x+pulse_y;
		clk[1] =  pulse_x+pulse_y;
		clk[2] =  -pulse_x+pulse_y;
		clk[3] =  pulse_x+pulse_y;
	}
	else if(cur_angle>175.0f&&cur_angle<185.0f)
	{
		float pulse_float_x = (-delta_x / wheel_circumference) * (PULSE_PER_ROUND+450);
		float pulse_float_y = (-delta_y / wheel_circumference) * (PULSE_PER_ROUND+90);
        pulse_x = (int32_t)roundf(pulse_float_x);
		pulse_y = (int32_t)roundf(pulse_float_y); // 四舍五入，保证精度
		clk[0] =  -pulse_x+pulse_y;
		clk[1] =  pulse_x+pulse_y;
		clk[2] =  -pulse_x+pulse_y;
		clk[3] =  pulse_x+pulse_y;
	}
	else if(cur_angle<-175.0f&&cur_angle>-185.0f)
	{
		float pulse_float_x = (-delta_x / wheel_circumference) * (PULSE_PER_ROUND+450);
		float pulse_float_y = (-delta_y / wheel_circumference) * (PULSE_PER_ROUND+90);
        pulse_x = (int32_t)roundf(pulse_float_x);
		pulse_y = (int32_t)roundf(pulse_float_y); // 四舍五入，保证精度
		clk[0] =  -pulse_x+pulse_y;
		clk[1] =  pulse_x+pulse_y;
		clk[2] =  -pulse_x+pulse_y;
		clk[3] =  pulse_x+pulse_y;
	}
	else if(cur_angle>85.0f&&cur_angle<95.0f)
	{
		float pulse_float_x = (-delta_x / wheel_circumference) * (PULSE_PER_ROUND+90);
		float pulse_float_y = (delta_y / wheel_circumference) * (PULSE_PER_ROUND+450);
        pulse_x = (int32_t)roundf(pulse_float_x);
		pulse_y = (int32_t)roundf(pulse_float_y); // 四舍五入，保证精度
		clk[0] =  -pulse_y+pulse_x;
		clk[1] =  pulse_y+pulse_x;
		clk[2] =  -pulse_y+pulse_x;
		clk[3] =  pulse_y+pulse_x;
	}
	else if(cur_angle>-95&&cur_angle<-85)
	{
		float pulse_float_x = (delta_x / wheel_circumference) * (PULSE_PER_ROUND+90);
		float pulse_float_y = (-delta_y / wheel_circumference) * (PULSE_PER_ROUND+450);
        pulse_x = (int32_t)roundf(pulse_float_x);
		pulse_y = (int32_t)roundf(pulse_float_y); // 四舍五入，保证精度
		clk[0] =  -pulse_y+pulse_x;
		clk[1] =  pulse_y+pulse_x;
		clk[2] =  -pulse_y+pulse_x;
		clk[3] =  pulse_y+pulse_x;
	}
	 // 装载速度加速度数组
    uint16_t temporary_v[4] = {v, v, v, v};
    uint8_t temporary_a[4] = {a, a, a, a};
	
	// 发送指令
    chassis_pos_send(clk, temporary_v, temporary_a);
    motor_set_total_pos();
}

uint8_t cali_xy_flag=0;
void cali_xy(float tar_x,float tar_y,float target_angle)
{
	correct_tag=0;
	correct_xandy(tar_x,tar_y,Move_Speed,Move_a);
	correct_tag=In_place_inspection();
	if(correct_tag)
	{
		correct_tag=0;
		correct_xandy(tar_x,tar_y,Move_Speed,Move_a);
		correct_tag=In_place_inspection();
	}
}

uint8_t cali_flag=0;
void cali_pos(float tar_x,float tar_y,float target_angle)
{
	if(cali_flag==0)//先校准z
	{
		Angle_verification=1;
		while(Angle_verification==1)
		{
			correct_Delicate_angle(target_angle);
		}
		cali_flag++;
	}
	if(cali_flag==1)//再校准x
	{
		cali_xy(tar_x,tar_y,target_angle);
		cali_flag++;
	}
	if(cali_flag==2)
	{
		Angle_verification=1;
		while(Angle_verification==1)
		{
			correct_Delicate_angle(target_angle);
		}
		cali_flag=0;
	}
}

void Move_xandy(float tar_x,float tar_y,uint8_t v,uint8_t a)
{
	float wheel_circumference = 2.0f * PI * WHEEL_RADIUS; // 单位m/圈
	int32_t pulse_x=0;
	int32_t pulse_y=0;
	int32_t clk[4];
	float pulse_float_x = (tar_x / wheel_circumference) * (PULSE_PER_ROUND+450);
	float pulse_float_y = (tar_y / wheel_circumference) * (PULSE_PER_ROUND+90);
    pulse_x = (int32_t)roundf(pulse_float_x);
    pulse_y = (int32_t)roundf(pulse_float_y); // 四舍五入，保证精度
	clk[0] =  -pulse_x+pulse_y;
	clk[1] =  pulse_x+pulse_y;
	clk[2] =  -pulse_x+pulse_y;
	clk[3] =  pulse_x+pulse_y;
	 // 装载速度加速度数组
    uint16_t temporary_v[4] = {v, v, v, v};
    uint8_t temporary_a[4] = {a, a, a, a};
	
	// 发送指令
    chassis_pos_send(clk, temporary_v, temporary_a);
    motor_set_total_pos();
}

//矫正角度后直走，输入距离和角度
void cali_Straight(float distance_x,float distance_y,float target_angle,float correct_angain)
{
	if(cali_flag==0)//先校准z
	{
		Angle_verification=1;
		while(Angle_verification==1)
		{
			correct_Delicate_angle(target_angle);
		}
		cali_flag++;
	}
	if(cali_flag==1)//再校准x
	{
		Move_xandy(distance_x,distance_y,Move_Speed,Move_a);
		In_place_inspection();
		cali_flag++;
	}
	if(cali_flag==2)
	{
		if(correct_angain==1)
		{
			Angle_verification=1;
			while(Angle_verification==1)
			{
				correct_Delicate_angle(target_angle);
			}
			cali_flag=0;
		}
		else{
			cali_flag=0;
		}
	}
}

static uint8_t x_axis_disabled = 0;
static uint8_t y_axis_disabled = 0;

void vision_cali(float x_kp, float y_kp)
{
    // ============== 一旦 dx 曾经为 0，永久禁用 X 轴校准 ==============
    if (parsed_data.dx == 0)
    {
        x_axis_disabled = 1;
    }

    // ============== 一旦 dy 曾经为 0，永久禁用 Y 轴校准 ==============
    if (parsed_data.dy == 0)
    {
        y_axis_disabled = 1;
    }

    // 只要还有任意一个轴允许校准，就继续执行
    if (!x_axis_disabled || !y_axis_disabled)
    {
        // ---------- X 轴：只有没被禁用，才执行校准 ----------
        if (!x_axis_disabled)
        {
            float delta_straight = parsed_data.dx * x_kp;
            calculate_wheel_cali_straight_revolution(delta_straight, 150, 100);
            In_place_inspection();
        }

        // ---------- Y 轴：只有没被禁用，才执行校准 ----------
        if (!y_axis_disabled)
        {
            float delta_lateral = parsed_data.dy * y_kp;
            calculate_wheel_cali_lateral_revolution(-delta_lateral, 150, 100);
            In_place_inspection();
        }

        // 执行完清空本次数据
        parsed_data.dx = 0;
        parsed_data.dy = 0;
    }
}

void object_pos_vary(uint8_t object_pos[3])
{
	object_pos[1] = (object_pos[1] % 3) + 1;
    object_pos[2] = (object_pos[2] % 3) + 1;
}

void object_pos_vary_clockwise(uint8_t object_pos[3])//顺时针情况
{
    // 顺时针逻辑：1->3, 2->1, 3->2
    object_pos[1] = ((object_pos[1] - 2 + 3) % 3) + 1;
    object_pos[2] = ((object_pos[2] - 2 + 3) % 3) + 1;
}

void swap13Keep2(uint8_t arr[], int len)
{
    int idx1 = -1;
    int idx3 = -1;

    // 遍历找到数值1和数值3的下标
    for (int i = 0; i < len; i++)
    {
        if (arr[i] == 1)
        {
            idx1 = i;
        }
        else if (arr[i] == 3)
        {
            idx3 = i;
        }
    }

    // 两个数值都找到时才执行交换
    if (idx1 != -1 && idx3 != -1)
    {
        uint8_t temp = arr[idx1];
        arr[idx1] = arr[idx3];
        arr[idx3] = temp;
    }
}

void Set_Object(float turntable_angle)//在转盘放置物体
{
	Set_Claw_Height_Relative(Set_Hight,slow_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	Set_Angle(3,Claw_Angle);
	vTaskDelay(500);
	Back_Zero(5,slow_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));		  
	servo_start_rotate(&gimbal_servo,1,rotate_angle_correct,200);
	Set_Turntable_Angle(turntable_angle);  
	vTaskDelay(1000);
	Set_Claw_Height_Relative(Turntable_Hight,fast_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	vTaskDelay(100); 
	Set_Angle(3,Release_Angle);
	vTaskDelay(500); 
	Back_Zero(5,fast_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200))); 
}

void catch_turn_1(void)
{
	Set_Claw_Height_Relative(Turntable_Hight,fast_z_speed,z_acc);
				do{
						Emm_V5_Read_Sys_Params(5, S_CPOS);
						osDelay(5);
				  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
						&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
				Set_Angle(3,Claw_Angle);
				vTaskDelay(500);
				Back_Zero(5,fast_z_speed,z_acc);
				do{
						Emm_V5_Read_Sys_Params(5, S_CPOS);
						osDelay(5);
				  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
						&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
				Move_Rack_Absolute_Y(zero_pos_y,fast_y_speed,fast_y_acc);  
				servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
				vTaskDelay(1000);
				Set_Claw_Height_Relative(Set_Hight,slow_z_speed,z_acc);
				do{
						Emm_V5_Read_Sys_Params(5, S_CPOS);
						osDelay(5);
				  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
						&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));  
				Set_Angle(3,Release_Angle);   
				vTaskDelay(500); 
				Back_Zero(5,slow_z_speed,z_acc);
				do{
						Emm_V5_Read_Sys_Params(5, S_CPOS);
						osDelay(5);
				  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
						&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
}
	
void catch_turn_2(void)
{
	Set_Claw_Height_Relative(Turntable_Hight,fast_z_speed,z_acc);
				do{
						Emm_V5_Read_Sys_Params(5, S_CPOS);
						osDelay(5);
				  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
						&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
				Set_Angle(3,Claw_Angle);
				vTaskDelay(500);
				Back_Zero(5,fast_z_speed,z_acc);
				do{
						Emm_V5_Read_Sys_Params(5, S_CPOS);
						osDelay(5);
				  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
						&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
				servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
				vTaskDelay(1000);
				Set_Claw_Height_Relative(Set_Hight,slow_z_speed,z_acc);
				do{
						Emm_V5_Read_Sys_Params(5, S_CPOS);
						osDelay(5);
				  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
						&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));  
				Set_Angle(3,Release_Angle);   
				vTaskDelay(500); 
				Back_Zero(5,slow_z_speed,z_acc);
				do{
						Emm_V5_Read_Sys_Params(5, S_CPOS);
						osDelay(5);
				  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
						&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));	
}

void catch_turn_3(void)
{
	Set_Claw_Height_Relative(Turntable_Hight,fast_z_speed,z_acc);
				do{
						Emm_V5_Read_Sys_Params(5, S_CPOS);
						osDelay(5);
				  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
						&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
				Set_Angle(3,Claw_Angle);
				vTaskDelay(500);
				Back_Zero(5,fast_z_speed,z_acc);
				do{
						Emm_V5_Read_Sys_Params(5, S_CPOS);
						osDelay(5);
				  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
						&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
				Move_Rack_Absolute_Y(middle_pos_y,fast_y_speed,fast_y_acc);  
				vTaskDelay(300); 
				Move_Rack_Absolute_Y(zero_pos_y,fast_y_speed,fast_y_acc);			  
				servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
				vTaskDelay(1000);
				Set_Claw_Height_Relative(Set_Hight,slow_z_speed,z_acc);
				do{
						Emm_V5_Read_Sys_Params(5, S_CPOS);
						osDelay(5);
				  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
						&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));  
				Set_Angle(3,Release_Angle);   
				vTaskDelay(500); 
				Back_Zero(5,slow_z_speed,z_acc);
				do{
						Emm_V5_Read_Sys_Params(5, S_CPOS);
						osDelay(5);
				  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
						&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
}

void Set_Object_Red(float roate_angle,float turntable_angle,float Len)
{
    Set_Claw_Height_Relative(Set_Hight,slow_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	Set_Angle(3,Claw_Angle);
	vTaskDelay(500);
	Back_Zero(5,slow_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	Move_Rack_Absolute_Y(Len,fast_y_speed,fast_y_acc);			  
	servo_start_rotate(&gimbal_servo,1,roate_angle,200);
	Set_Turntable_Angle(turntable_angle); 
	vTaskDelay(1000);
	Set_Claw_Height_Relative(Material_Hight,fast_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	vTaskDelay(100); 
	Set_Angle(3,Release_Angle);
	vTaskDelay(500); 
	Back_Zero(5,fast_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200))); 
}

void Set_Object_Green(float roate_angle,float turntable_angle,float Len)//放置物料函数，使用该函数前需要调整舵机角度到258以及车载转盘的角度
{
	Set_Claw_Height_Relative(Set_Hight,slow_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	Set_Angle(3,Claw_Angle);
	vTaskDelay(500);
	Back_Zero(5,slow_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	Move_Rack_Absolute_Y(Len,fast_y_speed,fast_y_acc);			  
	servo_start_rotate(&gimbal_servo,1,roate_angle,200);
	Set_Turntable_Angle(turntable_angle);  
	vTaskDelay(1000);
	Set_Claw_Height_Relative(Material_Hight,fast_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	vTaskDelay(100); 
	Set_Angle(3,Release_Angle);
	vTaskDelay(500); 
	Back_Zero(5,fast_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200))); 
}

void Set_Object_Bule(float roate_angle,float turntable_angle,float Len)
{
	Set_Claw_Height_Relative(Set_Hight,slow_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	Set_Angle(3,Claw_Angle);
	vTaskDelay(500);
	Back_Zero(5,slow_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	Move_Rack_Absolute_Y(Len,fast_y_speed,fast_y_acc);			  
	servo_start_rotate(&gimbal_servo,1,roate_angle,200);
	Set_Turntable_Angle(turntable_angle);  
	vTaskDelay(1000);
	Set_Claw_Height_Relative(Material_Hight,fast_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	vTaskDelay(100); 
	Set_Angle(3,Release_Angle);
	vTaskDelay(500); 
	Back_Zero(5,fast_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200))); 
}

void Set_Object_2(float roate_angle,float turntable_angle,float Len)
{
	Set_Claw_Height_Relative(Set_Hight,slow_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	Set_Angle(3,Claw_Angle);
	vTaskDelay(500);
	Back_Zero(5,slow_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	Move_Rack_Absolute_Y(Len,fast_y_speed,fast_y_acc);			  
	servo_start_rotate(&gimbal_servo,1,roate_angle,200);
	Set_Turntable_Angle(turntable_angle);  
	vTaskDelay(1000);
	Set_Claw_Height_Relative(Material_Hight_2,fast_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	vTaskDelay(200); 
	Set_Angle(3,Release_Angle);
	vTaskDelay(500); 
	Back_Zero(5,fast_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200))); 
}

void Catch_Object_Red(void)
{
	servo_start_rotate(&gimbal_servo,1,rotate_angle_red,200);
	Move_Rack_Absolute_Y(Right_LEN,fast_y_speed,fast_y_acc);
	vTaskDelay(1000);
	Set_Claw_Height_Relative(Material_Hight,fast_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	Set_Angle(3,Claw_Angle);
	vTaskDelay(500);  
	Back_Zero(5,fast_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	Move_Rack_Absolute_Y(0,fast_y_speed,fast_y_acc);			  
	servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
	vTaskDelay(1000);
	Set_Claw_Height_Relative(Set_Hight,slow_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	Set_Angle(3,Release_Angle);   
	vTaskDelay(500); 
	Back_Zero(5,slow_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	  Set_Angle(3,150);  
}
	
void Catch_Object_Green(void)
{
	servo_start_rotate(&gimbal_servo,1,rotate_angle_green,200);
	Move_Rack_Absolute_Y(Middle_LEN,fast_y_speed,fast_y_acc);
	vTaskDelay(1000);
	Set_Claw_Height_Relative(Material_Hight,fast_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	Set_Angle(3,Claw_Angle);
	vTaskDelay(500);  
	Back_Zero(5,fast_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	Move_Rack_Absolute_Y(0,fast_y_speed,fast_y_acc);			  
	servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
	vTaskDelay(1000);
	Set_Claw_Height_Relative(Set_Hight,slow_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	Set_Angle(3,Release_Angle);   
	vTaskDelay(500); 
	Back_Zero(5,slow_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	  Set_Angle(3,150);  
}
	
void Catch_Object_Bule(void)
{
	servo_start_rotate(&gimbal_servo,1,rotate_angle_bule,200);
	Move_Rack_Absolute_Y(Left_LEN,fast_y_speed,fast_y_acc);
	vTaskDelay(1000);
	Set_Claw_Height_Relative(Material_Hight,fast_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	Set_Angle(3,Claw_Angle);
	vTaskDelay(500);  
	Back_Zero(5,fast_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	Move_Rack_Absolute_Y(0,fast_y_speed,fast_y_acc);			  
	servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
    vTaskDelay(1000);
	Set_Claw_Height_Relative(Set_Hight,slow_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	Set_Angle(3,Release_Angle);   
	vTaskDelay(500); 
	Back_Zero(5,slow_z_speed,z_acc);
	do{
		Emm_V5_Read_Sys_Params(5, S_CPOS);
		osDelay(5);
	  }while(!((Emm_revolutions[4]-200)<=(Motor_Cur_Pos_robot_arm_Z)
			&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+200)));
	Set_Angle(3,150);   
}
	
uint8_t In_Serial=0;//指令发送标志位
void step0_extra_action(uint8_t step_index)//一次性发送任务
{
	if(In_Serial==0)
	{
		Serial_SendString("ONE");//发送识别二维码信息
		In_Serial=1;
	}
	if(In_Serial==1)
	{
		taskENTER_CRITICAL(); // 进入临界区
		motion_steps[step_index].action_exit_flag = 1;
		In_Serial=0;
		taskEXIT_CRITICAL();
	}
}

void step1_extra_action(uint8_t step_index)//底盘转到期望角度，循环干等，有信息后再跳到下一步
{
	if(In_Serial==0)
	{
		Back_Zero(5,1000,0);
		Move_Rack_Absolute_Y(0,300,200);
		servo_start_rotate(&gimbal_servo,1,rotate_angle_correct,200);
		In_Serial=1;
	}
	if(In_Serial==1)
	{
		if(parsed_data.QR_NUM1!=0&&parsed_data.QR_NUM2!=0)
		{
			taskENTER_CRITICAL(); // 进入临界区
			motion_steps[step_index].action_exit_flag = 1;
			In_Serial=0;
			taskEXIT_CRITICAL();
		}
    }
}

uint8_t object_pos[3];
uint8_t clockwise_flag=0;
void step2_extra_action(uint8_t step_index)//到达转盘位置，发送识别指令，夹取物料，循环任务
{
	if(In_Serial==0)//发送识别物料信息
	{
		Serial_SendString("TWO1");
		x_axis_disabled = 0;
        y_axis_disabled = 0;
		In_Serial=1;
	}
	if(In_Serial==1)
	{
		if(parsed_data.dx!=0||parsed_data.dy!=0)
		{
			if(parsed_data.dx==311&& parsed_data.dy == 311)
			{
				calculate_wheel_cali_straight_revolution(0.02, 150, 100);
				vTaskDelay(500);
				parsed_data.dx=0;
				parsed_data.dy=0;
				Serial_SendString("TWO1");
			}
			else if(parsed_data.dx==322&& parsed_data.dy == 322)
			{
				calculate_wheel_cali_straight_revolution(-0.02, 150, 100);
				vTaskDelay(500);
				parsed_data.dx=0;
				parsed_data.dy=0;
				Serial_SendString("TWO1");
			}
			else
			{
				vision_cali(Material_Area2_x_kp,Material_Area2_y_kp);
				if(parsed_data.func_code!=3)
				{
					Serial_SendString("TWO1");
				}
				else if(parsed_data.func_code==3)
				{
					x_axis_disabled = 0;
					y_axis_disabled = 0;
					In_Serial=2;
				}
			}
		}
		if(parsed_data.func_code==3)
		{
			x_axis_disabled = 0;
			y_axis_disabled = 0;
			In_Serial=2;
		}
	}
	if(In_Serial==2)
	{
		if(parsed_data.func_code==3)//判断功能码，存储位置数据
		{
			object_pos[0]=parsed_data.Material_Pos[0];
			object_pos[1]=parsed_data.Material_Pos[1];
			
			// 查找1-3中不在前两个位置的数字
			for(int i=1; i<=3; i++)
			{
				if(i!=object_pos[0] && i!=object_pos[1])
				{
					object_pos[2]=i;
					In_Serial=3;
					break;
				}
			}
			Set_Turntable_Angle(Object1_Angle);
		}
	}
	if(In_Serial==3)
	{
		if(object_pos[0]==1)//蓝左
		{
			Catch_Object_Bule();
			Set_Turntable_Angle(Object2_Angle);
			In_Serial=4;  
		}
		else if(object_pos[0]==2)//绿中
		{
		    Catch_Object_Green();
			Set_Turntable_Angle(Object2_Angle);
			In_Serial=4;
		}
		else if(object_pos[0]==3)//红右
		{
			Catch_Object_Red();
			Set_Turntable_Angle(Object2_Angle);
			In_Serial=4;
		}
	}
	if(In_Serial==4)
	{
		if(object_pos[1]==1)//蓝左
		{
			Catch_Object_Bule();
			Set_Turntable_Angle(Object3_Angle);
			In_Serial=5;  
		}
		else if(object_pos[1]==2)//绿中
		{
		    Catch_Object_Green();
			Set_Turntable_Angle(Object3_Angle);
			In_Serial=5;
		}
		else if(object_pos[1]==3)//红右
		{
			Catch_Object_Red();
			Set_Turntable_Angle(Object3_Angle);
			In_Serial=5;
		}
	}
	if(In_Serial==5)
	{
		if(object_pos[2]==1)//蓝左
		{
			Catch_Object_Bule();
			Set_Turntable_Angle(Object1_Angle);
			parsed_data.func_code=0;
			In_Serial=0; 
            motion_steps[step_index].action_exit_flag = 1;	
		}
		else if(object_pos[2]==2)//绿中
		{
			Catch_Object_Green();
			Set_Turntable_Angle(Object1_Angle);
			parsed_data.func_code=0;
			In_Serial=0; 
            motion_steps[step_index].action_exit_flag = 1;
		}
		else if(object_pos[2]==3)//红右
		{
			Catch_Object_Red();
			Set_Turntable_Angle(Object1_Angle);
			parsed_data.func_code=0;
			In_Serial=0; 
            motion_steps[step_index].action_exit_flag = 1;
		}
	}
}

void step3_extra_action(uint8_t step_index)//转到识别角度
{
	if(In_Serial==0)
	{
		Set_Angle(3,150);
		servo_start_rotate(&gimbal_servo,1,rotate_angle_correct-2,200);
		Move_Rack_Absolute_Y(Correct_Middle_LEN,fast_y_speed,fast_y_acc);
		OPS9_stop();
		In_Serial=1;
	}
	if(In_Serial==1)
	{
		taskENTER_CRITICAL(); // 进入临界区
		motion_steps[step_index].action_exit_flag = 1;
		In_Serial=0;
		taskEXIT_CRITICAL();
    }
}

void step4_extra_action(uint8_t step_index) //校准，放置
{
	if(In_Serial==0)
	{
		Serial_SendString("THREE1");
		x_axis_disabled = 0;
        y_axis_disabled = 0;
		In_Serial=1;
	}
	if(In_Serial==1)
	{
		if(parsed_data.dx!=0||parsed_data.dy!=0)//识别到了
		{
			vision_cali(Material_Area_x_kp,Material_Area_y_kp);
			if(parsed_data.func_code!=3)
			{
				Serial_SendString("THREE1");
			}
			else if(parsed_data.func_code==3)
			{
				x_axis_disabled = 0;
				y_axis_disabled = 0;
				In_Serial=2;
			}
		}
		if(parsed_data.func_code==3)
		{
			x_axis_disabled = 0;
			y_axis_disabled = 0;
			In_Serial=2;
		}
	}
	if(In_Serial==2)
	{
		if(parsed_data.func_code==3)//可以放置
		{
			object_pos[0]=parsed_data.Material_Pos[0];
			object_pos[1]=parsed_data.Material_Pos[1];
			
			//查找1-3中不在前两个位置的数字
			for(int i=1; i<=3; i++)
			{
				if(i!=object_pos[0] && i!=object_pos[1])
				{
					object_pos[2]=i;
					In_Serial=3;
					break;
				}
			}
		}
	}
	if(In_Serial==3)//放置第一个放进来的物体
	{
		Set_Angle(3,Release_Angle);
		servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
		Move_Rack_Absolute_Y(0,fast_y_speed,fast_y_acc);
		vTaskDelay(delay_time_turntable_car);
		if(object_pos[0]==3)//蓝左
		{
			Set_Object_Bule(rotate_angle_bule,Object2_Angle,Left_LEN+3);
			In_Serial=4;
		}
		if(object_pos[0]==2)//绿中
		{
			Set_Object_Green(rotate_angle_green,Object2_Angle,Middle_LEN+5);
			In_Serial=4;
		}
		if(object_pos[0]==1)//红右
		{
			Set_Object_Red(rotate_angle_red,Object2_Angle,Right_LEN+5); 
			In_Serial=4;
		}
	}
	if(In_Serial==4)//放置第二个放进来的物体
	{
		Set_Angle(3,Release_Angle);
		servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
		Move_Rack_Absolute_Y(0,fast_y_speed,fast_y_acc);
		vTaskDelay(delay_time_turntable_car);
		if(object_pos[1]==3)//蓝左
		{
			Set_Object_Bule(rotate_angle_bule,Object3_Angle,Left_LEN+3);
			In_Serial=5;
		}
		if(object_pos[1]==2)//绿中
		{
			Set_Object_Green(rotate_angle_green,Object3_Angle,Middle_LEN+5);
			In_Serial=5;
		}
		if(object_pos[1]==1)//红右
		{
			Set_Object_Red(rotate_angle_red,Object3_Angle,Right_LEN+5);
			In_Serial=5;
		}
	}
	if(In_Serial==5)
	{
		Set_Angle(3,Release_Angle);
		servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
		Move_Rack_Absolute_Y(0,fast_y_speed,fast_y_acc);
		vTaskDelay(delay_time_turntable_car);
		if(object_pos[2]==3)//蓝左
		{
			Set_Object_Bule(rotate_angle_bule,Object1_Angle,Left_LEN+3);
			Set_Angle(3,150);
			In_Serial=6; 
		}
		if(object_pos[2]==2)//绿中
		{
			Set_Object_Green(rotate_angle_green,Object1_Angle,Middle_LEN+5);
			Set_Angle(3,150);
			In_Serial=6; 			
		}
		if(object_pos[2]==1)//红右
		{
			Set_Object_Red(rotate_angle_red,Object1_Angle,Right_LEN+5);
			Set_Angle(3,150);
			In_Serial=6; 		
		}
	}
	//抓回物体
	if(In_Serial==6)
	{
		if(object_pos[0]==1)//红右
		{
			Catch_Object_Red();
			Set_Turntable_Angle(Object2_Angle);
			In_Serial=7;
		}
		if(object_pos[0]==2)//绿中
		{
			Catch_Object_Green();
			Set_Turntable_Angle(Object2_Angle);
			In_Serial=7;
		}
		if(object_pos[0]==3)//蓝左
		{
			Catch_Object_Bule();
			Set_Turntable_Angle(Object2_Angle);
			In_Serial=7;
		}
	}
	if(In_Serial==7)
	{
		if(object_pos[1]==1)//红右
		{
			Catch_Object_Red();
			Set_Turntable_Angle(Object3_Angle);
			In_Serial=8;
		}
		if(object_pos[1]==2)//绿中
		{
			Catch_Object_Green();
			Set_Turntable_Angle(Object3_Angle);
			In_Serial=8;
		}
		if(object_pos[1]==3)//蓝左
		{
			Catch_Object_Bule();
			Set_Turntable_Angle(Object3_Angle);
			In_Serial=8;
		}
	}
	if(In_Serial==8)
	{
		if(object_pos[2]==1)//红右
		{
			Catch_Object_Red();
			Set_Turntable_Angle(Object1_Angle);
			parsed_data.func_code=0;
			In_Serial=0; 
            motion_steps[step_index].action_exit_flag = 1;	
		}
		if(object_pos[2]==2)//绿中
		{
			Catch_Object_Green();
			Set_Turntable_Angle(Object1_Angle);
			parsed_data.func_code=0;
			In_Serial=0; 
            motion_steps[step_index].action_exit_flag = 1;
		}
		if(object_pos[2]==3)//蓝左
		{
			Catch_Object_Bule();
			Set_Turntable_Angle(Object1_Angle);
			parsed_data.func_code=0;
			In_Serial=0; 
            motion_steps[step_index].action_exit_flag = 1;
		}
	}		
}

void step5_extra_action(uint8_t step_index)//暂存区的放置
{
	if(In_Serial==0)
	{
		Serial_SendString("THREE1");
		x_axis_disabled = 0;
        y_axis_disabled = 0;
		In_Serial=1;
	}
	if(In_Serial==1)
	{
		if(parsed_data.dx!=0||parsed_data.dy!=0)//识别到了
		{
			vision_cali(Material_Area_x_kp,Material_Area_y_kp);
			if(parsed_data.func_code!=3)
			{
				Serial_SendString("THREE1");
			}
			else if(parsed_data.func_code==3)
			{
				x_axis_disabled = 0;
				y_axis_disabled = 0;
				In_Serial=2;
			}
		}
		if(parsed_data.func_code==3)
		{
			x_axis_disabled = 0;
			y_axis_disabled = 0;
			In_Serial=2;
		}
	}
	if(In_Serial==2)
	{
		if(parsed_data.func_code==3)//可以放置
		{
			object_pos[0]=parsed_data.Material_Pos[0];
			object_pos[1]=parsed_data.Material_Pos[1];
			
			// 查找1-3中不在前两个位置的数字
			for(int i=1; i<=3; i++)
			{
				if(i!=object_pos[0] && i!=object_pos[1])
				{
					object_pos[2]=i;
					In_Serial=3;
					break;
				}
			}
		}
	}
	if(In_Serial==3)//放置第一个放进来的物体
	{
		Set_Angle(3,Release_Angle);
		servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
		Move_Rack_Absolute_Y(0,fast_y_speed,fast_y_acc);
		vTaskDelay(delay_time_turntable_car);
		if(object_pos[0]==1)//红右
		{
			Set_Object_Red(rotate_angle_red,Object2_Angle,Right_LEN+5);
			In_Serial=4;
		}
		if(object_pos[0]==2)//绿中
		{
			Set_Object_Green(rotate_angle_green,Object2_Angle,Middle_LEN);
			In_Serial=4;
		}
		if(object_pos[0]==3)//蓝左
		{
			Set_Object_Bule(rotate_angle_bule,Object2_Angle,Left_LEN);
			In_Serial=4;
		}
	}
	if(In_Serial==4)//放置第二个放进来的物体
	{
		Set_Angle(3,Release_Angle);
		servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
		Move_Rack_Absolute_Y(0,fast_y_speed,fast_y_acc);
		vTaskDelay(delay_time_turntable_car);
		if(object_pos[1]==1)//红右
		{
			Set_Object_Red(rotate_angle_red,Object3_Angle,Right_LEN+5);
			In_Serial=5;
		}
		if(object_pos[1]==2)//绿中
		{
			Set_Object_Green(rotate_angle_green,Object3_Angle,Middle_LEN);
			In_Serial=5;
		}
		if(object_pos[1]==3)//蓝左
		{
			Set_Object_Bule(rotate_angle_bule,Object3_Angle,Left_LEN);
			In_Serial=5;
		}
	}
	if(In_Serial==5)
	{
		Set_Angle(3,Release_Angle);
		servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
		Move_Rack_Absolute_Y(0,fast_y_speed,fast_y_acc);
		vTaskDelay(delay_time_turntable_car);
		if(object_pos[2]==1)//红右
		{
			Set_Object_Red(rotate_angle_red,Object1_Angle,Right_LEN+5);
			Set_Angle(3,100);
		    servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
		    Move_Rack_Absolute_Y(0,fast_y_speed,fast_y_acc);
			parsed_data.func_code=0;
			In_Serial=0; 
            motion_steps[step_index].action_exit_flag = 1; 
		}
		if(object_pos[2]==2)//绿中
		{
			Set_Object_Green(rotate_angle_green,Object1_Angle,Middle_LEN);
			Set_Angle(3,100);
		    servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
		    Move_Rack_Absolute_Y(0,fast_y_speed,fast_y_acc);
			parsed_data.func_code=0;
			In_Serial=0; 
            motion_steps[step_index].action_exit_flag = 1;			
		}
		if(object_pos[2]==3)//蓝左
		{
			Set_Object_Bule(rotate_angle_bule,Object1_Angle,Left_LEN);
			Set_Angle(3,100);
		    servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
		    Move_Rack_Absolute_Y(0,fast_y_speed,fast_y_acc);
			parsed_data.func_code=0;
			In_Serial=0; 
            motion_steps[step_index].action_exit_flag = 1; 		
		}
	}
}

void step6_extra_action(uint8_t step_index)
{
	if(In_Serial==0)
	{
		Set_Angle(3,150);
		servo_start_rotate(&gimbal_servo,1,rotate_angle_correct,200);
		Move_Rack_Absolute_Y(0,fast_y_speed,fast_y_acc);
		In_Serial=1;
	}
	if(In_Serial==1)
	{
		taskENTER_CRITICAL(); // 进入临界区
		motion_steps[step_index].action_exit_flag = 1;
		In_Serial=0;
		taskEXIT_CRITICAL();
    }
}

void step7_extra_action(uint8_t step_index)
{
	if(In_Serial==0)//发送识别物料信息
	{
		Serial_SendString("TWO2");
		x_axis_disabled = 0;
        y_axis_disabled = 0;
		In_Serial=1;
	}
	if(In_Serial==1)
	{
		if(parsed_data.dx!=0||parsed_data.dy!=0)
		{
			if(parsed_data.dx==311&& parsed_data.dy == 311)
			{
				calculate_wheel_cali_straight_revolution(0.02, 150, 100);
				vTaskDelay(500);
				parsed_data.dx=0;
				parsed_data.dy=0;
				Serial_SendString("TWO2");
			}
			else if(parsed_data.dx==322&& parsed_data.dy == 322)
			{
				calculate_wheel_cali_straight_revolution(-0.02, 150, 100);
				vTaskDelay(500);
				parsed_data.dx=0;
				parsed_data.dy=0;
				Serial_SendString("TWO2");
			}
			else
			{
				vision_cali(Material_Area_x_kp,Material_Area_y_kp);
				if(parsed_data.func_code!=3)
				{
					Serial_SendString("TWO2");
				}
				else if(parsed_data.func_code==3)
				{
					x_axis_disabled = 0;
					y_axis_disabled = 0;
					In_Serial=2;
				}
			}
		}
		if(parsed_data.func_code==3)
		{
			x_axis_disabled = 0;
			y_axis_disabled = 0;
			In_Serial=2;
		}
	}
	if(In_Serial==2)
	{
		if(parsed_data.func_code==3)//判断功能码，存储位置数据
		{
			object_pos[0]=parsed_data.Material_Pos[0];
			object_pos[1]=parsed_data.Material_Pos[1];
			
			// 查找1-3中不在前两个位置的数字
			for(int i=1; i<=3; i++)
			{
				if(i!=object_pos[0] && i!=object_pos[1])
				{
					object_pos[2]=i;
					In_Serial=3;
					break;
				}
			}
			Set_Turntable_Angle(Object1_Angle);
		}
	}
	if(In_Serial==3)
	{
		if(object_pos[0]==1)//蓝左
		{
			Catch_Object_Bule();
			Set_Turntable_Angle(Object2_Angle);
			In_Serial=4;  
		}
		else if(object_pos[0]==2)//绿中
		{
		    Catch_Object_Green();
			Set_Turntable_Angle(Object2_Angle);
			In_Serial=4;
		}
		else if(object_pos[0]==3)//红右
		{
			Catch_Object_Red();
			Set_Turntable_Angle(Object2_Angle);
			In_Serial=4;
		}
	}
	if(In_Serial==4)
	{
		if(object_pos[1]==1)//蓝左
		{
			Catch_Object_Bule();
			Set_Turntable_Angle(Object3_Angle);
			In_Serial=5;  
		}
		else if(object_pos[1]==2)//绿中
		{
		    Catch_Object_Green();
			Set_Turntable_Angle(Object3_Angle);
			In_Serial=5;
		}
		else if(object_pos[1]==3)//红右
		{
			Catch_Object_Red();
			Set_Turntable_Angle(Object3_Angle);
			In_Serial=5;
		}
	}
	if(In_Serial==5)
	{
		if(object_pos[2]==1)//蓝左
		{
			Catch_Object_Bule();
			Set_Turntable_Angle(Object1_Angle);
			parsed_data.func_code=0;
			In_Serial=0; 
            motion_steps[step_index].action_exit_flag = 1;	
		}
		else if(object_pos[2]==2)//绿中
		{
			Catch_Object_Green();
			Set_Turntable_Angle(Object1_Angle);
			parsed_data.func_code=0;
			In_Serial=0; 
            motion_steps[step_index].action_exit_flag = 1;
		}
		else if(object_pos[2]==3)//红右
		{
			Catch_Object_Red();
			Set_Turntable_Angle(Object1_Angle);
			parsed_data.func_code=0;
			In_Serial=0; 
            motion_steps[step_index].action_exit_flag = 1;
		}
	}
}

void step8_extra_action(uint8_t step_index)
{
		if(In_Serial==0)
	{
		Serial_SendString("THREE2");
		x_axis_disabled = 0;
        y_axis_disabled = 0;
		In_Serial=1;
	}
	if(In_Serial==1)
	{
		if(parsed_data.dx!=0||parsed_data.dy!=0)//识别到了
		{
			vision_cali(Material_Area_x_kp,Material_Area_y_kp);
			if(parsed_data.func_code!=3)
			{
				Serial_SendString("THREE2");
			}
			else if(parsed_data.func_code==3)
			{
				x_axis_disabled = 0;
				y_axis_disabled = 0;
				In_Serial=2;
			}
		}
		if(parsed_data.func_code==3)
		{
			x_axis_disabled = 0;
			y_axis_disabled = 0;
			In_Serial=2;
		}
	}
	if(In_Serial==2)
	{
		if(parsed_data.func_code==3)//可以放置
		{
			object_pos[0]=parsed_data.Material_Pos[0];
			object_pos[1]=parsed_data.Material_Pos[1];
			
			// 查找1-3中不在前两个位置的数字
			for(int i=1; i<=3; i++)
			{
				if(i!=object_pos[0] && i!=object_pos[1])
				{
					object_pos[2]=i;
					In_Serial=3;
					break;
				}
			}
		}
	}
	if(In_Serial==3)//放置第一个放进来的物体
	{
		Set_Angle(3,Release_Angle);
		servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
		Move_Rack_Absolute_Y(0,fast_y_speed,fast_y_acc);
		vTaskDelay(delay_time_turntable_car);
		if(object_pos[0]==1)//红右
		{
			Set_Object_Red(rotate_angle_red,Object2_Angle,Right_LEN+3); 
			In_Serial=4;
		}
		if(object_pos[0]==2)//绿中
		{
			Set_Object_Green(rotate_angle_green,Object2_Angle,Middle_LEN+5);
			In_Serial=4;
		}
		if(object_pos[0]==3)//蓝左
		{
			Set_Object_Bule(rotate_angle_bule,Object2_Angle,Left_LEN+5);
			In_Serial=4;
		}
	}
	if(In_Serial==4)//放置第二个放进来的物体
	{
		Set_Angle(3,Release_Angle);
		servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
		Move_Rack_Absolute_Y(0,fast_y_speed,fast_y_acc);
		vTaskDelay(delay_time_turntable_car);
		if(object_pos[1]==1)//红右
		{
			Set_Object_Red(rotate_angle_red,Object3_Angle,Right_LEN+3);
			In_Serial=5;
		}
		if(object_pos[1]==2)//绿中
		{
			Set_Object_Green(rotate_angle_green,Object3_Angle,Middle_LEN+5);
			In_Serial=5;
		}
		if(object_pos[1]==3)//蓝左
		{
			Set_Object_Bule(rotate_angle_bule,Object3_Angle,Left_LEN+5);
			In_Serial=5;
		}
	}
	if(In_Serial==5)
	{
		Set_Angle(3,Release_Angle);
		servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
		Move_Rack_Absolute_Y(0,fast_y_speed,fast_y_acc);
		vTaskDelay(delay_time_turntable_car);
		if(object_pos[2]==1)//红右
		{
			Set_Object_Red(rotate_angle_red,Object1_Angle,Right_LEN+3);
			Set_Angle(3,150);
			In_Serial=6; 
		}
		if(object_pos[2]==2)//绿中
		{
			Set_Object_Green(rotate_angle_green,Object1_Angle,Middle_LEN+5);
			Set_Angle(3,150);
			In_Serial=6; 			
		}
		if(object_pos[2]==3)//蓝左
		{
			Set_Object_Bule(rotate_angle_bule,Object1_Angle,Left_LEN+5);
			Set_Angle(3,150);
			In_Serial=6; 		
		}
	}
	//抓回物体
	if(In_Serial==6)
	{
		if(object_pos[0]==1)//红右
		{
			Catch_Object_Red();
			Set_Turntable_Angle(Object2_Angle);
			In_Serial=7;
		}
		if(object_pos[0]==2)//绿中
		{
			Catch_Object_Green();
			Set_Turntable_Angle(Object2_Angle);
			In_Serial=7;
		}
		if(object_pos[0]==3)//蓝左
		{
			Catch_Object_Bule();
			Set_Turntable_Angle(Object2_Angle);
			In_Serial=7;
		}
	}
	if(In_Serial==7)
	{
		if(object_pos[1]==1)//红右
		{
			Catch_Object_Red();
			Set_Turntable_Angle(Object3_Angle);
			In_Serial=8;
		}
		if(object_pos[1]==2)//绿中
		{
			Catch_Object_Green();
			Set_Turntable_Angle(Object3_Angle);
			In_Serial=8;
		}
		if(object_pos[1]==3)//蓝左
		{
			Catch_Object_Bule();
			Set_Turntable_Angle(Object3_Angle);
			In_Serial=8;
		}
	}
	if(In_Serial==8)
	{
		if(object_pos[2]==1)//红右
		{
			Catch_Object_Red();
			Set_Turntable_Angle(Object1_Angle);
			parsed_data.func_code=0;
			In_Serial=0; 
            motion_steps[step_index].action_exit_flag = 1;	
		}
		if(object_pos[2]==2)//绿中
		{
			Catch_Object_Green();
			Set_Turntable_Angle(Object1_Angle);
			parsed_data.func_code=0;
			In_Serial=0; 
            motion_steps[step_index].action_exit_flag = 1;
		}
		if(object_pos[2]==3)//蓝左
		{
			Catch_Object_Bule();
			Set_Turntable_Angle(Object1_Angle);
			parsed_data.func_code=0;
			In_Serial=0; 
            motion_steps[step_index].action_exit_flag = 1;
		}
	}	
}

void step9_extra_action(uint8_t step_index)//转盘放置物体
{
	if(In_Serial==0)
	{
		Serial_SendString("FOUR1");
		x_axis_disabled = 0;
        y_axis_disabled = 0;
		In_Serial=1;
	}
	if(In_Serial==1)
	{
		if(parsed_data.dx!=0||parsed_data.dy!=0)//识别到了，开始标定
		{
			vision_cali(turntable_kp,turntable_kp);
			x_axis_disabled = 0;
            y_axis_disabled = 0;
			Serial_SendString("FIRST1");
			In_Serial=2;
		}
	}
	if(In_Serial==2)
	{
		if(parsed_data.func_code==4)//可以放置
		{
			Set_Angle(3,Release_Angle);
		    servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
		    Move_Rack_Absolute_Y(0,fast_y_speed,fast_y_acc);
			vTaskDelay(delay_time_turntable_car);
			In_Serial=3;
		}
	}
	if(In_Serial==3)//放置第一个放进来的物体
	{
		Set_Object(Object2_Angle);
		Serial_SendString("FIRST1");
		parsed_data.func_code=0;
		In_Serial=4;
	}
	if(In_Serial==4)//放置第二个放进来的物体
	{
		if(parsed_data.func_code==4)//可以放置
		{
			Set_Angle(3,Release_Angle);
		    servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
		    Move_Rack_Absolute_Y(0,fast_y_speed,fast_y_acc);
			vTaskDelay(delay_time_turntable_car);
			In_Serial=5;
		}
	}
	if(In_Serial==5)
	{
		Set_Object(Object3_Angle);
		Serial_SendString("FIRST1");
		parsed_data.func_code=0;
		In_Serial=6;
	}
	if(In_Serial==6)//放置第二个放进来的物体
	{
		if(parsed_data.func_code==4)//可以放置
		{
			Set_Angle(3,Release_Angle);
		    servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
		    Move_Rack_Absolute_Y(0,fast_y_speed,fast_y_acc);
			vTaskDelay(delay_time_turntable_car);
			In_Serial=7;
		}
	}
	if(In_Serial==7)
	{
		Set_Object(Object1_Angle);
		parsed_data.func_code=0;
		In_Serial=0;
		motion_steps[step_index].action_exit_flag = 1;
	}
}

void step10_extra_action(uint8_t step_index)
{
	if(In_Serial==0)
	{
		Serial_SendString("FOUR2");
		x_axis_disabled = 0;
        y_axis_disabled = 0;
		In_Serial=1;
	}
	if(In_Serial==1)
	{
		if(parsed_data.dx!=0||parsed_data.dy!=0)//识别到了，开始标定
		{
			vision_cali(turntable_kp,turntable_kp);
			x_axis_disabled = 0;
            y_axis_disabled = 0;
			Serial_SendString("FIRST2");
			In_Serial=2;
		}
	}
	if(In_Serial==2)
	{
		if(parsed_data.func_code==4)//可以放置
		{
			Set_Angle(3,Release_Angle);
		    servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
		    Move_Rack_Absolute_Y(0,fast_y_speed,fast_y_acc);
			vTaskDelay(delay_time_turntable_car);
			In_Serial=3;
		}
	}
	if(In_Serial==3)//放置第一个放进来的物体
	{
		Set_Object(Object2_Angle);
		Serial_SendString("FIRST2");
		parsed_data.func_code=0;
		In_Serial=4;
	}
	if(In_Serial==4)//放置第二个放进来的物体
	{
		if(parsed_data.func_code==4)//可以放置
		{
			Set_Angle(3,Release_Angle);
		    servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
		    Move_Rack_Absolute_Y(0,fast_y_speed,fast_y_acc);
			vTaskDelay(delay_time_turntable_car);
			In_Serial=5;
		}
	}
	if(In_Serial==5)
	{
		Set_Object(Object3_Angle);
		Serial_SendString("FIRST2");
		parsed_data.func_code=0;
		In_Serial=6;
	}
	if(In_Serial==6)//放置第二个放进来的物体
	{
		if(parsed_data.func_code==4)//可以放置
		{
			Set_Angle(3,Release_Angle);
		    servo_start_rotate(&gimbal_servo,1,rotate_angle_set,200);
		    Move_Rack_Absolute_Y(0,fast_y_speed,fast_y_acc);
			vTaskDelay(delay_time_turntable_car);
			In_Serial=7;
		}
	}
	if(In_Serial==7)
	{
		Set_Object(Object1_Angle);
		parsed_data.func_code=0;
		In_Serial=0;
		motion_steps[step_index].action_exit_flag = 1;
	}
}

void step11_extra_action(uint8_t step_index)
{
	Set_Angle(Gimbal_Servo_ID,246.0f);
	Move_Rack_Absolute_Y_One(60,50,20);
	Set_Claw_Height(16.32);
	Set_Angle (3,160);
	motion_steps[step_index].action_exit_flag = 1;
}

uint8_t need_skip_next_step = 0;  // 是否跳过下一步

/*
运动控制数组
*/
Move_Control_Step motion_steps[]={
	//转弯出暂存区
	{
		.type=STEP_DIFF_TURN,
		.skip_motor=0,
		.params.diffturn={500,-13000,100,70},//左右脉冲，速度加速度
		.need_wait=1,
		.action=step0_extra_action,//发送信息
		.action_repeat=-1,
		.action_exit_flag=0,
	},
	//二维码区
	{
		.type=STEP_COORDINATE,
		.skip_motor=0,
		.params.coordinate={-0.73,-0.10,-90},//x坐标,y坐标,期望角度
		.need_wait=0,
		.action=step1_extra_action,
		.action_repeat=-1,
		.action_exit_flag=0,
	},
	//走到十字路口准备转弯（第二个长度参数需要调整）
	{
		.type=STEP_STRAIGHT,
		.skip_motor=0,
		.params.straight={0.05,-0.40,-90,0},//运动距离，期望角，直走后是否进行角度矫正（0否1是）
		.need_wait=0,
		.action=NULL,
		.action_repeat=0,
		.action_exit_flag=0,
	},
	//过中间十字路口
	{
		.type=STEP_STRAIGHT,
		.skip_motor=0,
		.params.straight={0,-1.71,-1,0},//运动距离，期望角，直走后是否进行角度矫正（0否1是）
		.need_wait=0,
		.action=step3_extra_action,
		.action_repeat=-1,
		.action_exit_flag=0,
	},
	//校准完角度，进行物料夹取
	{
		.type=STEP_STRAIGHT,
		.skip_motor=0,
		.params.straight={0,0,90,1},//运动距离，期望角，直走后是否进行角度矫正（0否1是）
		.need_wait=0,
		.action=step2_extra_action,
		.action_repeat=-1,
		.action_exit_flag=0,
	},
	//夹取完走到拐角准备转弯
    {
		.type=STEP_STRAIGHT,
		.skip_motor=0,
		.params.straight={0.04,0.82,89,0},//运动距离，期望角，直走后是否进行角度矫正（0否1是）
		.need_wait=0,
		.action=step3_extra_action,
		.action_repeat=-1,
		.action_exit_flag=0,
	},
	//走到原暂存区，现粗加工区（第一轮）
	{
		.type=STEP_STRAIGHT,
		.skip_motor=0,
		.params.straight={-0.02,0.81,0,1},//运动距离，期望角，直走后是否进行角度矫正（0否1是）
		.need_wait=0,
		.action=step4_extra_action,
		.action_repeat=-1,
		.action_exit_flag=0,
	},
	//走到拐角处准备转弯
	{
		.type=STEP_STRAIGHT,
		.skip_motor=0,
		.params.straight={0.04,0.86,0,0},//运动距离，期望角，直走后是否进行角度矫正（0否1是）
		.need_wait=0,
		.action=step6_extra_action,
		.action_repeat=-1,
		.action_exit_flag=0,
	},
	//转弯后，运动到第一轮转盘处，进行识别放置
	{
		.type=STEP_STRAIGHT,
		.skip_motor=0,
		.params.straight={-0.04,0.40,-90,1},//运动距离，期望角，直走后是否进行角度矫正（0否1是）
		.need_wait=0,
		.action=step9_extra_action,
		.action_repeat=-1,
		.action_exit_flag=0,
	},
	//运动到十字路口处准备转弯（参数可能得调整）
	{
		.type=STEP_STRAIGHT,
		.skip_motor=0,
		.params.straight={0.05,0.39,-90,0},//运动距离，期望角，直走后是否进行角度矫正（0否1是）
		.need_wait=0,
		.action=NULL,
		.action_repeat=0,
		.action_exit_flag=0,
	},
	//直走通过十字路口
	{
		.type=STEP_STRAIGHT,
		.skip_motor=0,
		.params.straight={0,-1.71,-1,0},//运动距离，期望角，直走后是否进行角度矫正（0否1是）
		.need_wait=0,
		.action=step3_extra_action,
		.action_repeat=-1,
		.action_exit_flag=0,
	},
	//校准完角度，进行第二轮物料夹取
	{
		.type=STEP_STRAIGHT,
		.skip_motor=0,
		.params.straight={0,0,90,1},//运动距离，期望角，直走后是否进行角度矫正（0否1是）
		.need_wait=0,
		.action=step7_extra_action,
		.action_repeat=-1,
		.action_exit_flag=0,
	},
	//运动到拐角处准备转弯（第二轮）
	{
		.type=STEP_STRAIGHT,
		.skip_motor=0,
		.params.straight={0.04,0.81,89,0},//运动距离，期望角，直走后是否进行角度矫正（0否1是）
		.need_wait=0,
		.action=step3_extra_action,
		.action_repeat=-1,
		.action_exit_flag=0,
	},
	//走到原暂存区，现粗加工区（第一轮）
	{
		.type=STEP_STRAIGHT,
		.skip_motor=0,
		.params.straight={-0.02,0.81,0,1},//运动距离，期望角，直走后是否进行角度矫正（0否1是）
		.need_wait=0,
		.action=step8_extra_action,
		.action_repeat=-1,
		.action_exit_flag=0,
	},
	//运动到拐角处准备转弯（第二轮）
	{
		.type=STEP_STRAIGHT,
		.skip_motor=0,
		.params.straight={0.04,0.86,0,0},//运动距离，期望角，直走后是否进行角度矫正（0否1是）
		.need_wait=0,
		.action=step6_extra_action,
		.action_repeat=-1,
		.action_exit_flag=0,
	},
	//转弯后，运动到第二轮转盘处，进行识别放置
	{
		.type=STEP_STRAIGHT,
		.skip_motor=0,
		.params.straight={-0.04,0.40,-90,1},//运动距离，期望角，直走后是否进行角度矫正（0否1是）
		.need_wait=0,
		.action=step10_extra_action,
		.action_repeat=-1,
		.action_exit_flag=0,
	},
	//走回启停区（参数调试优先级最低）
	{
		.type=STEP_STRAIGHT,
		.skip_motor=0,
		.params.straight={0.032,1.25,-89.5,1},//运动距离，期望角，直走后是否进行角度矫正（0否1是）
		.need_wait=0,
		.action=step11_extra_action,
		.action_repeat=0,
		.action_exit_flag=0,
	},
	{
		.type=STEP_Slant,
		.skip_motor=0,
		.params.slant={8000,0,150,100},//左右斜脉冲，速度加速度
		.need_wait=1,
		.action=NULL,//发送信息
		.action_repeat=0,
		.action_exit_flag=0,
	},
//	{
//		.type=STEP_STRAIGHT,
//		.skip_motor=0,
//		.params.straight={0,0,90,1},//运动距离，期望角，直走后是否进行角度矫正（0否1是）
//		.need_wait=0,
//		.action=step8_extra_action,
//		.action_repeat=-1,
//		.action_exit_flag=0,
//	},
//	{
//		.type=STEP_STRAIGHT,
//		.skip_motor=0,
//		.params.straight={0.04,0.81,89,0},//运动距离，期望角，直走后是否进行角度矫正（0否1是）
//		.need_wait=0,
//		.action=step3_extra_action,
//		.action_repeat=-1,
//		.action_exit_flag=0,
//	},
//	{
//		.type=STEP_STRAIGHT,
//		.skip_motor=0,
//		.params.straight={-0.02,0.82,0,1},//运动距离，期望角，直走后是否进行角度矫正（0否1是）
//		.need_wait=0,
//		.action=step9_extra_action,
//		.action_repeat=-1,
//		.action_exit_flag=0,
//	},
//	{
//		.type=STEP_STRAIGHT,
//		.skip_motor=0,
//		.params.straight={0.04,0.86,0,0},//运动距离，期望角，直走后是否进行角度矫正（0否1是）
//		.need_wait=0,
//		.action=NULL,
//		.action_repeat=0,
//		.action_exit_flag=0,
//	},
//	{
//		.type=STEP_STRAIGHT,
//		.skip_motor=0,
//		.params.straight={-0.008,1.65,-89.5,1},//运动距离，期望角，直走后是否进行角度矫正（0否1是）
//		.need_wait=0,
//		.action=step10_extra_action,
//		.action_repeat=0,
//		.action_exit_flag=0,
//	},
//	{
//		.type=STEP_Slant,
//		.skip_motor=0,
//		.params.slant={8000,0,150,100},//左右斜脉冲，速度加速度
//		.need_wait=1,
//		.action=NULL,//发送信息
//		.action_repeat=0,
//		.action_exit_flag=0,
//	},
};

// 计算总步骤数（自动推导，避免硬编码）
const uint8_t TOTAL_STEPS = sizeof(motion_steps) / sizeof(Move_Control_Step);


