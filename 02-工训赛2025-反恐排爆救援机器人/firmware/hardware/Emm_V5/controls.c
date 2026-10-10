#include "controls.h"

float pos_robot_arm = 0.0f, Motor_Cur_Pos_robot_arm = 0.0f;
float pos_holder = 0.0f, Motor_Cur_Pos_holder = 0.0f;
float pos_robot_arm_1 = 0.0f, Motor_Cur_Pos_robot_arm_1 = 0.0f;
float pos_robot_arm_2 = 0.0f, Motor_Cur_Pos_robot_arm_2 = 0.0f;
volatile int32_t wheel_revolutions[5];				//脉冲累计

float Current_Claw_Height_Relative = MAX_HEIGHT;  //当前高度(初始最高)



void Readclk_robot_arm(void)//5
{
	Emm_V5_Read_Sys_Params(5, S_CPOS);
	vTaskDelay(1);
	while(Emm_V5_rxFrameFlag == false)
	{Emm_V5_rxFrameFlag = false;}
	if(Emm_V5_rxCmd[0] == 5 && Emm_V5_rxCmd[1] == 0x36 && Emm_V5_rxCount == 8)
	{
		// 拼接成uint32_t类型
		pos_robot_arm = (uint32_t)(
											((uint32_t)Emm_V5_rxCmd[3] << 24)    |
											((uint32_t)Emm_V5_rxCmd[4] << 16)    |
											((uint32_t)Emm_V5_rxCmd[5] << 8)     |
											((uint32_t)Emm_V5_rxCmd[6] << 0)
										);
		// 转换成角度
		Motor_Cur_Pos_robot_arm = (float)pos_robot_arm * 360.0f / 65536.0f;
		//转化成脉冲
		Motor_Cur_Pos_robot_arm = (int32_t)6400*(Motor_Cur_Pos_robot_arm/360.0f);
		// 符号
		if(Emm_V5_rxCmd[2]==1) { Motor_Cur_Pos_robot_arm = -Motor_Cur_Pos_robot_arm; }
	}
}

void Readclk_robot_arm_1(void)//6
{
	Emm_V5_Read_Sys_Params(6, S_CPOS);
	vTaskDelay(1);
	while(Emm_V5_rxFrameFlag == false)
	{Emm_V5_rxFrameFlag = false;}
	if(Emm_V5_rxCmd[0] == 6 && Emm_V5_rxCmd[1] == 0x36 && Emm_V5_rxCount == 8)
	{
		// 拼接成uint32_t类型
		pos_robot_arm_1 = (uint32_t)(
											((uint32_t)Emm_V5_rxCmd[3] << 24)    |
											((uint32_t)Emm_V5_rxCmd[4] << 16)    |
											((uint32_t)Emm_V5_rxCmd[5] << 8)     |
											((uint32_t)Emm_V5_rxCmd[6] << 0)
										);
		//转换成角度
		Motor_Cur_Pos_robot_arm_1 = (float)pos_robot_arm_1 * 360.0f / 65536.0f;
		//转化成脉冲
		Motor_Cur_Pos_robot_arm_1 = (int32_t)6400*(Motor_Cur_Pos_robot_arm_1/360.0f);
		// 符号
		if(Emm_V5_rxCmd[2]==0) { Motor_Cur_Pos_robot_arm_1 = -Motor_Cur_Pos_robot_arm_1; }
	}
}

void Readclk_robot_arm_2(void)//4
{
	Emm_V5_Read_Sys_Params(4, S_CPOS);
	vTaskDelay(1);
	while(Emm_V5_rxFrameFlag == false)
	{Emm_V5_rxFrameFlag = false;}
	if(Emm_V5_rxCmd[0] == 4 && Emm_V5_rxCmd[1] == 0x36 && Emm_V5_rxCount == 8)
	{
		// 拼接成uint32_t类型
		pos_robot_arm_2 = (uint32_t)(
											((uint32_t)Emm_V5_rxCmd[3] << 24)    |
											((uint32_t)Emm_V5_rxCmd[4] << 16)    |
											((uint32_t)Emm_V5_rxCmd[5] << 8)     |
											((uint32_t)Emm_V5_rxCmd[6] << 0)
										);
		// 转换成角度
		Motor_Cur_Pos_robot_arm_2 = (float)pos_robot_arm_2 * 360.0f / 65536.0f;
		//转化成脉冲
		Motor_Cur_Pos_robot_arm_2 = (int32_t)3200*(Motor_Cur_Pos_robot_arm_2/360.0f);
		// 符号
		if(Emm_V5_rxCmd[2]==0) { Motor_Cur_Pos_robot_arm_2 = -Motor_Cur_Pos_robot_arm_2; }
	}
}
int Read_robot_arm(void)//1
{
	Emm_V5_Read_Sys_Params(1, S_FLAG);
	vTaskDelay(1);
	while(Emm_V5_rxFrameFlag == false)
	{Emm_V5_rxFrameFlag = false;}
	if(Emm_V5_rxCmd[0] == 1 && Emm_V5_rxCmd[1] == 0x3A && Emm_V5_rxCount == 4)
	{
		if((0x03&Emm_V5_rxCmd[2])==0x01)
		{
			return 0;
		}
		else return 1;
	}
	else {
		return 0;
	}
}
int Read_holder(void)//2
{
	Emm_V5_Read_Sys_Params(2, S_FLAG);
	vTaskDelay(1);
	while(Emm_V5_rxFrameFlag == false)
	{Emm_V5_rxFrameFlag = false;}
	if(Emm_V5_rxCmd[0] == 2 && Emm_V5_rxCmd[1] == 0x3A && Emm_V5_rxCount == 4)
	{
		if((0x03&Emm_V5_rxCmd[2])==0x01)
		{
			return 0;
		}
		else return 1;
	}
	else {
		return 0;
	}
}

int Read_Z(void)
{
	Emm_V5_Read_Sys_Params(5, S_FLAG);
	vTaskDelay(1);
	while(Emm_V5_rxFrameFlag == false)
	{Emm_V5_rxFrameFlag = false;}
	if(Emm_V5_rxCmd[0] == 5 && Emm_V5_rxCmd[1] == 0x3A && Emm_V5_rxCount == 4)
	{
		if((0x03&Emm_V5_rxCmd[2])==0x01)
		{
			return 0;
		}
		else return 1;
	}
	else {
		return 0;
	}
}

int Read_Y(void) //地址6
{
	Emm_V5_Read_Sys_Params(6, S_FLAG);
	vTaskDelay(1);
	while(Emm_V5_rxFrameFlag == false)
	{Emm_V5_rxFrameFlag = false;}
	if(Emm_V5_rxCmd[0] == 6 && Emm_V5_rxCmd[1] == 0x3A && Emm_V5_rxCount == 4)
	{
		if((0x03&Emm_V5_rxCmd[2])==0x01)
		{
			return 0;
		}
		else return 1;
	}
	else {
		return 0;
	}
}

//Z和Y步进的函数封装
void Back_Zero(int Arr)
{
	if(Arr==5)
	{
		Emm_V5_Pos_Control(5,1,1600,200,0,1,0);
		wheel_revolutions[1]=0;
	}
	else if(Arr==6)
	{
		Emm_V5_Pos_Control(6,0,50,20,0,1,0);
		wheel_revolutions[0]=0;
	}	
}

void Strench_Pos_Z(uint8_t dir_Z,uint32_t clk_Z)
{
	Emm_V5_Pos_Control(5,dir_Z,1600,200,clk_Z,0,0);
}

void Strench_Pos_Y(uint8_t dir_Y,uint32_t clk_Y)
{
	Emm_V5_Pos_Control(6,dir_Y,100,50,clk_Y,0,0);
}

void Control_robot_arm_Y(uint8_t dir,int32_t clk,uint16_t v,uint8_t a)//3
{
	taskENTER_CRITICAL(); // 进入临界区
	if(dir==robot_arm_1_shen)
	{
		Emm_V5_Pos_Control(6,robot_arm_1_shen,v,a,clk,0,0);		
		vTaskDelay(1);
		wheel_revolutions[0]= wheel_revolutions[0] + clk;
	}else if(dir==robot_arm_1_shou)
	{
		Emm_V5_Pos_Control(6,robot_arm_1_shou,v,a,clk,0,0);		
		vTaskDelay(1);
		wheel_revolutions[0]= wheel_revolutions[0] - clk;
	}else 
	{
		Emm_V5_Stop_Now(6,0);
		vTaskDelay(1);
	}
	taskEXIT_CRITICAL(); // 退出临界区
}

void Control_robot_arm_Z(uint8_t dir,int32_t clk,uint16_t v,uint8_t a)//4
{
	taskENTER_CRITICAL(); // 进入临界区
	if(dir==robot_arm_2_shen)
	{
		Emm_V5_Pos_Control(5,robot_arm_2_shen,v,a,clk,0,0);		
		vTaskDelay(1);
		wheel_revolutions[1]= wheel_revolutions[1] + clk;
	}else if(dir==robot_arm_2_jian)
	{
		Emm_V5_Pos_Control(5,robot_arm_2_jian,v,a,clk,0,0);		
		vTaskDelay(1);
		wheel_revolutions[1]= wheel_revolutions[1] - clk;
	}else 
	{
		Emm_V5_Stop_Now(5,0);
		vTaskDelay(1);
	}
	taskEXIT_CRITICAL(); // 退出临界区
}

// 控制夹爪到指定高度（单位：厘米），使用相对脉冲
void Set_Claw_Height_Relative(float target_height, uint16_t v, uint8_t a)
{
	if(target_height < 0.0f) target_height = 0.0f;
	if(target_height > MAX_HEIGHT) target_height = MAX_HEIGHT;

	if(fabsf(target_height - Current_Claw_Height_Relative) < 0.01f) return;

	float height_diff = target_height - Current_Claw_Height_Relative;
	float pulse_val = height_diff * PULSE_PER_CM;
	int32_t fixed_pulse = (int32_t)(pulse_val + 0.5f);

	if(fixed_pulse > 0)
	{
		Control_robot_arm_Z(Z_UP, fixed_pulse, v, a);
	}
	else if(fixed_pulse < 0)
	{
		Control_robot_arm_Z(Z_DOWN, -fixed_pulse, v, a);
	}

	Current_Claw_Height_Relative = target_height;
}

int32_t Get_CLK(float kp,int32_t error)
{
	int32_t convert_clk=(int32_t)lround(kp*error);
	return convert_clk;
}

void Correct_Ball(float kp,int32_t error)//
{
	int32_t load_clk=Get_CLK(kp,error);
	if(load_clk<0)
	{
		load_clk=-1*load_clk;
		Control_robot_arm_Y(0,load_clk,20,0);
	}
	else if(load_clk>0)
	{
		Control_robot_arm_Y(1,load_clk,20,0);
	}
	else if(load_clk==0)
	{
		Emm_V5_Stop_Now(0,0);
	}
}

uint8_t In_Servo=0;
/*舵机控制函数*/
void rotate_angle_slow(float Current_Angle,float Angle)
{
	(void)Current_Angle;	//兼容旧接口，当前位置由 gimbal_servo.current_angle 维护
	In_Servo=0;
	Servo_StartRotate(&gimbal_servo, GIMBAL_SERVO_CH, Angle, GIMBAL_SERVO_SPEED_DPS);
	while(gimbal_servo.is_moving)
	{
		vTaskDelay(1);
	}
	In_Servo=1;//到位标志
}
/*
云台舵机旋转角度控制函数
*/
void rotate_angle(float Angle)
{
	Servo_SetCurrentAngle(GIMBAL_SERVO_CH,Angle);
}

/*
夹爪角度控制函数
*/
void claw_angle(float Angle)	
{
	Servo_SetAngle(2,Angle);
}

/*
云台俯仰角角度控制函数
*/
void pitch_angle(float Angle)
{
	Servo_SetAngle(3,Angle);
}

/*
num:输入的百位整数, data:长度为3的数组，用来存放分解后的三个数字
*/
void splitHundred(int num, int data[3])
{
    data[0] = num / 100;        // 百位
    data[1] = (num / 10) % 10; // 十位
    data[2] = num % 10;         // 个位
}


// 全局参数（示例：可在业务函数中修改）
float global_speed_scale = 1.0f;  // 速度缩放系数
uint8_t need_skip_next_step = 0;  // 是否跳过下一步

int16_t explosive_num=0;//第几个是目标爆炸物
int16_t target_num=0;//第几个是目标靶子
int16_t hostage_num=0;//第几个是目标人质

int32_t Speed_sum_last=0;
int32_t Speed_sum_error=0;

/*------------矫正比例-------------*/
#define KP_BALL_X 185.0f      //球的x轴处的矫正比例
#define KP_BALL_Y 25.0f      //球的y轴处的矫正比例

#define KP_TARGET_X 714.0f      //球的x轴处的矫正比例

#define KP_RENZHI_X 167.0f      //yolo方案的人质脉冲像素比(x轴)

#define KP_TOPCAP_X 83.0f       //顶部摄像头的脉冲比例(x轴)
#define KP_TOPCAP_Y 10.0        //顶部摄像头的脉冲比例(y轴)

/*------------靶位判定-------------*/
#define TARGET_POS_1_BOUND   (-35000)  //1号靶与2号靶分界，单位同Speed_sum
#define TARGET_POS_3_BOUND   ( 35000)  //2号靶与3号靶分界，正增量为3号靶

/*--------------夹爪舵机角度-----------------*/
#define Catch_Ball_Angle      60.0f     //夹取小球的舵机角度
#define Catch_Frustum_Angle   60.0f     //夹取圆台的舵机角度
#define Catch_Cylinder_Angle  70.0f     //夹取圆柱的舵机角度
#define Catch_Drum_Angle      65.0f     //夹取腰鼓的舵机角度
#define Claw_Free_Angle       180.0f    //释放球体的舵机角度

#define bool_error_max 15
#define bool_error_min	-15
#define bool_error_kp	100

#define target_error_x_max 15
#define target_error_x_min	-15
#define target_error_x_kp 7
#define target_error_y_kp -0.5
#define target_error_kp	-200

#define hostage_error_max 15
#define hostage_error_min	-15
#define hostage_error_kp	50

#define motor_params_distance_1 30000 //距离：抓球时，跟踪完后继续走，到爪子对应球的位置的距离
#define motor_params_distance_2 65000 //距离：抓球完，继续走，到爪子对应放球的位置的距离，策略2：110000
#define motor_params_distance_3 -45000 //距离：

#define robot_arm_a 150

/*机械臂抓球控制参数*/
//伸出抓球分支
#define Control_robot_arm_one_clk_1 46000 //驱动关节一，伸出,1600-48000
#define Control_robot_arm_two_clk_1 15000 //驱动关节二，伸出0-15000
#define Control_robot_arm_three_clk_1 725 //驱动关节三，动，用到
//抓到后，收缩抓球分支
#define Control_robot_arm_one_clk_2 28200 //驱动关节一，收回47000-18800
#define Control_robot_arm_two_clk_2 9800 //驱动关节二，伸出15000-24800
#define Control_robot_arm_three_clk_2 700 //驱动关节三，不动，未用到
//抓到后，旋转后，伸出放球分支
#define Control_robot_arm_one_clk_3 31700 //驱动关节一，伸出18800-50500
#define Control_robot_arm_two_clk_3 15000 //驱动关节二，伸出24800-39700
#define Control_robot_arm_three_clk_3 1450 //驱动关节三，动，用到
//放球后，收回机械臂分支
#define Control_robot_arm_one_clk_4 31700 //驱动关节一，收回
#define Control_robot_arm_two_clk_4 15000 //驱动关节二，收回
#define Control_robot_arm_three_clk_4 700
//驱动关节三，动，未用到
//放球后，旋转后，收回机械臂分支
#define Control_robot_arm_one_clk_5 (Control_robot_arm_one_clk_1-Control_robot_arm_one_clk_2) //驱动关节一，收回
#define Control_robot_arm_two_clk_5 (Control_robot_arm_two_clk_1+Control_robot_arm_two_clk_2) //驱动关节二，收回
#define Control_robot_arm_three_clk_5 0 //驱动关节三，动，用到

/*机械臂抓人质控制参数*/
//伸出人质分支
#define Control_robot_arm_one_clk_6 20000 //驱动关节一，伸出,1600-21600
#define Control_robot_arm_two_clk_6 10000 //驱动关节二，伸出,0-10000
#define Control_robot_arm_three_clk_6 0 //驱动关节三，不动，未用到
//伸出人质分支
#define Control_robot_arm_one_clk_7 18496 //驱动关节一，伸出,21600-40096
#define Control_robot_arm_two_clk_7 8000 //驱动关节二，伸出,10000-18000

#define Control_robot_arm_one_clk_7_2 8000 //驱动关节一，伸出,40096-48096
#define Control_robot_arm_two_clk_7_2 4000 //驱动关节二，伸出,18000-22000

#define Control_robot_arm_three_clk_7 0 //驱动关节三，不动，未用到
//抓到后，收缩人质分支
#define Control_robot_arm_one_clk_8 20000 //驱动关节一，收回
#define Control_robot_arm_two_clk_8 10000 //驱动关节二，伸出
#define Control_robot_arm_three_clk_8 0 //驱动关节三，不动，未用到
//抓到后，收缩人质分支
#define Control_robot_arm_one_clk_9 26496 //驱动关节一，收回
#define Control_robot_arm_two_clk_9 12000 //驱动关节二，伸出
#define Control_robot_arm_three_clk_9 0 //驱动关节三，不动，未用到

int32_t bool_luchen_san=0;
int32_t luchen_san=0;
int32_t hostage_luchen_san=0;

uint8_t In_position=0;//指令发送标志位
int QR_CODE=0;
int QR_code_data[3];

// 任务0:过完减速带后倒退，矫正
void step0_extra_action(uint8_t step_index)
{
	//等待电机任务完成
	while (check_all_reached_filter() != 1)
	{
			vTaskDelay(1);
	}
	motor_fixed_params(800,180);
	vTaskDelay(1000);
	do
	{
		Speed_sum=0;
		vTaskDelay(1000);
		Speed_sum_error=Speed_sum-Speed_sum_last;
		Speed_sum_last=Speed_sum;
	}while(!(Speed_sum_error<550||Speed_sum_error>-550));
	Load(0,0,0,0);
	motor_clear_and_init_params();
	motion_steps[step_index].action_exit_flag = 1;//到位置不动了，进入下一步
}
// 任务1:云台，夹爪回到初始位置
void step1_extra_action(uint8_t step_index)
{
	Back_Zero(6);
	rotate_angle(Rotate_Start_Angle);
	claw_angle(Claw_Release_Angle);
}

// 任务2:等待识别到二维码
void step2_extra_action(uint8_t step_index)
{
	if(In_position==0)
	{
		Serial_Vision_SendString("ONE");
		In_position=1;
	}
	if(In_position==1)
	{
		if((RXD_I[0]!=0&&RXD_I[1]!=0)&&(RXD_I[0]==RXD_I[1]))
		{
			taskENTER_CRITICAL(); // 进入临界区
			QR_CODE=RXD_I[0];     //将二维码值保存下来
			splitHundred(QR_CODE,QR_code_data);//将数据存到数组中
			motion_steps[step_index].action_exit_flag = 1;
			In_position=0;
			taskEXIT_CRITICAL();
		}
	}
}
// 任务3:机械臂向左转
void step3_extra_action(uint8_t step_index)
{
	vTaskDelay(2000);
	rotate_angle_slow(Rotate_Start_Angle,Rotate_Catch_Angle);
	vTaskDelay(1);
	Control_robot_arm_Y(1,3500,100,50);
	vTaskDelay(10);
			do{
					Readclk_robot_arm_1();
					vTaskDelay(1);
				}while(!((wheel_revolutions[0]-100)<=(Motor_Cur_Pos_robot_arm_1)
					&&(Motor_Cur_Pos_robot_arm_1)<=(wheel_revolutions[0]+100)));
}

//任务4: 发送识别球信号
void step4_extra_action(uint8_t step_index)
{
	RXD_I[0]=0;
	RXD_I[1]=0;
	vTaskDelay(3000);
	Serial_Vision_SendString("TWO");
}

uint8_t time_flag=0;     //计时标志位
uint32_t  timekeeping=0;
uint32_t  now_time=0;    //距离计时到现在的时间
int32_t Total_CLK=0;     //伸缩变化的累计脉冲
//任务5: 识别第一个位置，是目标则跟踪，完成抓取步骤，不是则继续往下走
void step5_extra_action(uint8_t step_index)
{
	if(In_position==0)
	{
		time_flag=1;   //开始计时
		In_position=1;
	}
	if(In_position==1)
	{
		now_time=timekeeping*5;//单位为ms
		if(RXD_I[0]!=0||RXD_I[1]!=0)//识别到了,开始追踪
		{
			time_flag=0;//将计时标志位请零
			while(!(RXD_I[0] == 1 && RXD_I[1] == 1))//当追踪到位时，退出追踪循环
			{
				while(RXD_I[0]==0&&RXD_I[1]==0)
				{
					vTaskDelay(1);
				}
				motor_fixed_distance_params(RXD_I[0]*KP_BALL_X,500,Fix_Angle);
				while (check_all_reached_filter() != 1)//等待车到位
				{
						vTaskDelay(1);
				}
				motor_clear_and_init_params();//清零
				vTaskDelay(200);
				Total_CLK+=Get_CLK(KP_BALL_Y,RXD_I[1]);
				Correct_Ball(KP_BALL_Y,RXD_I[1]);
				vTaskDelay(10);
				do{
					Readclk_robot_arm_1();
					vTaskDelay(1);
				}while(!((wheel_revolutions[0]-100)<=(Motor_Cur_Pos_robot_arm_1)
					&&(Motor_Cur_Pos_robot_arm_1)<=(wheel_revolutions[0]+100)));
				vTaskDelay(1000);
				if(RXD_I[0]!=1||RXD_I[1]!=1)
				{
					RXD_I[0]=0;
					RXD_I[1]=0;
					Serial_Vision_SendString("TWO");
				}
			}
			vTaskDelay(1);
			explosive_num=1;	//记录车在第一个位置
			Control_robot_arm_Z(0,70000,1600,200);
			vTaskDelay(10);
			do{
					Readclk_robot_arm();
					vTaskDelay(1);
				}while(!((wheel_revolutions[1]-100)<=(Motor_Cur_Pos_robot_arm)
					&&(Motor_Cur_Pos_robot_arm)<=(wheel_revolutions[1]+100)));
			vTaskDelay(10);
			claw_angle(Claw_Ball_Angle);
			vTaskDelay(Claw_Catch_Delay);
			Control_robot_arm_Z(1,70000,1600,200);
			vTaskDelay(10);
			do{
					Readclk_robot_arm();
					vTaskDelay(1);
				}while(!((wheel_revolutions[1]-100)<=(Motor_Cur_Pos_robot_arm)
					&&(Motor_Cur_Pos_robot_arm)<=(wheel_revolutions[1]+100)));
			vTaskDelay(10);
			Back_Zero(6);
			vTaskDelay(10);
			do{
					Readclk_robot_arm_1();
					vTaskDelay(1);
				}while(!((wheel_revolutions[0]-100)<=(Motor_Cur_Pos_robot_arm_1)
					&&(Motor_Cur_Pos_robot_arm_1)<=(wheel_revolutions[0]+100)));
			vTaskDelay(10);
			rotate_angle_slow(Rotate_Catch_Angle,Rotate_Set_Angle);
			motor_fixed_distance_params((motor_params_distance_2+2500),1000,Fix_Angle);
			while (check_all_reached_filter() != 1)//等待车到位
			{
					vTaskDelay(1);
			}
			motor_clear_and_init_params();//清零
			vTaskDelay(500);
			Control_robot_arm_Y(1,5000,100,50);
			vTaskDelay(10);
			do{
					Readclk_robot_arm_1();
					vTaskDelay(1);
				}while(!((wheel_revolutions[0]-100)<=(Motor_Cur_Pos_robot_arm_1)
					&&(Motor_Cur_Pos_robot_arm_1)<=(wheel_revolutions[0]+100)));
			vTaskDelay(10);
			claw_angle(Claw_Release_Angle);//松开夹爪
			vTaskDelay(1000);
			/*将Y轴，云台和夹爪设为初始角度*/
			Back_Zero(6);
			rotate_angle_slow(Rotate_Set_Angle,Rotate_Start_Angle);
			claw_angle(Claw_Release_Angle);
			vTaskDelay(10);
			do{
					Readclk_robot_arm_1();
					vTaskDelay(1);
				}while(!((wheel_revolutions[0]-100)<=(Motor_Cur_Pos_robot_arm_1)
					&&(Motor_Cur_Pos_robot_arm_1)<=(wheel_revolutions[0]+100)));
			vTaskDelay(10);
			
			In_position=0;
			motion_steps[step_index].action_exit_flag = 1;//完成步骤，进入下一步	
		}
		if(now_time>1500)//超过3秒，直接认为不是，继续走
		{
			motor_fixed_distance_params(70000,1000,Fix_Angle);
			while (check_all_reached_filter() != 1)//等待车到位
			{
					vTaskDelay(1);
			}
			motor_clear_and_init_params();//清零
			vTaskDelay(500);
			Serial_Vision_SendString("TWO");
			RXD_I[0]=0;
			RXD_I[1]=0;
			now_time=0;
			time_flag=0;
			timekeeping=0;
			In_position=0;
			vTaskDelay(1);
			motion_steps[step_index].action_exit_flag = 1;//完成步骤，进入下一步
		}			
	}
}
//任务6: 识别第二个位置，是目标则跟踪，完成抓取步骤
void step6_extra_action(uint8_t step_index)
{
	if(explosive_num==0)//未抓取,进行视觉判断和追踪
	{
			if(In_position==0)
		{
			time_flag=1;   //开始计时
			In_position=1;
		}
		if(In_position==1)
		{
			now_time=timekeeping*5;//单位为ms
			if(RXD_I[0]!=0||RXD_I[1]!=0)//识别到了,开始追踪
			{
				time_flag=0;//将计时标志位请零
				while(!(RXD_I[0] == 1 && RXD_I[1] == 1))//当追踪到位时，退出追踪循环
				{
					while(RXD_I[0]==0&&RXD_I[1]==0)
					{
						vTaskDelay(1);
					}
					motor_fixed_distance_params(RXD_I[0]*KP_BALL_X,500,Fix_Angle);
					while (check_all_reached_filter() != 1)//等待车到位
					{
							vTaskDelay(1);
					}
					motor_clear_and_init_params();//清零
					vTaskDelay(200);
					Total_CLK+=Get_CLK(KP_BALL_Y,RXD_I[1]);
					Correct_Ball(KP_BALL_Y,RXD_I[1]);
					vTaskDelay(10);
					do{
							Readclk_robot_arm_1();
							vTaskDelay(1);
						}while(!((wheel_revolutions[0]-100)<=(Motor_Cur_Pos_robot_arm_1)
							&&(Motor_Cur_Pos_robot_arm_1)<=(wheel_revolutions[0]+100)));
					vTaskDelay(1000);
					if(RXD_I[0]!=1||RXD_I[1]!=1)
					{
						RXD_I[0]=0;
						RXD_I[1]=0;
						Serial_Vision_SendString("TWO");
					}
				}
				vTaskDelay(1);
				explosive_num=2;	//记录车在第2个位置
				Control_robot_arm_Z(0,70000,1600,200);
				vTaskDelay(10);
				do{
						Readclk_robot_arm();
						vTaskDelay(1);
					}while(!((wheel_revolutions[1]-100)<=(Motor_Cur_Pos_robot_arm)
						&&(Motor_Cur_Pos_robot_arm)<=(wheel_revolutions[1]+100)));
				vTaskDelay(10);
				claw_angle(Claw_Ball_Angle);
				vTaskDelay(Claw_Catch_Delay);
				Control_robot_arm_Z(1,70000,1600,200);
				vTaskDelay(10);
				do{
						Readclk_robot_arm();
						vTaskDelay(1);
					}while(!((wheel_revolutions[1]-100)<=(Motor_Cur_Pos_robot_arm)
						&&(Motor_Cur_Pos_robot_arm)<=(wheel_revolutions[1]+100)));
				vTaskDelay(10);
				Back_Zero(6);
				vTaskDelay(10);
				do{
						Readclk_robot_arm_1();
						vTaskDelay(1);
					}while(!((wheel_revolutions[0]-100)<=(Motor_Cur_Pos_robot_arm_1)
						&&(Motor_Cur_Pos_robot_arm_1)<=(wheel_revolutions[0]+100)));
				vTaskDelay(10);
				rotate_angle_slow(Rotate_Catch_Angle,Rotate_Set_Angle);
				vTaskDelay(10);	
				Control_robot_arm_Y(1,5000,100,50);
				vTaskDelay(10);
				do{
						Readclk_robot_arm_1();
						vTaskDelay(1);
					}while(!((wheel_revolutions[0]-100)<=(Motor_Cur_Pos_robot_arm_1)
						&&(Motor_Cur_Pos_robot_arm_1)<=(wheel_revolutions[0]+100)));
				vTaskDelay(10);
				claw_angle(Claw_Release_Angle);//松开夹爪
				vTaskDelay(1000);
				/*将Y轴，云台和夹爪设为初始角度*/
				Back_Zero(6);
				rotate_angle_slow(Rotate_Set_Angle,Rotate_Start_Angle);	
				claw_angle(Claw_Release_Angle);
				vTaskDelay(10);
				do{
						Readclk_robot_arm_1();
						vTaskDelay(1);
					}while(!((wheel_revolutions[0]-100)<=(Motor_Cur_Pos_robot_arm_1)
						&&(Motor_Cur_Pos_robot_arm_1)<=(wheel_revolutions[0]+100)));
				vTaskDelay(10);
				
				In_position=0;
				motion_steps[step_index].action_exit_flag = 1;//完成步骤，进入下一步	
			}
			if(now_time>1500)//超过3秒，直接认为不是，继续走
			{
				motor_fixed_distance_params(70000,1000,Fix_Angle);
				while (check_all_reached_filter() != 1)//等待车到位
				{
						vTaskDelay(1);
				}
				motor_clear_and_init_params();//清零
				vTaskDelay(500);
				Serial_Vision_SendString("TWO");
				RXD_I[0]=0;
				RXD_I[1]=0;
				now_time=0;
				time_flag=0;
				timekeeping=0;
				In_position=0;
				vTaskDelay(1);
				motion_steps[step_index].action_exit_flag = 1;//完成步骤，进入下一步
			}			
		}
	}
	else
	{
		motion_steps[step_index].action_exit_flag = 1;//如果抓取过了，进入下一步
	}
}

//任务7: 识别第三个位置，是目标则跟踪，完成抓取步骤
void step7_extra_action(uint8_t step_index)
{
	if(explosive_num==0)//未抓取,进行视觉判断和追踪
	{
		if(RXD_I[0]!=0||RXD_I[1]!=0)//识别到了,开始追踪
		{
			time_flag=0;//将计时标志位请零
			while(!(RXD_I[0] == 1 && RXD_I[1] == 1))//当追踪到位时，退出追踪循环
			{
				while(RXD_I[0]==0&&RXD_I[1]==0)
				{
					vTaskDelay(1);
				}
				motor_fixed_distance_params(RXD_I[0]*KP_BALL_X,500,Fix_Angle);
				while (check_all_reached_filter() != 1)//等待车到位
				{
						vTaskDelay(1);
				}
				motor_clear_and_init_params();//清零
				vTaskDelay(200);
				Total_CLK+=Get_CLK(KP_BALL_Y,RXD_I[1]);
				Correct_Ball(KP_BALL_Y,RXD_I[1]);
				vTaskDelay(10);
				do{
						Readclk_robot_arm_1();
						vTaskDelay(1);
					}while(!((wheel_revolutions[0]-100)<=(Motor_Cur_Pos_robot_arm_1)
						&&(Motor_Cur_Pos_robot_arm_1)<=(wheel_revolutions[0]+100)));
				vTaskDelay(1000);
				if(RXD_I[0]!=1||RXD_I[1]!=1)
				{
					RXD_I[0]=0;
					RXD_I[1]=0;
					Serial_Vision_SendString("TWO");
				}
			}
			vTaskDelay(1);
			explosive_num=3;	//记录车在第一个位置
			Control_robot_arm_Z(0,70000,1600,200);
			vTaskDelay(10);
			do{
					Readclk_robot_arm();
					vTaskDelay(1);
				}while(!((wheel_revolutions[1]-100)<=(Motor_Cur_Pos_robot_arm)
					&&(Motor_Cur_Pos_robot_arm)<=(wheel_revolutions[1]+100)));
			vTaskDelay(10);
			claw_angle(Claw_Ball_Angle);
			vTaskDelay(Claw_Catch_Delay);
			Control_robot_arm_Z(1,70000,1600,200);
			vTaskDelay(10);
			do{
					Readclk_robot_arm();
					vTaskDelay(1);
				}while(!((wheel_revolutions[1]-100)<=(Motor_Cur_Pos_robot_arm)
					&&(Motor_Cur_Pos_robot_arm)<=(wheel_revolutions[1]+100)));
			vTaskDelay(10);
			Back_Zero(6);
			vTaskDelay(10);
			do{
					Readclk_robot_arm_1();
					vTaskDelay(1);
				}while(!((wheel_revolutions[0]-100)<=(Motor_Cur_Pos_robot_arm_1)
					&&(Motor_Cur_Pos_robot_arm_1)<=(wheel_revolutions[0]+100)));
			vTaskDelay(10);
			rotate_angle_slow(Rotate_Catch_Angle,Rotate_Set_Angle);
			motor_fixed_distance_params((-motor_params_distance_2),1000,Fix_Angle);
			while (check_all_reached_filter() != 1)//等待车到位
			{
				vTaskDelay(1);
			}
			motor_clear_and_init_params();//清零
			vTaskDelay(500);
			
			Control_robot_arm_Y(1,5000,100,50);
			vTaskDelay(10);
			do{
					Readclk_robot_arm_1();
					vTaskDelay(1);
				}while(!((wheel_revolutions[0]-100)<=(Motor_Cur_Pos_robot_arm_1)
					&&(Motor_Cur_Pos_robot_arm_1)<=(wheel_revolutions[0]+100)));
			vTaskDelay(10);
			claw_angle(Claw_Release_Angle);//松开夹爪
			vTaskDelay(1000);
			/*将Y轴，云台和夹爪设为初始角度*/
			Back_Zero(6);
			rotate_angle_slow(Rotate_Set_Angle,Rotate_Start_Angle);
			claw_angle(Claw_Release_Angle);
			vTaskDelay(10);
			do{
					Readclk_robot_arm_1();
					vTaskDelay(1);
				}while(!((wheel_revolutions[0]-100)<=(Motor_Cur_Pos_robot_arm_1)
					&&(Motor_Cur_Pos_robot_arm_1)<=(wheel_revolutions[0]+100)));
			vTaskDelay(10);
			In_position=0;
			motion_steps[step_index].action_exit_flag = 1;//完成步骤，进入下一步	
		}		
	}
	else
	{
		motion_steps[step_index].action_exit_flag = 1;//如果抓取过了，进入下一步
	}
}

// 任务8: 抓取完后前进,到位后发送识别信号
void step8_extra_action(uint8_t step_index)
{
	motor_fixed_distance_params(840000,1000,Fix_Angle);
	while (check_all_reached_filter() != 1)//等待车到位
	{
		vTaskDelay(1);
	}
	motor_clear_and_init_params();//清零
	vTaskDelay(1000);
	RXD_I[0]=0;
	RXD_I[1]=0;
	Serial_Vision_SendString("THREE");
	motion_steps[step_index].action_exit_flag = 1;
}

//void step8.5_extra_action(uint8_t step_index)
//{
//	 if(RXD_I[0]!=0&&RXD_I[1]==3)//获得了目标数据
//	 {
//		if(RXD_I[0]==1){	
//			motor_fixed_distance_params((-motor_params_distance_2),1000,-90);//向左移动调整
//			while (check_all_reached_filter() != 1)//等待车到位
//			{
//				vTaskDelay(1);
//			}
//			motor_clear_and_init_params();//清零
//			vTaskDelay(500);
//		}
//		else if(RXD_I[0]==3){	
//			motor_fixed_distance_params(motor_params_distance_2,1000,-90);//向右移动调整
//			while (check_all_reached_filter() != 1)//等待车到位
//			{
//				vTaskDelay(1);
//			}
//			motor_clear_and_init_params();//清零
//			vTaskDelay(500);
//		}
//		RXD_I[0]=0;
//		RXD_I[1]=0;
//		Serial_Vision_SendString("THREE2"); 
//		motion_steps[step_index].action_exit_flag = 1;
//	 }
//}

uint8_t Now_Target_Pos=0;
// 任务9: 识别跟踪目标靶子
void step9_extra_action(uint8_t step_index)
{
	if(RXD_I[0]!=0||RXD_I[1]!=0)//识别到了,开始追踪
		{
			while(!(RXD_I[0] == 1 && RXD_I[1] == 1))//当追踪到位时，退出追踪循环
			{
				while(RXD_I[0]==0)
				{
					vTaskDelay(1);
				}
				motor_fixed_distance_params(RXD_I[0]*KP_TARGET_X,500,Fix_Angle);
				while (check_all_reached_filter() != 1)//等待车到位
				{
						vTaskDelay(1);
				}
				bool_luchen_san=bool_luchen_san+Speed_sum;
				motor_clear_and_init_params();//清零
				vTaskDelay(1000);
				if(RXD_I[0]!=1)
				{
					RXD_I[0]=0;
					Serial_Vision_SendString("THREE");
				}
			}
			if(bool_luchen_san < TARGET_POS_1_BOUND)
			{
				Now_Target_Pos = 1;
			}
			else if(bool_luchen_san > TARGET_POS_3_BOUND)
			{
				Now_Target_Pos = 3;
			}
			else
			{
				Now_Target_Pos = 2;
			}
			Servo_SetAngle(3,Pitch_Aim_Angle);
			vTaskDelay(500);
			LED_TURN();
			vTaskDelay(1000);
			LED_TURN();
			motion_steps[step_index].action_exit_flag = 1; //完成步骤，进入下一步	
		}			
}
// 任务10: 打靶完后判断前进多少
void step10_extra_action(uint8_t step_index)
{
	if(Now_Target_Pos==1)
	{
		Servo_SetAngle(3,Pitch_Start_Angle);
		motor_fixed_distance_params(After_Aim_distance_1,1000,Fix_Angle);//多了减5000
		while (check_all_reached_filter() != 1)//等待车到位
		{
			vTaskDelay(1);
		}
		motor_clear_and_init_params();//清零
		vTaskDelay(500);
		motion_steps[step_index].action_exit_flag = 1;
	}
	else if(Now_Target_Pos==2)
	{
		Servo_SetAngle(3,Pitch_Start_Angle);
		motor_fixed_distance_params(After_Aim_distance_2,1000,Fix_Angle);
		while (check_all_reached_filter() != 1)//等待车到位
		{
			vTaskDelay(1);
		}
		motor_clear_and_init_params();//清零
		vTaskDelay(500);
		motion_steps[step_index].action_exit_flag = 1;
	}
	else if(Now_Target_Pos==3)
	{
		Servo_SetAngle(3,Pitch_Start_Angle);
		motor_fixed_distance_params(After_Aim_distance_3,1000,Fix_Angle);
		while (check_all_reached_filter() != 1)//等待车到位
		{
			vTaskDelay(1);
		}
		motor_clear_and_init_params();//清零
		vTaskDelay(500);
		motion_steps[step_index].action_exit_flag = 1;
	}
}
// 任务11: 移动到位后发送串口识别人质
void step11_extra_action(uint8_t step_index)
{
	//以下为新增逻辑
	vTaskDelay(500);
	rotate_angle_slow(Rotate_Start_Angle,Rotate_Catch_Angle);
	vTaskDelay(10);
	Control_robot_arm_Y(1,4000,100,50);
	//以上为新增逻辑,是塔吊摄像头,旋转前伸到可以大概看到人质的位置
	motor_fixed_distance_params(300000,1000,Fix_Angle_2);//320000是让底盘上的摄像头对准第一个人质，后续需要修改成顶部摄像头对准第二个人质
	while (check_all_reached_filter() != 1)//等待车到位
	{
		vTaskDelay(1);
	}
	motor_clear_and_init_params();//清零
	vTaskDelay(500);
	RXD_I[0]=0;
	RXD_I[1]=0;
	TYPE_I[0]=0;
	TYPE_I[1]=0;
	Func_Code=0;
	Serial_Vision_SendString("FOUR");
	motion_steps[step_index].action_exit_flag = 1;
}

uint8_t renzhi_flag=0;//用于记录在第个位置夹取了人质
uint8_t renzhi_type=0;//人质类型
// 任务12: 识别第一个位置，先进行精细矫正,然后根据视觉那边回传的数据判断是否是目标人质
void step12_extra_action(uint8_t step_index)
{
	if(Func_Code==4)//功能码为4，是人质类型，赋值人质类型，进行夹取
	{
		renzhi_type=TYPE_I[0];
		if(renzhi_type==1)
		{
			Control_robot_arm_Y(1,500,50,0);
			vTaskDelay(100);
			claw_angle(Claw_YuanZhu_Angle);
		}
		else if(renzhi_type==2)
		{
			claw_angle(Claw_YuanTai_Angle);
		}
		else if(renzhi_type==3)
		{
			claw_angle(Claw_YaoGu_Angle);
		}
		else return;//类型非法，等待下一帧
		vTaskDelay(Claw_Catch_Delay);
		Back_Zero(6);
		vTaskDelay(10);
		do{
			Readclk_robot_arm_1();
			vTaskDelay(1);
		}while(!((wheel_revolutions[0]-100)<=(Motor_Cur_Pos_robot_arm_1)
				&&(Motor_Cur_Pos_robot_arm_1)<=(wheel_revolutions[0]+100)));
		vTaskDelay(10);
		rotate_angle_slow(Rotate_Catch_Angle,Rotate_Start_Angle);
		vTaskDelay(500);
		RXD_I[0]=0;
		RXD_I[1]=0;
		Func_Code=0;
		renzhi_flag=1;//记录在第1个位置夹取了人质
		motion_steps[step_index].action_exit_flag = 1;//完成步骤，进入下一步
	}
	else if(Func_Code==1&&RXD_I[0]==1&&RXD_I[1]==1)//校准完毕，但是不是目标人质，转角度，并前伸到第二个人质的位置，判断第二个人质类型。
	{
		motor_fixed_distance_params(70000,1000,Fix_Angle_2);
			while (check_all_reached_filter() != 1)//等待车到位
			{
				vTaskDelay(1);
			}
			motor_clear_and_init_params();//清零
		vTaskDelay(500);
		//退出循环
		RXD_I[0]=0;
		RXD_I[1]=0;
		Func_Code=0;
		Serial_Vision_SendString("FOUR");//告诉视觉端，已经到了第二个人质的位置，开始识别判断。
		motion_steps[step_index].action_exit_flag = 1;//完成步骤，进入下一步
	}
	else if(Func_Code==1&&(RXD_I[0]!=0||RXD_I[1]!=0))//识别到了,开始追踪
	{
		Correct_Ball(KP_TOPCAP_Y,RXD_I[1]);
		vTaskDelay(10);
		do{
			Readclk_robot_arm_1();
			vTaskDelay(1);
		}while(!((wheel_revolutions[0]-100)<=(Motor_Cur_Pos_robot_arm_1)
				&&(Motor_Cur_Pos_robot_arm_1)<=(wheel_revolutions[0]+100)));
		vTaskDelay(10);
		motor_fixed_distance_params(RXD_I[0]*KP_TOPCAP_X,500,Fix_Angle_2);
		while (check_all_reached_filter() != 1)//等待车到位
		{
			vTaskDelay(1);
		}
		motor_clear_and_init_params();//清零
		vTaskDelay(1000);
		if(RXD_I[0]!=1||RXD_I[1]!=1)
		{
			RXD_I[0]=0;
			RXD_I[1]=0;
			Func_Code=0;
			Serial_Vision_SendString("FOUR");
		}
	}				
}
//任务13: 识别第二个位置，是目标人质则抓取,不是则直接走到第三个位置校准夹取
void step13_extra_action(uint8_t step_index)
{
	if(renzhi_flag==0)//未抓取,进行视觉判断和追踪
	{
		if(Func_Code==4)//功能码为4，是人质类型，赋值人质类型，进行夹取
		{
			renzhi_type=TYPE_I[0];
			if(renzhi_type==1)
			{
				Control_robot_arm_Y(1,500,50,0);
				vTaskDelay(100);
				claw_angle(Claw_YuanZhu_Angle);
			}
			else if(renzhi_type==2)
			{
				claw_angle(Claw_YuanTai_Angle);
			}
			else if(renzhi_type==3)
			{
				claw_angle(Claw_YaoGu_Angle);
			}
			else return;//类型非法，等待下一帧
			vTaskDelay(Claw_Catch_Delay);
			Back_Zero(6);
			vTaskDelay(10);
			do{
				Readclk_robot_arm_1();
				vTaskDelay(1);
			}while(!((wheel_revolutions[0]-100)<=(Motor_Cur_Pos_robot_arm_1)
					&&(Motor_Cur_Pos_robot_arm_1)<=(wheel_revolutions[0]+100)));
			vTaskDelay(10);
			rotate_angle_slow(Rotate_Catch_Angle,Rotate_Start_Angle);
			vTaskDelay(500);
			RXD_I[0]=0;
			RXD_I[1]=0;
			Func_Code=0;
			renzhi_flag=2;//记录在第1个位置夹取了人质
			motion_steps[step_index].action_exit_flag = 1;//完成步骤，进入下一步
		}
		else if(Func_Code==1&&RXD_I[0]==1&&RXD_I[1]==1)//不是目标人质
		{
			motor_fixed_distance_params(70000,1000,Fix_Angle_2);
			while (check_all_reached_filter() != 1)//等待车到位
			{
				vTaskDelay(1);
			}
			motor_clear_and_init_params();//清零
			vTaskDelay(500);
			//退出循环
			RXD_I[0]=0;
			RXD_I[1]=0;
			Func_Code=0;
			Serial_Vision_SendString("FOUR");//告诉视觉端，已经到了第二个人质的位置，开始识别判断。
			motion_steps[step_index].action_exit_flag = 1;//完成步骤，进入下一步
		}
		else if(Func_Code==1&&(RXD_I[0]!=0||RXD_I[1]!=0))//识别到了,开始追踪
		{
			Correct_Ball(KP_TOPCAP_Y,RXD_I[1]);
			vTaskDelay(10);
			do{
				Readclk_robot_arm_1();
				vTaskDelay(1);
			}while(!((wheel_revolutions[0]-100)<=(Motor_Cur_Pos_robot_arm_1)
					&&(Motor_Cur_Pos_robot_arm_1)<=(wheel_revolutions[0]+100)));
			vTaskDelay(10);
			motor_fixed_distance_params(RXD_I[0]*KP_TOPCAP_X,500,Fix_Angle_2);
			while (check_all_reached_filter() != 1)//等待车到位
			{
				vTaskDelay(1);
			}
			motor_clear_and_init_params();//清零
			vTaskDelay(1000);
			if(RXD_I[0]!=1||RXD_I[1]!=1)
			{
				RXD_I[0]=0;
				RXD_I[1]=0;
				Func_Code=0;
				Serial_Vision_SendString("FOUR");
			}
		}		
	}
	else
	{
		motion_steps[step_index].action_exit_flag = 1;//如果抓取过了，进入下一步
	}
}
//任务14: 识别第三个位置，是目标人质则跟踪，完成校准步骤
void step14_extra_action(uint8_t step_index)
{
	if(renzhi_flag==0)//未抓取,进行视觉判断和追踪
	{
		if(Func_Code==4)//功能码为4，是人质类型，赋值人质类型，进行夹取
		{
			renzhi_type=TYPE_I[0];
			if(renzhi_type==1)
			{
				Control_robot_arm_Y(1,500,50,0);
				vTaskDelay(100);
				claw_angle(Claw_YuanZhu_Angle);
			}
			else if(renzhi_type==2)
			{
				claw_angle(Claw_YuanTai_Angle);	
			}
			else if(renzhi_type==3)
			{
				claw_angle(Claw_YaoGu_Angle);
			}
			else return;//类型非法，等待下一帧
			vTaskDelay(Claw_Catch_Delay);
			Back_Zero(6);
			vTaskDelay(10);
			do{
				Readclk_robot_arm_1();
				vTaskDelay(1);
			}while(!((wheel_revolutions[0]-100)<=(Motor_Cur_Pos_robot_arm_1)
					&&(Motor_Cur_Pos_robot_arm_1)<=(wheel_revolutions[0]+100)));
			vTaskDelay(10);
			rotate_angle_slow(Rotate_Catch_Angle,Rotate_Start_Angle);
			vTaskDelay(500);
			RXD_I[0]=0;
			RXD_I[1]=0;
			Func_Code=0;
			renzhi_flag=3;//记录在第3个位置夹取了人质
			motion_steps[step_index].action_exit_flag = 1;//完成步骤，进入下一步
		}
		if(Func_Code==1&&(RXD_I[0]!=0||RXD_I[1]!=0))//识别到了,开始追踪
		{
			Correct_Ball(KP_TOPCAP_Y,RXD_I[1]);
			vTaskDelay(10);
			do{
				Readclk_robot_arm_1();
				vTaskDelay(1);
			}while(!((wheel_revolutions[0]-100)<=(Motor_Cur_Pos_robot_arm_1)
					&&(Motor_Cur_Pos_robot_arm_1)<=(wheel_revolutions[0]+100)));
			vTaskDelay(10);
			motor_fixed_distance_params(RXD_I[0]*KP_TOPCAP_X,500,Fix_Angle_2);
			while (check_all_reached_filter() != 1)//等待车到位
			{
				vTaskDelay(1);
			}
			motor_clear_and_init_params();//清零
			vTaskDelay(1000);
			if(RXD_I[0]!=1||RXD_I[1]!=1)
			{
				RXD_I[0]=0;
				RXD_I[1]=0;
				Func_Code=0;
				Serial_Vision_SendString("FOUR");
			}
		}			
	}
	else
	{
		motion_steps[step_index].action_exit_flag = 1;//如果抓取过了，进入下一步
	}
}
//任务15和16后续可直接跳过。
//任务15: 将云台旋转到夹取位置，发送串口信号
void step15_extra_action(uint8_t step_index)
{
	vTaskDelay(500);
	rotate_angle_slow(Rotate_Start_Angle,Rotate_Catch_Angle);
	vTaskDelay(10);
	Control_robot_arm_Y(1,3500,100,50);
	vTaskDelay(10);
			do{
					Readclk_robot_arm_1();
					vTaskDelay(1);
				}while(!((wheel_revolutions[0]-100)<=(Motor_Cur_Pos_robot_arm_1)
					&&(Motor_Cur_Pos_robot_arm_1)<=(wheel_revolutions[0]+100)));
	vTaskDelay(1000);
	RXD_I[0]=0;
	RXD_I[1]=0;			
	Serial_Vision_SendString("FOUR2");				
}
// 任务16: 进行校准夹取人质
void step16_extra_action(uint8_t step_index)
{
	if(RXD_I[0]!=0||RXD_I[1]!=0)//识别到了,开始追踪
		{
			while(!(RXD_I[0] == 1 && RXD_I[1] == 1))//当追踪到位时，退出追踪循环
			{
				while(RXD_I[0]==0&&RXD_I[1]==0)
				{
					vTaskDelay(1);
				}
				Correct_Ball(KP_TOPCAP_Y,RXD_I[1]);
				vTaskDelay(10);
				do{
						Readclk_robot_arm_1();
						vTaskDelay(1);
					}while(!((wheel_revolutions[0]-100)<=(Motor_Cur_Pos_robot_arm_1)
						&&(Motor_Cur_Pos_robot_arm_1)<=(wheel_revolutions[0]+100)));
				vTaskDelay(10);
				motor_fixed_distance_params(RXD_I[0]*KP_TOPCAP_X,500,180);
				while (check_all_reached_filter() != 1)//等待车到位
				{
						vTaskDelay(1);
				}
				motor_clear_and_init_params();//清零
				vTaskDelay(1000);
				if(RXD_I[0]!=1||RXD_I[1]!=1)
				{
					RXD_I[0]=0;
					RXD_I[1]=0;
					Serial_Vision_SendString("FOUR2");
				}
			}
			vTaskDelay(10);
			if(renzhi_type==1)//圆柱
			{
				claw_angle(Claw_YuanZhu_Angle);
			}
			else if(renzhi_type==2)//圆台
			{
				claw_angle(Claw_YuanTai_Angle);
			}
			else if(renzhi_type==3)//腰鼓
			{
				claw_angle(Claw_YaoGu_Angle);
			}
			vTaskDelay(2500);
			Back_Zero(6);
			vTaskDelay(10);
			do{
					Readclk_robot_arm_1();
					vTaskDelay(1);
				}while(!((wheel_revolutions[0]-100)<=(Motor_Cur_Pos_robot_arm_1)
					&&(Motor_Cur_Pos_robot_arm_1)<=(wheel_revolutions[0]+100)));
			vTaskDelay(10);
			rotate_angle_slow(Rotate_Catch_Angle,Rotate_Start_Angle);
			vTaskDelay(500);
			motion_steps[step_index].action_exit_flag = 1;//完成步骤，进入下一步	
		}
}
//任务15,16后续可直接跳过。

void step17_extra_action(uint8_t step_index)//判断前进多少
{
	if(renzhi_flag==1)
	{
		motor_fixed_distance_params(830000,1000,Fix_Angle_2);
		while (check_all_reached_filter() != 1)//等待车到位
		{
			vTaskDelay(1);
		}
		motor_clear_and_init_params();//清零
	}
	else if(renzhi_flag==2)
	{
		motor_fixed_distance_params(760000,1000,Fix_Angle_2);
		while (check_all_reached_filter() != 1)//等待车到位
		{
			vTaskDelay(1);
		}
		motor_clear_and_init_params();//清零	
	}
	else if(renzhi_flag==3)
	{
		motor_fixed_distance_params(690000,1000,Fix_Angle_2);
		while (check_all_reached_filter() != 1)//等待车到位
		{
			vTaskDelay(1);
		}
		motor_clear_and_init_params();//清零
	}
}

//通信测试
void steptx_extra_action(uint8_t step_index)
{
	RXD_I[0]=0;
	RXD_I[1]=0;
	vTaskDelay(500);
	Serial_Vision_SendString("FOUR");
}
//测试任务
void steptest_extra_action(uint8_t step_index)
{
		if(RXD_I[0]!=0||RXD_I[1]!=0)//识别到了,开始追踪
		{
			while(!(RXD_I[0] == 1 && RXD_I[1] == 1))//当追踪到位时，退出追踪循环
			{
				while(RXD_I[0]==0&&RXD_I[1]==0)
				{
					vTaskDelay(1);
				}
				motor_fixed_distance_params(RXD_I[0]*KP_RENZHI_X,500,0);
				while (check_all_reached_filter() != 1)//等待车到位
				{
						vTaskDelay(1);
				}
				motor_clear_and_init_params();//清零
				vTaskDelay(1000);
				if(RXD_I[0]!=1)
				{
					RXD_I[0]=0;
					RXD_I[1]=0;
					Serial_Vision_SendString("FOUR");
				}
			}
			claw_angle(120);
			motion_steps[step_index].action_exit_flag = 1;//完成步骤，进入下一步	
		}			
}

// 5. 定义步骤数组（按实际运动流程初始化，顺序就是执行顺序）
Motor_control_MotorStep motion_steps[] = {
       /*步骤 0 ：沿0度前进，无额外操作*/
   {	
       .type = STEP_FIXED_DIST,								//整体走直线，固定距离，固定速度
       .params.fixed_dist = {199500, 1000, 0},	// 目标距离，目标速度，目标角度
       .need_wait = 1,													// 1=需要等待完成，0=不需要
       .action = NULL, 												// 无额外业务
				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
   },
		/*步骤 1 ：向左差速走，无额外操作*/
   {		
       .type = STEP_SINGLE_SPEED,															//4轮分别速度控制，运行到目标距离
       .params.single_speed = {1000,1000,100,100, 152500},	// 左前速度，左后速度，右前速度，右后速度，目标距离
       .need_wait = 1,																					// 1=需要等待完成，0=不需要
       .action = NULL, 																				// 无额外业务
				.skip_motor = 0,   	 																		// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = 0   																		// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
   },
		/*步骤 2 ：原地转到-90度，无额外操作*/
	{
       .type = STEP_DIFF_TURN,									//差速转弯，到目标角度
       .params.diff_turn = {1000,-1000,-90},	// 左电机速度，右电机速度，目标角度
       .need_wait = 1,													// 1=需要等待完成，0=不需要
       .action = NULL, 												// 无额外业务
				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
   },
		/*步骤 4 ：不动，等待识别到二维码*/
   {
       .type = STEP_NONE,								//整体走直线，固定距离，固定速度
       .params.fixed_dist =	NULL,	// 目标距离，目标速度，目标角度
       .need_wait = 0,													// 1=需要等待完成，0=不需要
       .action = step2_extra_action, 					//任务2:等待识别到二维码 
				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = -1   									// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
   },
	{
		.type = STEP_FIXED_DIST,								//整体走直线，固定距离，固定速度
       .params.fixed_dist = {-305000,1000,-90},	// 目标距离，目标速度，目标角度(改318000)
       .need_wait = 1,													// 1=需要等待完成，0=不需要
       .action = NULL, 												// 无额外业务
				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）	
	},
		/*步骤 6 ：向右前方差速走，无额外操作*/
   {
      .type = STEP_SINGLE_SPEED,															//4轮分别速度控制，运行到目标距离
       .params.single_speed = {100,100,1000,1000,-155000},	// 左前速度，左后速度，右前速度，右后速度，目标距离
       .need_wait = 1,																					// 1=需要等待完成，0=不需要
       .action = NULL, 																				// 无额外业务
				.skip_motor = 0,   	 																		// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = 0   																		// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
   },
		/*步骤 7 ：原地旋转到0度，无额外操作*/
   {
		.type = STEP_DIFF_TURN,									//差速转弯，到目标角度
       .params.diff_turn = {1000,-1000,180},	// 左电机速度，右电机速度，目标角度
       .need_wait = 1,													// 1=需要等待完成，0=不需要
       .action = NULL, 												// 无额外业务
				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
   },
		/*步骤 8 ：沿0度前进，无额外操作,过减速带，任务：过完减速带后后退矫正*/
   {
		.type = STEP_FIXED_DIST,								//整体走直线，固定距离，固定速度
       .params.fixed_dist = {-290000, 500, 180},	// 目标距离，目标速度，目标角度
       .need_wait = 1,													// 1=需要等待完成，0=不需要
       .action = step0_extra_action, 					// 过完减速带后后退矫正
				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = -1   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）
   },
	{
		 .type = STEP_FIXED_DIST,								//整体走直线，固定距离，固定速度
       .params.fixed_dist = {-5000, 1000, 180},	// 目标距离，目标速度，目标角度
       .need_wait = 1,													// 1=需要等待完成，0=不需要
       .action = NULL, 												// 无额外业务
				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）
	},
		/*步骤 9 ：向右差速走，无额外操作*/
   {
		.type = STEP_SINGLE_SPEED,															//4轮分别速度控制，运行到目标距离
       .params.single_speed = {100,100,1000,1000,-155000},	// 左前速度，左后速度，右前速度，右后速度，目标距离
       .need_wait = 1,																					// 1=需要等待完成，0=不需要
       .action = NULL, 																				// 无额外业务
				.skip_motor = 0,   	 																		// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = 0   																		// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）	
   },
		/*步骤 10 ：原地旋转到90，无额外操作*/
   {
		.type = STEP_DIFF_TURN,									//差速转弯，到目标角度
       .params.diff_turn = {-1000, 1000, 90},		// 左电机速度，右电机速度，目标角度
       .need_wait = 1,													// 1=需要等待完成，0=不需要
       .action = NULL, 												// 无额外业务
				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
   },
		/*步骤 11 ：沿90度直走，无额外操作*/
   {
       .type = STEP_FIXED_DIST,								//整体走直线，固定距离，固定速度
       .params.fixed_dist = {407500, 1000, 90},	// 目标距离，目标速度，目标角度
       .need_wait = 1,													// 1=需要等待完成，0=不需要
       .action = NULL, 												// 无额外业务
				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）	
   },
		/*步骤 12 ：向左差速走，无额外操作*/
   {
		.type = STEP_SINGLE_SPEED,															//4轮分别速度控制，运行到目标距离
       .params.single_speed = {1000, 1000, 100, 100, 152500},	// 左前速度，左后速度，右前速度，右后速度，目标距离
       .need_wait = 1,																					// 1=需要等待完成，0=不需要
       .action = NULL, 																				// 无额外业务
				.skip_motor = 0,   	 																		// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = 0   																		// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）			
   },	
		/*步骤 13 ：原地旋转到0度，无额外操作*/
   {
        .type = STEP_DIFF_TURN,									//差速转弯，到目标角度
       .params.diff_turn = {1000, -1000, 0},	// 左电机速度，右电机速度，目标角度
       .need_wait = 1,													// 1=需要等待完成，0=不需要
       .action = NULL, 												// 无额外业务
				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
   },
		/*步骤 14 ：沿0度直走，无额外操作*/
   {
		.type = STEP_FIXED_DIST,								//整体走直线，固定距离，固定速度
       .params.fixed_dist = {272500, 1000, 0},	// 目标距离，目标速度，目标角度
       .need_wait = 1,													// 1=需要等待完成，0=不需要
       .action = NULL, 												// 无额外业务
				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
   },
		/*步骤 15 ：向左差速走，无额外操作*/
   {
		.type = STEP_SINGLE_SPEED,															//4轮分别速度控制，运行到目标距离
       .params.single_speed = {1000, 1000, 100, 100, 152500},	// 左前速度，左后速度，右前速度，右后速度，目标距离
       .need_wait = 1,																					// 1=需要等待完成，0=不需要
       .action = NULL, 																				// 无额外业务
				.skip_motor = 0,   	 																		// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = 0   																		// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）	
   },
		/*步骤 16 ：原地旋转到-90，云台旋转到识别球的位置*/
   {
       .type = STEP_DIFF_TURN,									//差速转弯，到目标角度
       .params.diff_turn = {1000, -1000, Fix_Angle},	// 左电机速度，右电机速度，目标角度
       .need_wait = 1,													// 1=需要等待完成，0=不需要
       .action = step3_extra_action, 							// 舵机云台和夹爪转到识别位置
				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
   },
		/*步骤 17 ：沿-90度直走，发送串口识别球*/
   {
		.type = STEP_FIXED_DIST,								//整体走直线，固定距离，固定速度
       .params.fixed_dist = {170000, 1000, Fix_Angle},	// 目标距离，目标速度，目标角度
       .need_wait = 1,													// 1=需要等待完成，0=不需要
       .action = step4_extra_action, 					
				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）	
   },
		/*步骤 18 ：判断是否识别到，任务5：识别第一个位置，到就跟踪，完成抓取，没识别到继续走*/
   {
       .type = STEP_NONE,								//整体走直线，固定距离，固定速度
       .params.fixed_dist = NULL,	// 目标距离，目标速度，目标角度
       .need_wait = 0,													// 1=需要等待完成，0=不需要
       .action = step5_extra_action, 					// 
				.skip_motor = 1,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = -1   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
   },
		/*步骤 19 ：判断是否识别到，任务6：识别第二个位置，到就跟踪，完成抓取，没识别到继续走*/
   {
      .type = STEP_NONE,								//整体走直线，固定距离，固定速度
       .params.fixed_dist = NULL,	// 目标距离，目标速度，目标角度
       .need_wait = 0,													// 1=需要等待完成，0=不需要
       .action = step6_extra_action, 					// 
				.skip_motor = 1,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = -1   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）			
   },
		/*步骤 20 ：判断是否识别到，任务7：识别第三个位置，到就跟踪，完成抓取，没识别到继续走*/
   {
		.type = STEP_NONE,								//整体走直线，固定距离，固定速度
       .params.fixed_dist = NULL,	// 目标距离，目标速度，目标角度
       .need_wait = 0,													// 1=需要等待完成，0=不需要
       .action = step7_extra_action, 					// 
				.skip_motor = 1,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = -1   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）	
   },
		/*步骤 21 ：判断是否识别到，任务8：到达标靶识别位置,串口发送识别信号*/
	{
		.type = STEP_NONE,								//整体走直线，固定距离，固定速度
       .params.fixed_dist = NULL,	// 目标距离，目标速度，目标角度
       .need_wait = 0,													// 1=需要等待完成，0=不需要
       .action = step8_extra_action, 					// 
				.skip_motor = 1,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = -1   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）	
   },
		/*步骤 22 ：判断是否识别到，任务6：追踪射靶子*/
	{
		.type = STEP_NONE,								//整体走直线，固定距离，固定速度
       .params.fixed_dist =	NULL,	// 目标距离，目标速度，目标角度
       .need_wait = 0,													// 1=需要等待完成，0=不需要
       .action = step9_extra_action, 					//任务2:等待识别到二维码 
				.skip_motor = 1,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = -1   									// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）	
//        .type = STEP_SINGLE_SPEED,															//4轮分别速度控制，运行到目标距离
//        .params.single_speed = {1400, 1400, 200, 200, 175000},	// 左前速度，左后速度，右前速度，右后速度，目标距离
//        .need_wait = 1,																					// 1=需要等待完成，0=不需要
//        .action = NULL, 																				// 无额外业务
//				.skip_motor = 0,   	 																		// 是否跳过电机动作（1=跳过，只执行业务）
//				.action_repeat = 0   																		// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
   },
		/*步骤 23 */
	{
		.type = STEP_NONE,								//整体走直线，固定距离，固定速度
       .params.fixed_dist = NULL,	// 目标距离，目标速度，目标角度
       .need_wait = 0,													// 1=需要等待完成，0=不需要
       .action = step10_extra_action, 					// 
				.skip_motor = 1,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = -1   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
   },	
		/*步骤 24 ：判断是否识别到，任务8：识别第一个位置，到就跟踪，完成击靶，没识别到继续走*/
   {
		.type = STEP_SINGLE_SPEED,															//4轮分别速度控制，运行到目标距离
       .params.single_speed = {1000, 1000, 100, 100, 152500},	// 左前速度，左后速度，右前速度，右后速度，目标距离
       .need_wait = 1,																					// 1=需要等待完成，0=不需要
       .action = NULL, 																				// 无额外业务
				.skip_motor = 0,   	 																		// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = 0   																		// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
   },
//		
		/*步骤 25 ：判断是否识别到，任务9：识别第二个位置，到就跟踪，完成击靶，没识别到继续走*/
	{
	
		.type = STEP_DIFF_TURN,									//差速转弯，到目标角度
       .params.diff_turn = {1000, -1000, Fix_Angle_2},	// 左电机速度，右电机速度，目标角度
       .need_wait = 1,													// 1=需要等待完成，0=不需要
       .action = NULL, 												// 无额外业务
				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）
   },
		/*步骤 26 ：判断是否识别到，任务10：识别第三个位置，到就跟踪，完成击靶*/
	{
       .type = STEP_NONE,								//整体走直线，固定距离，固定速度
       .params.fixed_dist = NULL,	// 目标距离，目标速度，目标角度
       .need_wait = 0,													// 1=需要等待完成，0=不需要
       .action = step11_extra_action, 					// 
				.skip_motor = 1,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = -1   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）			
   },
		/*步骤 27 ：判断是否识别到，任务11：根据射靶子的位置，判断走多少距离*/
	{
        .type = STEP_NONE,								//整体走直线，固定距离，固定速度
       .params.fixed_dist = NULL,	// 目标距离，目标速度，目标角度
       .need_wait = 0,													// 1=需要等待完成，0=不需要
       .action = step12_extra_action, 					// 
				.skip_motor = 1,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = -1   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）			
   },
		/*步骤 28 ：向右差速走，无额外操作*/
   {
        .type = STEP_NONE,								//整体走直线，固定距离，固定速度
       .params.fixed_dist = NULL,	// 目标距离，目标速度，目标角度
       .need_wait = 0,													// 1=需要等待完成，0=不需要
       .action = step13_extra_action, 					// 
				.skip_motor = 1,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = -1   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）			
   },
		/*步骤 29 ：原地旋转到180度，无额外操作*/
   {
        .type = STEP_NONE,								//整体走直线，固定距离，固定速度
       .params.fixed_dist = NULL,	// 目标距离，目标速度，目标角度
       .need_wait = 0,													// 1=需要等待完成，0=不需要
       .action = step14_extra_action, 					// 
				.skip_motor = 1,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = -1   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）			
   },
		/*步骤 32 ：判断是否识别到，任务5：识别第二个位置，到就跟踪，完成抓取，没识别到继续走*/
	{
       .type = STEP_NONE,								//整体走直线，固定距离，固定速度
       .params.fixed_dist = NULL,	// 目标距离，目标速度，目标角度
       .need_wait = 0,													// 1=需要等待完成，0=不需要
       .action = step17_extra_action, 					// 
				.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
				.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
   },
	//     /*步骤 0 ：沿0度前进，无额外操作*/
    // {	
    //     .type = STEP_FIXED_DIST,								//整体走直线，固定距离，固定速度
    //     .params.fixed_dist = {199500, 1000, 0},	// 目标距离，目标速度，目标角度
    //     .need_wait = 1,													// 1=需要等待完成，0=不需要
    //     .action = NULL, 												// 无额外业务
	// 			.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
	// 			.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
    // },
	// 	/*步骤 1 ：向左差速走，无额外操作*/
    // {		
    //     .type = STEP_SINGLE_SPEED,															//4轮分别速度控制，运行到目标距离
    //     .params.single_speed = {1000,1000,100,100, 152500},	// 左前速度，左后速度，右前速度，右后速度，目标距离
    //     .need_wait = 1,																					// 1=需要等待完成，0=不需要
    //     .action = NULL, 																				// 无额外业务
	// 			.skip_motor = 0,   	 																		// 是否跳过电机动作（1=跳过，只执行业务）
	// 			.action_repeat = 0   																		// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
    // },
	// 	/*步骤 2 ：原地转到-90度，无额外操作*/
	// {
    //     .type = STEP_DIFF_TURN,									//差速转弯，到目标角度
    //     .params.diff_turn = {1000,-1000,-90},	// 左电机速度，右电机速度，目标角度
    //     .need_wait = 1,													// 1=需要等待完成，0=不需要
    //     .action = NULL, 												// 无额外业务
	// 			.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
	// 			.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
    // },
	// 	/*步骤 4 ：不动，等待识别到二维码*/
    // {
    //     .type = STEP_NONE,								//整体走直线，固定距离，固定速度
    //     .params.fixed_dist =	NULL,	// 目标距离，目标速度，目标角度
    //     .need_wait = 1,													// 1=需要等待完成，0=不需要
    //     .action = NULL, 												// 无额外业务
	// 			.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
	// 			.action_repeat = -1   									// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
    // },
	// {
	// 	.type = STEP_FIXED_DIST,								//整体走直线，固定距离，固定速度
    //     .params.fixed_dist = {-307500,1000,-90},	// 目标距离，目标速度，目标角度(改318000)
    //     .need_wait = 1,													// 1=需要等待完成，0=不需要
    //     .action = NULL, 												// 无额外业务
	// 			.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
	// 			.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）	
	// },
	// 	/*步骤 6 ：向右前方差速走，无额外操作*/
    // {
    //    .type = STEP_SINGLE_SPEED,															//4轮分别速度控制，运行到目标距离
    //     .params.single_speed = {100,100,1000,1000,-155000},	// 左前速度，左后速度，右前速度，右后速度，目标距离
    //     .need_wait = 1,																					// 1=需要等待完成，0=不需要
    //     .action = NULL, 																				// 无额外业务
	// 			.skip_motor = 0,   	 																		// 是否跳过电机动作（1=跳过，只执行业务）
	// 			.action_repeat = 0   																		// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
    // },
	// 	/*步骤 7 ：原地旋转到0度，无额外操作*/
    // {
	// 	.type = STEP_DIFF_TURN,									//差速转弯，到目标角度
    //     .params.diff_turn = {1000,-1000,180},	// 左电机速度，右电机速度，目标角度
    //     .need_wait = 1,													// 1=需要等待完成，0=不需要
    //     .action = NULL, 												// 无额外业务
	// 			.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
	// 			.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
    // },
	// 	/*步骤 8 ：沿0度前进，无额外操作,过减速带，任务：过完减速带后后退矫正*/
    // {
	// 	.type = STEP_FIXED_DIST,								//整体走直线，固定距离，固定速度
    //     .params.fixed_dist = {-290000, 500, 180},	// 目标距离，目标速度，目标角度
    //     .need_wait = 1,													// 1=需要等待完成，0=不需要
    //     .action = step0_extra_action, 					// 过完减速带后后退矫正
	// 			.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
	// 			.action_repeat = -1   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）
    // },
	// {
	// 	 .type = STEP_FIXED_DIST,								//整体走直线，固定距离，固定速度
    //     .params.fixed_dist = {-5000, 1000, 180},	// 目标距离，目标速度，目标角度
    //     .need_wait = 1,													// 1=需要等待完成，0=不需要
    //     .action = NULL, 												// 无额外业务
	// 			.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
	// 			.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）
	// },
	// 	/*步骤 9 ：向右差速走，无额外操作*/
    // {
	// 	.type = STEP_SINGLE_SPEED,															//4轮分别速度控制，运行到目标距离
    //     .params.single_speed = {100,100,1000,1000,-155000},	// 左前速度，左后速度，右前速度，右后速度，目标距离
    //     .need_wait = 1,																					// 1=需要等待完成，0=不需要
    //     .action = NULL, 																				// 无额外业务
	// 			.skip_motor = 0,   	 																		// 是否跳过电机动作（1=跳过，只执行业务）
	// 			.action_repeat = 0   																		// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）	
    // },
	// 	/*步骤 10 ：原地旋转到90，无额外操作*/
    // {
	// 	.type = STEP_DIFF_TURN,									//差速转弯，到目标角度
    //     .params.diff_turn = {-1000, 1000, 90},		// 左电机速度，右电机速度，目标角度
    //     .need_wait = 1,													// 1=需要等待完成，0=不需要
    //     .action = NULL, 												// 无额外业务
	// 			.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
	// 			.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
    // },
	// 	/*步骤 11 ：沿90度直走，无额外操作*/
    // {
    //     .type = STEP_FIXED_DIST,								//整体走直线，固定距离，固定速度
    //     .params.fixed_dist = {395000, 1000, 90},	// 目标距离，目标速度，目标角度
    //     .need_wait = 1,													// 1=需要等待完成，0=不需要
    //     .action = NULL, 												// 无额外业务
	// 			.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
	// 			.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）	
    // },
	// 	/*步骤 12 ：向左差速走，无额外操作*/
    // {
	// 	.type = STEP_SINGLE_SPEED,															//4轮分别速度控制，运行到目标距离
    //     .params.single_speed = {1000, 1000, 100, 100, 152500},	// 左前速度，左后速度，右前速度，右后速度，目标距离
    //     .need_wait = 1,																					// 1=需要等待完成，0=不需要
    //     .action = NULL, 																				// 无额外业务
	// 			.skip_motor = 0,   	 																		// 是否跳过电机动作（1=跳过，只执行业务）
	// 			.action_repeat = 0   																		// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）			
    // },	
	// 	/*步骤 13 ：原地旋转到0度，无额外操作*/
    // {
    //      .type = STEP_DIFF_TURN,									//差速转弯，到目标角度
    //     .params.diff_turn = {1000, -1000, 0},	// 左电机速度，右电机速度，目标角度
    //     .need_wait = 1,													// 1=需要等待完成，0=不需要
    //     .action = NULL, 												// 无额外业务
	// 			.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
	// 			.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
    // },
	// 	/*步骤 14 ：沿0度直走，无额外操作*/
    // {
	// 	.type = STEP_FIXED_DIST,								//整体走直线，固定距离，固定速度
    //     .params.fixed_dist = {267500, 1000, 0},	// 目标距离，目标速度，目标角度
    //     .need_wait = 1,													// 1=需要等待完成，0=不需要
    //     .action = NULL, 												// 无额外业务
	// 			.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
	// 			.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
    // },
	// 	/*步骤 15 ：向左差速走，无额外操作*/
    // {
	// 	.type = STEP_SINGLE_SPEED,															//4轮分别速度控制，运行到目标距离
    //     .params.single_speed = {1000, 1000, 100, 100, 152500},	// 左前速度，左后速度，右前速度，右后速度，目标距离
    //     .need_wait = 1,																					// 1=需要等待完成，0=不需要
    //     .action = NULL, 																				// 无额外业务
	// 			.skip_motor = 0,   	 																		// 是否跳过电机动作（1=跳过，只执行业务）
	// 			.action_repeat = 0   																		// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）	
    // },
	// 	/*步骤 16 ：原地旋转到-90，云台旋转到识别球的位置*/
    // {
    //     .type = STEP_DIFF_TURN,									//差速转弯，到目标角度
    //     .params.diff_turn = {1000, -1000, -90},	// 左电机速度，右电机速度，目标角度
    //     .need_wait = 1,													// 1=需要等待完成，0=不需要
    //     .action = step3_extra_action, 												// 无额外业务
	// 			.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
	// 			.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）		
    // },
	// 	/*步骤 17 ：沿-90度直走，发送串口识别球*/
    // {
	// 	.type = STEP_FIXED_DIST,								//整体走直线，固定距离，固定速度
    //     .params.fixed_dist = {170000, 1000, -90},	// 目标距离，目标速度，目标角度
    //     .need_wait = 1,													// 1=需要等待完成，0=不需要
    //     .action = NULL, 												// 无额外业务
	// 			.skip_motor = 0,   	 										// 是否跳过电机动作（1=跳过，只执行业务）
	// 			.action_repeat = 0   										// 业务代码重复次数（0=执行1次，n>0=执行n次，-1=无限循环直到外部终止）	
    // },
};

// 计算总步骤数（自动推导，避免硬编码）
const uint8_t TOTAL_STEPS = sizeof(motion_steps) / sizeof(Motor_control_MotorStep);





