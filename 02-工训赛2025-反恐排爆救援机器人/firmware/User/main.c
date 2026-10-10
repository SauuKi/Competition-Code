/*

										   _oo0oo_
										  o8888888o
										  88" . "88
										  (| -_- |)
									  	0\  =  /0
								   ____/`---'\____	  
							  . '  \\|     |//  `.
						 	/	  \\|||  :  |||//  \
						  /   _||||| -:- |||||-  \
						  |    | \\\  -  /// |   |
						  |  \_|  ''\---/''  |   |
						  \   .-\__  `-`  ___/-. /
						  _`. .'   /--.--\  '. . __
			  ."" '<   `.____\_<|>_/___.'  >'""
				| |:  `- \ `. ;`\ _ /`;.`/ - ` : | |
				\ \ `-.   \_ ___ \ /__ _/    .-`/  /
		======`-.___`-.___\________/___.-`___.-'======
											`=------='
		 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
				    佛主保佑            永无BUG

 * @Author: wangjieming su-ming 2977972809@qq.com
 * @Date: 2024-11-08
 * @LastEditors: su-ming 2977972809@qq.com
 * @LastEditTime: 2024-11-08
 * @FilePath: \delivering_car\User\main.c
 * @Description: 佛主保佑永无BUG！！！
 */

/*
*************************************************************************
*                             包含的头文件
*************************************************************************
*/
/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "queue.h"
/* 开发板硬件bsp头文件 */
#include "stdio.h"
#include <math.h>
#include <stdbool.h>
#include "stdlib.h"
#include "sys.h"
/**************************** 任务句柄 ********************************/
/*
 * 任务句柄是一个指针，用于指向一个任务，当任务创建好之后，它就具有了一个任务句柄
 * 以后我们要想操作这个任务都需要通过这个任务句柄，如果是自身的任务操作自己，那么
 * 这个句柄可以为NULL。
 */

QueueHandle_t Task_Number_Handle = NULL;



static TaskHandle_t AppTaskCreate_Handle = NULL;   //+创建任务句柄
static TaskHandle_t Task__ONE_Handle = NULL;       //+任务1句柄
static TaskHandle_t Task__TWO_Handle = NULL;       //+任务2句柄
static TaskHandle_t Task__THREE_Handle = NULL;     //+任务3句柄
static TaskHandle_t Task__FOUR_Handle = NULL;      //+任务4句柄

/******************************* Global variable declaration ************************************/
/*
 * 当我们在写应用程序的时候，可能需要用到一些全局变量。
 */

int16_t now_speed[4]={0};

/*
*************************************************************************
*                             函数声明
*************************************************************************
*/

static void AppTaskCreate(void);           /* 用于创建任务 */
static void Task__TWO(void *pvParameters); /* Test_Task任务实现 */
static void Task__ONE(void *pvParameters); /* Test_Task任务实现 */
static void Task__THREE(void *pvParameters);             //
static void Task__FOUR(void *pvParameters);              //
void motor_fixed_distance_params(float tar_pos, float tar_speed, float tar_angle);
void motor_diff_turn_params( float tar_speed_left, float tar_speed_right, float tar_angle);
void motor_single_speed_params( float tar_speed_left_qian, float tar_speed_left_hou,float tar_speed_right_qian,float tar_speed_right_hou, float tar_pos);



/*****************************************************************
  * @brief  主函数
  * @param  无
  * @retval 无
  * @note   第一步：开发板硬件初始化
            第二步：创建APP应用任务
            第三步：启动FreeRTOS，开始多任务调度
  ****************************************************************/
int main(void)
{

    /* 开发板硬件初始化 */
		RCC_ClocksTypeDef get_rcc_clock;
        RCC_GetClocksFreq(&get_rcc_clock);
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
		delay_init(168);     //初始化延时函数
		pid_init();
		OLED_Init();
		Key_Init();
		LED_Init();
		board_init();
		HWT101_board_init();
		Serial_Init();
		SERVO_Init();
		Motor_Init();
		Encoder_TIM3_Init();
		Encoder_TIM4_Init();
		Encoder_TIM2_Init();
		Encoder_TIM5_Init();
		
		Timer6_Init();
	
		LED_OFF();
		rotate_angle(66);
		claw_angle(180);
		pitch_angle(140);
		
    /* 创建AppTaskCreate任务 */
     xTaskCreate((TaskFunction_t)AppTaskCreate,          /* 任务入口函数 */
                          (const char *)"AppTaskCreate",          /* 任务名字 */
                          (uint16_t)256,                          /* 任务栈大小 */
                          (void *)NULL,                           /* 任务入口函数参数 */
                          (UBaseType_t)1,                         /* 任务的优先级 */
                          (TaskHandle_t *)&AppTaskCreate_Handle); /* 任务控制块指针 */
     
	 /* 启动任务调度 */
     vTaskStartScheduler(); /* 启动任务，开启调度 */

         /* 正常不会执行到这里 */
}

/***********************************************************************
 * @ 函数名  ： AppTaskCreate
 * @ 功能说明： 为了方便管理，所有的任务创建函数都放在这个函数里面
 * @ 参数    ： 无
 * @ 返回值  ： 无
 **********************************************************************/
static void AppTaskCreate(void)
{
    taskENTER_CRITICAL(); // 进入临界区

    /* 创建Test_Task任务 */
 
    
	xTaskCreate((TaskFunction_t)Task__ONE,          /* 任务入口函数 */
                          (const char *)"Task__ONE",          /* 任务名字 */
                          (uint16_t)512,                      /* 任务栈大小 */
                          (void *)NULL,                       /* 任务入口函数参数 */
                          (UBaseType_t)1,                     /* 任务的优先级 */
                          (TaskHandle_t *)&Task__ONE_Handle); /* 任务控制块指针 */
   
	xTaskCreate((TaskFunction_t)Task__TWO,          /* 任务入口函数 */
                          (const char *)"Task__TWO",          /* 任务名字 */
                          (uint16_t)512,                      /* 任务栈大小 */
                          (void *)NULL,                       /* 任务入口函数参数 */
                          (UBaseType_t)2,                     /* 任务的优先级 */
                          (TaskHandle_t *)&Task__TWO_Handle); /* 任务控制块指针 */
  

    xTaskCreate((TaskFunction_t)Task__THREE,          /* 任务入口函数 */
                          (const char *)"Task__THREE",          /* 任务名字 */
                          (uint16_t)512,                       /* 任务栈大小 */
                          (void *)NULL,                         /* 任务入口函数参数 */
                          (UBaseType_t)3,                       /* 任务的优先级 */
                          (TaskHandle_t *)&Task__THREE_Handle); /* 任务控制块指针 */
  
    xTaskCreate((TaskFunction_t)Task__FOUR,          /* 任务入口函数 */
                          (const char *)"Task__FOUR",          /* 任务名字 */
                          (uint16_t)512,                       /* 任务栈大小 */
                          (void *)NULL,                        /* 任务入口函数参数 */
                          (UBaseType_t)4,                      /* 任务的优先级 */
                          (TaskHandle_t *)&Task__FOUR_Handle); /* 任务控制块指针 */
 


    // 挂机任务，等待选择任务

    vTaskDelete(AppTaskCreate_Handle); // 删除AppTaskCreate任务

    taskEXIT_CRITICAL(); // 退出临界区
}
int Speed_left_hou=0;
int Speed_left_qian=0;
int Speed_right_qian=0;
int Speed_right_hou=0;
int Speed_average=0;

int32_t Speed_sum=0;

volatile int mode=0;
int zigbee_mode=0;
int key=0;
int num=0;
int ched_is=0;
int Z_is=0;
int Y_is=0;
float i=0;
uint32_t Servo_Angle=180;
uint32_t my_test_clk=0;
uint8_t current_step = 0;  // 当前步骤索引（替代原来的num）
static void Task__ONE(void *parameter)
{
	
    while (1)
    {		
			ched_is=check_all_reached_filter();
//			Readclk_robot_arm();
//			Readclk_robot_arm_1();
			OLED_Clear();
			OLED_ShowFloatNum_My(1,1,Yaw,3,2,OLED_8X16);
			OLED_ShowNum_My(2,1,Func_Code,2,OLED_8X16);	
//			OLED_ShowSignedNum_My(2,1,Speed_sum,8,OLED_8X16);
			// OLED_ShowSignedNum_My(2,1,my_test_clk,8,OLED_8X16);			

			// OLED_ShowSignedNum_My(3,1,now_speed[0],4,OLED_8X16);
		    // OLED_ShowSignedNum_My(3,7,now_speed[1],4,OLED_8X16);
			
			// OLED_ShowSignedNum_My(4,1,now_speed[2],4,OLED_8X16);
		    // OLED_ShowSignedNum_My(4,7,now_speed[3],4,OLED_8X16);
		
			// OLED_ShowSignedNum_My(3,1,wheel_revolutions[1],7,OLED_8X16);
		
			// OLED_ShowSignedNum_My(4,1,Motor_Cur_Pos_robot_arm,7,OLED_8X16);
		
			OLED_ShowSignedNum_My(3,1,RXD_I[0],4,OLED_8X16);
		    OLED_ShowSignedNum_My(3,7,RXD_I[1],4,OLED_8X16);

			OLED_ShowSignedNum_My(4,1,TYPE_I[0],4,OLED_8X16);
		    OLED_ShowSignedNum_My(4,7,TYPE_I[1],4,OLED_8X16);
		
//			OLED_ShowNum_My(3,1,Servo_Angle,3,OLED_8X16);
	
//////			OLED_ShowNum_My(3,1,timekeeping*5,8,OLED_8X16);
////		
//			OLED_ShowNum_My(4,1,ched_is,2,OLED_8X16);
//			OLED_ShowNum_My(4,5,In_Servo,2,OLED_8X16);

			OLED_Update();
			
			vTaskDelay(1);
    }
}

uint32_t Ball_clk=0;
static void Task__TWO(void *parameter)
{
	while (1)
	{	
		key=Key_GetNum();
		if(key==1)
		{
			claw_angle(Claw_YuanZhu_Angle);
			// Servo_SetAngle(3,Pitch_Aim_Angle);
			// rotate_angle_slow(66,203);
			// Control_robot_arm_Z(0,10000,200,0);
//			Servo_Angle-=10;
//			claw_angle(Servo_Angle);
//			motor_fixed_distance_params(20000,500,0);
//			while (check_all_reached_filter() != 1)//等待车到位
//			{
//						vTaskDelay(1);
//			}
//			motor_clear_and_init_params();//清零
//			rotate_angle(20);
			// rotate_angle_slow(90,180);
		}
		else if(key==2)
		{
			Control_robot_arm_Y(1,500,50,0);
			vTaskDelay(100);
			claw_angle(Claw_YuanZhu_Angle);
			// Readclk_robot_arm();
//		    Servo_Angle-=5;
//			claw_angle(Servo_Angle);
//			rotate_angle_slow(47,184);
//			motor_fixed_distance_params(5000,500,0);
//			while (check_all_reached_filter() != 1)//等待车到位
//			{
//						vTaskDelay(1);
//			}
//			motor_clear_and_init_params();//清零
//			vTaskDelay(500);
		}
		else if(key==5)
		{
			// rotate_angle_slow(66,203);
//			Servo_Angle-=1;
//			claw_angle(Servo_Angle);
//			Control_robot_arm_Y(1,5000,50,0);
//			Control_robot_arm_Z(0,50000,200,0);
//			LED_TURN();
//			rotate_angle(184);
			mode=1;
		}
		
		vTaskDelay(1);	
	}
}



static void Task__THREE(void *parameter)
{	
    while (1)
    {
			 // 读取mode时先通过临界区获取最新值
        taskENTER_CRITICAL();
        uint8_t current_mode = mode;  // 临时变量保存当前mode
        taskEXIT_CRITICAL();
        if (current_mode == 1 && current_step < TOTAL_STEPS)
        {
					// 检查是否需要跳过当前步骤
            if (need_skip_next_step)
            {
                need_skip_next_step = 0;  // 清零标志
                current_step++;           // 直接进入下一步
                continue;
            }
            // 获取当前步骤
            Motor_control_MotorStep *step = &motion_steps[current_step];
						 // 判断是否需要执行电机控制（根据skip_motor参数）
						taskENTER_CRITICAL();
            if (!step->skip_motor)
            {
							// 根据步骤类型执行对应的电机控制函数
							switch (step->type)
							{
								case STEP_NONE:
										motor_clear_and_init_params();
									break;
								case STEP_FIXED_DIST:
										motor_fixed_distance_params(
												step->params.fixed_dist.tar_pos,
												step->params.fixed_dist.tar_speed,
												step->params.fixed_dist.tar_angle
										);
										break;
								case STEP_DIFF_TURN:
										motor_diff_turn_params(
												step->params.diff_turn.left_speed,
												step->params.diff_turn.right_speed,
												step->params.diff_turn.tar_angle
										);
										break;
								case STEP_SINGLE_SPEED:
										motor_single_speed_params(
												step->params.single_speed.lf_speed,
												step->params.single_speed.lb_speed,
												step->params.single_speed.rf_speed,
												step->params.single_speed.rb_speed,
												step->params.single_speed.tar_pos
										);
										break;
							}
						}
            taskEXIT_CRITICAL();
            // 执行当前步骤的额外业务代码
            if (step->action != NULL)
            {
							int16_t repeat_cnt = step->action_repeat;
							if (repeat_cnt == 0)// 根据重复次数执行业务代码
							{
								// 执行1次
								step->action(current_step);// 传入当前步骤索引
							}
							else if (repeat_cnt > 0)
							{
								// 执行指定次数
								for (int16_t i = 0; i < repeat_cnt; i++)
								{
									step->action(current_step);// 传入当前步骤索引
									vTaskDelay(1);// 可根据需要添加间隔延时
								}
							}
							else // repeat_cnt == -1
							{
								// 无限循环，直到退出标志置位
								step->action_exit_flag = 0; // 重置退出标志
								while (!step->action_exit_flag)
								{
										step->action(current_step);
										vTaskDelay(1); // 防止占满CPU
								} 
							}
            }
            // 等待动作完成（和之前一样）
            if (step->need_wait)
            {
							// 如果执行了电机动作，等待电机完成
							while (check_all_reached_filter() != 1)
							{
									vTaskDelay(1);
							}
							motor_clear_and_init_params();
							// 无论是否执行电机动作，都等待指定时间（可调整）
							vTaskDelay(1000);
            }
            current_step++;  // 进入下一步
        }
        vTaskDelay(1);
    }
}

static void Task__FOUR(void *parameter)
{
    while (1)
    {
		MPUData_processing();
		Servo_UpdateRotate(&gimbal_servo);
		if(Serial_RxFlag==1)//接收标志位置1，处理接收数据
		{
			Serial_Vision_Unpack();
			Serial_RxFlag=0;
		}
        vTaskDelay(1);	
    }
}

void TIM6_DAC_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
				now_speed[0]=Read_Speed(3)*2;
				now_speed[1]=Read_Speed(2)*2;
                now_speed[2]=Read_Speed(4)*2;
				now_speed[3]=Read_Speed(5)*2;
				
				Speed_average=(now_speed[0]+now_speed[1]+now_speed[2]+now_speed[3])/8;
				Speed_sum=Speed_sum+Speed_average;
		
				if(time_flag==1)
				{
					timekeeping++;
				}
				else if(time_flag==0)
				{
					timekeeping=0;
				}
				if(motor_control_mode==1)
				{
					Motor_control_fixed_distance(Speed_sum,now_speed,&fixed_dist_params);//整体走直线，固定距离，固定速度
				}
				else if(motor_control_mode==2)
				{
					Motor_control_diff_turn(now_speed,&diff_turn_params);//差速转弯，到目标角度
				}
				else if(motor_control_mode==3)
				{
					Motor_control_single_speed(Speed_sum,now_speed,&single_speed_params);//4轮分别速度控制，运行到目标距离
				}
				else if(motor_control_mode==4)
				{
					Motor_control_fixed(&fixed_dist_params);//整体走直线，固定角度
				}
    }
}





