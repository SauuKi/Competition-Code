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

int16_t now_speed[2]={0};

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
void motor_single_speed_params( float left_speed, float right_speed, float tar_pos);



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
		Gray_Init();
		Motor_Init();
		Encoder_TIM2_Init();
		Encoder_TIM5_Init();
		
		Timer6_Init();
	
		LED_OFF();
		
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

/* 底盘通信（Serial.c 中定义） */
extern volatile uint8_t chassis_cmd;
extern volatile uint8_t chassis_cmd_flag;
volatile uint8_t chassis_done_flag = 0;   // ISR通知任务发送0xB0应答
void Chassis_SendRSP(uint8_t rsp);
int key=0;
int num=0;
int ched_is=0;
int Z_is=0;
int Y_is=0;
float i=0;
uint32_t Servo_Angle=180;

uint8_t current_step = 0;  // 当前步骤索引（替代原来的num）

/*显示任务*/
static void Task__ONE(void *parameter)
{
    uint8_t Gray_Digtal[SENSOR_NUM];
    uint8_t i;

    while (1)
    {		
        /* 读取循迹传感器 */
        GetDigtal(Gray_Digtal);

        OLED_Clear();
        /* Line1: M:mode C:0xcmd */
        OLED_ShowString_My(1, 1, "M:", OLED_8X16);
        OLED_ShowNum_My(1, 3, mode, 1, OLED_8X16);
        OLED_ShowString_My(1, 5, "C:0x", OLED_8X16);
        OLED_ShowHexNum_My(1, 9, chassis_cmd, 2, OLED_8X16);

        // /* Line2: 循迹 1~8 */
        // OLED_ShowString_My(3, 1, "G:", OLED_8X16);
        // for (i = 0; i < 8; i++)
        //     OLED_ShowNum_My(3, 3 + i, Gray_Digtal[i], 1, OLED_8X16);

        // /* Line3: 循迹 9~12 */
        // for (i = 8; i < 12; i++)
        //     OLED_ShowNum_My(4, 3 + (i - 8), Gray_Digtal[i], 1, OLED_8X16);

        /* Line4: 偏差值 */
        OLED_ShowString_My(4, 1, "P:", OLED_8X16);
        OLED_ShowFloatNum_My(4, 3, Line_Position_Calc(Gray_Digtal), 3, 3, OLED_8X16);

       /* Yaw + 总里程 */
       OLED_ShowFloatNum_My(2,1,Yaw,3,1,OLED_8X16);
       OLED_ShowSignedNum_My(2,6, Speed_sum, 12, OLED_8X16);
       /* 计时(s) */
       OLED_ShowString_My(3, 1, "T:", OLED_8X16);
       OLED_ShowFloatNum_My(3, 3, timekeeping / 200.0f, 4, 1, OLED_8X16);
       OLED_ShowString_My(3, 10, "s", OLED_8X16); 
       //    /* 轮速 */
    //    OLED_ShowSignedNum_My(3, 1, now_speed[0], 4, OLED_8X16);
    //    OLED_ShowSignedNum_My(3, 7, now_speed[1], 4, OLED_8X16);

        OLED_Update();
        vTaskDelay(50);
    }
}

/*按键任务*/
uint32_t Ball_clk=0;
static void Task__TWO(void *parameter)
{
	while (1)
	{	
		key=Key_GetNum();
		if(key==1)
		{
			motor_line_track_params(1800, 2500, 2660000, 2500000, 0, 0, 0, 0,0);  // 快速循迹（无加减速）
		}
		else if(key==2)
		{
            motor_line_track_params(1100, 1528, 2720000, 2500000, 1, 300, 150000, 150000,0); // 慢速循迹（加减速）
		    // motor_fixed_distance_params(837564, 1000, 0);
        }
		else if(key==4)
		{
			motor_fixed_distance_params(700300,1000,0);
		}
		else if(key==5)
		{
            motor_line_track_params(1450, 2014, 710300, 2500000, 1, 300, 150000, 150000,0);         
        }
		vTaskDelay(10);	
	}
}


/*控制任务顺序轮询*/
static void Task__THREE(void *parameter)
{	
    static uint8_t last_mode = 0;  // 记录上次mode，用于检测切换

    while (1)
    {
        // 读取mode时先通过临界区获取最新值
        taskENTER_CRITICAL();
        uint8_t current_mode = mode;
        taskEXIT_CRITICAL();

        // mode 切换时重置步骤计数器
        if (current_mode != last_mode)
        {
            current_step = 0;
            motor_clear_and_init_params();
            last_mode = current_mode;
        }

        if (current_mode != MODE_IDLE && current_mode < MOTION_PATH_COUNT)
        {
            MotionPath *path = &motion_paths[current_mode];

            if (current_step < path->total_steps)
            {
                // 检查是否需要跳过当前步骤
                if (need_skip_next_step)
                {
                    need_skip_next_step = 0;
                    current_step++;
                    continue;
                }
                // 获取当前步骤
                Motor_control_MotorStep *step = &path->steps[current_step];
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
                                step->params.single_speed.left_speed,
                                step->params.single_speed.right_speed,
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
                    if (repeat_cnt == 0)
                    {
                        step->action(current_step);
                    }
                    else if (repeat_cnt > 0)
                    {
                        for (int16_t i = 0; i < repeat_cnt; i++)
                        {
                            step->action(current_step);
                            vTaskDelay(1);
                        }
                    }
                    else // repeat_cnt == -1
                    {
                        step->action_exit_flag = 0;
                        while (!step->action_exit_flag)
                        {
                            step->action(current_step);
                            vTaskDelay(1);
                        }
                    }
                }
                // 等待动作完成
                if (step->need_wait)
                {
                    while (check_all_reached_filter() != 1)
                    {
                        vTaskDelay(1);
                    }
                    motor_clear_and_init_params();
                    vTaskDelay(1000);
                }
                current_step++;  // 进入下一步
            }
            if (current_step >= path->total_steps)
            {
                Chassis_SendRSP(0xB0);
                motor_clear_and_init_params();
                taskENTER_CRITICAL();
                mode = MODE_IDLE;
                taskEXIT_CRITICAL();
            }   
        } 
        vTaskDelay(1);
    }
}

/*处理陀螺仪数据+底盘指令*/
static void Task__FOUR(void *parameter)
{
    while (1)
    {
        MPUData_processing();
        if(Serial_RxFlag==1)
        {
            CharToInt(Rx_data,RXD_I);
            Serial_RxFlag=0;
        }
        if(chassis_cmd_flag)
        {
            taskENTER_CRITICAL();
            uint8_t cmd = chassis_cmd;
            chassis_cmd = 0;            // 清零旧指令
            chassis_cmd_flag = 0;
            taskEXIT_CRITICAL();
            motor_clear_and_init_params();
            switch(cmd)
            {
                case 0xA1: motor_line_track_params(1800, 2500, 2663000, 2500000, 0, 0, 0, 0, 1);   timekeeping = 0; break;  // 快速循迹（无加减速, Yaw提前停止）
                case 0xA2: motor_line_track_params(1200, 1667, 710300, 2500000, 1, 300, 100000, 100000, 0); timekeeping = 0; break;  // 慢速循迹（加减速）A->B
                case 0xA3: motor_line_track_params(1100, 1528, 2720000, 2500000, 1, 300, 150000, 150000, 0); timekeeping = 0; break;  // 慢速循迹（加减速）
                case 0xA4: motor_line_track_params(1800, 2500, 5320000, 2500000, 0, 0, 0, 0, 0);timekeeping = 0; break;
                case 0xA0: mode = MODE_IDLE; Load(0,0);           time_flag = 0;   break;  // 停止
            }
        }
		if(chassis_done_flag)
		{
			chassis_done_flag = 0;
			Chassis_SendRSP(0xB0);
			mode = MODE_IDLE;
		}
        vTaskDelay(1);
    }
}

/*电机控制*/
void TIM6_DAC_IRQHandler(void)
{
	static uint8_t track_timer_started = 0;
	
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
				now_speed[0]=Read_Speed(2)*2;
				now_speed[1]=Read_Speed(5)*2;
				
				Speed_average=(now_speed[0]+now_speed[1])/4;
				Speed_sum=Speed_sum+Speed_average;
		
				if(time_flag==1)
				{
					timekeeping++;
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
				else if(motor_control_mode==5)
				{
					if(!track_timer_started)
					{
						time_flag = 1;
						track_timer_started = 1;
					}
					
					uint8_t Digtal[SENSOR_NUM];
					GetDigtal(Digtal);
					
					if(Speed_sum >= line_track_params.stop_hard)
					{
						motor_control_mode = 0;
						Load(0, 0);
						Speed_sum = 0;
						time_flag = 0;
						track_timer_started = 0;
						chassis_done_flag = 1;
					}
					else if (line_track_params.yaw_stop_enable
					         && Speed_sum >= line_track_params.detect_start
					         && fabsf(Yaw) < 33.0f)
					{
						motor_control_mode = 0;
						Load(0, 0);
						Speed_sum = 0;
						time_flag = 0;
						track_timer_started = 0;
						chassis_done_flag = 1;
					}
					else
				{
					// 动态调速：加减速使能时，根据已走距离调整 base_speed
					if (line_track_params.accel_enable)
					{
						if (Speed_sum < line_track_params.accel_distance)
						{
							// 加速段：线性从 min_speed → base_speed_orig
							line_track_params.base_speed = line_track_params.accel_min_speed
								+ (line_track_params.base_speed_orig - line_track_params.accel_min_speed)
								* Speed_sum / line_track_params.accel_distance;
						}
						else if (Speed_sum > line_track_params.stop_hard - line_track_params.decel_distance)
						{
							// 减速段：线性从 base_speed_orig → min_speed
							int32_t remaining = line_track_params.stop_hard - Speed_sum;
							line_track_params.base_speed = line_track_params.accel_min_speed
								+ (line_track_params.base_speed_orig - line_track_params.accel_min_speed)
								* remaining / line_track_params.decel_distance;
						}
						else
						{
							// 匀速段
							line_track_params.base_speed = line_track_params.base_speed_orig;
						}
						// steer_gain 随 base_speed 等比缩放，避免低速时转向过猛
						line_track_params.steer_gain = line_track_params.steer_gain_orig
							* line_track_params.base_speed / line_track_params.base_speed_orig;
					}
					Motor_control_line_track(now_speed, &line_track_params);//循迹控制
				}
				}
				else
				{
					track_timer_started = 0;  // 退出循迹模式时重置
				}
    }
}





