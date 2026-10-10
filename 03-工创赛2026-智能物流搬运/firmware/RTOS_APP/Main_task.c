#include "Main_task.h"

uint8_t current_step = 0; //当前步骤的索引
uint8_t mode=0;
uint8_t in_pos=0;
void Main_task(void)
{
	while(1)
	{
		taskENTER_CRITICAL();
        uint8_t current_mode = mode;  // 临时变量保存当前mode
        taskEXIT_CRITICAL();
		if(current_mode == 1 && current_step < TOTAL_STEPS)
		{
			// 检查是否需要跳过当前步骤
            if (need_skip_next_step)
            {
                need_skip_next_step = 0;  // 清零标志
                current_step++;           // 直接进入下一步
                continue;
            }
			Move_Control_Step *step=&motion_steps[current_step];
			// 判断是否需要执行电机控制（根据skip_motor参数）
			if(!step->skip_motor)
			{
				switch(step->type)
				{
					case STEP_NONE:
						Emm_V5_Stop_Now(0,0);
						break;
					case STEP_COORDINATE:
						cali_pos(step->params.coordinate.tar_x,
					             step->params.coordinate.tar_y,
					             step->params.coordinate.tar_angle);
						osDelay(500);
						break;
					case STEP_DIFF_TURN:
						Move_Diff_Turn(step->params.diffturn.tar_left_clk,
					                   step->params.diffturn.tar_right_clk,
									   step->params.diffturn.tar_speed,
					                   step->params.diffturn.tar_Acceleration);
						break;
					case STEP_STRAIGHT:
						cali_Straight(step->params.straight.distance_x,
					               step->params.straight.distance_y,
					               step->params.straight.tar_angle,
					               step->params.straight.correct_angain);
						break;
					case STEP_Slant:
						MOVE_SLANT(step->params.slant.Right_Slant_clk,
					               step->params.slant.Left_Slant_clk,
					               step->params.slant.v,
					               step->params.slant.a);
						break;
					default:
						break;		
				}
			}
			if(step->action!=NULL)
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
			// 等待动作完成
			if(step->need_wait)//进行到位检测(仅直行和转弯需要)
			{
				in_pos=In_place_inspection();
				Emm_V5_Stop_Now(0,0);
				vTaskDelay(100);
			}
			Reset_CurPos_To_Zero();
			current_step++;
		}
		vTaskDelay(1);
	}
}
