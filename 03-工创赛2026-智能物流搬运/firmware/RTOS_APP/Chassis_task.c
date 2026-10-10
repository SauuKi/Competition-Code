#include "Chassis_task.h"
#include "Main_task.h"


uint8_t my_flag=0;
float angle=95;
float test_mm=0;
uint32_t test_clk=0;
void Chassis_task(void)
{
	uint8_t keynum=0;
	uint32_t Speed=0;
	float Angle_set=0;
	char send_buf_1[32];
	char send_buf_2[32];
	osDelay(500);
//	Emm_V5_Origin_Trigger_Return(6,2,1);
//	osDelay(1);
	Emm_V5_Origin_Trigger_Return(7,1,1);
	osDelay(1);
	Emm_V5_Synchronous_motion(0);
	osDelay(1);
	Move_Rack_Absolute_Y_One(60,50,20);
	Set_Claw_Height(16.32);
	while(1)
	{
		keynum=Key_GetNum();
		if(keynum==1)
		{
			//Set_Angle(3,Claw_Angle);
			mode=1;
		}
		if(keynum==2)
		{
			//Set_Angle(3,Release_Angle);
			//mode=1;
			//clockwise_flag=1;//è½¬ç›˜ä¸ºé¡ºæ—¶é’ˆ
		}
		if(keynum==3)
		{
//			Set_Angle(3,69);
//			angle-=1;
//			Set_Angle(3,angle);
//			Move_Rack_Absolute_Y(159,300,200);
//			servo_start_rotate(&gimbal_servo,1,114,200); 
//		    Set_Turntable_Angle(15.0f);
//			do{
//					Emm_V5_Read_Sys_Params(6, S_CPOS);
//					osDelay(5);
//			  }while(!((Emm_revolutions[5]-100)<=(Motor_Cur_Pos_robot_arm_Y)
//					&&(Motor_Cur_Pos_robot_arm_Y)<=(Emm_revolutions[5]+100)));
//			Set_Claw_Height_Relative(14.4f);
//			do{
//					Emm_V5_Read_Sys_Params(5, S_CPOS);
//					osDelay(5);
//			  }while(!((Emm_revolutions[4]-100)<=(Motor_Cur_Pos_robot_arm_Z)
//					&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+100)));
//			Set_Angle(3,74);
//			vTaskDelay(500);
//			Back_Zero(5);
//			do{
//					Emm_V5_Read_Sys_Params(5, S_CPOS);
//					osDelay(5);
//			  }while(!((Emm_revolutions[4]-100)<=(Motor_Cur_Pos_robot_arm_Z)
//					&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+100)));
//			Move_Rack_Absolute_Y(0,300,200);  
//			servo_start_rotate(&gimbal_servo,1,258,200);
//			vTaskDelay(1000);
//			Set_Claw_Height_Relative(18.32f);
//			do{
//					Emm_V5_Read_Sys_Params(5, S_CPOS);
//					osDelay(5);
//			  }while(!((Emm_revolutions[4]-100)<=(Motor_Cur_Pos_robot_arm_Z)
//					&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+100)));  
//			Set_Angle(3,100);   
//			vTaskDelay(500); 
//            Back_Zero(5);
//			do{
//					Emm_V5_Read_Sys_Params(5, S_CPOS);
//					osDelay(5);
//			  }while(!((Emm_revolutions[4]-100)<=(Motor_Cur_Pos_robot_arm_Z)
//					&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+100)));			  
//			servo_start_rotate(&gimbal_servo,1,95,200);
//			calculate_wheel_cali_lateral_revolution(0.01,20,5);
//			Set_Turntable_Angle(15.0f);
//			angle+=1;
//			Back_Zero(5);
//			Serial_SendString("FIRST");
//			Back_Zero(5);
//			test_clk+=10.0f;
//			Move_Rack_Absolute_Y(test_clk,100,50);
//			Set_Pos_Y(0,test_clk,50,20);
//			Set_Angle(1,angle);
//			servo_start_rotate(&gimbal_servo,1,180,100);
//			Set_Angle(3,89);
		}
		if(keynum==4)
		{
			Set_Turntable_Angle(Object2_Angle);
//			test_clk-=10.0f;
//			Move_Rack_Absolute_Y(test_clk,100,50);
//			Set_Turntable_Angle(15.0f);
//			Move_Rack_Absolute_Y(159,300,200);
//			servo_start_rotate(&gimbal_servo,1,79,200); 
//		    Set_Turntable_Angle(15.0f);
//			do{
//					Emm_V5_Read_Sys_Params(6, S_CPOS);
//					osDelay(5);
//			  }while(!((Emm_revolutions[5]-100)<=(Motor_Cur_Pos_robot_arm_Y)
//					&&(Motor_Cur_Pos_robot_arm_Y)<=(Emm_revolutions[5]+100)));
//			Set_Claw_Height_Relative(14.4f);
//			do{
//					Emm_V5_Read_Sys_Params(5, S_CPOS);
//					osDelay(5);
//			  }while(!((Emm_revolutions[4]-100)<=(Motor_Cur_Pos_robot_arm_Z)
//					&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+100)));
//			Set_Angle(3,74);
//			vTaskDelay(500);
//			Back_Zero(5);
//			do{
//					Emm_V5_Read_Sys_Params(5, S_CPOS);
//					osDelay(5);
//			  }while(!((Emm_revolutions[4]-100)<=(Motor_Cur_Pos_robot_arm_Z)
//					&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+100)));
//			Move_Rack_Absolute_Y(95,300,200);  
//			do{
//					Emm_V5_Read_Sys_Params(6, S_CPOS);
//					osDelay(5);
//			  }while(!((Emm_revolutions[5]-100)<=(Motor_Cur_Pos_robot_arm_Y)
//					&&(Motor_Cur_Pos_robot_arm_Y)<=(Emm_revolutions[5]+100)));
//            Move_Rack_Absolute_Y(0,300,200);			  
//			servo_start_rotate(&gimbal_servo,1,258,200);
//			vTaskDelay(1000);
//			Set_Claw_Height_Relative(18.32f);
//			do{
//					Emm_V5_Read_Sys_Params(5, S_CPOS);
//					osDelay(5);
//			  }while(!((Emm_revolutions[4]-100)<=(Motor_Cur_Pos_robot_arm_Z)
//					&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+100)));  
//			Set_Angle(3,100);   
//			vTaskDelay(500); 
//            Back_Zero(5);
//			do{
//					Emm_V5_Read_Sys_Params(5, S_CPOS);
//					osDelay(5);
//			  }while(!((Emm_revolutions[4]-100)<=(Motor_Cur_Pos_robot_arm_Z)
//					&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+100)));			  
//			servo_start_rotate(&gimbal_servo,1,114,200);
//		    Set_Turntable_Angle(135.0f);  
//			vTaskDelay(1000); 
//            Move_Rack_Absolute_Y(159,300,200);	
//            do{
//					Emm_V5_Read_Sys_Params(6, S_CPOS);
//					osDelay(5);
//			  }while(!((Emm_revolutions[5]-100)<=(Motor_Cur_Pos_robot_arm_Y)
//					&&(Motor_Cur_Pos_robot_arm_Y)<=(Emm_revolutions[5]+100)));
//			Set_Claw_Height_Relative(14.4f);
//			do{
//					Emm_V5_Read_Sys_Params(5, S_CPOS);
//					osDelay(5);
//			  }while(!((Emm_revolutions[4]-100)<=(Motor_Cur_Pos_robot_arm_Z)
//					&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+100)));
//			Set_Angle(3,74);
//			vTaskDelay(500);
//			Back_Zero(5);
//			do{
//					Emm_V5_Read_Sys_Params(5, S_CPOS);
//					osDelay(5);
//			  }while(!((Emm_revolutions[4]-100)<=(Motor_Cur_Pos_robot_arm_Z)
//					&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+100)));
//			Move_Rack_Absolute_Y(0,300,200);  
//			servo_start_rotate(&gimbal_servo,1,258,200);
//			vTaskDelay(1000);
//			Set_Claw_Height_Relative(18.32f);
//			do{
//					Emm_V5_Read_Sys_Params(5, S_CPOS);
//					osDelay(5);
//			  }while(!((Emm_revolutions[4]-100)<=(Motor_Cur_Pos_robot_arm_Z)
//					&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+100)));  
//			Set_Angle(3,100);   
//			vTaskDelay(500); 
//            Back_Zero(5);
//			do{
//					Emm_V5_Read_Sys_Params(5, S_CPOS);
//					osDelay(5);
//			  }while(!((Emm_revolutions[4]-100)<=(Motor_Cur_Pos_robot_arm_Z)
//					&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+100)));	
//            servo_start_rotate(&gimbal_servo,1,95,200);
//			Set_Turntable_Angle(255.0f); 
//			vTaskDelay(1000);
//			Set_Claw_Height_Relative(14.4f);
//			do{
//					Emm_V5_Read_Sys_Params(5, S_CPOS);
//					osDelay(5);
//			  }while(!((Emm_revolutions[4]-100)<=(Motor_Cur_Pos_robot_arm_Z)
//					&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+100)));
//			Set_Angle(3,74);
//			vTaskDelay(500);
//			Back_Zero(5);
//			do{
//					Emm_V5_Read_Sys_Params(5, S_CPOS);
//					osDelay(5);
//			  }while(!((Emm_revolutions[4]-100)<=(Motor_Cur_Pos_robot_arm_Z)
//					&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+100)));
//			servo_start_rotate(&gimbal_servo,1,258,200);
//			vTaskDelay(1000);
//			Set_Claw_Height_Relative(18.32f);
//			do{
//					Emm_V5_Read_Sys_Params(5, S_CPOS);
//					osDelay(5);
//			  }while(!((Emm_revolutions[4]-100)<=(Motor_Cur_Pos_robot_arm_Z)
//					&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+100)));  
//			Set_Angle(3,100);   
//			vTaskDelay(500); 
//            Back_Zero(5);
//			do{
//					Emm_V5_Read_Sys_Params(5, S_CPOS);
//					osDelay(5);
//			  }while(!((Emm_revolutions[4]-100)<=(Motor_Cur_Pos_robot_arm_Z)
//					&&(Motor_Cur_Pos_robot_arm_Z)<=(Emm_revolutions[4]+100)));
//			servo_start_rotate(&gimbal_servo,1,95,200);		
			  
//			calculate_wheel_cali_lateral_revolution(-0.01,20,5);
//			Set_Angle(3,74);
//		    Serial_SendString("SECOND");
		}
		if(keynum==5)
		{
//			Set_Claw_Height_Relative(Material_Hight_2,2000,0);
			Move_Rack_Absolute_Y(50,50,20);
			Back_Zero(5,200,0);
//			Serial_SendString("FIVE");
		}
		if(parsed_data.is_valid)
		{
//			if(parsed_data.func_code==1)//×ø±ê
//			{
//				memset(&send_buf_1, 0, sizeof(send_buf_1));
//                memset(&send_buf_2, 0, sizeof(send_buf_2));
//				Generate_Nextion_n0_Val_Cmd(send_buf_1,32,parsed_data.dx);
//				Generate_Nextion_n1_Val_Cmd(send_buf_2,32,parsed_data.dy);
//				USART5_SendString(send_buf_1);
//	            vTaskDelay(100);
//				USART5_SendString(send_buf_2);
//			}
			if(parsed_data.func_code==2)//¶þÎ¬Âë
			{
				memset(&send_buf_1, 0, sizeof(send_buf_1));
                memset(&send_buf_2, 0, sizeof(send_buf_2));
				Generate_Nextion_n0_Val_Cmd(send_buf_1,32,parsed_data.QR_NUM1);
				Generate_Nextion_n1_Val_Cmd(send_buf_2,32,parsed_data.QR_NUM2);
				USART5_SendString(send_buf_1);
	            vTaskDelay(100);
				USART5_SendString(send_buf_2);
			}
			parsed_data.is_valid=0;
		}
		osDelay(1);
	}
}
