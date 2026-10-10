#include "Output_task.h"
#include "Main_task.h"

uint8_t reach_flag=0;
void Output_task(void)	
{
	osDelay(100);
	uint32_t count=0;
	ops9_para_t *pData = get_ops9_data_point();
	while(1)
	{
//		Emm_V5_Read_Sys_Params (5,S_CPOS);
//		ReadClk_Z();
//		count++;
        float z_angle = ConvertToAngle(pData->OPS9_Rx_Data.z);
//		OLED_ShowNum_My(2,1,Angle_verification,1,OLED_8X16);
//		OLED_ShowNum_My(2,5,jiaodu_num,2,OLED_8X16);
//		OLED_ShowNum_My(1,8,in_pos,1,OLED_8X16);
//		OLED_ShowFloatNum_My(1,1,angle,5,2,OLED_8X16);
//		OLED_ShowFloatNum_My(2,1,test_clk,5,2,OLED_8X16);
//		OLED_ShowNum_My(3,1,is_done,1,OLED_8X16);
//		OLED_ShowFloatNum_My(3,1,Emm_revolutions[0],5,2,OLED_8X16);
//		OLED_ShowFloatNum_My(2,1,Emm_revolutions[1],5,2,OLED_8X16);
//		OLED_ShowFloatNum_My(3,1,Motor_Cur_Pos_robot_arm_Z,5,2,OLED_8X16);
//		OLED_ShowFloatNum_My(4,1,Emm_revolutions[4],5,2,OLED_8X16);
		/*底盘脉冲区数据*/
//		OLED_ShowFloatNum_My(1,1,Motor_Cur_Pos_wheel[0],7,0,OLED_8X16);
//		OLED_ShowFloatNum_My(2,1,Motor_Cur_Pos_wheel[1],7,0,OLED_8X16);
//		OLED_ShowFloatNum_My(3,1,Motor_Cur_Pos_wheel[2],7,0,OLED_8X16);
//		OLED_ShowFloatNum_My(4,1,Motor_Cur_Pos_wheel[3],7,0,OLED_8X16);
//        OLED_ShowFloatNum_My(1,9,Emm_revolutions[0],7,0,OLED_8X16);
//		OLED_ShowFloatNum_My(2,9,Emm_revolutions[1],7,0,OLED_8X16);
//		OLED_ShowFloatNum_My(3,9,Emm_revolutions[2],7,0,OLED_8X16);
//		OLED_ShowFloatNum_My(4,9,Emm_revolutions[3],7,0,OLED_8X16);
//        OLED_ShowNum_My(4,10,reach_flag,1,OLED_8X16);
		OLED_ShowFloatNum_My(1,1,pData->OPS9_Rx_Data.x,3,4,OLED_8X16);
		OLED_ShowFloatNum_My(2,1,pData->OPS9_Rx_Data.y,3,4,OLED_8X16);
//		OLED_ShowFloatNum_My(4,1,pData->OPS9_Rx_Data.z,3,4,OLED_8X16);
//		OLED_ShowNum_My(2,1,count,5,OLED_8X16);
//		OLED_ShowFloatNum_My(3,1,pData->OPS9_Rx_Data.x,3,4,OLED_8X16);
//		OLED_ShowFloatNum_My(4,1,Motor_Cur_Pos_wheel[0],5,2,OLED_8X16);
//		OLED_ShowNum_My(4,1,G6220_flag,1,OLED_8X16);
        /*通信信息显示*/
//		OLED_ShowSignedNum_My(1,1,parsed_data.QR_NUM1,4,OLED_8X16);
//		OLED_ShowSignedNum_My(2,1,parsed_data.QR_NUM2,4,OLED_8X16);
//		OLED_ShowSignedNum_My(1,6,parsed_data.dx,4,OLED_8X16);
//		OLED_ShowSignedNum_My(2,6,parsed_data.dy,4,OLED_8X16);
		OLED_ShowNum_My(3,1,parsed_data.func_code,1,OLED_8X16);
		OLED_ShowNum_My(4,1,parsed_data.test_num1,1,OLED_8X16);
		OLED_ShowNum_My(4,3,parsed_data.test_num2,1,OLED_8X16);
//		OLED_ShowNum_My(4,1,parsed_data.Material_Pos[0],1,OLED_8X16);
//		OLED_ShowNum_My(4,3,parsed_data.Material_Pos[1],1,OLED_8X16);
		OLED_ShowFloatNum_My(3,6,z_angle,3,2,OLED_8X16);
		OLED_ShowFloatNum_My(4,6,Yaw,3,2,OLED_8X16);
//		OLED_ShowSignedNum_My(4,1,return_num,2,OLED_8X16);
		
		OLED_Update();
		osDelay(100);
	}
	
}



