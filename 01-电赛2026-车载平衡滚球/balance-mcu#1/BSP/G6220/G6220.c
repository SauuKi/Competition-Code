#include "G6220.h"

void G6220_Init(void)
{
	CAN_Start(CAN_NUM );
	HAL_Delay(1000);
	G6220_Command (1); //使能
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		 G6220命令
//  @return     		 /
//  @param					 CMD  基础命令 使能:1,失能:2,零点重置:3,清除报错:4
//  @Sample usage:   G6220_Command(&hcan1,0x01,1);//电机使能
//-------------------------------------------------------------------------------------------------------------------
void G6220_Command(uint8_t CMD)
{
	uint8_t Data[8];
	Data[0] = 0xFF;
	Data[1] = 0xFF;
	Data[2] = 0xFF;
	Data[3] = 0xFF;
	Data[4] = 0xFF;
	Data[5] = 0xFF;
	Data[6] = 0xFF;
	switch(CMD)
	{
		case G6220_Enable :
			Data[7] = 0xFC;
		break;
		case G6220_Disable :
		Data[7] = 0xFD;
		break;
		case G6220_Save_Zero_Position :
		Data[7] = 0xFE;
		break;
		case G6220_Clear_Error :
		Data[7] = 0xFB;
		break;
		default:
		break;
	}
	CAN_SendData(CAN_NUM,G6220_Modo+G6220_CAN_ID,(uint8_t*)Data,8);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		 G6220位置速度模式
//  @return     		 /
//  @param					 position   位置,这里采用的是角度制,单圈范围为0-3600，即分辨率为0.1度,可以设置多圈角度
//  @param					 speed			速度,采用的也是角度制，速度最大为2300度每秒
//  @Sample usage:   G6220_Position(1800,1800);//以180度每秒的速度转到180度的位置
//-------------------------------------------------------------------------------------------------------------------
void G6220_Position(int32_t position,uint16_t speed)
{
	float radian;
	radian = position*0.00174533;//这个系数是pai（3.14159265）除以1800得到的，因为浮点数乘的计算速度更快
	uint8_t *Postion_Tmp;
  Postion_Tmp = (uint8_t *)&radian;
	uint8_t Data[8];
	Data[0] = *(Postion_Tmp);
	Data[1] = *(Postion_Tmp + 1);
	Data[2] = *(Postion_Tmp + 2);
	Data[3] = *(Postion_Tmp + 3);
	
	float velocity;
	velocity = speed*0.0174533;
	uint8_t *Velocity_Tmp;
	Velocity_Tmp = (uint8_t *)&velocity;
	Data[4] = *(Velocity_Tmp);
	Data[5] = *(Velocity_Tmp + 1);
	Data[6] = *(Velocity_Tmp + 2);
	Data[7] = *(Velocity_Tmp + 3);
	CAN_SendData(CAN_NUM,G6220_POZ+G6220_CAN_ID,(uint8_t*)Data,8);//当输出为位置速度模式时,id要加0x100
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      		 G6220速度模式
//  @return     		 /
//  @param					 speed			速度,采用的也是角度制，速度最大为2300度每秒
//  @Sample usage:   G6220_Speed(1800);//以180度每秒的速度持续转动
//-------------------------------------------------------------------------------------------------------------------
void G6220_Speed(uint16_t speed)
{
	float velocity;
	velocity = speed*0.0174533;
	uint8_t *Velocity_Tmp;
	uint8_t Data[4];
	Velocity_Tmp = (uint8_t *)&velocity;
	Data[0] = *(Velocity_Tmp);
	Data[1] = *(Velocity_Tmp + 1);
	Data[2] = *(Velocity_Tmp + 2);
	Data[3] = *(Velocity_Tmp + 3);
	CAN_SendData(CAN_NUM,G6220_SPEED+G6220_CAN_ID,(uint8_t*)Data,4);//当输出为位置速度模式时,id要加0x200
}

