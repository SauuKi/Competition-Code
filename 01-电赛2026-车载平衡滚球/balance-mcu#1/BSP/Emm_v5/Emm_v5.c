#include "Emm_v5.h"

/*内含延时，切换操作系统时需要修改*/

bool Emm_V5_rxFrameFlag = false;
uint8_t Emm_V5_rxCmd [Emm_V5_FIFO_SIZE ] = {0};
uint8_t Emm_V5_rxCount = 0;

/**
  * @brief    读取系统参数
  * @param    addr  ：电机地址
  * @param    s     ：系统参数类型
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[i] = addr; ++i;                   // 地址

  switch(s)                             // 功能码
  {
    case S_VER  : cmd[i] = 0x1F; ++i; break;
    case S_RL   : cmd[i] = 0x20; ++i; break;
    case S_PID  : cmd[i] = 0x21; ++i; break;
    case S_VBUS : cmd[i] = 0x24; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_ORG  : cmd[i] = 0x3B; ++i; break;
    case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // 校验字节
  
  // 发送命令
  HAL_UART_Transmit (&huart1,(uint8_t *)cmd,i,HAL_MAX_DELAY);
}

///*
//  * @brief    读取两轴的脉冲(阻塞)
//*/

//void ReadClk_Z(void)
//{
//	Emm_V5_Read_Sys_Params(5, S_CPOS);
//	osDelay(1);
//	while(Emm_V5_rxFrameFlag == false)
//	{Emm_V5_rxFrameFlag = false;}
//	if(Emm_V5_rxCmd[0] == 5 && Emm_V5_rxCmd[1] == 0x36 && Emm_V5_rxCount == 8)
//	{
//		// 拼接成uint32_t类型
//		pos_robot_arm_Z = (uint32_t)(
//											((uint32_t)Emm_V5_rxCmd[3] << 24)    |
//											((uint32_t)Emm_V5_rxCmd[4] << 16)    |
//											((uint32_t)Emm_V5_rxCmd[5] << 8)     |
//											((uint32_t)Emm_V5_rxCmd[6] << 0)
//										);
//		// 转换成角度
//		Motor_Cur_Pos_robot_arm_Z = (float)pos_robot_arm_Z * 360.0f / 65536.0f;
//		//转化成脉冲
//		Motor_Cur_Pos_robot_arm_Z = (int32_t)6400*(Motor_Cur_Pos_robot_arm_Z/360.0f);
//		// 符号
//		if(Emm_V5_rxCmd[2]==0) { Motor_Cur_Pos_robot_arm_Z = -Motor_Cur_Pos_robot_arm_Z; }
//	}
//}

//void ReadClk_Y(void)
//{
//	Emm_V5_Read_Sys_Params(6, S_CPOS);
//	delay_ms(1);
//	while(Emm_V5_rxFrameFlag == false)
//	{Emm_V5_rxFrameFlag = false;}
//	if(Emm_V5_rxCmd[0] == 6 && Emm_V5_rxCmd[1] == 0x36 && Emm_V5_rxCount == 8)
//	{
//		// 拼接成uint32_t类型
//		pos_robot_arm_Y = (uint32_t)(
//											((uint32_t)Emm_V5_rxCmd[3] << 24)    |
//											((uint32_t)Emm_V5_rxCmd[4] << 16)    |
//											((uint32_t)Emm_V5_rxCmd[5] << 8)     |
//											((uint32_t)Emm_V5_rxCmd[6] << 0)
//										);
//		// 转换成角度
//		Motor_Cur_Pos_robot_arm_Y = (float)pos_robot_arm_Y * 360.0f / 65536.0f;
//		//转化成脉冲
//		Motor_Cur_Pos_robot_arm_Y = (int32_t)6400*(Motor_Cur_Pos_robot_arm_Y/360.0f);
//		// 符号
//		if(Emm_V5_rxCmd[2]==0) { Motor_Cur_Pos_robot_arm_Y = -Motor_Cur_Pos_robot_arm_Y; }
//	}
//}

//void Emm_Data_Unpack(uint8_t Emm_V5_rxCount,uint8_t Emm_V5_rxCmd [Emm_V5_FIFO_SIZE ])
//{
//	if(Emm_V5_rxCmd[0] == 5 && Emm_V5_rxCmd[1] == 0x36 && Emm_V5_rxCount == 8)
//	{
//		// 拼接成uint32_t类型
//		pos_robot_arm_Z = (uint32_t)(
//											((uint32_t)Emm_V5_rxCmd[3] << 24)    |
//											((uint32_t)Emm_V5_rxCmd[4] << 16)    |
//											((uint32_t)Emm_V5_rxCmd[5] << 8)     |
//											((uint32_t)Emm_V5_rxCmd[6] << 0)
//										);
//		// 转换成角度
//		Motor_Cur_Pos_robot_arm_Z = (float)pos_robot_arm_Z * 360.0f / 65536.0f;
//		//转化成脉冲
//		Motor_Cur_Pos_robot_arm_Z = (int32_t)6400*(Motor_Cur_Pos_robot_arm_Z/360.0f);
//		// 符号
//		if(Emm_V5_rxCmd[2]==0) { Motor_Cur_Pos_robot_arm_Z = -Motor_Cur_Pos_robot_arm_Z; }
//	}
//	else if(Emm_V5_rxCmd[0] == 6 && Emm_V5_rxCmd[1] == 0x36 && Emm_V5_rxCount == 8)
//	{
//		// 拼接成uint32_t类型
//		pos_robot_arm_Y = (uint32_t)(
//											((uint32_t)Emm_V5_rxCmd[3] << 24)    |
//											((uint32_t)Emm_V5_rxCmd[4] << 16)    |
//											((uint32_t)Emm_V5_rxCmd[5] << 8)     |
//											((uint32_t)Emm_V5_rxCmd[6] << 0)
//										);
//		// 转换成角度
//		Motor_Cur_Pos_robot_arm_Y = (float)pos_robot_arm_Y * 360.0f / 65536.0f;
//		//转化成脉冲
//		Motor_Cur_Pos_robot_arm_Y = (int32_t)6400*(Motor_Cur_Pos_robot_arm_Y/360.0f);
//		// 符号
//		if(Emm_V5_rxCmd[2]==0) { Motor_Cur_Pos_robot_arm_Y = -Motor_Cur_Pos_robot_arm_Y; }
//	}
//}

/**
  * @brief    速度模式
  * @param    addr：电机地址
  * @param    dir ：方向       ，0为CW，其余值为CCW
  * @param    vel ：速度       ，范围0 - 5000RPM
  * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动
  * @param    snF ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF6;                       // 功能码
  cmd[2] =  dir;                        // 方向
  cmd[3] =  (uint8_t)(vel >> 8);        // 速度(RPM)高8位字节
  cmd[4] =  (uint8_t)(vel >> 0);        // 速度(RPM)低8位字节
  cmd[5] =  acc;                        // 加速度，注意：0是直接启动
  cmd[6] =  snF;                        // 多机同步运动标志
  cmd[7] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit(&huart1,(uint8_t *)cmd,8,HAL_MAX_DELAY);
}

/**
  * @brief    位置模式
  * @param    addr：电机地址
  * @param    dir ：方向        ，0为CW，其余值为CCW
  * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
  * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
  * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个
  * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动
  * @param    snF ：多机同步标志 ，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 功能码
  cmd[2]  =  dir;                       // 方向
  cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
  cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节 
  cmd[5]  =  acc;                       // 加速度，注意：0是直接启动
  cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
  cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
  cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
  cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
  cmd[10] =  raF;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
  cmd[11] =  snF;                       // 多机同步运动标志，false为不启用，true为启用
  cmd[12] =  0x6B;                      // 校验字节
  
  // 发送命令
  HAL_UART_Transmit(&huart1,(uint8_t *)cmd,13,HAL_MAX_DELAY);
}

/*四轮速度控制-(无多机同步)*/
void Emm_vel_control(int MotorSpeed1 ,int MotorSpeed2 ,int MotorSpeed3 ,int MotorSpeed4 )
{
	uint8_t MCommand1[8]={0x01,0xF6,0x00,0x00,0x10,0x00,0x00,0x6B};
	uint8_t MCommand2[8]={0x02,0xF6,0x00,0x00,0x10,0x00,0x00,0x6B};
	uint8_t MCommand3[8]={0x03,0xF6,0x00,0x00,0x10,0x00,0x00,0x6B};
	uint8_t MCommand4[8]={0x04,0xF6,0x00,0x00,0x10,0x00,0x00,0x6B};
	if (MotorSpeed1<0) {MotorSpeed1=-MotorSpeed1; MCommand1[2]=0x01;}  //电机方向指令
	MCommand1[3]=(MotorSpeed1 >> 8); MCommand1[4]=(MotorSpeed1 >> 0);  //电机速度指令
	
	if (MotorSpeed2<0) {MotorSpeed2=-MotorSpeed2; MCommand2[2]=0x01;}
	MCommand2[3]=(MotorSpeed2 >> 8); MCommand2[4]=(MotorSpeed2 >> 0); 
	
	if (MotorSpeed3<0) {MotorSpeed3=-MotorSpeed3; MCommand3[2]=0x01;}
	MCommand3[3]=(MotorSpeed3 >> 8); MCommand3[4]=(MotorSpeed3 >> 0); 
	
	if (MotorSpeed4<0) {MotorSpeed4=-MotorSpeed4; MCommand4[2]=0x01;}
	MCommand4[3]=(MotorSpeed4 >> 8); MCommand4[4]=(MotorSpeed4 >> 0);
	
	delay_ms(1);
	HAL_UART_Transmit(&huart1,(uint8_t *)MCommand1,8,HAL_MAX_DELAY);
	delay_ms(1);
	HAL_UART_Transmit(&huart1,(uint8_t *)MCommand2,8,HAL_MAX_DELAY);
	delay_ms(1);
	HAL_UART_Transmit(&huart1,(uint8_t *)MCommand3,8,HAL_MAX_DELAY);
	delay_ms(1);
	HAL_UART_Transmit(&huart1,(uint8_t *)MCommand4,8,HAL_MAX_DELAY);
	delay_ms(1);
}

/**
  * @brief    立即停止（所有控制模式都通用）
  * @param    addr  ：电机地址
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFE;                       // 功能码
  cmd[2] =  0x98;                       // 辅助码
  cmd[3] =  snF;                        // 多机同步运动标志
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  HAL_UART_Transmit(&huart1,(uint8_t *)cmd,5,HAL_MAX_DELAY);
}

/**
  * @brief    多机同步运动
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFF;                       // 功能码
  cmd[2] =  0x66;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
 HAL_UART_Transmit(&huart1,(uint8_t *)cmd,4,HAL_MAX_DELAY);
}

/**
  * @brief    将当前位置清零
  * @param    addr  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
    uint8_t cmd[16] = {addr, 0x0A, 0x6D, 0x6B};
    HAL_UART_Transmit(&huart1,(uint8_t *)cmd,4,HAL_MAX_DELAY);
}

/**
  * @brief    触发回零
  * @param    addr   ：电机地址
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
    uint8_t cmd[5] = {addr, 0x9A, o_mode, snF, 0x6B};
	HAL_UART_Transmit(&huart1,(uint8_t *)cmd,5,HAL_MAX_DELAY);
}

