#include "sys.h"

void sys_Init(void)
{
	  OLED_Init();
	  OPS9_init();
	  Servo_Init();
	  HWT101_board_init();
//	  Set_Angle (1,246.0f);
	  Set_Angle (3,160);
//	  Servo_Init_Angle(1, 246.0f);  
      Servo_Init_Angle(3, 160.0f);
}


