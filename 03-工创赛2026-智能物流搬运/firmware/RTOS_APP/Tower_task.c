#include "Tower_task.h"

bool is_done;

void Tower_task(void)
{
	gimbal_servo.current_angle=246.0f;
	Set_Angle(Gimbal_Servo_ID,246.0f);
	while(1)
	{
		//更新舵机运动
		MPUData_processing();
        is_done = servo_update_rotate(&gimbal_servo);
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}
