#include "Tim_Delay.h"

void delay_us_timer(uint32_t us) {
    __HAL_TIM_SET_COUNTER(&htim7, 0);
    HAL_TIM_Base_Start(&htim7);
    
    while(__HAL_TIM_GET_COUNTER(&htim7) < (us * 2)) { 
    }
	
    HAL_TIM_Base_Stop(&htim7);
}

