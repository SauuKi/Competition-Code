#include "LED.h"

void LED_OFF(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,GPIO_PIN_RESET);
}

void LED_ON(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,GPIO_PIN_SET);
}

void LED_TURN(void)
{
	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_0);
}
