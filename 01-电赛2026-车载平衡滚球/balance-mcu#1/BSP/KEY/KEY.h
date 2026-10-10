#ifndef __KEY_H
#define __KEY_H

#include "GPIO.h"
#include "sys.h"

// 按键去抖动延时
#define KEY_DELAY_MS        20

uint8_t Key_GetNum(void);
//uint8_t Key_ScanTask(void);


//extern volatile uint32_t sys_tick;

#endif
