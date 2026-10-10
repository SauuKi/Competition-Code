#ifndef __HWT101_H
#define __HWT101_H

#include "sys.h"  // 替换原标准库头文件

#define HWT101_FIFO_SIZE 64  // 原代码未定义，必须添加（根据帧长调整，建议≥32）

extern __IO bool HWT101_rxFrameFlag;
extern __IO uint8_t HWT101_rxCmd[HWT101_FIFO_SIZE];
extern float Yaw;

void HWT101_board_init(void);
void HWT_Settings(void);
void HWT_CALSW_IN(void);
void HWT_CALSW_OUT(void);
void MPUData_processing(void);
void HWT101(void);

#endif
