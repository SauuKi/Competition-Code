#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "sys.h"
#include <stdbool.h>

void Chassis_task(void);

/* ========== 底盘UART通信接口 ========== */
void uart_send_chassis_cmd(uint8_t cmd);
uint8_t uart_get_chassis_rsp(void);
void chassis_rsp_set(uint8_t rsp);       // 由UART接收调用

/* ========== 按键接口 ========== */
bool key_start_pressed(void);

/* ========== 偏移量选择（OLED显示用） ========== */
extern bool   offset_select_mode;
extern int8_t offset_value;

#endif
