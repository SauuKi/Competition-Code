#ifndef __MAIN_TASK_H
#define __MAIN_TASK_H

#include "sys.h"
#include "Chassis_task.h"
#include "Output_task.h"
#include <stdbool.h>

/* ========== 比赛状态机 ========== */
typedef enum {
    STATE_IDLE = 0,      // 待机，等待按键选mode
    STATE_RUNNING,       // 运动中（底盘+球控制）
    STATE_DONE,          // 完成，显示结果
} CompetitionState;

/* ========== 比赛模式 ========== */
#define MODE_CIRCLE      1   // 顺时针一圈（要求2）
#define MODE_AB          2   // A→B 直线段+球稳定（要求4）
#define MODE_CIRCLE_BALL 3   // 顺时针一圈+球稳定（要求5/6）
#define MODE_TASK3       4   // 任务3
#define MODE_OTHER       5   // 其他底盘运动

/* ========== → 底盘MCU的UART指令 ========== */
#define CHASSIS_CMD_START_CIRCLE       0xA1   // 底盘快速一圈（无球）
#define CHASSIS_CMD_START_CIRCLE_SLOW  0xA3   // 底盘慢速一圈（有球控制）
#define CHASSIS_CMD_START_AB           0xA2   // 底盘走AB段
#define CHASSIS_CMD_STOP               0xA0   // 紧急停止
#define CHASSIS_CMD_START_OTHER        0xA4   // 其他底盘运动

/* ========== ← 底盘MCU的UART应答 ========== */
#define CHASSIS_RSP_DONE          0xB0   // 到位
#define CHASSIS_RSP_TIMEOUT       0xB1   // 超时/失败

/* ========== 全局状态变量 ========== */
extern CompetitionState comp_state;   // 当前状态
extern uint8_t selected_mode;         // 用户选择的mode
extern bool ball_control_enable;      // 是否启用球控制
extern float target_ball_pos;         // 目标球位置(mm), 0=中心
extern float dbg_target_angle;        // 调试: PID输出的目标角度(°)

void Main_task(void);

#endif
