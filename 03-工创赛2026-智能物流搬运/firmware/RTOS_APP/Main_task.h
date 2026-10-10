#ifndef __MAIN_TASK_H
#define __MAIN_TASK_H

#include "sys.h"
#include "Chassis_task.h"
#include "Output_task.h"
#include "Tower_task.h"

extern uint8_t mode;
extern uint8_t in_pos;

void Main_task(void);

#endif
