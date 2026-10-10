#ifndef __TJC_H
#define __TJC_H

#include "sys.h"
#include "usart.h"

void USART5_SendString(const char *str);
int Generate_Nextion_n0_Val_Cmd(char *buf, size_t buf_size, int32_t value);
int Generate_Nextion_n1_Val_Cmd(char *buf, size_t buf_size, int32_t value);

#endif
