#ifndef __SERIAL_H
#define __SERIAL_H

#include "sys.h"
#include "usart.h"

// 协议参数
#define FRAME_HEADER      0xABCD  // 帧头
#define FRAME_TAIL        0xEF12  // 帧尾
#define FRAME_LEN         14      // 固定帧长
#define RX_BUF_SIZE       128     // DMA接收缓冲区大小

// 解析后的数据结构体
typedef struct {
    uint8_t  func_code;       // 功能码
    int32_t  dx;              // X坐标差
    int32_t  dy;              // Y坐标差
	int32_t  QR_NUM1;         // 二维码数据1
	int32_t  QR_NUM2;         // 二维码数据2
	int32_t  test_num1;
	int32_t  test_num2;
	uint8_t  Material_Pos[3]; //物料位置数组
    uint8_t  is_valid;        // 数据有效标志
} ParsedData_t;

extern ParsedData_t parsed_data;
extern uint8_t return_num;

void Serial_SendString(const char *str);
int Vision_Data_Unpack(uint8_t *rw_data, ParsedData_t *parsed_data);

#endif
