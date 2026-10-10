#include "Serial.h"

ParsedData_t parsed_data;
uint8_t return_num=0;


void Serial_SendString(const char *str)
{
    uint16_t len = 0;
    const char *p = str;
    
    while (*p++ != '\0')
    {
        len++;
    }
    
    if (len > 0)
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)str, len, HAL_MAX_DELAY);
    }
}

/**
 * @brief 解析视觉数据
 * 
 * @param rw_data 视觉数据指针
 * @param parsed_data 解析后的数据指针
 * @return int 解析结果:0=成功，-1=帧头错，-2=帧尾错，-3=校验错，-4=未知功能码
 */
int Vision_Data_Unpack(uint8_t *rw_data, ParsedData_t *parsed_data)
{
    // 1. 帧头校验
    uint16_t header = (rw_data[0] << 8) | rw_data[1];
    if(header != FRAME_HEADER) return -1;

    // 2. 帧尾校验
    uint16_t tail = (rw_data[12] << 8) | rw_data[13];
    if(tail != FRAME_TAIL) return -2;

    // 3. 校验和校验
    uint8_t sum = 0;
    for(int i = 2; i <= 10; i++) sum += rw_data[i];
    if(sum != rw_data[11]) return -3;

    // 4. 统一先解析出功能码和数据段的两个 int32（只写一次移位逻辑）
    parsed_data->func_code = rw_data[2];
    
    // 用临时变量或直接用联合体/指针强转（这里保持移位写法以兼容大小端逻辑）
    int32_t data1 = (int32_t)(
        (rw_data[3] << 24) | (rw_data[4] << 16) |
        (rw_data[5] << 8)  | rw_data[6]
    );
    int32_t data2 = (int32_t)(
        (rw_data[7] << 24) | (rw_data[8] << 16) |
        (rw_data[9] << 8)  | rw_data[10]
    );

    // 5. 根据功能码“分发”数据 + 清零无关变量
    if(parsed_data->func_code == 1)
    {
        parsed_data->dx = data1;
        parsed_data->dy = data2;
        parsed_data->QR_NUM1 = 0;
        parsed_data->QR_NUM2 = 0;
    }
    else if(parsed_data->func_code == 2)
    {
        parsed_data->QR_NUM1 = data1;
        parsed_data->QR_NUM2 = data2;
        parsed_data->dx = 0;
        parsed_data->dy = 0;
    }
	else if(parsed_data->func_code == 3)
	{
		parsed_data->Material_Pos[0]=data1;
		parsed_data->Material_Pos[1]=data2;
	}
	else if(parsed_data->func_code == 6)
	{
		parsed_data->test_num1=data1;
		parsed_data->test_num2=data2;
	}
    else
    {
        return -4; // 未知功能码
    }

    parsed_data->is_valid = 1;
    return 0;
}



