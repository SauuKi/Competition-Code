#include "Gray.h"

/* 传感器权重（左正右负，12路，中心在索引5和6之间） */
static const float SENSOR_WEIGHTS[SENSOR_NUM] = {
    +5.5f, +4.5f, +3.5f, +2.5f, +1.5f, +0.5f,
    -0.5f, -1.5f, -2.5f, -3.5f, -4.5f, -5.5f
};

/** 
  * @brief  初始化全部 12 路循迹传感器 GPIO（输入下拉）
  *         引脚: PG2 ~ PG13
  */
void Gray_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* 开启 GPIOG 时钟 */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

    /* 输入模式 + 下拉 */
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    /* PG2 ~ PG13 */
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2  | GPIO_Pin_3  | GPIO_Pin_4  |
                               GPIO_Pin_5  | GPIO_Pin_6  | GPIO_Pin_7  |
                               GPIO_Pin_8  | GPIO_Pin_9  | GPIO_Pin_10 |
                               GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_Init(GPIOG, &GPIO_InitStruct);
}

/** 
  * @brief  读取单路循迹传感器 
  * @param  channel  传感器通道号 (1 ~ SENSOR_NUM)
  * @return 1: 检测到线, 0: 未检测到
  *         （下拉输入，检测到黑线 → 输出高电平 = 读到1）
  */
static unsigned char digtal(unsigned char channel)
{
    GPIO_TypeDef *port;
    uint16_t      pin;

    switch (channel)
    {
        case  1: port = GPIO_GRAY_PIN_0_PORT;   pin = GPIO_GRAY_PIN_0_PIN;   break;
        case  2: port = GPIO_GRAY_PIN_1_PORT;   pin = GPIO_GRAY_PIN_1_PIN;   break;
        case  3: port = GPIO_GRAY_PIN_2_PORT;   pin = GPIO_GRAY_PIN_2_PIN;   break;
        case  4: port = GPIO_GRAY_PIN_3_PORT;   pin = GPIO_GRAY_PIN_3_PIN;   break;
        case  5: port = GPIO_GRAY_PIN_4_PORT;   pin = GPIO_GRAY_PIN_4_PIN;   break;
        case  6: port = GPIO_GRAY_PIN_5_PORT;   pin = GPIO_GRAY_PIN_5_PIN;   break;
        case  7: port = GPIO_GRAY_PIN_6_PORT;   pin = GPIO_GRAY_PIN_6_PIN;   break;
        case  8: port = GPIO_GRAY_PIN_7_PORT;   pin = GPIO_GRAY_PIN_7_PIN;   break;
        case  9: port = GPIO_GRAY_PIN_8_PORT;   pin = GPIO_GRAY_PIN_8_PIN;   break;
        case 10: port = GPIO_GRAY_PIN_9_PORT;   pin = GPIO_GRAY_PIN_9_PIN;   break;
        case 11: port = GPIO_GRAY_PIN_10_PORT;  pin = GPIO_GRAY_PIN_10_PIN;  break;
        case 12: port = GPIO_GRAY_PIN_11_PORT;  pin = GPIO_GRAY_PIN_11_PIN;  break;
        default: return 0;
    }

    /* 低电平有效: 读到 0 表示检测到线 */
    return (GPIO_ReadInputDataBit(port, pin) == 0) ? 1 : 0;
}

/** 
  * @brief  读取全部 12 路传感器状态 
  * @param  Digtal  输出数组，长度 >= SENSOR_NUM
  */
void GetDigtal(uint8_t *Digtal)
{
    uint8_t i;
    for (i = 0; i < SENSOR_NUM; i++)
    {
        Digtal[i] = digtal(i + 1);
    }
}

/** 
  * @brief 统计有效传感器的数量 
  */
uint8_t Active_Count(uint8_t *Digtal)
{
    uint8_t count = 0;
    uint8_t i;
    for (i = 0; i < SENSOR_NUM; i++)
    {
        count += Digtal[i];
    }
    return count;
}

/** 
  * @brief  计算线位置偏差 (左正右负)
  * @param  Digtal  传感器状态数组 (1=检测到线)
  * @return 偏差值 -1.0 ~ +1.0，正值=线偏左需左转，0=居中或丢线
  */
float Line_Position_Calc(uint8_t *Digtal)
{
    float total_weight = 0.0f;
    uint8_t active_count = 0;
    uint8_t i;

    for (i = 0; i < SENSOR_NUM; i++)
    {
        if (Digtal[i])
        {
            total_weight += SENSOR_WEIGHTS[i];
            active_count++;
        }
    }

    if (active_count == 0)
        return 0.0f;

    return total_weight / (5.5f * (float)active_count);
}
