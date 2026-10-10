#include "TJC.h"

void USART5_SendString(const char *str)
{
    uint16_t len = 0;
    const char *p = str;
    
    while (*p++ != '\0')
    {
        len++;
    }
    
    if (len > 0)
    {
        HAL_UART_Transmit(&huart5, (uint8_t *)str, len, HAL_MAX_DELAY);
    }
}

/**
 * @brief  生成串口屏数值赋值指令：n0.val=数字\xff\xff\xff\0
 * @param  buf: 输出缓冲区（建议大小≥32字节）
 * @param  buf_size: 缓冲区总大小
 * @param  value: 要赋值的整数（支持正负，范围int32_t）
 * @retval 成功返回指令总长度（不含末尾'\0'，可直接用于串口发送）
 *         -1=参数错误/缓冲区过小
 *         -2=格式化失败
 */
int Generate_Nextion_n0_Val_Cmd(char *buf, size_t buf_size, int32_t value)
{
    // 输入参数合法性检查
    if (buf == NULL || buf_size < 12) // 最小需要12字节：7(前缀)+1(数字)+3(0xff)+1('\0')
    {
        return -1;
    }

    // 1. 写入前缀和数字（自动处理正负号）
    // 若需要修改控件名，只需把"n0.val"改成"n1.val"/"t0.txt"等
    int ret = snprintf(buf, buf_size, "n0.val=%ld", (long)value);
    
    // 检查格式化是否成功，且预留足够空间放3个0xff和1个'\0'
    if (ret < 0 || (size_t)ret >= buf_size - 4)
    {
        return -2;
    }

    size_t cmd_len = (size_t)ret;

    // 2. 追加Nextion指令固定结尾：三个0xFF字节
    buf[cmd_len++] = 0xFF;
    buf[cmd_len++] = 0xFF;
    buf[cmd_len++] = 0xFF;

    // 3. 添加字符串结束符（方便调试打印，不影响串口发送）
    buf[cmd_len] = '\0';

    // 返回指令实际长度（串口发送时直接用这个长度）
    return (int)cmd_len;
}

int Generate_Nextion_n1_Val_Cmd(char *buf, size_t buf_size, int32_t value)
{
    // 输入参数合法性检查
    if (buf == NULL || buf_size < 12) // 最小需要12字节：7(前缀)+1(数字)+3(0xff)+1('\0')
    {
        return -1;
    }

    // 1. 写入前缀和数字（自动处理正负号）
    // 若需要修改控件名，只需把"n0.val"改成"n1.val"/"t0.txt"等
    int ret = snprintf(buf, buf_size, "n1.val=%ld", (long)value);
    
    // 检查格式化是否成功，且预留足够空间放3个0xff和1个'\0'
    if (ret < 0 || (size_t)ret >= buf_size - 4)
    {
        return -2;
    }

    size_t cmd_len = (size_t)ret;

    // 2. 追加Nextion指令固定结尾：三个0xFF字节
    buf[cmd_len++] = 0xFF;
    buf[cmd_len++] = 0xFF;
    buf[cmd_len++] = 0xFF;

    // 3. 添加字符串结束符（方便调试打印，不影响串口发送）
    buf[cmd_len] = '\0';

    // 返回指令实际长度（串口发送时直接用这个长度）
    return (int)cmd_len;
}

