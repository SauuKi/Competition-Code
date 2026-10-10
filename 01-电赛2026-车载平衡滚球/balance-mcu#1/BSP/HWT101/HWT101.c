#include "HWT101.h"

// 声明HAL库生成的UART句柄（必须添加，huart2在main.c中定义）
extern UART_HandleTypeDef huart2;

__IO bool HWT101_rxFrameFlag = false;
__IO uint8_t HWT101_rxCmd[HWT101_FIFO_SIZE] = {0};   // DMA接收缓冲区

/*--------------命令集（完全不变）--------------*/
static const uint8_t unlock_reg[] = {0xFF,0xAA,0x69,0x88,0xB5};
static const uint8_t reset_Z[]    = {0xFF,0xAA,0x76,0x00,0x00};
static const uint8_t set_1000Hz[]  = {0xFF,0xAA,0x03,0x0D,0x00};
static const uint8_t set_baud[]   = {0xFF,0xAA,0x04,0x07,0x00};
static const uint8_t save_cfg[]   = {0xFF,0xAA,0x00,0x00,0x00};
static const uint8_t restart[]    = {0xFF,0xAA,0x00,0xFF,0x00};
static const uint8_t calsw_in[]   = {0xFF,0xAA,0x01,0x01,0x00};
static const uint8_t calsw_out[]  = {0xFF,0xAA,0x01,0x00,0x00};

/*--------------发送函数改写（阻塞式，和原逻辑一致）--------------*/
static void SendArray(const uint8_t *a, uint16_t len)
{
    // 替换原标准库逐字节发送，HAL阻塞发送超时设为最大
    HAL_UART_Transmit(&huart2, a, len, HAL_MAX_DELAY);
    
    // 原代码末尾发送回车换行，必须保留
    uint8_t crlf[] = {0x0D, 0x0A};
    HAL_UART_Transmit(&huart2, crlf, 2, HAL_MAX_DELAY);
    
    // 原代码200ms延迟，替换为HAL库毫秒级延迟（更准确）
    delay_us_timer(2000);
}

/*--------------陀螺仪配置命令（仅替换延迟函数）--------------*/
void HWT_Settings(void)
{
    SendArray(unlock_reg, 5);
    SendArray(reset_Z,    5);
    SendArray(set_1000Hz,  5);
    SendArray(set_baud,   5);
    SendArray(save_cfg,   5);
    SendArray(restart,    5);
}

void HWT_CALSW_IN(void)  { SendArray(calsw_in,  5); }
void HWT_CALSW_OUT(void) { SendArray(calsw_out, 5); }

/*--------------板级初始化（简化，复用CubeMX生成的代码）--------------*/
void HWT101_board_init(void)
{
//    // 手动开启IDLE中断（CubeMX无法自动配置）
//    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    
    // 替换为官方空闲中断DMA接收函数
    // 当接收到一帧数据（空闲中断触发）时，会自动调用HAL_UARTEx_RxEventCallback
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)HWT101_rxCmd, HWT101_FIFO_SIZE);
	__HAL_UART_DISABLE_IT(&huart2, UART_IT_ERR);  // 关闭错误中断（溢出中断）
    
//    // 执行陀螺仪初始化配置
    HWT_Settings();
}

/*--------------数据解析（完全不变，和库无关）--------------*/
float Yaw=0;

void MPUData_processing(void)
{
    // 原解析逻辑完全保留，注意HWT101的Yaw数据是小端模式
    Yaw = (int16_t)(((uint16_t)HWT101_rxCmd[18] << 8) | HWT101_rxCmd[17]) / 32768.0f * 180.0f;
}
//void MPUData_processing(void)
//{
//    // 原解析逻辑完全保留，注意HWT101的Yaw数据是小端模式
//    Yaw = (int16_t)(((uint16_t)HWT101_rxCmd[7] << 8) | HWT101_rxCmd[6]) / 32768.0f * 180.0f;
//}

/*--------------原HWT101函数（仅替换USART开关宏）--------------*/
void HWT101(void)
{
    __HAL_UART_ENABLE(&huart2);   // 替换原USART_Cmd(ENABLE)
    HAL_Delay(20);                // 原代码20ms延迟
    MPUData_processing();
    __HAL_UART_DISABLE(&huart2);  // 替换原USART_Cmd(DISABLE)
}

///*--------------USART2中断服务函数（核心改写）--------------*/
//// 注意：需要删除CubeMX自动生成的空USART2_IRQHandler函数，替换为以下内容
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//    if (huart->Instance == USART2)
//    {
//        // 标记一帧数据接收完成
//        HWT101_rxFrameFlag = true;
//        
//        // 重启空闲中断DMA接收，准备下一帧
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t*)HWT101_rxCmd, HWT101_FIFO_SIZE);
//    }
//}
