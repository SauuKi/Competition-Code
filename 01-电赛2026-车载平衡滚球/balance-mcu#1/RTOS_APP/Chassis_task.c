#include "Chassis_task.h"
#include "Main_task.h"
#include "TJC.h"
#include "Controls.h"       // 调试用: beam_set_angle, Back_Zero

/* ========== 底盘UART通信缓冲区 ========== */
static uint8_t chassis_rx_buf = 0;     // 接收缓冲区

/* ========== 发送指令给底盘MCU ========== */
void uart_send_chassis_cmd(uint8_t cmd)
{
    USART5_SendByte(cmd);
}

/* ========== 获取底盘MCU应答 ========== */
uint8_t uart_get_chassis_rsp(void)
{
    uint8_t rsp = chassis_rx_buf;
    chassis_rx_buf = 0;  // 读后清零
    return rsp;
}

/* ========== 底盘应答写入（由UART RX中断/任务调用） ========== */
void chassis_rsp_set(uint8_t rsp)
{
    chassis_rx_buf = rsp;
}

/* ========== 按键5标志 ========== */
static uint8_t key5_pressed = 0;

/* ========== 偏移量选择模式 ========== */
bool    offset_select_mode = false;   // 是否处于偏移量选择模式
int8_t  offset_value = 0;             // 当前选择的偏移量 (-12 ~ 12)

bool key_start_pressed(void)
{
    if (key5_pressed)
    {
        key5_pressed = 0;
        return true;
    }
    return false;
}

/*
 * Chassis_task - 按键选mode + UART底盘通信
 * 优先级：高, 周期：10ms
 */
void Chassis_task(void)
{
    uint8_t keynum = 0;
    Set_Pos_Z(Z_UP,34400,500,0);
    osDelay(2000);
    Emm_V5_Reset_CurPos_To_Zero(5); 
    Serial_SendString("ONE");

    while (1)
    {
        keynum = Key_GetNum();

        // /* ===== 正式比赛: 按键选模式 ===== */
        // if (comp_state == STATE_IDLE)
        // {
        //     if (keynum == 1)      selected_mode = MODE_CIRCLE;
        //     else if (keynum == 2) selected_mode = MODE_AB;
        //     else if (keynum == 3) selected_mode = MODE_CIRCLE_BALL;
        //     else if (keynum == 4) {
        //         Emm_V5_Reset_CurPos_To_Zero(5);   // 清零脉冲计数
        //         Serial_SendString("ONE");         // 通知树莓派执行标定程序
        //     }
        // }

        /* ===== 偏移量选择模式：按键5发送后退出 ===== */
        if (offset_select_mode)
        {
            if (keynum == 5)
            {
                char buf[16];
                sprintf(buf, "%d", offset_value);
                Serial_SendString(buf);
                offset_select_mode = false;
                osDelay(500);
                ball_control_enable = true;
            }
        }
        else
        {
            if (keynum == 5) key5_pressed = 1;
        }

        /* ===== 调试模式: 按键直接控制 ===== */
         if (keynum == 1){
            Serial_SendString("TWO");
            osDelay(500);
            ball_control_enable = true;   // 开启球控（不再翻转）
            target_ball_pos = 0.0f;
        }
         else if (keynum == 2) {
            ball_control_enable = false;
            Back_Zero(5, 500, 0);
            osDelay(1000);
            Serial_SendString("ONE");
        }
         else if (keynum == 3) {
            /* 循环切换: CIRCLE -> CIR+BALL -> A->B -> TASK3 -> OTHER -> CIRCLE */
            if (selected_mode == MODE_CIRCLE)
                selected_mode = MODE_CIRCLE_BALL;
            else if (selected_mode == MODE_CIRCLE_BALL)
                selected_mode = MODE_AB;
            else if (selected_mode == MODE_AB)
                selected_mode = MODE_TASK3;
            else if (selected_mode == MODE_TASK3)
                selected_mode = MODE_OTHER;
            else
                selected_mode = MODE_CIRCLE;
         }
         else if (keynum == 4) {
            /* 偏移量选择：-12 → -11 → ... → 12 → -12 循环 */
            if (!offset_select_mode) {
                offset_select_mode = true;
                offset_value = -12;
            } else {
                offset_value++;
                if (offset_value > 12)
                    offset_value = -12;
            }
         }
        osDelay(10);
    }
}
