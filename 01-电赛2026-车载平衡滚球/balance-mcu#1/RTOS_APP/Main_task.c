#include "Main_task.h"

/* ========== 全局状态变量 ========== */
CompetitionState comp_state = STATE_IDLE;
uint8_t selected_mode = 0;
bool ball_control_enable = false;
float target_ball_pos = 0.0f;       // 0=中心O, ±50=±5cm
float dbg_target_angle = 0.0f;

/* ========== 外部函数声明 ========== */
extern void uart_send_chassis_cmd(uint8_t cmd);
extern uint8_t uart_get_chassis_rsp(void);
extern bool key_start_pressed(void);

/*
 * Main_task - 比赛流程控制（状态机）
 * 优先级：中, 周期：50ms
 */
void Main_task(void)
{
    static uint8_t last_mode = 0;

    while (1)
    {
        switch (comp_state)
        {

        /* ============================================================
         * 待机状态：按键选mode，等待启动
         * ============================================================ */
        case STATE_IDLE:
        {
            // 按键切换mode
            if (selected_mode != last_mode)
            {
                // TODO: 根据mode预设球控制参数
                switch (selected_mode)
                {
                case MODE_CIRCLE:
                    ball_control_enable = false;     // 要求2：不控球
                    break;
                case MODE_AB:
                    target_ball_pos = 0.0f;          // 要求4：球在中心O
                    ball_control_enable = true;
                    break;
                case MODE_CIRCLE_BALL:
                    target_ball_pos = 0.0f;          // 要求5/6：球在O或指定位置
                    ball_control_enable = true;
                    break;
                case MODE_TASK3:
                    ball_control_enable = false;     // 等待按键5触发
                    break;
                case MODE_OTHER:
                    ball_control_enable = false;
                    break;
                }
                last_mode = selected_mode;
            }

            // 启动键按下 → 执行对应任务
            if (key_start_pressed() && selected_mode != 0)
            {
                if (selected_mode == MODE_TASK3)
                {
                    Serial_SendString("FOUR");
                    ball_control_enable = true;
                    // 任务3不涉及底盘运动, 保持在待机状态
                }
                else
                {
                    // 发指令给底盘MCU
                    if (selected_mode == MODE_CIRCLE)
                    {
                        Serial_SendString("START");
                        uart_send_chassis_cmd(CHASSIS_CMD_START_CIRCLE);        // 快速
                    }
                    else if (selected_mode == MODE_CIRCLE_BALL)
                    {                               
                        uart_send_chassis_cmd(CHASSIS_CMD_START_CIRCLE_SLOW);   // 慢速
                    }
                    else if (selected_mode == MODE_AB)
                    {
                        uart_send_chassis_cmd(CHASSIS_CMD_START_AB);
                    }
                    else if (selected_mode == MODE_OTHER)
                    {
                        uart_send_chassis_cmd(CHASSIS_CMD_START_OTHER);
                    }

                    comp_state = STATE_RUNNING;
                }
            }
            break;
        }

        /* ============================================================
         * 运行状态：底盘在跑，球控制在后台任务中运行
         * ============================================================ */
        case STATE_RUNNING:
        {
            // 底盘到位（底盘MCU也会发超时信号）
            uint8_t rsp = uart_get_chassis_rsp();
            if (rsp == CHASSIS_RSP_DONE || rsp == CHASSIS_RSP_TIMEOUT)
            {
                if (selected_mode == MODE_CIRCLE)
                    Serial_SendString("FINISH");
                comp_state = STATE_DONE;
            }
            break;
        }

        /* ============================================================
         * 完成状态：显示结果，等待按键回待机
         * ============================================================ */
        case STATE_DONE:
        {
            if (key_start_pressed())
            {
                selected_mode = 0;
                last_mode = 0;
                comp_state = STATE_IDLE;
            }
            break;
        }

        default:
            comp_state = STATE_IDLE;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
