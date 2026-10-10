#include "Output_task.h"
#include "Main_task.h"

static const char *mode_names[] = {
    [0] = "IDLE",
    [MODE_CIRCLE]      = "CIRCLE  ",
    [MODE_AB]          = "A->B    ",
    [MODE_CIRCLE_BALL] = "CIR+BALL",
    [MODE_TASK3]       = "TASK3   ",
    [MODE_OTHER]       = "OTHER   ",
};

static const char *state_names[] = {
    [STATE_IDLE]   = "READY",
    [STATE_RUNNING]= "RUN  ",
    [STATE_DONE]   = "DONE ",
};

/*
 * Output_task - OLED显示
 * 优先级：低, 周期：100ms
 */
void Output_task(void)
{
    osDelay(100);

    while (1)
    {
        /* 第1行：状态 */
        OLED_ShowString_My(1, 1, "ST:", OLED_8X16);
        OLED_ShowString_My(1, 5, (char *)state_names[comp_state], OLED_8X16);

        /* 第2行：模式 */
        OLED_ShowString_My(2, 1, "MD:", OLED_8X16);
        OLED_ShowString_My(2, 5, (char *)mode_names[selected_mode], OLED_8X16);

        /* 第3行：球控制 + 偏移量选择 */
        OLED_ShowString_My(3, 1, "BL:", OLED_8X16);
        OLED_ShowString_My(3, 5, ball_control_enable ? "ON " : "OFF", OLED_8X16);
        if (offset_select_mode) {
            OLED_ShowString_My(3, 9, "OF:", OLED_8X16);
            OLED_ShowSignedNum_My(3, 12, offset_value, 3, OLED_8X16);
        }

        /* 第4行：球坐标差(mm) + 球速度(mm/s) */
        // OLED_ShowString_My(4, 1, "TP:", OLED_8X16);
        // OLED_ShowSignedNum_My(4, 5, (int16_t)(target_ball_pos * 10), 4, OLED_8X16);
        // OLED_ShowString_My(4, 10, "mm", OLED_8X16);
        OLED_ShowString_My(4, 1, "D:", OLED_8X16);
        OLED_ShowSignedNum_My(4, 3, parsed_data.dx, 4, OLED_8X16);
        OLED_ShowString_My(4, 8, "V:", OLED_8X16);
        OLED_ShowFloatNum_My(4, 10, parsed_data.ball_v, 3, 2, OLED_8X16);

        OLED_Update();
        osDelay(100);
    }
}
