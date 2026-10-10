# 2025 广东省工科大学生实验综合技能竞赛 —— 反恐排爆救援机器人

**成绩**：广东省二等奖（广东省教育厅，2025 年 12 月，证书编号 250201157）　|　**我的角色**：电控，负责全部单片机代码

## 机器人构成

差速底盘（4 路电机 + 编码器）+ 机械臂（步进齿条 + 舵机夹爪）+ IMU + OLED 状态显示，通过串口接收视觉模块数据，完成场地定位与目标抓取动作。

## 目录说明（`firmware/` 为完整工程）

| 路径 | 内容 |
| --- | --- |
| `User/main.c` | 4 个 FreeRTOS 任务（底盘运动、机械臂与夹爪动作、视觉数据处理、状态显示）与任务间协作 |
| `User/FreeRTOSConfig.h`、`User/stm32f4xx_it.c` | FreeRTOS 配置与中断处理 |
| `hardware/DSP/` | 自写驱动：`Encoder` 编码器测速、`Motor` 电机控制、`PID`、`PWM`、`Serial` 视觉数据解包、`Servo` 夹爪舵机、`HWT101` IMU、`Key`、`LED` |
| `hardware/Emm_V5/`、`hardware/OLED/`、`hardware/SYSTEM/`、`hardware/TFT/` | 步进电机协议驱动、显示驱动、系统层 |
| `FreeRTOS/`、`Libraries/` | FreeRTOS 内核与 STM32F4 标准库（厂商代码） |
| `Project/RVMDK（uv5）/` | Keil MDK5 工程文件 |

## 实现要点

- 用 4 个 FreeRTOS 任务 + 队列 / 临界区分工：底盘运动控制、机械臂动作、视觉数据解析、OLED 显示。
- 底盘支持单速、定距、差速转弯三种参数化运动模式，配合编码器反馈闭环（`Read_Speed`、`motor_single_speed_params`、`motor_fixed_distance_params`、`motor_diff_turn_params`）。
- 机械臂由步进电机驱动齿条升降，舵机控制夹爪开合，包含回零、到位判断（`check_all_reached_filter`）、慢速转动（`rotate_angle_slow`）等处理。
- 与视觉模块通过串口通信，自行解包视觉帧（`Serial_Vision_Unpack`）。

## 编译说明

Keil MDK5 打开 `firmware/Project/RVMDK（uv5）/Fire_FreeRTOS.uvprojx`，工程自带厂商库，安装好 STM32F4 器件包后可直接编译；下载用 ST-Link，接口 SWD。

## 说明

`User/main.c` 顶部的注释头保留了工程模板的上游作者信息，任务框架在此基础上改写；FreeRTOS 内核、STM32 标准库、OLED 显示驱动与 Emm_V5 步进协议驱动为第三方或基于第三方修改。
