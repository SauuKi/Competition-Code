# 2026 广东省大学生工程实践与创新能力大赛 —— 智能物流搬运

**成绩**：广东省二等奖　|　**我的角色**：电控，负责全部单片机代码

## 系统构成

STM32F407 + HAL + STM32CubeMX + FreeRTOS（CMSIS-OS v2）。一台物料搬运机器人：差速底盘 + 齿条升降 / 伸缩机构（步进）+ 舵机夹爪 + 云台舵机 + 云台电机（CAN）+ 视觉模块 + 串口屏交互。

## 任务划分（`Core/Src/freertos.c`）

| 任务 | 作用 |
| --- | --- |
| `Main_Task` | 比赛流程与模式管理（按键选择、启动与状态切换） |
| `Chassis_Task` | 机构回零、齿条绝对位移、夹爪高度控制、底盘运动 |
| `Output_Task` | 视觉数据处理与输出执行（OPS 模块数据换算为角度） |
| `Tower_Task` | 云台舵机角度控制与 IMU 姿态更新 |
| `UART_RX_Task` | 串口数据接收与解析 |

## 板级驱动（`BSP/`）

| 模块 | 说明 |
| --- | --- |
| `Emm_v5/` | 步进电机（齿条 / 升降机构）协议驱动，`Controls.c` 中封装运动参数、同步运动与绝对位移 |
| `G6220/` + `hcan` | 达妙 G6220 云台电机，CAN 总线控制（使能、失能、零点重置、位置与速度模式） |
| `HWT101/` | 陀螺仪，提供 Yaw 角度反馈 |
| `Servo/` | 舵机（夹爪 / 云台），带角度插值运动 |
| `OPS/` | 视觉模块串口驱动（定长帧 + CRC 校验） |
| `Serial/` | 调试与通信串口 |
| `TJC/`、`OLED/`、`KEY/`、`LED/` | 串口屏、OLED 显示、按键、指示灯 |
| `Support/CRC8_CRC16.c` | 通信校验 |

## 目录说明（`firmware/` 为完整工程）

| 路径 | 内容 |
| --- | --- |
| `Core/` | CubeMX 生成的外设初始化（CAN、DMA、TIM、USART、GPIO）与 FreeRTOS 任务创建 |
| `BSP/` | 板级驱动（见上表） |
| `RTOS_APP/` | 任务实现代码 |
| `System/` | 延时与系统层 |
| `Drivers/`、`Middlewares/` | STM32F4 HAL 与 FreeRTOS（厂商代码） |
| `MDK-ARM/` | Keil MDK5 工程文件与启动文件 |
| `工训test.ioc` | STM32CubeMX 配置 |

`images/` 是比赛现场与作品照片。

## 编译说明

Keil MDK5 打开 `firmware/MDK-ARM/工训test.uvprojx`，工程自带厂商库，安装好 STM32F4 器件包后可直接编译；下载用 ST-Link，接口 SWD。

## 说明

STM32 HAL 库、FreeRTOS 内核与 Emm_V5 步进协议驱动为第三方或基于第三方修改；任务划分、通信协议、外设应用层驱动与机构动作逻辑为自行实现。
