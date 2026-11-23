# CQUelectest2026 · 两轮自平衡小车

2026 年大学生电子设计竞赛重庆大学校级比赛 **B 题：两轮自平衡小车**的嵌入式控制工程。

主控使用 STM32F407ZGT6，软件基于 STM32CubeMX + FreeRTOS + Keil MDK。当前仓库包含 ATK-MS901M 九轴姿态采集（USART3 + DMA）、三环 PID、差速控制到电机 PWM 抽象的控制骨架。

> **2026-11 架构变更**：IMU 已从 MPU6050（I2C1 + TIM7）换成 **ATK-MS901M 九轴（USART3 921600 + DMA1 Stream1 + `IMUdatadoneBinarySem`）**。PID 任务由 DMA 完成信号量驱动，不再使用 `osDelayUntil()`；新增 `controlTask` 骨架。下文涉及 MPU6050、TIM7、IMUtask 的段落为历史描述。

> **转向架构（2026-11）**：编码器只提供共模位置/速度；yaw 使用 ATK-MS901M 九轴相对角（位置模式首次进入时归零）；位置模式 Controlqueue `value2` = 目标 yaw 角，单位 0.1°；速度模式 `value2` = 目标 yaw rate，单位 1°/s，不使用 yaw 零点；中环位置模式做 yaw 角度 PD → 目标 yaw rate 限幅，速度模式直接限幅；内环 yaw rate PI → 差模 PWM；**不再使用编码器差模，也不再使用 yaw 2 状态 Kalman**。

> **单位变更（2026-11）**：PID 内部长度单位已改为 **m**，速度单位为 **m/s**；角度/角速度仍为 deg、deg/s。`Encoder_Update()` 在读取时用轮径和 `ENCODER_COUNTS_PER_REV=385×4` 换算；Controlqueue 的 `value1` 仍为 int16，按 **mm / mm·s⁻¹** 编码，进入 PID 后换算为 m / m·s⁻¹。

> **controlTask（2026-11）**：已实现五个顶层流程——无线调参 DEBUG、原地停止、路径测试、巡线对齐、四次 zigzag；KEY0（PE4）轮询进入/退出 DEBUG，KEY_UP（PA0）循环运行状态；TIM11_CH1/PF7 同时驱动蜂鸣器与 LED，频率 2500→50 Hz 扫描 1 s。详见 [`prj/elet2026/tasks/CONTROL_TASK.md`](prj/elet2026/tasks/CONTROL_TASK.md)。

## 目录

- [1. 项目概览](#1-项目概览)
- [2. 快速开始](#2-快速开始)
- [3. 仓库结构](#3-仓库结构)
- [4. 系统架构](#4-系统架构)
- [5. FreeRTOS 任务与调度](#5-freertos-任务与调度)
- [6. 控制链路](#6-控制链路)
- [7. Controlqueue 协议](#7-controlqueue-协议)
- [8. 单位、反馈与 DWT 时间基准](#8-单位反馈与-dwt-时间基准)
- [9. 当前状态与待办](#9-当前状态与待办)
- [10. 调试与安全](#10-调试与安全)
- [11. 相关文档与 Git](#11-相关文档与-git)

## 1. 项目概览

| 项目 | 内容 |
| --- | --- |
| 赛题 | 2026 年大学生电子设计竞赛重庆大学校级比赛 B 题 |
| 对象 | 两轮自平衡自动行驶小车 |
| 主控 | STM32F407ZGT6，168 MHz |
| RTOS | FreeRTOS / CMSIS-RTOS2 |
| 姿态传感器 | ATK-MS901M 九轴，USART3（PB10/PB11）921600，DMA1 Stream1 28 B/周期 |
| 执行器 | 左右直流电机，TIM2 PWM（20 kHz）+ 方向 GPIO + STBY |
| 反馈 | TIM3/TIM4 正交编码器（共模位置/速度）；yaw 由 ATK-MS901M 九轴提供 |
| 按键 | KEY_UP=PA0（运行状态切换）；KEY0=PE4（GPIO 轮询，HOLD/STOP ↔ DEBUG）；START 留待下一版 |
| 提示 | TIM11_CH1=PF7/BEEP_LED，蜂鸣器 + LED，2500→50 Hz 频率扫描 |
| 控制结构 | controlTask 五流程 → Controlqueue → PID 位置 10 Hz / 速度 50 Hz / 姿态 250 Hz |
| 通信 | `Controlqueue` 48 位命令，50 Hz 轮询；`dataToLCDQueue` 80 B，50 Hz |
| 调试串口 | USART6：PG14 TX / PC7 RX，115200 8N1；TX DMA2 Stream6 发 86 B 遥测，RX DMA2 Stream1 收 64 B 调参帧 |
| 构建 | Keil MDK-ARM 5 / `prj/elet2026/MDK-ARM/elet2026.uvprojx` |

赛题禁止使用图像传感器/摄像头；本工程不使用视觉，姿态与运动反馈分别来自 ATK-MS901M 和编码器。完整题目原文与逐条释义见 [`target/README.md`](target/README.md)。

## 2. 快速开始

### 2.1 打开工程

使用 Keil MDK-ARM 打开：

```text
prj/elet2026/MDK-ARM/elet2026.uvprojx
```

目标器件为 `STM32F407ZGTx`。

### 2.2 任务源文件与 Include Path（已配置）

`elet2026.uvprojx` 已包含 `Application/User/Tasks` 组，并在 `Options for Target -> C/C++ -> Include Paths` 中加入了 `../tasks`。若用 CubeMX 重新生成工程后丢失，请重新加入：

```text
prj/elet2026/tasks/dwt_time.c
prj/elet2026/tasks/imu_task.c
prj/elet2026/tasks/pid_task.c
prj/elet2026/tasks/control_task.c
prj/elet2026/tasks/encoder.c
prj/elet2026/tasks/line_sensor.c
prj/elet2026/tasks/display_task.c
prj/elet2026/tasks/oled.c
```

所有可调参数集中在：

```text
prj/elet2026/tasks/robot_config.h
```

`robot_config.h` 通过 `../tasks` Include Path 被各任务头文件包含，不需要单独加入 Keil 的 `Files` 列表。

```text
../tasks
```

现有的 `../Core/Inc`、HAL、CMSIS、FreeRTOS 包含路径保持不变。

### 2.3 编译与下载

1. 确认 `Application/User/Tasks` 组中有上述源文件；
2. 确认 Include Paths 中有 `../tasks`；
3. Rebuild；
4. 下载到 STM32F407ZGT6；
5. 上电后保持小车静止约 1 秒，等待 MPU6050 陀螺零偏校准完成；
6. PID 任务会等待 IMU 首帧，之后进入 `PID_STATE_STOP`，等待 `Controlqueue` 命令。

> 第一次上电调试建议拆下轮子或把车架空，先验证 IMU 姿态、PWM 方向和编码器符号，再落地闭环。

## 3. 仓库结构

```text
.
├─ README.md                        本文件
├─ .gitignore                       排除 datasheet/ 与构建产物
├─ target/                          赛题资料
│  ├─ README.md                     题目原文、翻译与逐条释义
│  └─ 2026年...B题.pdf              原始赛题
├─ prj/elet2026/                    STM32CubeMX / Keil 工程
│  ├─ Core/
│  │  ├─ Inc/                       main.h、FreeRTOSConfig.h 等
│  │  └─ Src/                       main.c、freertos.c、i2c.c、tim.c 等
│  ├─ Drivers/                      STM32F4 HAL / CMSIS
│  ├─ Middlewares/                  FreeRTOS
│  ├─ MDK-ARM/
│  │  ├─ elet2026.uvprojx           Keil 工程
│  │  ├─ elet2026.uvoptx
│  │  └─ startup_stm32f407xx.s
│  ├─ tasks/
│  │  ├─ dwt_time.c/.h              DWT CYCCNT 共享时间基准
│  │  ├─ imu_task.c/.h              ATK-MS901M USART3 DMA 帧解析
│  │  ├─ pid_task.c/.h              三环控制、PID 状态机、Controlqueue
│  │  ├─ control_task.c/.h          controlTask 五流程/按键/巡线/蜂鸣器
│  │  ├─ line_sensor.c/.h           前/后 8 路光电读取与掩码
│  │  ├─ encoder.c/.h               TIM3/TIM4 共模编码器
│  │  ├─ display_task.c/.h          OLED 调试显示
│  │  ├─ oled.c/.h                  SSD1306/SSD1315 I2C DMA
│  │  ├─ CONTROL_TASK.md            ★ 状态机/巡线/对齐详细说明
│  │  └─ README.md                  控制架构与详细调度说明
│  ├─ elet2026.ioc                  CubeMX 配置
│  └─ .mxproject
└─ datasheet/                       本地资料，已被 .gitignore 排除
```

`datasheet/` 约 1.6 GB，未提交到 Git；仓库中保留源码和必要工程文件。

## 4. 系统架构

### 4.1 控制数据流

> 以下为旧“差模位置/速度”控制链历史描述。当前转向架构见顶部
> “2026-11 转向架构”：编码器只共模；位置模式 `value2` = 0.1° 目标 yaw 角
> （相对首次进入位置模式的零点），速度模式 `value2` = 1°/s 目标 yaw rate；
> 中环位置模式做 yaw 角度 PD → 目标 yaw rate 限幅，速度模式直接限幅；
> 内环 yaw rate PI → 差模 PWM。

```text
                 ┌─────────────────────────────┐
Controlqueue ───►│ PID 状态机（STOP/POSITION/SPEED）│
                 └──────────────┬──────────────┘
                                │ 目标位置/速度/差模量
                                ▼
       ┌──────────────────────────────────────────┐
       │ 位置外环 10 Hz（仅 POSITION）             │
       │ 速度中环 50 Hz                            │
       │ 姿态/yaw rate 内环 250 Hz                 │
       └──────────────────┬───────────────────────┘
                          │ 左右 PWM
                          ▼
                    PIDTask_WriteOutput()
                          │
                          ▼
                    SETPWM()  ← TIM2 PWM + 方向 GPIO
```

### 4.2 IMU 数据流

> 以下为旧 MPU6050 + TIM7 + I2C DMA 方案，仅作历史参考。当前实现为
> ATK-MS901M 九轴：USART3 921600 → DMA1 Stream1（28 B）→
> `IMUdatadoneBinarySem` → PID 的 `IMU_Parse()`，详见 `tasks/README.md` 顶部。

```text
TIM7 2 ms
  → IMU_OnTimer7()
  → HAL_I2C_Mem_Read_DMA()（MPU6050，I2C1）
  → HAL_I2C_MemRxCpltCallback()
  → 释放 IMUtaskBinarySem
  → IMUTask：Kalman_Update()
  → 写入 s_kalman_out
  → kalmaneddataQueue（深度 1，按值存放 KalmanData_t）
  → PIDTask_UpdateIMU()
```

- MPU6050 配置：500 Hz、DLPF 94/98 Hz、±1000 dps、±2 g。
- `IMU_Init()` 在初始化任务中执行约 1 s 静止陀螺零偏校准（pitch/yaw 两轴）；校准失败不阻止启动，pitch 由卡尔曼在线估计，yaw 由 PID task 内的 2 状态 Kalman 在线修正残余零偏。
- 首次上电保持静止约 1 s；`IMU_IsReady()` 在没有首帧时会让 PID 任务等待。
- IMU 输出 `pitch / pitch_rate / yaw_rate`：`pitch/pitch_rate` 用于共模平衡，`yaw_rate` 用于差模 yaw rate 内环；差模目标仍来自编码器位置外环或 Controlqueue。

### 4.3 编码器反馈

编码器由 `tasks/encoder.c` 读取：

```text
右编码器：TIM3 CH1/CH2 = PA6/PA7
左编码器：TIM4 CH1/CH2 = PD12/PD13
```

- `Encoder_Init()`：启动 TIM3/TIM4 编码器接口，CNT 初始值设为
  **6554**（16 位总量 65536 的约 1/10）；
- `Encoder_Update(&position_m, &speed_mps)`：PID 中环 50 Hz 每 20 ms 调用一次，
  用 `int16_t` 差值累计，自动处理 16 位回绕，并转换为 m / m·s⁻¹；
- 只计算共模量，并由 PID 内部写入 `s_pid_input.current_position/current_speed`；
  PID 不向其他任务暴露反馈写入接口。

- `ENCODER_LEFT_SIGN` / `ENCODER_RIGHT_SIGN` 用于实车方向修正；
- 编码器差模量不再参与 yaw 控制。

> **四倍频与米制**：TIM3/TIM4 使用 `TIM_ENCODERMODE_TI12`，硬件 4 倍频。
> `ENCODER_COUNTS_PER_REV = 385 × 4 = 1540`，轮径为 `WHEEL_DIAMETER_M`；
> `Encoder_Update()` 用 `π × 轮径 / 1540` 把共模计数转换为 m / m·s⁻¹。
> 轮距 `WHEEL_TRACK_M` 当前预留给差速/里程计，共模控制暂未使用。

位置模式由 `PositionController_Start()` 锁存共模起点，因此位置反馈应传**累计值**，不要传增量。

### 4.4 八路巡线传感器

前 8 路 F1-F8、后 8 路 B1-B8 全部配置为 GPIO 输入，由 `tasks/line_sensor.c` 读取：

```text
前：F1=PE8   F2=PD15  F3=PD14  F4=PD9
    F5=PF4   F6=PF2   F7=PF0   F8=PB15
后：B1=PG8   B2=PG6   B3=PC8   B4=PC12
    B5=PC11  B6=PG15  B7=PD6   B8=PC6
```

- `LineSensor_ReadLevels()` 返回 16 路原始电平（0/1）；
- `ControlTask_GetFrontSensorLevels()` / `ControlTask_GetBackSensorLevels()` 返回 controlTask 内保存的前/后 8 路数组；
- `ControlTask_GetFrontLineMask()` / `ControlTask_GetBackLineMask()` 返回“检测到线”的位掩码，bit0 = x1；
- 默认 `LINE_SENSOR_ACTIVE_LEVEL = 0`（黑线低电平）；若实测相反，只改 `tasks/robot_config.h`；
- controlTask 周期为 20 ms，因此巡线电平刷新率为 50 Hz；

## 5. FreeRTOS 任务与调度

### 5.1 任务

| 任务 | 入口 | 优先级 | 周期/触发 | 栈 | 职责 |
| --- | --- | --- | --- | --- | --- |
| defaultTask | `StartDefaultTask` | `osPriorityHigh4`（44） | 初始化一次后永久挂起 | 256 words | `Encoder_Init()`、`IMU_Init()` |
| PID | `StartPIDTask` | `osPriorityNormal4`（28） | `IMUdatadoneBinarySem` 驱动，约 250 Hz | 256 words | IMU 解析、三环 PID、Controlqueue、显示快照 |
| controlTask | `StartcontrolTask` | `osPriorityLow5`（13） | 20 ms（50 Hz） | 512 words | 五流程、按键、巡线、对齐、蜂鸣器 |
| displayTask | `StartdisplayTask` | `osPriorityLow`（8） | 队列事件驱动 | 256 words | 50 Hz 快照；USART6 50 Hz，OLED 按 DMA 就绪刷新（约 20~25 Hz） |

调度关系：

```text
defaultTask > PID > controlTask > displayTask
```

`defaultTask` 在初始化完成后必须永久挂起，否则会长期占用 CPU。

### 5.2 FreeRTOS 队列/信号量

| 对象 | 类型 | 深度/初值 | 用途 |
| --- | --- | --- | --- |
| `Controlqueue` | 消息队列 | 深度 1，项 6 字节 | controlTask → PID 命令 |
| `dataToLCDQueue` | 消息队列 | 深度 1，项 80 字节 | PID → displayTask 快照（含 IMU 时间戳） |
| `IMUdatadoneBinarySem` | 二值信号量 | 创建后 acquire 到 0 | USART3 RX DMA 完成通知 |

> `IMUdatadoneBinarySem` 在 `MX_FREERTOS_Init()` 创建后立即 acquire 到 0，PID 只由 USART3 RX DMA 完成中断释放后运行。

### 5.3 PID 任务时间结构

PID 基础循环标称 **4 ms（250 Hz）**，实际由 IMU USART3 RX DMA 完成信号量触发：

```c
PID_TASK_PERIOD_MS = 4U;   /* 标称 250 Hz */
```

任务循环顺序（不再调用 `osDelayUntil()`）：

```c
osSemaphoreAcquire(IMUdatadoneBinarySemHandle, osWaitForever);
IMU_Parse();                     /* 解析 DMA 快照 */
PIDTask_UpdateIMU();             /* 取最新姿态/角速度 */
PIDTask_CheckTimestampOverflow();/* DWT 32 位回绕保护 */
PIDTask_UpdateControlQueue();    /* 计数器分频，实际 50 Hz */
switch (s_pid_state) { ... }     /* 三环状态机 */
PIDTask_UpdateLoopTime(...);
PIDTask_UpdateDisplayQueue();    /* 50 Hz 快照 */
```

三环分频：

| 环 | 频率 | 周期 | 执行条件 |
| --- | --- | --- | --- |
| 内环（姿态/yaw rate） | 250 Hz | 4 ms | 每个基础 tick |
| 中环（速度） | 50 Hz | 20 ms | `tick % 5 == 0` |
| 外环（位置） | 10 Hz | 100 ms | `tick % 25 == 0` |

`s_pid_loop_counter` 是**循环执行次数计数器**，不是真实时间计时器：

- `POSITION`/`SPEED` 每执行一次状态函数增加 1；
- `tick` 范围 0~24，计到 25 回零；
- `STOP` 不运行控制环，不增加该计数器；
- 模式切换时清零，下一拍从 0 开始；
- 如果 IMU 任务抢占导致某一拍晚到，它不会补加多次。

因此实际控制 `dt` 由共享 DWT 时间戳测量，而不是假设每拍一定 4 ms。

### 5.4 Controlqueue 轮询

`PIDTask_UpdateControlQueue()` 每个基础 tick 被调用，但只有 `s_control_queue_counter == 0` 时才真正读取队列：

```text
250 Hz / 5 = 50 Hz（20 ms）
```

Controlqueue 在状态机 `switch` 之前读取：

1. 读取并更新状态/目标；
2. 同一基础 tick 的 `switch` 进入新状态；
3. 如果只是更新当前模式目标，目标在后续对应控制环生效；
4. 如果模式切换，控制环计数器清零，新状态第一拍从 `tick 0` 开始。

停止命令的检测延迟最多约 20 ms；如果安全上需要更快停止，应设计独立的紧急停止通道。

## 6. 控制链路

> 本节 6.1~6.3 为旧“差模编码器 + yaw Kalman”方案的历史描述。
> 当前实现：共模位置/速度 + 九轴相对 yaw 角度环 + 陀螺 yaw rate 内环，
> 详见 `tasks/README.md` 顶部“当前转向架构”。
> 其中的脉冲单位、`PID_YAW_DEG_PER_DIFF_PULSE` 和旧参数名均已不再使用；
> 当前参数集中在 `tasks/robot_config.h`，位置 m、速度 m/s。

### 6.1 位置模式：`PID_STATE_POSITION`

```text
共模位置 PD 10 Hz → 目标速度
                         ↓
共模速度 PD 50 Hz → 目标倾角 → 姿态 PD 250 Hz → 共模 PWM
差模位置 PD 10 Hz → 目标差模速度（脉冲/s）
                         ↓ × PID_YAW_DEG_PER_DIFF_PULSE
中环限幅 50 Hz → 目标 yaw rate（deg/s）→ yaw rate PI 250 Hz → 差模 PWM
```

- 外环：`PositionController_Update()`，`dt` 由 DWT 实测；
- 中环：`SpeedController_Update()`，差模通道把目标差模速度换算为目标 yaw rate；
- 内环：`AttitudeController_Update()`，姿态 PD + yaw rate PI；yaw rate 反馈来自 PID task 内的 2 状态 Kalman；
- 左右 PWM：

```text
PWM_L = PWM_common - PWM_diff
PWM_R = PWM_common + PWM_diff
```

### 6.2 速度模式：`PID_STATE_SPEED`

```text
Controlqueue value1 → 目标共模速度
Controlqueue value2 → 目标差模速度（脉冲/s）
          ↓ × PID_YAW_DEG_PER_DIFF_PULSE
速度 PD 50 Hz → 目标倾角 → 姿态 PD 250 Hz → 共模 PWM
目标 yaw rate 50 Hz 限幅 → yaw rate PI 250 Hz → 差模 PWM
```

### 6.3 控制器参数

所有增益和限幅集中在 `prj/elet2026/tasks/pid_task.h`：

```c
PID_POS_KP_DISTANCE
PID_POS_KD_DISTANCE
PID_POS_KP_DIFF_POSITION
PID_POS_KD_DIFF_POSITION
PID_SPEED_KP
PID_SPEED_KD
PID_ATT_KP_PITCH
PID_ATT_KD_PITCH
PID_YAW_KP
PID_YAW_KI
```

当前全部为 `0.0f`，需要实车整定。输出限幅/换算包括：

```text
PID_POS_MAX_SPEED_PULSES_S        50
PID_POS_MAX_DIFF_SPEED_PULSES_S   200
PID_SPEED_MAX_PITCH_DEG           10
PID_ATT_MAX_PWM                   4199
PID_ATT_MAX_DIFF_PWM              4199
PID_YAW_MAX_INTEGRAL              4199
PID_YAW_MAX_RATE_DPS              180
PID_YAW_DEG_PER_DIFF_PULSE        1.0  ← 必须按轮径/CPR/轮距标定
```

> `PID_YAW_DEG_PER_DIFF_PULSE` 的标定公式：`180 * D / (CPR_wheel * L)`，其中 D 为轮径、L 为完整轮距。默认 `1.0f` 只是占位，实车必须先标定。

### 6.4 电机输出

`PIDTask_WriteOutput()` 把左右有符号 PWM 转成方向 + 比较值：

```c
SETPWM(left_wheel, forward, compare);
```

- `left_wheel`：1 = 左轮，0 = 右轮；
- `forward`：1 = 正转，0 = 反转；
- `compare`：PID 输出 0~4199，`SETPWM()` 会按 TIM2 ARR 缩放。

电机驱动逻辑：

| AIN1 | AIN2 | PWM | 状态 |
| --- | --- | --- | --- |
| 1 | 0 | PWM | 正转 |
| 0 | 1 | PWM | 反转 |
| 0 | 0 | 任意 | 短路刹车 |
| 1 | 1 | 任意 | 自由停车 |

接线：

```text
左轮：LF=PD11 -> AIN1，LB=PD10 -> AIN2，LPWM=PA2/TIM2_CH3
右轮：RF=PA4  -> AIN1，RB=PA5  -> AIN2，RPWM=PA1/TIM2_CH2
STBY：PC1 -> 驱动芯片 STBY（高使能，低待机）
```

`compare=0` 时使用短路刹车。`SETPWM()` 首次调用会启动 TIM2 CH3/CH2。STOP 模式下先输出 0，再把 STBY 拉低进入待机。

TIM2 当前已配置 `Prescaler=0、Period=4199`，APB1 定时器时钟 84 MHz，对应 **20 kHz PWM**。

> USART6 当前 TX 使用 **PG14**、RX 使用 **PC7**；PC6 已作为 B8 巡线输入。
> 如果无线模块实际接在 PC6（USART6 默认 TX），需要重新分配 B8 或改接 PG14。


## 7. Controlqueue 协议

`Controlqueue` 由 CubeMX 创建：

```text
名称：Controlqueue
深度：1
队列项：6 字节（48 位）
句柄：ControlqueueHandle
```

### 7.1 位定义

按 48 位消息从 MSB 开始：

| 位（MSB 起） | LSB 编号 | 含义 |
| --- | --- | --- |
| 1~2 | 47~46 | 模式 |
| 3~16 | 45~32 | 保留 |
| 17~32 | 31~16 | `value1`，`int16` |
| 33~48 | 15~0 | `value2`，`int16` |

模式：

| 模式 | 含义 | value1 | value2 |
| --- | --- | --- | --- |
| `00` | 停止 | 忽略 | 忽略 |
| `01` | 位置 | 目标共模位移（mm） | 目标 yaw 角（0.1°，相对首次进入位置模式的零点） |
| `10` | 速度 | 目标共模速度（mm/s） | 目标 yaw rate（1°/s，不使用 yaw 零点） |
| `11` | 姿态（只跑内环） | 目标 Pitch 角（0.1°） | 目标 yaw rate（1°/s） |

> 姿态模式（11）只运行 250 Hz 内环，不运行位置外环、速度中环和 yaw 角度环；
> 目标 Pitch 限幅 ±30°，目标 yaw rate 限幅 ±180°/s，用于直接调内环参数。

发送示例：

```c
uint64_t message = ((uint64_t)0x1U << 46)
                 | ((uint64_t)(uint16_t)target_position_mm << 16)
                 | (uint64_t)(uint16_t)target_yaw_value;  /* 位置 mm；速度 mm/s；姿态 0.1° */

(void)osMessageQueuePut(ControlqueueHandle, &message, 0U, 0U);
```

- 队列深度为 1；发送失败表示上一条尚未被 PID 任务取走。
- 位置模式：`value2 = 900` 表示目标 +90.0°，相对**首次进入位置模式**时的九轴 yaw 零点；
  同一位置模式内新命令直接覆盖目标角，不累加、也不相对当前实测角。
- 速度模式：`value2 = 90` 表示目标 +90°/s；不使用 yaw 零点，也不做 yaw 角度环，
  中环只对目标 yaw rate 限幅。
- 逆时针为正；编码器差模不再参与 yaw 控制。

### 7.2 命令生效时序

`PIDTask_UpdateControlQueue()` 按 50 Hz 在状态机之前运行：

- 位置/速度命令先写入 `s_pid_input`；
- 切换模式时调用对应复位函数并清零控制环计数器；
- 同一 tick 进入新状态；
- 同一模式内的目标更新等下一次对应控制环生效。

## 8. 单位、反馈与 DWT 时间基准

### 8.1 单位约定

| 量 | 单位 | 说明 |
| --- | --- | --- |
| 共模位置 | m | 相对位置模式起点 |
| 共模速度 | m/s | 编码器读取时换算 |
| 目标倾角 | deg | 速度环输出 |
| 当前倾角 | deg | ATK-MS901M 九轴 pitch |
| 目标 yaw 角 | deg | 相对模式进入时的零点；Controlqueue 0.1° 增量 |
| 当前 yaw 角 | deg | ATK-MS901M 九轴 yaw（相对零点） |
| 目标 yaw rate | deg/s | yaw 角度 PD 输出，限幅 |
| 当前 yaw rate | deg/s | ATK-MS901M 陀螺 Gz |
| PWM | 比较值 | 当前限幅 `4199`（20 kHz / ARR 4199） |

位置模式下：

```text
current_common = position_current_m - position_start_m
current_yaw_rel = wrap180(imu_yaw - yaw_zero)
```

`PositionController_Start()` 进入位置模式时锁存共模起点，因此
`PositionController_Start()` 进入位置模式时锁存共模起点；编码器在 PID 内部读取，其他任务不写反馈。

### 8.2 DWT 实测 dt

共享模块：

```text
prj/elet2026/tasks/dwt_time.c
prj/elet2026/tasks/dwt_time.h
```

使用 Cortex-M4 DWT `CYCCNT`：

```text
dt = (now_cycles - last_cycles) / SystemCoreClock
```

当前 `SystemCoreClock = 168 MHz`，周期分辨率约 6 ns。

- DWT 在 `PIDTask_Init()` 中初始化一次；当前所有 DWT 读取都在 PID 任务上下文（PID 三环 + PID 中环里的编码器读取）；`DWTTime_Init()` 是幂等的；
- PID 初始化不会再次清零 `CYCCNT`，避免破坏 IMU 时间戳；
- PID 内/中/外环分别保存时间戳；
- IMU 在 DMA 启动时记录 DWT 时间戳，DMA 完成后带到任务侧；
- 模式切换时 PID 回退三个环的标称时间戳，避免第一拍 `dt≈0`；
- DWT 不可用时回退到标称 `dt`（IMU 使用 TIM7 2 ms 计数）。

### 8.3 溢出保护

- DWT 是 32 位计数器，168 MHz 下约 **25.55 s** 回绕一次；
- PID 用 1 ms FreeRTOS tick 检测循环间隔，超过 `PID_DT_OVERFLOW_GUARD_MS = 20000` 时重置时间戳；
- IMU 用 TIM7 2 ms 计数检测相邻成功帧间隔，超过 `IMU_KALMAN_DT_GUARD_TICKS = 10000`（20 s）时重置 DWT 时间戳；
- PID 实测 `dt` 上限：内环 20 ms、中环 100 ms、外环 500 ms；
- Kalman `dt` 上限：20 ms。

这些保护只避免把回绕差值当成正常时间；如果任务真的停摆 20 s 以上，控制性能已经无法保证。

## 9. 当前状态与待办

| 项目 | 状态 | 说明 |
| --- | --- | --- |
| ATK-MS901M UART 采集 | 已实现 | USART3 921600、DMA1 Stream1、28 B/周期、`IMUdatadoneBinarySem` |
| IMU 帧解析 | 已实现 | `0x01` 姿态 + `0x03` 陀螺/加速度；`0x55 0x55` + 累加和 |
| 三环 PID | 已实现 | 位置 10 Hz / 速度 50 Hz / 姿态/yaw rate 250 Hz |
| 编码器 | 已实现 | TIM3 右/TIM4 左，共模位置/速度，m / m·s⁻¹ |
| 电机 PWM | 已实现 | TIM2 20 kHz、方向 GPIO、STBY 待机 |
| Controlqueue | 已实现 | 48 位命令，50 Hz 轮询，深度 1 |
| controlTask 状态机 1/2/3 | 已实现 | 无线调参 DEBUG、原地停止、110 cm + 丢线停车 |
| controlTask 状态机 4 | 已实现（待实车标定） | 直行 + 巡线 + 后退 5 cm + 对齐 + 二次巡线停车 |
| controlTask 状态机 5 | 已实现（待实车标定） | 四次 zigzag：右转/130 cm/巡线/后退/左转/二次巡线，编码器 3 cm 到点 |
| 巡线 / 对齐 P 控制 | 初始参数 | `CONTROL_LF_*` / `CONTROL_ALIGN_*` 需按实车标定 |
| 巡线末段降速 | 已实现 | 每次巡线记录共模起点，30π→40π cm 最大速度 400→200 mm/s |
| TIM11 蜂鸣器 / LED | 已实现 | PF7/BEEP_LED，2500→50 Hz 扫描，1 s `osTimerOnce` |
| KEY0 / KEY_UP | 已实现 | KEY0=PE4 GPIO 轮询，KEY_UP=PA0 EXTI 上升沿 |
| PID 参数 | **未整定** | `pid_task.h` 全部为 0.0 |
| IMU 安装方向 | 待确认 | `IMU_PITCH_SIGN` / `IMU_YAW_SIGN` / `IMU_PITCH_GYRO_AXIS` |
| 九轴磁力计校准 | 待实车确认 | 电机/电源线/铁件会干扰绝对 yaw |
| IMU 数据超时保护 | **未实现** | PID 由信号量驱动；UART 停流时建议增加看门狗停车 |

### 建议的下一步

1. 先整定 pitch 内环，再整定 yaw rate 内环、速度环和位置环；
2. 确认 `LINE_SENSOR_ACTIVE_LEVEL`、前后传感器排列方向和巡线/对齐转向符号；
3. 标定全局变量 `alinetarget`（单位：光电间距个数）、对齐容差、`CONTROL_SM4_REVERSE_WAIT_MS`、状态机 5 的找线速度和转角容差；
4. 低速验证状态机 4 的“直行 110 cm → 巡线 → 后退 5 cm → 对齐 → 二次巡线停车”全流程；
5. 在空地单步验证状态机 5 的右转/左转方向、130 cm 编码器到点和 4 个周期流程；
6. 实现完整 A/B/C/D 路径、到点声光提示和计时；
7. 增加 IMU 停流/超时停车保护。

## 10. 调试与安全

- **先架空/拆轮**：首次测试时不要让车落地闭环；
- **先内环**：姿态环稳定后再接速度环/位置环；
- **限流供电**：使用可调限流电源或保险丝，防止 PWM 方向错误造成堵转；
- **确认符号**：`pitch`、`pitch_rate`、`yaw_rate`、左右电机方向、编码器差模符号必须统一；
- **确认 DWT**：调试器中观察 `DWT->CYCCNT` 是否递增；如果 DWT 不可用，系统会回退标称 `dt`；
- **不要依赖 50 Hz 队列急停**：需要快速停车时增加独立急停输入/中断；
- **校准期间保持静止**：上电约 1 s 内不要碰车；
- **参数从小开始**：所有 PID 增益先小后大，每次只改一个环。

## 11. 相关文档与 Git

- 赛题原文/翻译：[`target/README.md`](target/README.md)
- 控制架构、任务调度、DWT 细节：[`prj/elet2026/tasks/README.md`](prj/elet2026/tasks/README.md)
- controlTask 状态机/巡线/对齐：[`prj/elet2026/tasks/CONTROL_TASK.md`](prj/elet2026/tasks/CONTROL_TASK.md)
- 无线调参协议/CubeMX 配置：[`prj/elet2026/tasks/WIRELESS_TUNING.md`](prj/elet2026/tasks/WIRELESS_TUNING.md)
- OLED 显示字段说明：[`prj/elet2026/tasks/DISPLAY_LEGEND.md`](prj/elet2026/tasks/DISPLAY_LEGEND.md)
- 原始赛题 PDF：[`target/2026年大学生电子设计竞赛重庆大学校级比赛-B题.pdf`](target/2026年大学生电子设计竞赛重庆大学校级比赛-B题.pdf)
- CubeMX 配置：[`prj/elet2026/elet2026.ioc`](prj/elet2026/elet2026.ioc)
- Keil 工程：[`prj/elet2026/MDK-ARM/elet2026.uvprojx`](prj/elet2026/MDK-ARM/elet2026.uvprojx)

Git 远程：

```text
https://github.com/zitzengzit-png/CQUelectest2026
```

- 默认分支：`main`
- `datasheet/` 不进入 Git（约 1.6 GB 参考资料）
- `.gitignore` 同时排除 Keil 构建产物和常见临时文件

> 本仓库目前是“五流程 + 巡线/对齐 + TIM11 提示音”的控制骨架，不是可直接参赛的完成版本。PID 参数、巡线/对齐符号与目标差、后退等待时间、状态机 5 的转角/到点参数、巡线末段降速阈值/比例仍需实车整定后使用。


