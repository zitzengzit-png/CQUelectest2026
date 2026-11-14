# CQUelectest2026 · 两轮自平衡小车

2026 年大学生电子设计竞赛重庆大学校级比赛 **B 题：两轮自平衡小车**的嵌入式控制工程。

主控使用 STM32F407ZGT6，软件基于 STM32CubeMX + FreeRTOS + Keil MDK。当前仓库包含从 MPU6050 姿态采集、卡尔曼滤波、三环 PID、差速控制到电机 PWM 抽象的控制骨架。

> **当前状态**：控制结构和任务调度已完成，但 `SETPWM()` 仍是空实现、PID 参数仍为 0，且 MDK 工程尚未加入 `tasks` 源文件。因此当前代码不能直接让实车闭环行驶，详见 [当前状态与待办](#9-当前状态与待办)。

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
| 姿态传感器 | MPU6050，I2C1（PB6/PB7），TIM7 2 ms 触发 DMA 采集 |
| 执行器 | 左右直流电机，方向 + PWM |
| 反馈 | 外部编码器：共模/差模位置、共模/差模速度 |
| 控制结构 | 位置外环 10 Hz、速度中环 50 Hz、姿态/差速内环 250 Hz |
| 通信 | `Controlqueue`，48 位命令，25 Hz 轮询 |
| 构建 | Keil MDK-ARM 5 / `prj/elet2026/MDK-ARM/elet2026.uvprojx` |

赛题禁止使用图像传感器/摄像头；本工程不使用视觉，姿态与运动反馈分别来自 MPU6050 和编码器。完整题目原文与逐条释义见 [`target/README.md`](target/README.md)。

## 2. 快速开始

### 2.1 打开工程

使用 Keil MDK-ARM 打开：

```text
prj/elet2026/MDK-ARM/elet2026.uvprojx
```

目标器件为 `STM32F407ZGTx`。

### 2.2 把任务源文件加入 MDK 工程

仓库中的 `prj/elet2026/tasks` 目录目前没有被写入 `elet2026.uvprojx` 的源文件列表。首次编译前需要手动添加：

```text
prj/elet2026/tasks/dwt_time.c
prj/elet2026/tasks/pid_task.c
prj/elet2026/tasks/imu_task.c
```

并在 `Options for Target -> C/C++ -> Include Paths` 中加入：

```text
../tasks
```

现有的 `../Core/Inc`、HAL、CMSIS、FreeRTOS 包含路径保持不变。

### 2.3 编译与下载

1. 确认上面的 3 个 `.c` 文件已经加入工程；
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
│  │  ├─ imu_task.c/.h              MPU6050、DMA、卡尔曼、姿态发布
│  │  ├─ pid_task.c/.h              三环控制、状态机、Controlqueue
│  │  └─ README.md                  控制架构与详细调度说明
│  ├─ elet2026.ioc                  CubeMX 配置
│  └─ .mxproject
└─ datasheet/                       本地资料，已被 .gitignore 排除
```

`datasheet/` 约 1.6 GB，未提交到 Git；仓库中保留源码和必要工程文件。

## 4. 系统架构

### 4.1 控制数据流

```text
                 ┌─────────────────────────────┐
Controlqueue ───►│ PID 状态机（STOP/POSITION/SPEED）│
                 └──────────────┬──────────────┘
                                │ 目标位置/速度/差模量
                                ▼
       ┌──────────────────────────────────────────┐
       │ 位置外环 10 Hz（仅 POSITION）             │
       │ 速度中环 50 Hz                            │
       │ 姿态/差速内环 250 Hz                      │
       └──────────────────┬───────────────────────┘
                          │ 左右 PWM
                          ▼
                    PIDTask_WriteOutput()
                          │
                          ▼
                    SETPWM()  ← 当前空实现
```

### 4.2 IMU 数据流

```text
TIM7 2 ms
  → IMU_OnTimer7()
  → HAL_I2C_Mem_Read_DMA()（MPU6050，I2C1）
  → HAL_I2C_MemRxCpltCallback()
  → 释放 IMUtaskBinarySem
  → IMUTask：Kalman_Update()
  → 双缓冲
  → kalmaneddataQueue（深度 1，存放结构体指针）
  → PIDTask_UpdateIMU()
```

- MPU6050 配置：500 Hz、DLPF 94/98 Hz、±1000 dps、±2 g。
- `IMU_Init()` 在初始化任务中执行约 1 s 静止陀螺零偏校准；校准失败不阻止启动，滤波器会在线估计零偏。
- 首次上电保持静止约 1 s；`IMU_IsReady()` 在没有首帧时会让 PID 任务等待。
- IMU 只输出 `pitch / pitch_rate`，不参与差模旋转目标/反馈。

### 4.3 编码器反馈

编码器不由 STM32 内部读取，外部需要把左右编码器换算成以下四个量并调用：

```c
PIDTask_SetFeedback(position_pulses,
                    diff_position_pulses,
                    speed_pulses_s,
                    diff_speed_pulses_s);
```

- `position_pulses`：共模位置累计值，单位脉冲；
- `diff_position_pulses`：差模位置累计值，单位脉冲；
- `speed_pulses_s`：共模速度，单位脉冲/s；
- `diff_speed_pulses_s`：差模速度，单位脉冲/s。

位置模式由 `PositionController_Start()` 锁存共模/差模起点，所以位置反馈应传**累计值**，不要传增量。

## 5. FreeRTOS 任务与调度

### 5.1 任务

| 任务 | 入口 | 优先级 | 周期/触发 | 栈 | 职责 |
| --- | --- | --- | --- | --- | --- |
| defaultTask | `StartDefaultTask` | `osPriorityHigh4` | 初始化一次后永久挂起 | 256 words | `IMU_Init()`、启动 TIM7 |
| IMUTask | `StartIMUTask` | `osPriorityAboveNormal` | TIM7 2 ms + 二值信号量 | 256 words | DMA 帧解析、卡尔曼、发布 |
| PID | `StartPIDTask` | `osPriorityNormal4` | 4 ms 基础任务 | 256 words | 三环控制、状态机、Controlqueue |

调度关系：

```text
defaultTask > IMUTask > PID
```

`defaultTask` 在初始化完成后必须永久挂起，否则会长期占用 CPU。

### 5.2 FreeRTOS 队列/信号量

| 对象 | 类型 | 深度/初值 | 用途 |
| --- | --- | --- | --- |
| `Controlqueue` | 消息队列 | 深度 1，项 6 字节 | 上位机控制命令 |
| `kalmaneddataQueue` | 消息队列 | 深度 1，项 4 字节 | `KalmanData_t *` 指针 |
| `IMUtaskBinarySem` | 二值信号量 | CubeMX 当前初值 1 | DMA 完成通知 |

> `StartIMUTask()` 启动时会先清掉一次 `IMUtaskBinarySem`；更规范的做法是在 CubeMX 中把初值改为 **0**。

### 5.3 PID 任务时间结构

PID 基础任务周期：

```c
PID_TASK_PERIOD_MS = 4U;   /* 250 Hz */
```

任务循环顺序：

```c
PIDTask_UpdateIMU();             // 读 Kalman 队列，250 Hz
PIDTask_CheckTimestampOverflow();// DWT 32 位回绕保护
PIDTask_UpdateControlQueue();    // 计数器分频，实际 25 Hz
switch (s_pid_state) { ... }     // 状态机
PIDTask_DelayUntil();            // osDelayUntil，下一 4 ms 节拍
```

三环分频：

| 环 | 频率 | 周期 | 执行条件 |
| --- | --- | --- | --- |
| 内环（姿态/差速） | 250 Hz | 4 ms | 每个基础 tick |
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
250 Hz / 10 = 25 Hz（40 ms）
```

Controlqueue 在状态机 `switch` 之前读取：

1. 读取并更新状态/目标；
2. 同一基础 tick 的 `switch` 进入新状态；
3. 如果只是更新当前模式目标，目标在后续对应控制环生效；
4. 如果模式切换，控制环计数器清零，新状态第一拍从 `tick 0` 开始。

停止命令的检测延迟最多约 40 ms；如果安全上需要更快停止，应设计独立的紧急停止通道。

## 6. 控制链路

### 6.1 位置模式：`PID_STATE_POSITION`

```text
共模位置 PD 10 Hz → 目标速度
                         ↓
共模速度 PD 50 Hz → 目标倾角 → 姿态 PD 250 Hz → 共模 PWM
差模位置 PD 10 Hz → 目标差模速度
                         ↓
中环透传/限幅 50 Hz → 差模速度 PI 250 Hz → 差模 PWM
```

- 外环：`PositionController_Update()`，`dt` 由 DWT 实测；
- 中环：`SpeedController_Update()`，差模通道只透传，不做 PID；
- 内环：`AttitudeController_Update()`，姿态 PD + 差速 PI；
- 左右 PWM：

```text
PWM_L = PWM_common - PWM_diff
PWM_R = PWM_common + PWM_diff
```

### 6.2 速度模式：`PID_STATE_SPEED`

```text
Controlqueue value1 → 目标共模速度
Controlqueue value2 → 目标差模速度
          ↓
速度 PD 50 Hz → 目标倾角 → 姿态 PD 250 Hz → 共模 PWM
差模目标速度 50 Hz 透传 → 差速 PI 250 Hz → 差模 PWM
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
PID_ATT_KP_DIFF
PID_ATT_KI_DIFF
```

当前全部为 `0.0f`，需要实车整定。输出限幅包括：

```text
PID_POS_MAX_SPEED_PULSES_S        50
PID_POS_MAX_DIFF_SPEED_PULSES_S   200
PID_SPEED_MAX_PITCH_DEG           10
PID_ATT_MAX_PWM                   1000
PID_ATT_MAX_DIFF_PWM              1000
PID_ATT_MAX_DIFF_INTEGRAL         10000
```

### 6.4 电机输出

`PIDTask_WriteOutput()` 把左右有符号 PWM 转成方向 + 比较值：

```c
SETPWM(left_wheel, forward, compare);
```

- `left_wheel`：1 = 左轮，0 = 右轮；
- `forward`：1 = 正转，0 = 反转；
- `compare`：PWM 比较值。

**当前 `SETPWM()` 仍是空实现**，不会真正驱动电机。需要按实际电机驱动接线填入 `__HAL_TIM_SET_COMPARE` / `HAL_TIM_PWM_Start`。


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
| `01` | 位置 | 目标共模位移（脉冲） | 目标差模旋转量（脉冲） |
| `10` | 速度 | 目标共模速度（脉冲/s） | 目标差模速度（脉冲/s） |
| `11` | 保留 | 按停止处理 | 按停止处理 |

发送示例：

```c
uint64_t message = ((uint64_t)0x1U << 46)
                 | ((uint64_t)(uint16_t)target_position << 16)
                 | (uint64_t)(uint16_t)target_diff_position;

(void)osMessageQueuePut(ControlqueueHandle, &message, 0U, 0U);
```

- 队列深度为 1；发送失败表示上一条尚未被 PID 任务取走。
- 位置模式的 `value2` 是**编码器差模脉冲**，不是 IMU 角度/deg。
- 差模正方向必须与 `value2`、编码器差模符号和 PWM 转向一致。

### 7.2 命令生效时序

`PIDTask_UpdateControlQueue()` 按 25 Hz 在状态机之前运行：

- 位置/速度命令先写入 `s_pid_input`；
- 切换模式时调用对应复位函数并清零控制环计数器；
- 同一 tick 进入新状态；
- 同一模式内的目标更新等下一次对应控制环生效。

## 8. 单位、反馈与 DWT 时间基准

### 8.1 单位约定

| 量 | 单位 | 说明 |
| --- | --- | --- |
| 共模位置 | 编码器脉冲 | 相对位置模式起点 |
| 差模位置 | 编码器脉冲 | 左右编码器差 |
| 共模速度 | 脉冲/s | 外部换算后传入 |
| 差模速度 | 脉冲/s | 外部换算后传入 |
| 目标倾角 | deg | 速度环输出 |
| 当前倾角 | deg | MPU6050 + 卡尔曼 |
| 角速度 | deg/s | MPU6050 陀螺 |
| PWM | 比较值 | 当前限幅 `1000` |

位置模式下：

```text
current_common = encoder_current - encoder_start
current_diff   = encoder_diff_current - encoder_diff_start
```

`PositionController_Start()` 进入位置模式时锁存两个起点，因此 `PIDTask_SetFeedback()` 应传累计位置。

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

- DWT 在 `IMU_Init()` 中初始化一次，`DWTTime_Init()` 是幂等的；
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
| MPU6050 初始化/校准 | 已实现 | 500 点静止零偏校准 |
| TIM7 + I2C DMA | 已实现 | 2 ms 触发，双缓冲 |
| 卡尔曼滤波 | 已实现 | TKJ 二状态滤波 + 自适应 R |
| 三环调度 | 已实现 | 250/50/10 Hz |
| Controlqueue | 已实现 | 25 Hz 轮询，状态机前读取 |
| DWT dt | 已实现 | PID/IMU 共用；需实车确认 CYCCNT 递增 |
| `SETPWM()` | **未实现** | 空函数，电机不会动作 |
| PID 参数 | **未整定** | `pid_task.h` 全部为 0.0 |
| 编码器接入 | 待外部实现 | 需调用 `PIDTask_SetFeedback()` |
| MDK 源文件列表 | **未完成** | `tasks` 三个 `.c` 尚未加入 `uvprojx` |
| IMU 安装方向 | 待确认 | 默认 X 朝前、pitch 绕 Y；必要时改 `IMU_PITCH_SIGN` |
| Controlqueue 停止响应 | 25 Hz | 最坏约 40 ms，安全要求高时需独立急停 |
| `IMUtaskBinarySem` 初值 | 建议改 0 | 当前 CubeMX 初值为 1，任务启动时会清掉 |

### 建议的下一步

1. 根据实际电机驱动填写 `SETPWM()`；
2. 加入 `tasks` 源文件和 `../tasks` Include Path；
3. 用示波器/调试器确认 DWT `CYCCNT` 正常递增；
4. 确认 MPU6050 安装方向和 pitch 符号；
5. 确认编码器共模/差模符号、速度换算和 PWM 方向；
6. 先整定姿态内环，再整定速度中环，最后整定位置外环；
7. 最后再运行赛题路径和声光提示逻辑。

## 10. 调试与安全

- **先架空/拆轮**：首次测试时不要让车落地闭环；
- **先内环**：姿态环稳定后再接速度环/位置环；
- **限流供电**：使用可调限流电源或保险丝，防止 PWM 方向错误造成堵转；
- **确认符号**：`pitch`、`pitch_rate`、左右电机方向、编码器差模符号必须统一；
- **确认 DWT**：调试器中观察 `DWT->CYCCNT` 是否递增；如果 DWT 不可用，系统会回退标称 `dt`；
- **不要依赖 25 Hz 队列急停**：需要快速停车时增加独立急停输入/中断；
- **校准期间保持静止**：上电约 1 s 内不要碰车；
- **参数从小开始**：所有 PID 增益先小后大，每次只改一个环。

## 11. 相关文档与 Git

- 赛题原文/翻译：[`target/README.md`](target/README.md)
- 控制架构、任务调度、DWT 细节：[`prj/elet2026/tasks/README.md`](prj/elet2026/tasks/README.md)
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

> 本仓库目前是控制骨架版本，不是可直接参赛的完成版本。请先完成 `SETPWM()`、编码器接入和参数整定，再进行实车路径测试。

