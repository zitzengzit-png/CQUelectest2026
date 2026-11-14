# tasks —— FreeRTOS 任务实现

本目录存放各 FreeRTOS 任务的实现。任务本身在 STM32CubeMX 的 FreeRTOS 配置中创建，
创建方式选择 **As external**；任务入口函数在本目录的 `.c` 文件中实现，由
`Core/Src/freertos.c` 中的 `osThreadNew()` 注册并启动。

## 现有任务

| 任务 | 入口函数 | 文件 | 周期 | 优先级 | 栈 |
| --- | --- | --- | --- | --- | --- |
| 初始化（defaultTask） | `StartDefaultTask` | `Core/Src/freertos.c` | 一次性初始化（IMU + TIM7）后永久挂起 | `osPriorityHigh4` | 256 words（1 KB） |
| IMU | `StartIMUTask` | `imu_task.c` / `imu_task.h` | 2 ms（TIM7 触发）；上电校准约 1 s | `osPriorityAboveNormal` | 256 words（1 KB） |
| PID | `StartPIDTask` | `pid_task.c` / `pid_task.h` | 4 ms 基础任务（250 Hz）；三环由内置计数器分为 250/50/10 Hz | `osPriorityNormal4` | 256 words（1 KB） |

> **共享时间模块**：`dwt_time.c` / `dwt_time.h` 提供 DWT CYCCNT 初始化、
> 时间戳和实测 `dt`，供 IMU 与 PID 共用；DWT 不可用时自动回退到各自标称时间。
>
> **调度关系**：`defaultTask`（44）> `IMUTask`（32）> `PID`（28），
> `configUSE_PREEMPTION=1`。FreeRTOS 优先级数值越大越高；defaultTask 在初始化
> 期间不会被 PID 抢占，但初始化完成后必须永久挂起，否则会饿死 PID/IMU。
> 栈大小只影响栈空间，不影响抢占关系。启动同步见下文“IMU 卡尔曼滤波”的
> “启动同步”小节。

## PID 任务控制方案

PID 任务采用**三环串级 + 差速内环**结构：

- **最外环（位置）**：目标共模位移（脉冲）、目标差模旋转量（脉冲） → 目标速度（脉冲/s）、目标差模速度（脉冲/s）；
- **中环（速度/姿态）**：共模速度 PD → 目标倾角；差模目标速度**直接透传**，不做 PID；
- **内环**：目标倾角 → 共模 PWM；目标差模速度 → 差模 PWM（PI）。

> **单位约定**：本任务内部共模位置、差模位置、速度、差模速度全部使用编码器脉冲
> （速度 = 脉冲/秒）；IMU 只提供俯仰角/角速度（deg、deg/s），只用于姿态内环，
> 不参与位置模式的旋转目标或旋转反馈。
>
> **旋转量**：位置模式的 `value2 / psi_ref` 是**差模编码器脉冲数**，不是 IMU 角度、
> 不是 deg；目标与反馈使用同一脉冲坐标系，进入位置模式时锁存差模起点。
>
> 状态：差模位置 PD 直接输出目标差模速度；中环只透传，不做差模 PID。

### 1. 最外环：位置控制

| 项目 | 内容 |
| --- | --- |
| 目标输入 | 目标共模位移 `s_ref`（脉冲）、目标差模旋转量 `psi_ref`（脉冲） |
| 反馈输入 | 共模当前位置 `s`（编码器计数差，脉冲）、差模当前位置 `psi`（编码器差速脉冲） |
| 控制器 | 共模位置 **PD**、差模位置 **PD** |
| 输出 | 目标速度 `v_ref`（脉冲/s）、目标差模速度 `n_diff_ref`（脉冲/s） |

- 共模位置 PD：`e_s = s_ref - s`，`v_ref = Kp_s * e_s + Kd_s * de_s/dt`（脉冲/s）。
- 差模位置 PD：`e_psi = psi_ref - psi`，
  `n_diff_ref = Kp_psi * e_psi + Kd_psi * de_psi/dt`（脉冲/s）。
  - `Kp_psi` 单位 `1/s`，`Kd_psi` 无量纲；输出是**目标差模速度**，不是差模位置指令。
- 共模当前位置 `s = encoder_current - encoder_start`（脉冲）；
  差模当前位置 `psi = encoder_diff_current - encoder_diff_start`（脉冲）。
- 进入位置模式时同时锁存共模起点 `encoder_start` 和差模起点
  `encoder_diff_start`；`s_ref`、`psi_ref` 都是相对该起点的脉冲增量。
- `psi` 由左右编码器计数差得到（脉冲），与 IMU 无关；IMU 只参与下面的姿态内环。

### 2. 中环：速度/姿态

| 项目 | 内容 |
| --- | --- |
| 目标输入 | 目标速度 `v_ref`（脉冲/s）、目标差模速度 `n_diff_ref`（脉冲/s） |
| 反馈输入 | 当前共模速度 `v`（脉冲/s） |
| 控制器 | 共模速度 **PD**；差模目标速度**直接透传**（不做 PID） |
| 输出 | 目标倾角 `pitch_ref`（deg）、差模目标速度 `n_diff`（脉冲/s） |

- 共模速度 PD：`e_n = v_ref - v`，
  `pitch_ref = Kp_n * e_n + Kd_n * de_n/dt`（限幅）。
- `e_n > 0`（目标速度高于当前速度）时输出正的目标倾角（前倾），使车体向前加速；
  `e_n < 0` 时输出负的目标倾角。
- 差模通道：`n_diff = n_diff_ref`，中环不做差模 PID，只做安全限幅。
- 位置模式（Controlqueue 模式 01）：外环差模位置 PD 直接给出 `n_diff_ref`；
  中环原样透传。
- 速度模式（Controlqueue 模式 10）：`value2` 就是 `n_diff_ref`（脉冲/s）；
  中环同样原样透传。
- 内环再用 `n_diff - n_diff_actual` 做差速 PI，输出差模 PWM。

### 3. 内环：目标姿态/差速 → PWM

| 项目 | 内容 |
| --- | --- |
| 目标输入 | 目标倾角 `pitch_ref`（deg）、差模目标速度 `n_diff`（脉冲/s） |
| 反馈输入 | 当前倾角 `pitch`（deg，IMU）、当前差模速度 `n_diff_actual`（脉冲/s，编码器） |
| 控制器 | 共模：倾角 PD；差模：**PI** |
| 输出 | 共模 PWM `PWM_cmd`、差模 PWM `PWM_diff` |

- 共模 PWM：`e_theta = pitch - pitch_ref`，
  `PWM_cmd = Kp_theta * e_theta + Kd_theta * pitch_rate`（已含陀螺角速度阻尼）。
- 误差方向：`pitch > pitch_ref`（前倾超过目标）时输出正 PWM，驱动轮子向前加速；
  若 IMU 安装方向相反，`pitch` 与 `pitch_rate` 需同时取反。
- 差模 PWM：`PWM_diff = PI(n_diff - n_diff_actual)`。
- 合成左右 PWM：`PWM_L = PWM_cmd - PWM_diff`，`PWM_R = PWM_cmd + PWM_diff`
  （符号按实车验证）。

### 4. 信号流

```
s_ref ──► [共模位置 PD] ──► v_ref ──► [中环 速度 PD] ──► pitch_ref ──► [内环 倾角 PD] ──► PWM_cmd
psi_ref ► [差模位置 PD] ──► n_diff_ref ──► [中环透传] ──► n_diff ──► [内环 差速 PI] ──► PWM_diff

反馈：编码器共模 → s；编码器共模速度 v、差模速度 n_diff_actual；编码器差模 → psi；
      IMU pitch、pitch_rate 只进姿态内环
输出：PWM_L = PWM_cmd - PWM_diff，PWM_R = PWM_cmd + PWM_diff
```

### 5. 输入与职责边界

| 模块 | 职责 |
| --- | --- |
| 上层路径/指令模块 | 通过 `Controlqueue` 给出目标共模位移/差模旋转量（模式 01，均为脉冲）或目标速度/差模速度（模式 10） |
| PID 任务（本任务） | 位置 → 速度/姿态 → PWM；`Controlqueue` 在任务循环开头（IMU 之后）、状态机之前按 25 Hz 读取 |
| 编码器 | 外部换算为 position / diff position / speed / diff speed，经 `PIDTask_SetFeedback()` 写入 |
| IMU | TIM7 2 ms → I2C DMA → `IMUTask` 卡尔曼；PID 从 `kalmaneddataQueue` 取指针 |
| 电机驱动 | `SETPWM()` 设置方向与比较值（当前空实现） |

### 6. 执行周期与参数

#### 多环调度

PID 基础任务按 `PID_TASK_PERIOD_MS = 4 ms` 运行（250 Hz），三环不创建额外任务，
而是共用 `pid_task.c` 内的软件计数器 `s_pid_loop_counter` 分频：

| 环 | 频率 | 周期 | 分频条件 | 标称 `dt`（实际由 DWT 测量） |
| --- | --- | --- | --- | --- |
| 内环（姿态/差速） | 250 Hz | 4 ms | 每个基础 tick | `PID_INNER_DT_S = 0.004 s` |
| 中环（速度） | 50 Hz | 20 ms | `tick % 5 == 0` | `PID_MIDDLE_DT_S = 0.02 s` |
| 外环（位置） | 10 Hz | 100 ms | `tick % 25 == 0` | `PID_OUTER_DT_S = 0.1 s` |

- 表中的 `dt` 是标称值，只用于 DWT 不可用时的回退；正常运行时三个环分别用
  `DWT->CYCCNT` 实测两次执行之间的真实间隔。
- `PIDTask_NextLoopTick()` 返回当前控制环 `tick`（`0 ~ 24`），然后把计数器推进
  一格；计到 25 自动回零。
- `PIDTask_UpdateControlQueue()` 在 `StartPIDTask()` 的每个基础循环中调用：
  `PIDTask_UpdateIMU()` 之后、状态机 `switch` 之前按 25 Hz 读取 Controlqueue；
  读取后先更新状态/目标，再由 `switch` 进入新状态。
- `PIDTask_StatePosition()` 内顺序固定为：外环（若到期）→ 中环（若到期）→
  内环（每拍）。外环先算，保证同一拍中环能立即使用新的目标速度。
- `PIDTask_StateSpeed()` 只运行 50 Hz 中环和 250 Hz 内环，跳过位置外环。
- 环间输出保存在 `s_outer_target_speed`、`s_outer_target_diff_speed`、
  `s_middle_target_pitch`、`s_middle_diff_speed_command` 中；未到执行周期的环沿用
  上一次结果，不能用栈上的临时变量。

#### Controlqueue 轮询（25 Hz）

`Controlqueue` 与三环解耦，固定按 **25 Hz（40 ms）** 读取，即每 10 个 250 Hz
基础 tick 读取一次：

- 计数器：`s_control_queue_counter`，范围 `0 ~ 9`；
- `PIDTask_UpdateControlQueue()` 每个基础 tick 调用一次，只有计数器为 0 时才调用
  `PIDTask_ProcessControlQueue()`；
- 读取后立即更新 `s_pid_state`、目标位置/目标速度和差模目标；随后同一个基础 tick
  的 `switch` 会进入新状态；
- IMU 仍按 250 Hz 读取；只有 Controlqueue 降为 25 Hz；
- 计数器在所有状态下持续运行，不随模式切换清零；控制环计数器
  `s_pid_loop_counter` 仍在模式切换时清零。

由此：

- 命令检测延迟最多约 40 ms；
- 如果命令只更新当前模式的目标：位置目标在下一次 10 Hz 外环生效（最多再加
  100 ms），速度目标在下一次 50 Hz 中环生效（最多再加 20 ms）；
- 如果命令切换状态：当前基础 tick 就进入新状态；位置/速度状态的控制环计数器
  已清零，第一拍从 `tick 0` 开始；
- STOP 命令也在同一个 25 Hz 读取点处理，处理后当前 tick 进入 `PID_STATE_STOP`
  并输出 0。若对停止响应有更高要求，需单独设计紧急停止通道，不能只依赖 25 Hz
  队列轮询。

#### 实测 dt（DWT CYCCNT）

250 Hz 基础任务会被更高优先级的 IMUTask 抢占，实际执行间隔不严格等于 4 ms。
因此控制器不再使用固定 `dt`，而是用 Cortex-M4 的 DWT 周期计数器测量：

- 共享 DWT 初始化由 `IMU_Init()`（TIM7 启动前）和 `PIDTask_Init()` 通过
  `DWTTime_Init()` 完成，函数是幂等的：
  - 第一次调用置 `CoreDebug->DEMCR` 的 `TRCENA`，清零并使能 `DWT->CYCCNT`；
  - 之后 PID 初始化不会再次清零，避免破坏 IMU 正在使用的时间戳；
  - 若 `DWT_CTRL_NOCYCCNT_Msk` 指示无周期计数器，则标记 DWT 不可用。
- 分别保存三个环上一次执行的时间戳：
  `s_pid_last_inner_cycles`、`s_pid_last_middle_cycles`、
  `s_pid_last_outer_cycles`。
- 每个环执行时读取 `DWT->CYCCNT`，由 `PIDTask_CalculateDt()` 计算：

  ```c
  dt = (now - last_cycles) / SystemCoreClock;
  ```

  当前 `SystemCoreClock = 168 MHz`，CYCCNT 分辨率约 6 ns；32 位计数器约
  25.55 s 回绕一次。只要两次时间戳间隔小于 2³² 个周期，无符号
  `uint32_t` 差值就是准确的。
- `PIDTask_CheckTimestampOverflow()` 在每个基础循环用 1 ms FreeRTOS tick 检查
  循环间隔；超过 `PID_DT_OVERFLOW_GUARD_MS = 20000 ms` 时调用
  `PIDTask_ResetLoopTimers()`，不把可能回绕的 DWT 差值当作真实 `dt`。
- 内环、中环、外环各自更新时间戳，因此中环使用两次 50 Hz 中环之间的实际
  间隔，外环使用两次 10 Hz 外环之间的实际间隔，不会把 4 ms 基础周期误当成
  20 ms/100 ms。
- 模式切换时 `PIDTask_ResetLoopTimers()` 把三个时间戳回退到各自一个标称周期前，
  避免新模式第一拍 `dt≈0` 导致微分项异常。
- 若 DWT 不可用或 `SystemCoreClock == 0`，`PIDTask_CalculateDt()` 自动返回标称
  `dt`。
- 实测 `dt` 设有上限：
  - 内环 `PID_INNER_DT_MAX_S = 0.020 s`
  - 中环 `PID_MIDDLE_DT_MAX_S = 0.100 s`
  - 外环 `PID_OUTER_DT_MAX_S = 0.500 s`

  仅在任务长时间停顿、超过上限时截断，防止 D/I 项被异常放大。
- 溢出保护边界：正常运行时环间隔为 4/20/100 ms，远小于 20 s 保护窗口；
  若任务停顿超过 20 s，保护逻辑会重置时间戳并使用标称 `dt`，不会使用回绕后的
  错误差值。若整个系统连续运行超过 FreeRTOS tick 的约 49.7 天回绕周期，则属于
  另一层 tick 溢出问题，需要独立的运行时长管理。

> 这只修正 PID 的 `de/dt`、积分增量等时间基准，**不会改变环的调度节拍**。
> 若 IMU 任务连续抢占导致外环/中环漏拍，实际 `dt` 会变大，控制带宽仍会下降；
> 需要从任务优先级、栈和 CPU 负载上进一步保证实时性。

#### 节拍

- 模式切换时由 `PIDTask_ResetLoopCounter()` 把控制环计数器清零，使新模式第一拍
  从 `tick 0` 开始；已在同一模式内更新目标时不清控制环计数器。
- `PIDTask_DelayUntil()` 仍使用 `osDelayUntil()` 和 `pdMS_TO_TICKS(PID_TASK_PERIOD_MS)`
  保持 4 ms 基础节拍。FreeRTOS tick 为 1 kHz，4 ms 对应 4 个 tick；任务调度抖动
  会带来约 ±1 ms 的节拍误差，但控制器的 `dt` 已由 DWT 实测。

#### 参数与单位

- 三环的标称 `dt` 只用于 DWT 不可用时的回退；正常运行时使用实测值，不能把
  4 ms 硬编码给 10 Hz/50 Hz 控制器。
- 共模/差模都直接使用编码器脉冲差（`encoder_current - encoder_start`、
  `encoder_diff_current - encoder_diff_start`），不需要 cm/deg 换算。
- `de_s/dt`、`de_psi/dt`、`de_n/dt` 与 `pitch_rate` 建议低通滤波。
- 前进、左转、前倾的正方向需统一定义；`s_ref`、`psi_ref`、`pitch_ref`、`v_ref`、
  `n_diff`、PWM 与各积分项均需限幅。

## 控制器函数与状态机

工程使用 C 语言，三个控制器以**函数**形式实现。所有宏、参数结构体、状态枚举与
函数声明集中在 `pid_task.h`；结构体实例、控制器内部状态与函数实现放在 `pid_task.c` 中。

### 控制器函数与参数结构体

| 控制器 | 函数 | 参数结构体 | 输入 → 输出 |
| --- | --- | --- | --- |
| 位置 | `PositionController_Update()` | `PositionControllerParams_t` | 共模/差模编码器起始/当前值、目标共模位移（脉冲）、目标差模旋转量（脉冲） → 目标速度（脉冲/s）、目标差模速度（脉冲/s） |
| 速度/姿态 | `SpeedController_Update()` | `SpeedControllerParams_t` | 目标/当前共模速度（脉冲/s）、目标差模速度（脉冲/s） → 目标倾角（deg）、透传的差模速度（脉冲/s） |
| 姿态/差速 | `AttitudeController_Update()` | `AttitudeControllerParams_t` | 目标倾角（deg）、当前倾角/角速度、目标/当前差模速度（脉冲/s） → 左右 PWM |

每个控制器提供：

- `xxxController_Update()`：按 `dt` 执行一次 PD/PI 计算，结果通过输出指针返回；
- `xxxController_Reset()`：清零微分历史 / 积分；
- `PositionController_Start()`：进入位置模式时同时锁存共模/差模起点
  （`encoder_start = encoder_current`、`encoder_diff_start = encoder_diff_current`）；
- 参数（增益与限幅）统一放在对应的 `xxxControllerParams_t` 结构体中，
  实例为 `s_position_params`、`s_speed_params`、`s_attitude_params`。
- 位置参数结构体保存共模 `encoder_start/current` 和差模
  `encoder_diff_start/current`；共模位置、差模旋转量都取“当前值 - 起点值”，
  单位均为编码器脉冲。

### IMU 数据链路

| 环节 | 实现 |
| --- | --- |
| 定时触发 | TIM7 2 ms → `IMU_OnTimer7()` 启动 `HAL_I2C_Mem_Read_DMA(..., 0x3B, 14)` |
| DMA 完成 | `HAL_I2C_MemRxCpltCallback()` 把原始数据拷贝到 `s_imu_ready[14]`，释放 `IMUtaskBinarySem` |
| 解算 | `IMUTask` 等待信号量 → 解析 accel/gyro → `Kalman_Update()` → 写入双缓冲 |
| 发布 | `IMU_Publish()` 用 `osMessageQueueReset()` + `osMessageQueuePut()` 把当前缓冲指针覆盖进 `kalmaneddataQueue`（深度 1，4 字节指针） |
| PID 取数 | `PIDTask_UpdateIMU()` 从 `kalmaneddataQueue` 取指针，在临界区内拷贝 `pitch/pitch_rate` |

- `IMU_Init()` 在初始化任务中完成 MPU6050 配置（500 Hz、DLPF 94/98 Hz、±1000 dps、±2g），成功后才启动 TIM7。
- `IMU_ReadData()` 已删除，PID 不再直接读 I2C。
- 双缓冲 + 指针队列：IMU 任务写另一块缓冲，PID 取到指针后立即在临界区拷贝，避免被下一帧覆盖。
- CubeMX 当前的 `IMUtaskBinarySem` 初始计数是 1，`StartIMUTask()` 会先清掉它；
  建议在 CubeMX 中把二值信号量初始计数改为 **0**。
- 初始化任务当前栈为 256 words（1 KB）；`defaultTask` 优先级为 44，初始化完成后
  必须永久挂起，否则会饿死 PID/IMU。
- `IMU_Process()` 默认按“pitch 绕模块 Y 轴、X 轴朝前”解算；实际安装不同，需修改
  `imu_task.c` 中的加速度轴、陀螺轴与符号。

### PWM 硬件抽象

| 函数 | 作用 | 现状 |
| --- | --- | --- |
| `SETPWM(left_wheel, forward, compare)` | 设置单侧电机方向与比较值 | 空实现；TODO 填 `__HAL_TIM_SET_COMPARE` / `HAL_TIM_PWM_Start` |

- `PIDTask_WriteOutput()` 把有符号 PWM 转成方向 + 比较值，分别调用两次 `SETPWM()`：
  - `left_wheel`：1 = 左轮，0 = 右轮；
  - `forward`：1 = 正转，0 = 反转；
  - `compare`：写入比较寄存器的值。
- 编码器数据由外部换算成 `共模位置 / 差模位置 / 共模速度 / 差模速度`
  （脉冲、脉冲/s）后，通过 `PIDTask_SetFeedback()` 写入。位置/差模位置应传
  **编码器累计值**，位置模式由 `PositionController_Start()` 锁存起点；差模正方向
  必须与 Controlqueue `value2` 和 PWM 转向一致。

### 控制模式与状态机

| 状态 | 说明 | 控制器链 |
| --- | --- | --- |
| `PID_STATE_STOP` | 输出 0；`Controlqueue` 由主循环按 25 Hz 轮询 | — |
| `PID_STATE_POSITION` | 位置模式（共模位移 + 差模旋转，均为编码器脉冲） | 外环 10 Hz `PositionController_Update` → 中环 50 Hz `SpeedController_Update` → 内环 250 Hz `AttitudeController_Update` |
| `PID_STATE_SPEED` | 速度控制模式 | 中环 50 Hz `SpeedController_Update` → 内环 250 Hz `AttitudeController_Update`（跳过位置外环） |

- 模式与目标：全部由 `Controlqueue` 命令决定（不再提供 SetMode / SetTarget 接口）；
- 编码器反馈：`PIDTask_SetFeedback(position_pulses, diff_position_pulses, speed_pulses_s, diff_speed_pulses_s)`；
- IMU：`PIDTask_UpdateIMU()` 在循环开头从 `kalmaneddataQueue` 取最新指针并拷贝 `pitch / pitch_rate`；
- Controlqueue：`StartPIDTask()` 在 `PIDTask_UpdateIMU()` 之后、`switch` 之前调用
  `PIDTask_UpdateControlQueue()`，固定按 25 Hz 读取；读取后立即更新状态/目标，
  同一基础 tick 的 `switch` 进入新状态；
- 若命令只更新当前模式的目标，目标会在下一次对应控制环生效；若命令切换状态，
  当前基础 tick 就进入新状态；
- `for (;;)` 内只有 `PIDTask_UpdateIMU()` + `PIDTask_UpdateControlQueue()` +
  状态机 + `PIDTask_DelayUntil()`；状态处理函数先用 `PIDTask_NextLoopTick()`
  取调度 tick，再按分频调用 `PIDTask_PositionOuterLoop()` /
  `PIDTask_PositionMiddleLoop()` / `PIDTask_SpeedMiddleLoop()` /
  `PIDTask_AttitudeInnerLoop()`。

## Controlqueue（CubeMX 生成）

- CubeMX FreeRTOS 队列：`Controlqueue`，深度 **1**，队列项 **6 字节（48 位）**，
  句柄 `ControlqueueHandle`，在 `MX_FREERTOS_Init()` 中创建；
- PID 任务直接使用 `ControlqueueHandle`，不另建队列；
- 固定每 40 ms（25 Hz）读取一次；每次任务循环先更新 IMU，再轮询 Controlqueue，
  最后进入状态机；
- 命令由上层通过 `osMessageQueuePut(ControlqueueHandle, &message, 0U, 0U)` 发送，
  队列深度为 1；发送失败表示上一条尚未被 PID 任务取走。

### 位定义

按 48 位字的第 1 位为 MSB：

| 位（MSB 起） | 位（LSB 编号） | 含义 |
| --- | --- | --- |
| 1~2 | 47~46 | 模式：`00` 停止、`01` 位置、`10` 速度、`11` 保留 |
| 3~16 | 45~32 | 保留（跳过第一个 16 位） |
| 17~32 | 31~16 | value1：`int16`，共模位置或共模速度（编码器脉冲 / 脉冲/s） |
| 33~48 | 15~0 | value2：`int16`，差模位置或差模速度（编码器脉冲 / 脉冲/s） |

| 模式 | value1 | value2 |
| --- | --- | --- |
| `00` 停止 | 忽略 | 忽略 |
| `01` 位置 | 目标共模位移（脉冲） | 目标差模旋转量（脉冲） |
| `10` 速度 | 目标速度（脉冲/s） | 目标差模速度（脉冲/s） |

> 位置模式的 `value2` 是编码器差模脉冲，不是 IMU 角度/deg；IMU 的 `pitch` 只用于
> 姿态内环。若上层只有物理旋转角度，需先按编码器/轮距换算成差模脉冲再发送。

- 两个字段均为 **16 位有符号**（-32768~32767）；行程或速度超过范围时需上层缩放/分段。
- STM32 为小端；6 字节队列项对应 `uint64_t` 的低 48 位。
- 发送示例：

  ```c
  uint64_t message = ((uint64_t)0x1U << 46)
                   | ((uint64_t)(uint16_t)target_position << 16)
                   | (uint64_t)(uint16_t)target_diff_position;
  (void)osMessageQueuePut(ControlqueueHandle, &message, 0U, 0U);
  ```

### 状态机读取

`PIDTask_ProcessControlQueue()` 不是每个 4 ms 基础 tick 都调用；
`PIDTask_UpdateControlQueue()` 每个基础 tick 检查 `s_control_queue_counter`，
仅在计数为 0 时读取，因此实际频率为 **25 Hz（40 ms）**。它在状态机之前执行，
读取后立即更新状态和目标，再由同一 tick 的 `switch` 进入新状态。

读取逻辑：

1. `osMessageQueueGetCount(ControlqueueHandle) == 0` → 队列为空，直接返回；
2. 非空 → `osMessageQueueGet()` 非阻塞读取 48 位消息；
3. 按最高两位切换：
   - `00` → `PID_STATE_STOP`，三个控制器复位；
   - `01` → 写入目标共模位移/差模旋转量，锁存共模/差模起点，进入 `PID_STATE_POSITION`；
   - `10` → 写入目标速度/差模速度，进入 `PID_STATE_SPEED`；
   - `11` → 按停止处理。

## IMU 卡尔曼滤波（imu_task.c）

### 算法与来源

采用 **TKJ Electronics 的二状态离散卡尔曼滤波**（状态：俯仰角 `pitch`、陀螺零偏 `bias`），
这是 MPU6050 平衡车/四轴最常用的轻量融合算法：

- 预测：
  - `pitch_k = pitch_{k-1} + (gyro - bias_{k-1}) * dt`
  - `P_k = F * P_{k-1} * F^T + Q`，`F = [[1, -dt], [0, 1]]`，
    `Q = diag(Q_angle*dt, Q_bias*dt)`
- 更新：
  - `K = P_k * H^T / (H * P_k * H^T + R)`，`H = [1, 0]`
  - `x_k = x_k + K * (z - pitch_k)`
  - `P_k = (I - K * H) * P_k`
- 输入：
  - `z`：加速度计倾角 `atan2(-ax, sqrt(ay^2 + az^2))`，单位 deg；
  - `gyro`：绕模块 Y 轴的角速度，单位 deg/s；
  - `dt`：实际采样间隔；正常 2 ms，按相邻成功帧 DMA 启动时刻的 DWT 时间戳差计算，
    最大 20 ms；TIM7 2 ms 计数保留用于 DWT 不可用时的回退和 20 s 回绕保护。

> 只做 pitch 单轴融合，不做三轴四元数/EKF：平衡车只需要俯仰角，
> 二状态滤波器在 Cortex-M4F 上单次约几十条浮点指令，500 Hz 无压力。
> 若以后要做全姿态，再换 Mahony/Madgwick 或四元数 EKF。

### 参数（deg 体系）

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `Q_angle` | `0.001` | 角度过程噪声；越大越信加速度计，响应快但抖动大 |
| `Q_bias` | `0.003` | 零偏过程噪声；越大零偏跟踪快，但易被线加速度带偏 |
| `R_measure` | `0.03 deg²` | 静止测量噪声，约 `0.17 deg` 标准差；TKJ/STM32 HAL 参考值 |
| `P0_angle` | `1.0 deg²` | 首帧角度来自含噪加速度计，不能像 TKJ 示例那样设 0 |
| `P0_bias` | `0.01 (deg/s)²`（校准成功）/ `1.0`（未校准） | 校准后零偏初值可信 |
| 自适应 R | `0.03 → 0.6 deg²` | `abs(accel_norm - 1g)` 超过 0.05g 后按 20 倍/g 放大，抑制加减速/振动 |
| 零偏限幅 | `±30 deg/s` | MPU6050 初始零偏约 ±20 dps，防止发散 |

参数来源与对照：

- TKJ 官方 `Kalman.cpp`：`Q_angle=0.001`、`Q_bias=0.003`、`R_measure=0.03`、`P=0`；
  本项目保留前三个值，但把 `P` 改为非零，原因见上表。
- STM32 HAL 版 `leech001/MPU6050`：同样使用 TKJ 的这组参数，并用 `HAL_GetTick()`
  计算 `dt`；本项目改用 DWT CYCCNT 记录每帧 DMA 启动时刻，按相邻成功帧的 DWT
  差值计算 `dt`，分辨率约 6 ns；DWT 不可用时才回退到 TIM7 2 ms 计数。
- TKJ `Balanduino` 平衡车：上电调用 `calibrateGyro()` 校准 `gyroXzero`，并用
  `micros()` 计算实际 `dt`；本项目的 DWT CYCCNT 和 500 点静止校准对应这两点。
- 国内博客常见另一组 `Q_angle=0.001, Q_gyro=0.003, R=0.5, P=[[1,0],[0,1]]`；
  其 R 更保守，但部分文章的协方差预测公式写错（把 Q 直接相加、漏掉陀螺输入），
  不要直接照抄；若实车加速度噪声大，可先把 `R_measure` 调到 `0.1~0.5` 试。

### 上电零偏校准

`IMU_Init()` 在配置 MPU6050 后执行 `IMU_CalibrateGyro()`：

1. 连续读 500 帧（约 1 s，`HAL_Delay(2)`），取 pitch 轴陀螺均值；
2. 采样期间角速度峰峰值必须 `< 5 deg/s`，否则判为“被碰动”，自动重试，最多 3 次；
3. 成功则 `bias = 均值`、`P0_bias=0.01`；失败则 `bias=0`、`P0_bias=1.0`，
   由卡尔曼在线学习。

> 上电后约 1 s 内保持小车静止。校准失败不会阻止 IMU 启动，但零偏收敛会慢一些，
> 可通过 `KalmanData_t.gyro_bias` 观察。校准在初始化任务（defaultTask）中阻塞执行；
> 当前 defaultTask 栈为 256 words（1 KB），可容纳校准期间的 HAL I2C 阻塞调用。
>
> MPU6050 数据手册给出的 Initial ZRO Tolerance 为 ±20 dps，ZRO 随温度变化也可达
> ±20 dps；上电校准只解决常温初始零偏，滤波器仍保留 `bias` 状态跟踪温漂。

### 启动同步（重要）

`IMU_Init()` 在 defaultTask（优先级 44）中运行，初始化期间 PID（28）和 IMUTask（32）
都不能抢占它，校准可以相对完整地跑完。但 defaultTask 挂起后，PID 可能先于 IMU
首帧运行，仍存在启动风险：

- TIM7 刚启动或尚未启动，`kalmaneddataQueue` 为空；
- `PIDTask_Init()` 会把 `s_pid_input` 清零，`current_pitch = 0`；
- 如果 Controlqueue 在启动阶段收到位置/速度命令，PID 可能用 `pitch=0` 进入
  `PID_STATE_POSITION` 或 `PID_STATE_SPEED` 并计算 PWM。

代码已在 `StartPIDTask()` 进入控制循环前加入启动门控：

```c
while (IMU_IsReady() == 0U)
{
  (void)osDelay(10U);
}
```

`IMU_IsReady()` 在 `IMU_Process()` 首次成功发布数据后置 1。PID 在 defaultTask
挂起后才开始运行，门控期间通过 `osDelay()` 让出 CPU，让 IMUTask 完成首帧。
等待放在 `PIDTask_Init()` 之前，避免 `s_pid_wake_tick` 在等待期间变旧、
随后连续追赶多个周期。若 IMU 初始化失败，PID 会一直等待，输出保持关闭；
这是有意的安全行为。

> defaultTask 现在已经是最高优先级（44），初始化期间不会被 PID 抢占；代价是
> `HAL_Delay()` 忙等会在约 1 s 内饿死 PID/IMU，所以初始化完成后必须永久挂起。
> 即使优先级调高，IMU ready 门控仍然需要：defaultTask 挂起后 PID 可能先于
> IMU 首帧运行；若 IMU_Init 失败，PID 也应当保持输出关闭。

### 安装方向

默认假设：模块 **X 轴朝车头、pitch 绕模块 Y 轴**，即

```c
accel_pitch = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * RAD_TO_DEG;
gyro_pitch_rate = gy_dps;
```

若只是前后装反（绕 Z 转 180°），改 `IMU_PITCH_SIGN`（`+1.0f` / `-1.0f`），
让 `pitch` 与 `pitch_rate` 同时取反；不要只改其中一个，否则内环阻尼符号会错。
若模块绕 X/Y 翻转、pitch 改绕其他轴，或 Z 轴不是朝上，必须同时修改加速度计公式
和陀螺仪读取轴，不能只改一个符号宏。
若模块有固定安装倾角，用 PID 的 `pitch_ref` 做零点补偿，不要改滤波公式。

### 调参建议

| 现象 | 调整 |
| --- | --- |
| 角度跟随慢、刹车/启动时滞后 | 增大 `Q_angle`（0.001 → 0.002~0.005） |
| 静止时角度缓慢漂移 | 减小 `Q_bias`（0.003 → 0.001）或增大 `R_measure` |
| 静止时角度抖动/电机振动串入 | 增大 `R_measure`（0.03 → 0.1~0.5），或降低 DLPF 带宽 |
| 加减速时角度被“拉走” | 保持/增大自适应 R 的 `R_MOTION_GAIN`，或降低 `R_SCALE_MAX` |
| 角度出现快速跳变 | 检查 `IMU_PITCH_SIGN`、加速度计量程和 `ACCEL_LSB_PER_G` |

> 所有 Q/R 都是 **deg 体系**；若改成 rad，R 要乘 `(pi/180)^2`，Q 也要换算，
> 否则参数含义完全变了。

### 参考链接

- TKJ Electronics, *A practical approach to Kalman filter and how to implement it*：
  <https://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/>
- TKJ Electronics, `KalmanFilter`（C++ 参考实现）：
  <https://github.com/TKJElectronics/KalmanFilter>
- TKJ Electronics, `Balanduino`（平衡车实车使用，含 `calibrateGyro()`）：
  <https://github.com/TKJElectronics/Balanduino>
- `leech001/MPU6050`（STM32 HAL + TKJ 卡尔曼）：
  <https://github.com/leech001/MPU6050>
- 中文对照（参数顺序/P=1 的另一种写法，公式需甄别）：
  <https://www.cnblogs.com/zghr/p/19449657>
- IEEE, *2024 15th International Conference on Computing Communication and Networking
  Technologies (ICCCNT)*：*Analysis of the Kalman Filter with the MPU6050 Accelerometer
  and Gyroscope*，<https://doi.org/10.1109/icccnt61001.2024.10724704>

## 强制编码规则

### 规则 1：主循环只允许“状态机 + 函数调用”

任务的 `for (;;)` 内**只允许**出现以下两类内容：

1. **状态机**：`switch` / `if` 的状态分支；
2. **函数调用**：状态处理、数据更新、延时等。

禁止在 `for (;;)` 内直接写业务算法、寄存器操作、忙等待、长循环或大段计算。
所有具体动作都必须封装成函数，循环中只保留调用。

```c
for (;;)
{
  PIDTask_UpdateIMU();                  /* 函数调用：取 Kalman 最新姿态 */
  PIDTask_UpdateControlQueue();         /* 函数调用：25 Hz 轮询 Controlqueue */

  switch (s_pid_state)                  /* 状态机 */
  {
    case PID_STATE_POSITION: PIDTask_StatePosition(); break;
    case PID_STATE_SPEED:    PIDTask_StateSpeed();    break;
    case PID_STATE_STOP:     PIDTask_StateStop();     break;
    default:                 PIDTask_StateStop();     break;
  }

  PIDTask_DelayUntil();                 /* 函数调用：任务节拍 */
}
```

### 规则 2：函数实现统一放在文件末尾

任务入口函数 `StartXXXTask()` 之后不再穿插具体实现；所有内部函数集中放在文件末尾的
“以下为函数实现”区，并按固定格式注释**功能与输入输出**：

```c
/*
 * 功能：函数用途（一句话说明）
 * 输入：输入参数及含义（无输入写“无”）
 * 输出：返回值 / 对外影响（无输出写“无”）
 */
static void XXXTask_YYY(void)
{
  /* ... */
}
```

### 规则 3：其他约定

- 任务入口固定为 `void Start<Name>Task(void *argument)`，必须与 `freertos.c` 中
  `osThreadNew()` 使用的函数名一致。
- 内部函数、状态变量一律 `static`，只在任务文件内可见；头文件只暴露任务入口、
  周期宏和必要的对外接口。
- 状态用 `typedef enum` 定义，状态变量命名 `s_<name>_state`，状态处理函数命名
  `<Name>Task_State<Xxx>()`。
- 任务周期用宏定义（如 `PID_TASK_PERIOD_MS`），循环末尾只调用一次延时函数；
  延时统一使用 `osDelayUntil()` 按绝对 tick 定时，避免周期累计误差：

  ```c
  s_pid_wake_tick += pdMS_TO_TICKS(PID_TASK_PERIOD_MS);
  (void)osDelayUntil(s_pid_wake_tick);
  ```

  首次唤醒 tick 在任务初始化时用 `osKernelGetTickCount()` 取得，之后每周期只做累加。
- 任务参数若不使用，在入口函数内写 `(void)argument;`，避免编译告警。
- 源代码注释使用中文，统一保存为 UTF-8 编码；若 Keil 中显示乱码，请在
  `Edit > Configuration > Editor > Encoding` 中选择 UTF-8。

## 新增任务流程

1. 在 CubeMX 中新建 Task，`Code Generation` 选项选择 **As external**；生成后
   `freertos.c` 中会出现 `extern void StartXXXTask(void *argument);` 和对应的
   `osThreadNew(...)`。
2. 在本目录新建 `xxx_task.c` / `xxx_task.h`，按上述规则实现 `StartXXXTask()`、
   状态机与内部函数。
3. 在 MDK-ARM 工程中把 `.c` 文件加入工程（Add Files to Group）。
   > 注意：当前 `elet2026.uvprojx` 尚未加入 `tasks` 目录，编译前必须完成这一步。
4. 若其他目录需要包含本目录的头文件，请在 Keil 的 `Include Paths` 中加入
   `../tasks`。

## 参考实现

`pid_task.c` 是当前的标准示例，包含：

- 任务状态枚举与状态变量；
- `StartPIDTask()` 入口与“状态机 + 函数调用”的主循环；
- 三个状态处理：`PIDTask_StateStop()`、`PIDTask_StatePosition()`、
  `PIDTask_StateSpeed()`；
- `PIDTask_UpdateControlQueue()` 在任务循环中、状态机之前按 25 Hz 调用
  `PIDTask_ProcessControlQueue()`，读取 CubeMX 生成的 `Controlqueue`
  （深度 1、6 字节）；
- `dwt_time.c` / `dwt_time.h`：共享的 `DWTTime_Init()`、`DWTTime_GetCycles()`、
  `DWTTime_CalculateDt()`；PID 和 IMU 共用一个 DWT 时间基准；
- `PIDTask_InitHighResClock()` / `PIDTask_CalculateDt()` 使用 DWT CYCCNT
  为三个环计算实测 `dt`；`PIDTask_CheckTimestampOverflow()` 用 1 ms tick
  防止 32 位 DWT 回绕差值被误用；DWT 不可用时回退标称值；
- IMU 链路在 `imu_task.c`：TIM7 → I2C DMA → 信号量 → Kalman（含上电零偏校准）→ 双缓冲 → `kalmaneddataQueue`；
- `SETPWM()` 仍是待填 HAL 的空实现；
- 文件末尾集中放置的状态处理、输出、硬件抽象与延时函数，并附
  功能 / 输入 / 输出 注释。

三个控制器函数、三个参数结构体实例（`s_position_params`、`s_speed_params`、
`s_attitude_params`）以及控制器内部状态都集中在 `pid_task.c` 中；
结构体类型、宏与函数声明在 `pid_task.h` 中。

`imu_task.c` 提供 MPU6050 采集、卡尔曼滤波、双缓冲与 `kalmaneddataQueue` 发布，
卡尔曼 `dt` 使用共享 DWT 模块实测，DWT 不可用时回退 TIM7 计数；同样按
“函数实现在文件末尾 + 功能 / 输入 / 输出注释”组织。
