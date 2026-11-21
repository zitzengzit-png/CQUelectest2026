# tasks —— FreeRTOS 任务实现

本目录存放各 FreeRTOS 任务的实现。任务本身在 STM32CubeMX 的 FreeRTOS 配置中创建，
创建方式选择 **As external**；任务入口函数在本目录的 `.c` 文件中实现，由
`Core/Src/freertos.c` 中的 `osThreadNew()` 注册并启动。

> 当前 PID 层级、频率、输入输出与模式差异见
> [`PID_CONTROL_DIAGRAM.md`](PID_CONTROL_DIAGRAM.md)。

> OLED 每个缩写、单位和符号说明见
> [`DISPLAY_LEGEND.md`](DISPLAY_LEGEND.md)。

> controlTask 的状态机、巡线与对齐公式见
> [`CONTROL_TASK.md`](CONTROL_TASK.md)；状态机/巡线参数在 `control_task.c` 顶部。

> **硬件与 PID 可调参数集中到 [`robot_config.h`](robot_config.h)**：PID 增益/限幅、
> 任务频率、PID 环频率、Controlqueue 轮询频率、IMU/编码器/OLED 参数都在这里。
> 其他任务头文件会自动包含它；调参时优先只改这个文件。

## 当前架构（2026-11，ATK-MS901M 九轴）

> 本工程已从 MPU6050 + I2C1/TIM7 方案切换为 **ATK-MS901M 九轴 + USART3 DMA**。
> 下文旧的 MPU6050、TIM7、IMUtaskBinarySem、kalmaneddataQueue 描述仅作历史参考。

| 任务/模块 | 入口 / 文件 | 优先级 | 周期 | 说明 |
| --- | --- | --- | --- | --- |
| 初始化 | `StartDefaultTask` / `Core/Src/freertos.c` | `osPriorityHigh4`（44） | 一次性 | `IMU_Init()`：USART3 921600、250 Hz、姿态 + 陀螺/加速度 + 磁力计、九轴；成功后永久挂起 |
| PID | `StartPIDTask` / `pid_task.c` | `osPriorityNormal4`（28） | 由 `IMUdatadoneBinarySem` 驱动，约 250 Hz | 位置模式：位置/速度 + yaw 角度环 → 内环；速度模式：速度 + 目标 yaw rate → 内环；姿态模式（11）：只跑内环；编码器只共模 |
| 主控 | `StartcontrolTask` / `control_task.c` | `osPriorityLow5`（13） | 20 ms（50 Hz） | 五流程（调参/原地停止/路径/巡线对齐/四次 zigzag）、按键、巡线、对齐、蜂鸣器 |
| USART3 RX DMA | DMA1 Stream1，Normal，28 B | IRQ 5 | 250 Hz | 完成回调拷贝 `s_imu_snapshot`、释放 `IMUdatadoneBinarySem`，再挂起下一次 DMA |
| 编码器 | `Encoder_Init/Update` / `encoder.c` | — | 50 Hz（PID 中环调用） | TIM3 右/TIM4 左，TI12 **4 倍频**，CNT 初值 6554；`Encoder_Update` 在读取时换算并返回共模位置(m)/速度(m/s)，PID 内部写入私有输入 |
| 巡线传感器 | `LineSensor_ReadLevels` / `line_sensor.c` | — | 50 Hz（controlTask 调用） | 前 F1-F8、后 B1-B8 共 16 路 GPIO 输入；controlTask 保存前后各 8 个原始电平数组，并生成“检测到线”的位掩码 |
| 显示 | `StartdisplayTask` / `display_task.c` | `osPriorityLow`（8） | 50 ms（20 Hz） | 消费 `dataToLCDQueue`，刷新 128×64 OLED；PID 按值发送 68 字节完整快照 |

### 八路巡线传感器

- 前 8 路 F1-F8，后 8 路 B1-B8，均为 GPIO 输入；
- `line_sensor.c` 只读电平，`controlTask` 每 20 ms 调用一次；
- `ControlTask_GetFrontSensorLevels()` / `GetBackSensorLevels()` 返回原始电平数组；
- `ControlTask_GetFrontLineMask()` / `GetBackLineMask()` 返回按 `LINE_SENSOR_ACTIVE_LEVEL` 归一化的位掩码，bit0 = x1；
- 数组下标 0..7 对应 x1..x8；默认低电平有效，实测相反时改 `robot_config.h`；

### controlTask 状态机

`control_task.c` 目前实现五个顶层流程，由全局 `currentstatemachine` 选择：

| 值 | 状态机 | 摘要 |
| --- | --- | --- |
| 0 | IDLE | 占位 |
| 1 | TUNING | KEY0 切 6 个子模式，每次进入发一次命令 |
| 2 | HOLD | `POSITION(0,0)` 原地保持 |
| 3 | RUN | 直行 110 cm，F1~F8 黑线 → `POSITION(0,0)` + 蜂鸣 1 s |
| 4 | SM4 | 直行 + 巡线 + 后退 5 cm + 对齐 + 二次巡线停车 |
| 5 | SM5 | 四次 zigzag：右转/130 cm/巡线/后退/左转/二次巡线，编码器到点 |
| 6 | STOP | 占位 |

- KEY_UP（PA0）按 1 → 2 → 3 → 4 → 5 → 1 循环，同时把 `currentmode` 复位为 0；
- KEY0（PE4）切 `currentmode`；
- TIM11_CH1/PF7 驱动蜂鸣器 + LED，2500 → 50 Hz 频率扫描，1 s `osTimerOnce`；
- 巡线用前后 8 路加权中心的共模/差模偏差：yaw rate 正比例 P，速度反比例 P；
- 每次开始巡线记录编码器共模起点，行进 30π cm 后最大速度线性降到 40π cm 的 50%；
- 对齐用前后中心差做 P，`SPEED(0, yaw rate)`，连续 10 帧在容差内通过；
- 状态机 5 的 130 cm 到点用编码器共模位置（±3 cm），后退 5 cm 用 ±1 cm，
  转角按 yaw 零点 + 51.3°（连续 3 帧误差 ≤2°），连续执行 4 个周期。

详细阶段表、公式和参数见 [`CONTROL_TASK.md`](CONTROL_TASK.md)。

OLED 第 0 行显示实际运行频率：

```text
POS RUN P250 C50 E0
```

- `P250`：PID 基础循环实际频率；
- `C50`：controlTask 实际频率；
- 频率用两端的 DWT 时间差和任务运行计数差计算；
- 由于单次显示间隔只有 50 ms，controlTask 50 Hz 只有 2~3 次计数，直接相除会量化成
  40/60 Hz；代码默认使用 **8 个显示周期（约 400 ms）** 的统计窗口，仍用 DWT 实测
  窗口时间；
- 末尾 `!` 表示 IMU 已停更；`E0` 是 IMU/I2C/队列错误计数（超过 9 显示为 `9`）。

数据链：

```text
USART3 921600
  → DMA1 Stream1（28 B）
  → HAL_UART_RxCpltCallback()
  → memcpy 到 s_imu_snapshot + osSemaphoreRelease(IMUdatadoneBinarySem)
  → PID 任务：IMU_Parse()（0x55 0x55 + 累加和校验）
  → IMU_GetData()
  → 三环控制
```

IMU 周期数据：`0x01` 姿态 11 B + `0x03` 陀螺/加速度 17 B = **28 B/周期**。
九轴 Yaw 已在模块内部融合磁力计，控制不需要 `0x04` 原始磁力计帧；当前 `RETURNSET=0x05`。

> `IMUdatadoneBinarySem` 初值为 **0**；PID 只在 DMA 完成中断释放后运行。
> `defaultTask` 的 44 只用于初始化；PID 的 28 低于 defaultTask、高于 controlTask（13）。
> defaultTask 初始化并挂起后，PID 才成为运行中的最高优先级任务。

### 当前转向架构（2026-11）

- **位置模式**：第一次进入时锁存当前九轴 yaw 作为零点；同一位置模式内，
  新命令的 `value2` 直接覆盖目标角（相对该零点），不累加、也不相对当前实测；
- 位置模式 `value2` = 目标 yaw 角，单位 **0.1°**，逆时针为正；
- **速度模式**：`value2` = 目标 yaw rate，单位 **1°/s**（180 = 180°/s）；
  速度模式**不使用 yaw 零点**，也不做 yaw 角度环；
- **姿态模式（11）**：`value1` = 目标 Pitch（0.1°），`value2` = 目标 yaw rate（1°/s）；
  只运行 250 Hz 内环，用于调参；
- 外环只算共模直行距离 -> 目标速度；转向目标直接来自 `value2`；
- 位置模式外环 10 Hz：共模位置 PD -> 目标速度；
- 位置模式中环 50 Hz：共模速度 PD -> 目标倾角；目标 yaw 角 PD -> 目标 yaw rate；
- 速度模式中环：`value2` 直接限幅后作为目标 yaw rate；
- 内环：yaw rate PI -> 差模 PWM；
- 编码器只提供共模位置/速度；**不使用编码器差模，不使用 yaw 2 状态 Kalman**；
- yaw 角度直接使用 ATK-MS901M 九轴 yaw，yaw rate 直接使用陀螺 Gz。

## PID 任务控制方案

> **2026-11 转向架构**：位置外环只算共模直行距离 -> 目标速度；位置模式的目标转向角
> 由 Controlqueue `value2`（0.1°，相对首次进入位置模式的零点）给出；速度模式
> `value2` 是目标 yaw rate（1°/s），不使用 yaw 零点。位置模式中环做 yaw 角度 PD ->
> 目标 yaw rate 并限幅；内环做 yaw rate PI -> 差模 PWM。编码器只提供共模
> 位置/速度，不再使用编码器差模，也不再使用 yaw 2 状态 Kalman。
>
> 下面 1~4.1 小节的“差模位置 PD / 差模目标速度 / yaw Kalman”是旧方案，
> 仅作历史参考；其中“脉冲/脉冲每秒”单位也已过时，当前实现使用 m / m·s⁻¹。

PID 任务采用**三环串级 + yaw rate 内环**结构：

- **最外环（位置）**：目标共模位移（脉冲）、目标差模旋转量（脉冲） → 目标速度（脉冲/s）、目标差模速度（脉冲/s）；
- **中环（速度/姿态）**：共模速度 PD → 目标倾角；差模目标速度经 `PID_YAW_DEG_PER_DIFF_PULSE` 换算为目标 yaw rate（deg/s）；
- **内环**：目标倾角 → 共模 PWM；目标 yaw rate → 差模 PWM（PI，反馈来自 MPU6050 `gz`）。

> **单位约定**：编码器位置、位置差、速度仍为脉冲、脉冲/s；yaw 目标/反馈为 deg/s。
> 位置模式的 `value2 / psi_ref` 仍是差模编码器脉冲；速度模式的 `value2` 仍是差模脉冲/s，
> 中环统一换算成目标 yaw rate。yaw rate 在 PID task 内做 2 状态 Kalman：
> 50 Hz 用编码器 yaw 角度修正陀螺零偏，250 Hz 内环使用融合后的 yaw rate。

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
- `psi` 由左右编码器计数差得到（脉冲），与 IMU 无关；IMU 只参与下面的姿态/yaw 内环。

### 2. 中环：速度/姿态

| 项目 | 内容 |
| --- | --- |
| 目标输入 | 目标速度 `v_ref`（脉冲/s）、目标差模速度 `n_diff_ref`（脉冲/s） |
| 反馈输入 | 当前共模速度 `v`（脉冲/s） |
| 控制器 | 共模速度 **PD**；差模目标速度换算为目标 yaw rate |
| 输出 | 目标倾角 `pitch_ref`（deg）、目标 yaw rate `omega_ref`（deg/s） |

- 共模速度 PD：`e_n = v_ref - v`，
  `pitch_ref = Kp_n * e_n + Kd_n * de_n/dt`（限幅）。
- `e_n > 0`（目标速度高于当前速度）时输出正的目标倾角（前倾），使车体向前加速；
  `e_n < 0` 时输出负的目标倾角。
- 差模通道：`omega_ref = clamp(n_diff_ref * PID_YAW_DEG_PER_DIFF_PULSE,
  ±PID_YAW_MAX_RATE_DPS)`。
- 位置模式（Controlqueue 模式 01）：外环差模位置 PD 给出 `n_diff_ref`；
  中环换算成目标 yaw rate。
- 速度模式（Controlqueue 模式 10）：`value2` 就是 `n_diff_ref`（脉冲/s）；
  中环同样换算成目标 yaw rate。
- 内环再用 `omega_ref - current_yaw_rate` 做 yaw rate PI，输出差模 PWM。

### 3. 内环：目标姿态/yaw rate → PWM

| 项目 | 内容 |
| --- | --- |
| 目标输入 | 目标倾角 `pitch_ref`（deg）、目标 yaw rate `omega_ref`（deg/s） |
| 反馈输入 | 当前倾角 `pitch`（deg，IMU）、当前 yaw rate `omega`（deg/s，IMU `gz`） |
| 控制器 | 共模：倾角 PD；差模：**yaw rate PI** |
| 输出 | 共模 PWM `PWM_cmd`、差模 PWM `PWM_diff` |

- 共模 PWM：`e_theta = pitch - pitch_ref`，
  `PWM_cmd = Kp_theta * e_theta + Kd_theta * pitch_rate`（已含陀螺角速度阻尼）。
- 误差方向：`pitch > pitch_ref`（前倾超过目标）时输出正 PWM，驱动轮子向前加速；
  若 IMU 安装方向相反，`pitch` 与 `pitch_rate` 需同时取反。
- 差模 PWM：`e_yaw = omega_ref - current_yaw_rate`，
  `PWM_diff = Kp_yaw * e_yaw + Ki_yaw * ∫e_yaw dt`。
- 合成左右 PWM：`PWM_L = PWM_cmd - PWM_diff`，`PWM_R = PWM_cmd + PWM_diff`
  （符号按实车验证）。

### 4. 信号流

```
s_ref ──► [共模位置 PD] ──► v_ref ──► [中环 速度 PD] ──► pitch_ref ──► [内环 倾角 PD] ──► PWM_cmd
psi_ref ► [差模位置 PD] ──► n_diff_ref ──► [× PID_YAW_DEG_PER_DIFF_PULSE] ──► omega_ref
          ──► [内环 yaw rate PI] ──► PWM_diff

反馈：编码器共模 → s；编码器共模速度 v、差模速度 n_diff_actual（保留给后续融合）；
      编码器差模 → psi；IMU pitch、pitch_rate、yaw_rate 进姿态/yaw 内环
输出：PWM_L = PWM_cmd - PWM_diff，PWM_R = PWM_cmd + PWM_diff
```

### 4.1 yaw 2 状态 Kalman（PID task，50 Hz）

- 状态：`x = [θ_yaw, b_gz]`，角度 deg、零偏 deg/s；
- 预测：`θ += (gz - b) * dt`，`b` 随机游走；
- 测量：编码器差模位置积分成 yaw 角度，`θ_enc += ΔN_diff * PID_YAW_DEG_PER_DIFF_PULSE`；
- 更新：50 Hz，在位置/速度中环中调用 `PIDTask_UpdateYawKalman(dt)`；
- 输出：`s_yaw_rate_fused = gz - b`，250 Hz 内环使用；
- 打滑门控：编码器 yaw rate 与陀螺 yaw rate 偏差过大时放大 `R_enc`；
- 参数：`PID_YAW_KALMAN_*`、`PID_YAW_SLIP_*` 在 `pid_task.h`；
- 队列保持 16 字节：融合在 PID task 本地完成，不把 yaw angle 放回 `kalmaneddataQueue`。

### 5. 输入与职责边界

| 模块 | 职责 |
| --- | --- |
| 上层路径/指令模块 | 通过 `Controlqueue` 给出目标共模位移(mm)/速度(mm/s)；位置模式 `value2` = 0.1° 目标 yaw 角（相对首次进入位置模式的零点），速度模式 `value2` = 1°/s 目标 yaw rate |
| PID 任务（本任务） | 位置模式：位置/速度 + yaw 角度环 → 内环；速度模式：速度 + 目标 yaw rate → 内环；`Controlqueue` 按 50 Hz 读取；位置 m、速度 m/s |
| 编码器 | `encoder.c`：TIM3 右/TIM4 左，PID 中环 50 Hz 调用 `Encoder_Update(&pos_m, &speed_mps)`；CNT 初值 6554，PID 内部写入私有输入 |
| IMU | ATK-MS901M 九轴：USART3 DMA → `IMUdatadoneBinarySem` → PID 内 `IMU_Parse()` |
| 电机驱动 | `SETPWM()`：TIM2 PWM + LF/LB/RF/RB 方向；compare=0 短路刹车 |

> **四倍频与米制**：TIM3/TIM4 为 `TIM_ENCODERMODE_TI12`，计数为 4 倍频；
> `ENCODER_COUNTS_PER_REV = 385 × 4 = 1540`，`WHEEL_DIAMETER_M` 为轮径，
> `ENCODER_METERS_PER_COUNT = π × 轮径 / 1540`。
> `Encoder_Update()` 已把共模位置/速度转成 **m / m·s⁻¹**，PID 内部不再使用脉冲。
> `WHEEL_TRACK_M` 为轮距，当前共模控制暂未使用，预留给后续差速/里程计。
>
> Controlqueue 仍是 int16：位置 `value1` 单位 **mm**，速度 `value1` 单位 **mm/s**；
> 进入 PID 后分别乘 `PID_CONTROL_DISTANCE_UNIT_M` / `PID_CONTROL_SPEED_UNIT_MPS`
> 转成 m / m·s⁻¹。角度单位仍为 deg / deg·s⁻¹。

### 6. 执行周期与参数

#### 多环调度

PID 基础任务按 `PID_TASK_PERIOD_MS = 4 ms` 运行（250 Hz），三环不创建额外任务，
而是共用 `pid_task.c` 内的软件计数器 `s_pid_loop_counter` 分频：

| 环 | 频率 | 周期 | 分频条件 | 标称 `dt`（实际由 DWT 测量） |
| --- | --- | --- | --- | --- |
| 内环（姿态/yaw rate） | 250 Hz | 4 ms | 每个基础 tick | `PID_INNER_DT_S = 0.004 s` |
| 中环（速度） | 50 Hz | 20 ms | `tick % 5 == 0` | `PID_MIDDLE_DT_S = 0.02 s` |
| 外环（位置） | 10 Hz | 100 ms | `tick % 25 == 0` | `PID_OUTER_DT_S = 0.1 s` |

- 表中的 `dt` 是标称值，只用于 DWT 不可用时的回退；正常运行时三个环分别用
  `DWT->CYCCNT` 实测两次执行之间的真实间隔。
- `PIDTask_NextLoopTick()` 返回当前控制环 `tick`（`0 ~ 24`），然后把计数器推进
  一格；计到 25 自动回零。
- `PIDTask_UpdateControlQueue()` 在 `StartPIDTask()` 的每个基础循环中调用：
  `PIDTask_UpdateIMU()` 之后、状态机 `switch` 之前按 50 Hz 读取 Controlqueue；
  读取后先更新状态/目标，再由 `switch` 进入新状态。
- `PIDTask_StatePosition()` 内顺序固定为：外环（若到期）→ 中环（若到期）→
  内环（每拍）。外环先算，保证同一拍中环能立即使用新的目标速度。
- 收到 POSITION 命令的当拍，`s_force_position_outer` 会强制位置外环立即执行一次，
  即使当前 tick 不是 10 Hz 时隙；中环仍按 50 Hz 调度。
- POSITION 是**锁存状态**：队列为空不会自动退出位置模式，外环继续按 10 Hz 运行，
  直到收到新的 STOP/SPEED/ATTITUDE/POSITION 命令。
- `PIDTask_StateSpeed()` 只运行 50 Hz 中环和 250 Hz 内环，跳过位置外环。
- 环间输出保存在 `s_outer_target_speed`、`s_middle_target_pitch`、
  `s_middle_target_yaw_rate` 中；未到执行周期的环沿用上一次结果，不能用栈上的
  临时变量。

#### Controlqueue 轮询（50 Hz）

`Controlqueue` 与三环解耦，固定按 **50 Hz（20 ms）** 读取，即每 5 个 250 Hz
基础 tick 读取一次：

- 计数器：`s_control_queue_counter`，范围 `0 ~ 4`；
- `PIDTask_UpdateControlQueue()` 每个基础 tick 调用一次，只有计数器为 0 时才调用
  `PIDTask_ProcessControlQueue()`；
- 读取后立即更新 `s_pid_state`、目标位置/目标速度和目标 yaw rate；随后同一个基础
  tick 的 `switch` 会进入新状态；
- IMU 仍按 250 Hz 读取；Controlqueue 为 50 Hz；
- 计数器在所有状态下持续运行，不随模式切换清零；控制环计数器
  `s_pid_loop_counter` 仍在模式切换时清零。

由此：

- 命令检测延迟最多约 20 ms；
- 如果命令只更新当前模式的目标：POSITION 命令当拍强制外环，位置目标立即生效；
  速度目标在下一次 50 Hz 中环生效（最多再加 20 ms）；
- 如果命令切换状态：当前基础 tick 就进入新状态；位置/速度状态的控制环计数器
  已清零，第一拍从 `tick 0` 开始；
- STOP 命令也在同一个 50 Hz 读取点处理，处理后当前 tick 进入 `PID_STATE_STOP`
  并输出 0。若对停止响应有更高要求，需单独设计紧急停止通道，不能只依赖 50 Hz
  队列轮询。

#### 实测 dt（DWT CYCCNT）

PID 的 250 Hz 基础节拍由 USART3 RX DMA 完成信号量驱动；实际间隔由 IMU 帧到达时刻决定，
不严格等于 4 ms。因此控制器不使用固定 `dt`，而是用 Cortex-M4 的 DWT 周期计数器测量：

- 共享 DWT 初始化只由 `PIDTask_Init()` 通过 `DWTTime_Init()` 完成；IMU 不读 DWT，
  编码器在 PID 中环调用 `Encoder_Update()` 时读 DWT，所有 DWT 读取都在 PID 上下文：
  - 第一次调用置 `CoreDebug->DEMCR` 的 `TRCENA`，清零并使能 `DWT->CYCCNT`；
  - `DWTTime_Init()` 幂等，运行中不会再次清零零点；
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
- PID 不再调用 `osDelayUntil()`：基础节拍由 `IMUdatadoneBinarySem` 驱动，
  每个 USART3 RX DMA 完成执行一拍（约 250 Hz）。
- 三环分频仍使用 `s_pid_loop_counter`；控制器的 `dt` 由 DWT 实测，不假设每拍严格 4 ms。

#### 参数与单位

- 三环的标称 `dt` 只用于 DWT 不可用时的回退；正常运行时使用实测值，不能把
  4 ms 硬编码给 10 Hz/50 Hz 控制器。
- 共模位置/速度在 `Encoder_Update()` 内转换为 m / m·s⁻¹；PID 内部不再使用脉冲，
  位置模式使用 `position_current_m - position_start_m`；当前不使用编码器差模。
- `de_s/dt`、`de_psi/dt`、`de_n/dt` 与 `pitch_rate` 建议低通滤波。
- 前进、左转、前倾的正方向需统一定义；`s_ref`、`psi_ref`、`pitch_ref`、`v_ref`、
  `omega_ref`、PWM 与各积分项均需限幅。

## 控制器函数与状态机

工程使用 C 语言，三个控制器以**函数**形式实现。所有宏、参数结构体、状态枚举与
函数声明集中在 `pid_task.h`；结构体实例、控制器内部状态与函数实现放在 `pid_task.c` 中。

### 控制器函数与参数结构体

| 控制器 | 函数 | 参数结构体 | 输入 → 输出 |
| --- | --- | --- | --- |
| 位置 | `PositionController_Update()` | `PositionControllerParams_t` | 共模起始/当前位置（m）、目标共模位移（m） → 目标速度（m/s） |
| 速度/姿态 | `SpeedController_Update()` | `SpeedControllerParams_t` | 目标/当前共模速度（m/s） → 目标倾角（deg） |
| 姿态/yaw | `AttitudeController_Update()` | `AttitudeControllerParams_t` | 目标倾角（deg）、当前倾角/角速度、目标/当前 yaw rate（deg/s） → 左右 PWM |

每个控制器提供：

- `xxxController_Update()`：按 `dt` 执行一次 PD/PI 计算，结果通过输出指针返回；
- `xxxController_Reset()`：清零微分历史 / 积分；
- `PositionController_Start()`：进入位置模式时同时锁存共模/差模起点
  （`encoder_start = encoder_current`、`encoder_diff_start = encoder_diff_current`）；
- 参数（增益与限幅）统一放在对应的 `xxxControllerParams_t` 结构体中，
  实例为 `s_position_params`、`s_speed_params`、`s_attitude_params`。
- 位置参数结构体保存 `position_start_m/position_current_m`，单位均为 m。

### IMU 数据链路

| 环节 | 实现 |
| --- | --- |
| 定时触发 | TIM7 2 ms → `IMU_OnTimer7()` 启动 `HAL_I2C_Mem_Read_DMA(..., 0x3B, 14)` |
| DMA 完成 | `HAL_I2C_MemRxCpltCallback()` 把原始数据拷贝到 `s_imu_ready[14]`，释放 `IMUtaskBinarySem` |
| 解算 | `IMUTask` 等待信号量 → 解析 accel/gyro → `Kalman_Update()` → 写入 `s_kalman_out` |
| 发布 | `IMU_Publish()` 用 `osMessageQueueReset()` + `osMessageQueuePut()` 把 `KalmanData_t` 按值覆盖进 `kalmaneddataQueue`（深度 1，16 字节） |
| PID 取数 | `PIDTask_UpdateIMU()` 从 `kalmaneddataQueue` 取出结构体副本，不再访问共享缓冲 |

- `IMU_Init()` 在初始化任务中完成 MPU6050 配置（500 Hz、DLPF 94/98 Hz、±1000 dps、±2g），成功后才启动 TIM7。
- `IMU_ReadData()` 已删除，PID 不再直接读 I2C。
- 队列按值传递：IMU 任务发布时把 `KalmanData_t` 复制进队列，PID 取出的是独立副本，不共享缓冲。
- CubeMX 当前的 `IMUtaskBinarySem` 初始计数是 1，`StartIMUTask()` 会先清掉它；
  建议在 CubeMX 中把二值信号量初始计数改为 **0**。
- 初始化任务当前栈为 256 words（1 KB）；`defaultTask` 优先级为 44，初始化完成后
  必须永久挂起，否则会饿死 PID/IMU。
- `IMU_Process()` 默认按“pitch 绕模块 Y 轴、X 轴朝前”解算；实际安装不同，需修改
  `imu_task.c` 中的加速度轴、陀螺轴与符号。

### PWM 硬件抽象

| 函数 | 作用 | 现状 |
| --- | --- | --- |
| `SETPWM(left_wheel, forward, compare)` | 设置单侧电机方向与比较值 | 已实现：TIM2 CH3/CH2 PWM + LF/LB/RF/RB GPIO；STBY=PC1 |

- 左轮：`LF=PD11 -> AIN1`、`LB=PD10 -> AIN2`、`LPWM=PA2/TIM2_CH3`；
- 右轮：`RF=PA4 -> AIN1`、`RB=PA5 -> AIN2`、`RPWM=PA1/TIM2_CH2`；
- `STBY=PC1`：高电平使能，低电平待机；STOP 模式先 PWM=0 再拉低；
- `AIN1=1, AIN2=0` 正转；`AIN1=0, AIN2=1` 反转；
- `compare=0` 时 `AIN1=AIN2=0`，短路刹车；
- `SETPWM()` 首次调用启动 TIM2 两路 PWM；
- **TIM2 必须在 CubeMX 设置合理 Period**：当前 `Period=4294967295` 会导致
  PWM 频率约 0.02 Hz，代码会检测并保持刹车。推荐 `Prescaler=0、Period=4199`（20 kHz）。

- `PIDTask_WriteOutput()` 把有符号 PWM 转成方向 + 比较值，分别调用两次 `SETPWM()`：
  - `left_wheel`：1 = 左轮，0 = 右轮；
  - `forward`：1 = 正转，0 = 反转；
  - `compare`：写入比较寄存器的值。
- 编码器由 `encoder.c` 在 PID 中环读取：`Encoder_Update(&position_m, &speed_mps)`，
  PID 内部写入 `s_pid_input.current_position/current_speed`；位置为累计米值，
  位置模式由 `PositionController_Start()` 锁存起点；**差模量不再使用**。

### 控制模式与状态机

| 状态 | 说明 | 控制器链 |
| --- | --- | --- |
| `PID_STATE_STOP` | 输出 0；`Controlqueue` 由主循环按 50 Hz 轮询 | — |
| `PID_STATE_POSITION` | 位置模式（共模位移 + 相对 yaw 目标） | 外环 10 Hz `PositionController_Update` → 中环 50 Hz `SpeedController_Update` + `YawAngleController_Update` → 内环 250 Hz `AttitudeController_Update` |
| `PID_STATE_SPEED` | 速度控制模式（共模速度 + 目标 yaw rate） | 中环 50 Hz `SpeedController_Update` → 目标 yaw rate 限幅 → 内环 250 Hz `AttitudeController_Update`（跳过位置外环和 yaw 角度环） |

- 模式与目标：全部由 `Controlqueue` 命令决定（不再提供 SetMode / SetTarget 接口）；
- 编码器反馈：`Encoder_Update(&position_m, &speed_mps)` 在 PID 中环 50 Hz 读取 TIM3/TIM4，PID 内部写入私有输入；
- IMU：`PIDTask_UpdateIMU()` 在信号量醒来后通过 `IMU_GetData()` 取最新 `pitch / pitch_rate / yaw_rate / yaw_angle`；
- Controlqueue：`StartPIDTask()` 在 `IMU_Parse()` / `PIDTask_UpdateIMU()` 之后、`switch` 之前调用
  `PIDTask_UpdateControlQueue()`，固定按 50 Hz 读取；读取后立即更新状态/目标，
  同一基础 tick 的 `switch` 进入新状态；
- 若命令只更新当前模式的目标，目标会在下一次对应控制环生效；若命令切换状态，
  当前基础 tick 就进入新状态；
- `for (;;)` 内先 `osSemaphoreAcquire(IMUdatadoneBinarySemHandle, osWaitForever)`，
  再 `IMU_Parse()`、`PIDTask_UpdateIMU()`、`PIDTask_UpdateControlQueue()` 和状态机；
  不再调用 `PIDTask_DelayUntil()`。状态处理函数先用 `PIDTask_NextLoopTick()`
  取调度 tick，再按分频调用 `PIDTask_PositionOuterLoop()` /
  `PIDTask_PositionMiddleLoop()` / `PIDTask_SpeedMiddleLoop()` /
  `PIDTask_AttitudeInnerLoop()`。

## dataToLCDQueue（CubeMX 生成）

- 深度 **1**，队列项 **68 字节**，句柄 `dataToLCDQueueHandle`；
- 68 字节正好等于 `sizeof(LCD_Data_t)`，PID 直接按值发送完整快照：
  ```c
  osMessageQueuePut(dataToLCDQueueHandle, &s_lcd_snapshot, 0U, 0U);
  ```
- 显示任务直接接收完整副本，不需要指针、双缓冲或序号校验：
  ```c
  osMessageQueueGet(dataToLCDQueueHandle, &s_lcd_data, NULL, 0U);
  ```
- 发送频率：PID 侧 250 Hz 累加器分频，平均 **20 Hz**；
- `LCD_QueueItemSizeCheck` 在编译期确认 `sizeof(LCD_Data_t) == 68`；
- `osMessageQueueGetMsgSize()` 在运行时再次确认队列 itemsize，避免
  CubeMX 改了队列大小但代码没同步而导致拷贝越界。

## Controlqueue（CubeMX 生成）

- CubeMX FreeRTOS 队列：`Controlqueue`，深度 **1**，队列项 **6 字节（48 位）**，
  句柄 `ControlqueueHandle`，在 `MX_FREERTOS_Init()` 中创建；
- PID 任务直接使用 `ControlqueueHandle`，不另建队列；
- 固定每 20 ms（50 Hz）读取一次；每次任务循环先更新 IMU，再轮询 Controlqueue，
  最后进入状态机；
- 命令由上层通过 `osMessageQueuePut(ControlqueueHandle, &message, 0U, 0U)` 发送，
  队列深度为 1；发送失败表示上一条尚未被 PID 任务取走。

### 位定义

按 48 位字的第 1 位为 MSB：

| 位（MSB 起） | 位（LSB 编号） | 含义 |
| --- | --- | --- |
| 1~2 | 47~46 | 模式：`00` 停止、`01` 位置、`10` 速度、`11` 姿态（只跑内环） |
| 3~16 | 45~32 | 保留（跳过第一个 16 位） |
| 17~32 | 31~16 | value1：位置 = 共模位移（mm）；速度 = 共模速度（mm/s）；姿态 = 目标 Pitch（0.1°） |
| 33~48 | 15~0 | value2：位置 = 0.1° 目标 yaw 角；速度 = 1°/s 目标 yaw rate；姿态 = 1°/s 目标 yaw rate |

| 模式 | value1 | value2 |
| --- | --- | --- |
| `00` 停止 | 忽略 | 忽略 |
| `01` 位置 | 目标共模位移（mm） | 目标 yaw 角（0.1°，相对首次进入位置模式的零点） |
| `10` 速度 | 目标速度（mm/s） | 目标 yaw rate（1°/s，不使用 yaw 零点） |
| `11` 姿态 | 目标 Pitch 角（0.1°） | 目标 yaw rate（1°/s） |

> 姿态模式：只运行 250 Hz 内环，不运行位置/速度/yaw 角度环；
> 目标 Pitch 限幅 ±30°，目标 yaw rate 限幅 ±180°/s，用于调参。

> 位置模式：`value2 = 900` 表示目标 +90.0°（相对首次进入位置模式时的九轴 yaw 零点）。
> 同一位置模式内新命令直接**覆盖**目标角，不累加、也不相对当前实测角。
> 速度模式：`value2 = 90` 表示目标 +90°/s；中环只做限幅，不做 yaw 角度环。
> 逆时针为正。

- 两个字段均为 **16 位有符号**（-32768~32767）。
- STM32 为小端；6 字节队列项对应 `uint64_t` 的低 48 位。
- 发送示例：

  ```c
  uint64_t message = ((uint64_t)0x1U << 46)
                   | ((uint64_t)(uint16_t)target_position << 16)
                   | (uint64_t)(uint16_t)target_yaw_value;  /* 位置 0.1° / 速度 1°/s */
  (void)osMessageQueuePut(ControlqueueHandle, &message, 0U, 0U);
  ```

### 状态机读取

`PIDTask_ProcessControlQueue()` 不是每个 4 ms 基础 tick 都调用；
`PIDTask_UpdateControlQueue()` 每个基础 tick 检查 `s_control_queue_counter`，
仅在计数为 0 时读取，因此实际频率为 **50 Hz（20 ms）**。它在状态机之前执行，
读取后立即更新状态和目标，再由同一 tick 的 `switch` 进入新状态。

读取逻辑：

1. `osMessageQueueGetCount(ControlqueueHandle) == 0` → 队列为空，直接返回；
2. 非空 → `osMessageQueueGet()` 非阻塞读取 48 位消息；
3. 按最高两位切换：
   - `00` → `PID_STATE_STOP`，三个控制器复位；
   - `01` → 写入共模位移；首次进入位置模式时锁存九轴 yaw 零点；`value2` 直接作为
     相对该零点的目标角；同一位置模式内新命令覆盖目标角，不累加；
   - `10` → 写入目标速度；`value2` 作为目标 yaw rate（1°/s），不使用 yaw 零点；
     速度模式不做 yaw 角度环，只做 yaw rate 限幅；
   - `11` → 写入目标 Pitch（0.1°）和目标 yaw rate（1°/s）；进入姿态模式，
     复位内环积分，只运行 250 Hz 内环。

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
> yaw rate 由 PID task 内的 2 状态 Kalman 处理：`gz` 提供高频预测，
> 编码器差模角度在 50 Hz 修正陀螺零偏，250 Hz 内环使用融合后的 yaw rate。
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

1. 连续读 500 帧（约 1 s，`HAL_Delay(2)`），同时取 pitch 轴 `gy` 和 yaw 轴 `gz` 的均值；
2. 采样期间任一轴角速度峰峰值必须 `< 5 deg/s`，否则判为“被碰动”，自动重试，最多 3 次；
3. 成功则 pitch/yaw 各自保存零偏；失败则两轴零偏为 0，pitch 由卡尔曼在线学习，
   yaw 由 PID task 内的 2 状态 Kalman 在线估计残余零偏；上电校准值只作为初始参考。

> 上电后约 1 s 内保持小车静止。校准失败不会阻止 IMU 启动，但零偏收敛会慢一些，
> pitch 零偏可通过 `IMU_GetPitchBias()` 观察。校准在初始化任务（defaultTask）中阻塞执行；
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
  if (osSemaphoreAcquire(IMUdatadoneBinarySemHandle, osWaitForever) == osOK)
  {
    (void)IMU_Parse();                  /* 函数调用：解析 DMA 快照 */
    PIDTask_UpdateIMU();                /* 函数调用：取最新姿态/角速度 */
    PIDTask_UpdateControlQueue();       /* 函数调用：50 Hz 轮询 Controlqueue */

    switch (s_pid_state)                /* 状态机 */
    {
      case PID_STATE_POSITION: PIDTask_StatePosition(); break;
      case PID_STATE_SPEED:    PIDTask_StateSpeed();    break;
      case PID_STATE_STOP:     PIDTask_StateStop();     break;
      default:                 PIDTask_StateStop();     break;
    }
  }
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
- 周期任务（如 `controlTask`）用宏定义周期，循环末尾只调用一次 `osDelayUntil()`
  按绝对 tick 定时，避免周期累计误差：

  ```c
  s_control_wake_tick += pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS);
  (void)osDelayUntil(s_control_wake_tick);
  ```

  PID 任务例外：它由 `IMUdatadoneBinarySem` 驱动，**不调用 `osDelayUntil()`**；
  每个 USART3 RX DMA 完成执行一拍。
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
   > 当前 `elet2026.uvprojx` 已包含 `Application/User/Tasks` 组；若 CubeMX
   > 重新生成工程后丢失，需要重新加入。
4. 若其他目录需要包含本目录的头文件，请在 Keil 的 `Include Paths` 中加入
   `../tasks`（当前工程已配置）。

## 参考实现

`pid_task.c` 是当前的标准示例，包含：

- 任务状态枚举与状态变量；
- `StartPIDTask()` 入口与“状态机 + 函数调用”的主循环；
- 三个状态处理：`PIDTask_StateStop()`、`PIDTask_StatePosition()`、
  `PIDTask_StateSpeed()`；
- `PIDTask_UpdateControlQueue()` 在任务循环中、状态机之前按 50 Hz 调用
  `PIDTask_ProcessControlQueue()`，读取 CubeMX 生成的 `Controlqueue`
  （深度 1、6 字节）；
- `dwt_time.c` / `dwt_time.h`：共享的 `DWTTime_Init()`、`DWTTime_GetCycles()`、
  `DWTTime_CalculateDt()`；PID 和 IMU 共用一个 DWT 时间基准；
- `PIDTask_InitHighResClock()` / `PIDTask_CalculateDt()` 使用 DWT CYCCNT
  为三个环计算实测 `dt`；`PIDTask_CheckTimestampOverflow()` 用 1 ms tick
  防止 32 位 DWT 回绕差值被误用；DWT 不可用时回退标称值；
- IMU 链路在 `imu_task.c`：USART3 921600 → DMA1 Stream1（28 B）→ 完成回调释放 `IMUdatadoneBinarySem` → PID 的 `IMU_Parse()` 解析 0x55 0x55 帧；
- `SETPWM()` 已实现（TIM2 PWM + 方向 GPIO），但 TIM2 的 Period 需在 CubeMX 修正；
- 文件末尾集中放置的状态处理、输出、硬件抽象与延时函数，并附
  功能 / 输入 / 输出 注释。

三个控制器函数、三个参数结构体实例（`s_position_params`、`s_speed_params`、
`s_attitude_params`）以及控制器内部状态都集中在 `pid_task.c` 中；
结构体类型、宏与函数声明在 `pid_task.h` 中。

`imu_task.c` 提供 ATK-MS901M 九轴 UART 采集、`0x55 0x55` 帧解析和 `IMU_GetData()`；
`control_task.c` 实现五个状态机流程、KEY0/KEY_UP、巡线/对齐和 TIM11 提示音；同样按“函数实现在文件末尾 + 功能 / 输入 / 输出注释”组织。

