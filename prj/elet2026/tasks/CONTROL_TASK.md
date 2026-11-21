# controlTask 状态机与巡线说明

`control_task.c` 以 **20 ms（50 Hz）** 周期运行，负责：

- 轮询前/后各 8 路光电，保存原始电平与“检测到线”掩码；
- 通过 `Controlqueue` 向 PID 任务发布目标；
- KEY0 / KEY_UP 按键处理；
- TIM11 蜂鸣器/LED 频率扫描；
- 四个顶层状态机。

## 1. 顶层状态机与按键

全局变量：

```c
volatile uint8_t currentstatemachine;
volatile uint8_t currentmode;
```

| `currentstatemachine` | 状态机 | 说明 |
| --- | --- | --- |
| 0 | IDLE | 占位 |
| 1 | TUNING | 调参六模式 |
| 2 | HOLD | 原地停止 |
| 3 | RUN | 110 cm + 丢线停车测试 |
| 4 | SM4 | 直行 + 巡线 + 后退 + 对齐 |
| 5 | STOP | 顶层停止占位 |

按键：

| 按键 | 引脚 | 触发 | 行为 |
| --- | --- | --- | --- |
| KEY0 | PE4 / EXTI4 | 下降沿（CubeMX） | `currentmode` 0~5 循环，调参子模式 |
| KEY_UP | PA0 / EXTI0 | 下降沿（CubeMX） | 状态机 1→2→3→4→1 循环，并把 `currentmode` 复位为 0 |

- 两个键都在 `HAL_GPIO_EXTI_Callback()` 中处理，200 ms 消抖；
- 上电默认 `currentstatemachine = 2`（HOLD）；
- 从 HOLD 按 KEY_UP 两次可进入状态机 4，也可在调试器里直接把 `currentstatemachine` 设为 4；
- `currentstatemachine = 5`（STOP）仍是占位，不产生停车命令；停车使用状态机 2/4 的 `POSITION(0,0)`。

## 2. 状态机 1：调参

`currentmode` 决定六种子模式，每次进入只发布一次命令：

| `currentmode` | 模式 | Controlqueue |
| --- | --- | --- |
| 0 | 速度 | SPEED，`value1=+100 mm/s`，`value2=0` |
| 1 | 位置 | POSITION，`value1=+500 mm`，`value2=0` |
| 2 | 倾角 | ATTITUDE，`value1=+50`（5.0°），`value2=0` |
| 3 | 角度 | POSITION，`value1=0`，`value2=+900`（90.0°） |
| 4 | 角速度 | SPEED，`value1=0`，`value2=+30`（30°/s） |
| 5 | 停止 | STOP，`value1=0`，`value2=0` |

## 3. 状态机 2：原地停止

进入时发布一次：

```text
POSITION, value1=0（不前进）, value2=0（不转向）
```

PID 收到 POSITION 会锁存编码器起点，因此保持当前位置和航向。

## 4. 状态机 3：路径测试

1. 进入时发布一次 `POSITION(1100, 0)`，向前 110 cm；
2. 每 20 ms 读取 `s_front_line_mask`（F1~F8，bit0=F1）；
3. 任一路检测到黑线：立即发布 `POSITION(0, 0)` 原地停车；
4. 停车成功后启动 1 s 蜂鸣器频率扫描。

## 5. 状态机 4：直行 + 巡线 + 后退 + 对齐

### 5.1 阶段

| 阶段 | 行为 | 退出条件 |
| --- | --- | --- |
| `STRAIGHT_1` | `POSITION(1100,0)` 直行 110 cm | 前轮检测结果 `0→1` 上升沿（黑线出现） |
| `FOLLOW_1` | 巡线，发布 `SPEED(速度, yaw rate)` | 前轮检测结果 `1→0` 下降沿（黑线消失） |
| `REVERSE` | `POSITION(-50,0)` 后退 5 cm | 等待 `CONTROL_SM4_REVERSE_WAIT_MS`（默认 600 ms） |
| `ALIGN` | 前后加权中心差 - `alinetarget` 做 P，发布 `SPEED(0, yaw rate)` | 连续 10 帧在容差内 |
| `STRAIGHT_2` | `POSITION(1100,0)` 再直行 110 cm | 前轮检测结果 `0→1` 上升沿 |
| `FOLLOW_2` | 巡线 | 前轮检测结果 `1→0` 下降沿（第二次消失） |
| `STOPPING` | `POSITION(0,0)` 原地停车 | 命令入队成功 |
| `DONE` | 保持停车 | — |

- 每个周期把“前轮是否检测到黑线”存到 `s_sm4_front_detected`，再根据
  `s_sm4_front_prev_detected` 计算出现/消失的上升沿/下降沿；
- `STRAIGHT_1` / `STRAIGHT_2` 只使用**出现上升沿**，命令未发出时会把边沿锁存到
  `s_sm4_front_seen`，命令发出后立即使用；
- 停车由 `FOLLOW_2` 的下降沿触发；离开对齐线时的下降沿只鸣叫，不会提前停车；
- 对齐没有超时停车，会一直尝试直到连续 10 帧满足容差；
- 状态机 4 假设小车出发时前轮不在黑线上；若一开始压线，不会立即进入 `FOLLOW_1`，
  需要等黑线先消失再出现。

### 5.2 黑线出现/消失提示

每次 `s_sm4_front_detected` 发生 `0 → 1`（出现）或 `1 → 0`（消失）都会调用：

```c
ControlTask_BuzzerBeepOneSecond();
```

蜂鸣器使用 TIM11 频率扫描（见第 6 节），每次边沿重新从 2500 Hz 开始计时 1 s。

### 5.3 光电加权中心

8 路传感器等间距 `LINE_SENSOR_PITCH_MM = 11.5 mm`，以传感器排中心为 0：

> 模块 PCB 按 16 针 2.54 mm 排针做比例尺实测，相邻探头中心距约 **11.3 mm**；
> 当前 `robot_config.h` 使用 11.5 mm，误差约 1.7%，对计数型目标影响很小。

```text
x1 = -3.5 × 11.5 mm, x2 = -2.5 × 11.5 mm, ... x8 = +3.5 × 11.5 mm

center = Σ(x_i × active_i) / Σactive_i
```

- `active_i = 1` 表示该路掩码为 1（检测到黑线）；
- 前/后各算一次 `front_center`、`rear_center`；
- 若某排没有检测到线，则该排中心无效。

### 5.4 巡线算法

```text
common = (front_center + rear_center) / 2   // 整车横向偏离

## 6. 蜂鸣器 / LED

- 引脚：**PF7 / BEEP_LED**，TIM11_CH1，AF3；
- 同一引脚同时驱动蜂鸣器和 LED；
- `controlTask` 在初始化时把 TIM11 设为 `PSC=51`，
  计数时钟 = 168 MHz / 52 = **3.230769 MHz**；
- 起始频率 **2500 Hz**，每个 controlTask 周期减 **50 Hz**，最低 **50 Hz**；
- `ARR = 计数时钟 / 频率 - 1`，`CCR = (ARR+1)/2`，保持 50% 占空比；
- 1 s 单次 FreeRTOS 软件定时器 `osTimerOnce` 关闭 PWM；
- 2500 Hz 需要 PSC=51，PSC=0 时 16 位 ARR 最低只能到约 2563 Hz。

## 7. 主要参数

| 宏 | 默认值 | 说明 |
| --- | --- | --- |
| `CONTROL_SM4_FORWARD_MM` | 1100 | 每段直行 110 cm |
| `CONTROL_SM4_REVERSE_MM` | -50 | 后退 5 cm |
| `CONTROL_SM4_REVERSE_WAIT_MS` | 600 | 后退等待时间，按实车调 |
| `CONTROL_LF_BASE_SPEED_MM_S` | 400 | 零误差基础速度 |
| `CONTROL_LF_MIN_SPEED_MM_S` | 80 | 最小前进速度 |
| `CONTROL_LF_MAX_SPEED_MM_S` | 400 | 最大前进速度 |
| `CONTROL_LF_SPEED_KP` | 8.0 | 速度反比例 P，mm/s per mm |
| `CONTROL_LF_YAW_KP` | 6.0 | 转向 P，°/s per mm |
| `CONTROL_LF_MAX_RATE_DPS` | 45 | 巡线 yaw rate 限幅 ±45°/s |
| `CONTROL_LF_STEER_SIGN` | -1.0 | 实车转向反了改 +1 |
| `CONTROL_LF_FRONT_POS_SIGN` / `REAR_POS_SIGN` | +1.0 | 某排安装方向相反时改 -1 |
| `CONTROL_LF_COMMON_TARGET_MM` | 0.0 | 横向中心目标 |
| `CONTROL_LF_DIFF_TARGET_MM` | 0.0 | 方向偏差目标 |
| `alinetarget`（全局变量） | 0.0 | 前-后中心目标差，单位=光电间距个数 |
| `CONTROL_ALIGN_KP` | 8.0 | 对齐 P，°/s per mm |
| `CONTROL_ALIGN_MAX_RATE_DPS` | 60 | 对齐 yaw rate 限幅 |
| `CONTROL_ALIGN_STEER_SIGN` | -1.0 | 对齐转向反了改 +1 |
| `CONTROL_ALIGN_TOLERANCE_MM` | 2.0 | 对齐成功容差 |
| `CONTROL_ALIGN_SETTLE_FRAMES` | 10 | 连续成功帧数 |
| `CONTROL_BUZZER_START_FREQ_HZ` | 2500 | 蜂鸣器起始频率 |
| `CONTROL_BUZZER_FREQ_STEP_HZ` | 50 | 每 20 ms 减小量 |
| `CONTROL_BUZZER_MIN_FREQ_HZ` | 50 | 最低频率 |
| `CONTROL_BUZZER_TIM_PRESCALER` | 51 | TIM11 预分频 |

## 8. 实车调试清单

1. **黑线有效电平**：确认 `robot_config.h` 的 `LINE_SENSOR_ACTIVE_LEVEL`（默认 0 = 黑线低电平）；
2. **传感器排列方向**：若某排 x1..x8 顺序相反，改 `CONTROL_LF_FRONT_POS_SIGN` / `REAR_POS_SIGN`；
3. **巡线转向方向**：若车往错误方向转，改 `CONTROL_LF_STEER_SIGN`；
4. **对齐转向方向**：若对齐越转越偏，改 `CONTROL_ALIGN_STEER_SIGN`；
5. **对齐目标差**：在调试器里修改全局变量 `alinetarget`，单位是“光电间距个数”，建议用 0、1、2 等整数；
6. **后退 5 cm**：当前用固定 600 ms 等待，按实际后退速度调 `CONTROL_SM4_REVERSE_WAIT_MS`；
7. **巡线 P 参数**：先低速（降低 `CONTROL_LF_BASE_SPEED_MM_S`），再逐步增大 `CONTROL_LF_YAW_KP`；
8. **PID 内环**：`pid_task.h` 的 PID 参数仍需要先整定，否则上面所有目标都不会真正跟踪；
9. **Controlqueue 深度 1**：巡线目标 50 Hz 发布，个别周期可能因队列满被丢弃，下一周期会重新发布；
10. **起点**：状态机 4 假设出发时前轮不在黑线上。

diff   = front_center - rear_center          // 车身方向偏差

common_error = common - CONTROL_LF_COMMON_TARGET_MM
diff_error   = diff   - CONTROL_LF_DIFF_TARGET_MM

steer_mm = common_error + CONTROL_LF_STEER_DIFF_WEIGHT * diff_error
yaw_rate = CONTROL_LF_YAW_KP * steer_mm * CONTROL_LF_STEER_SIGN
yaw_rate = clamp(yaw_rate, ±CONTROL_LF_MAX_RATE_DPS)

error_mag = |common_error| + CONTROL_LF_SPEED_DIFF_WEIGHT * |diff_error|
speed = CONTROL_LF_BASE_SPEED_MM_S - CONTROL_LF_SPEED_KP * error_mag
speed = clamp(speed, CONTROL_LF_MIN_SPEED_MM_S, CONTROL_LF_MAX_SPEED_MM_S)
```

通过 SPEED 模式发布：

```text
Controlqueue: SPEED
value1 = 目标速度（mm/s）
value2 = 目标 yaw rate（°/s）
```

效果：

- 偏差为 0：速度最大、转向为 0；
- 共模或差模偏差增大：速度下降、转向角速度增大；
- 当前只实现 P，参数在 `control_task.c` 顶部；
- 前/后只有一排有效时，用有效的一排作为 common，diff 置 0；两排都无效时发布 `SPEED(0,0)`。

### 5.5 对齐

```text
align_error = (front_center - rear_center) - alinetarget * LINE_SENSOR_PITCH_MM
yaw_rate = CONTROL_ALIGN_KP * align_error * CONTROL_ALIGN_STEER_SIGN
yaw_rate = clamp(yaw_rate, ±CONTROL_ALIGN_MAX_RATE_DPS)
```

- `alinetarget` 是全局 `volatile float`，单位是**相邻光电间距个数**，不是 mm；
  例如 `1.0` 表示前中心比后中心大 1 个探头间距，`0.5` 表示半个间距；
- 因为探头是离散的，优先用 0、1、2 等整数个间距，避免写一个物理上到不了的 mm 值；
- 默认 0 表示对齐后前后加权中心重合；
- 使用速度模式发布 `value1=0`（不前进）、`value2=yaw_rate`（原地转向）；
- 前后两排都必须检测到线，否则发布 `SPEED(0,0)` 并重置成功计数；
- 连续 `CONTROL_ALIGN_SETTLE_FRAMES`（默认 10）帧满足
  `|align_error| <= CONTROL_ALIGN_TOLERANCE_MM`（默认 2 mm）即认为对齐；
- 对齐成功后发布一次 `SPEED(0,0)` 停止转向，进入 `STRAIGHT_2`；
- **没有超时停车**：对齐会持续尝试，直到连续 10 帧成功。
