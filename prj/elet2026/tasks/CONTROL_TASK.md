# controlTask 状态机与巡线说明

`control_task.c` 以 **20 ms（50 Hz）** 周期运行，负责：

- 轮询前/后各 8 路光电，保存原始电平与“检测到线”掩码；
- 通过 `Controlqueue` 向 PID 任务发布目标；
- KEY0 / KEY_UP 按键处理；
- TIM11 蜂鸣器/LED 频率扫描；
- 顶层状态机 1~5。状态机 1 为无线调参 DEBUG；状态机 4/5 共用巡线 P 控制。

## 1. 顶层状态机与按键

全局变量：

```c
volatile uint8_t currentstatemachine;
```

| `currentstatemachine` | 状态机 | 说明 |
| --- | --- | --- |
| 0 | IDLE | 占位 |
| 1 | DEBUG | 无线调参 |
| 2 | HOLD | 原地停止 |
| 3 | RUN | 110 cm + 丢线停车测试 |
| 4 | SM4 | 直行 + 巡线 + 后退 + 对齐 |
| 5 | SM5 | 四次 zigzag + 巡线 |
| 6 | STOP | 顶层停止占位 |

按键：

| 按键 | 引脚 | 触发 | 行为 |
| --- | --- | --- | --- |
| KEY0 | PE4 | GPIO 轮询、上拉、低有效 | HOLD/STOP 与 DEBUG 切换 |
| KEY_UP | PA0 / EXTI0 | 上升沿 | HOLD→RUN→SM4→SM5→HOLD |

- KEY0 不在 `HAL_GPIO_EXTI_Callback()` 中处理，避免与 PC4/EXTI4 冲突；
- 上电默认 `currentstatemachine = 2`（HOLD）；
- DEBUG 进入时只发一次 `ATTITUDE(0,0)`，之后等待 USART6 调参帧；
- `currentstatemachine = 6`（STOP）仍是占位，不产生停车命令；停车使用状态机 2/4/5 的 `POSITION(0,0)`。

## 2. 状态机 1：无线调参 DEBUG

KEY0 进入 DEBUG 后：

1. 清空 `paramtocontroltask` / `paramQueue`；
2. 发一次 `ATTITUDE(0,0)`；
3. 等待 USART6 固定 64 B 调参帧；
4. controlTask 校验 CRC、参数 ID、范围和目标；
5. 校验通过：参数写入 `paramQueue`（PID 10 Hz 应用），目标通过 `Controlqueue` 发布一次；
6. 校验失败：只把 NACK 状态交给 PID 回传；
7. 无新帧不重复发姿态命令。

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

蜂鸣器使用 TIM11 频率扫描（见第 7 节），每次边沿重新从 2500 Hz 开始计时 1 s。
状态机 5 使用同一套前轮边沿提示。

### 5.3 光电加权中心

8 路传感器等间距 `LINE_SENSOR_PITCH_MM = 11.5 mm`，以传感器排中心为 0：

> 实测相邻探头中心距 **11.5 mm**（本车卡尺测量，2026-12）；
> 早期按 16 针 2.54 mm 排针比例尺估算的 11.3 mm 已废弃，`robot_config.h` 的 11.5 为准。

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
diff   = front_center - rear_center          // 车身方向偏差

common_error = common - CONTROL_LF_COMMON_TARGET_MM
diff_error   = diff   - CONTROL_LF_DIFF_TARGET_MM

steer_mm = common_error + CONTROL_LF_STEER_DIFF_WEIGHT * diff_error
yaw_rate = CONTROL_LF_YAW_KP * steer_mm * CONTROL_LF_STEER_SIGN
yaw_rate = clamp(yaw_rate, ±CONTROL_LF_MAX_RATE_DPS)

error_mag = |common_error| + CONTROL_LF_SPEED_DIFF_WEIGHT * |diff_error|
speed = CONTROL_LF_BASE_SPEED_MM_S - CONTROL_LF_SPEED_KP * error_mag

travel = |current_common - line_follow_start_common|
scale = 1.0                                        , travel <= 30π cm
scale = 1.0 - 0.5 * (travel - 30π cm) / (10π cm)   , 30π < travel < 40π cm
scale = 0.5                                        , travel >= 40π cm

max_speed = CONTROL_LF_MAX_SPEED_MM_S * scale
speed = clamp(speed, CONTROL_LF_MIN_SPEED_MM_S, max_speed)
```

巡线末段降速：

- 状态机 4/5 **每次进入 `FOLLOW_1` 或 `FOLLOW_2`** 时调用一次
  `ControlTask_LineFollowStart()`，记录当时的编码器共模位置；
- 此后每个 20 ms 控制周期在 `ControlTask_LineFollow()` 中读取当前共模位置，
  计算 `travel = |current_common - line_follow_start_common|`；
- `30π cm ≈ 94.25 cm` 开始降速，`40π cm ≈ 125.66 cm` 降到最大速度的 50%；
  40π cm 对应半径为 40 cm 的半圆弧长 `πR`，之后一直保持 50% 上限；
- 降速只降低**最大前进速度上限**，巡线误差速度 P 仍然生效；
  默认 `CONTROL_LF_MAX_SPEED_MM_S = 400`，即 30π cm 后从 400 线性降到 200 mm/s；
- 每次重新进入巡线（包括状态机 5 的每个周期）都会重新记录起点，重新从完整速度开始；
- 若动态上限低于 `CONTROL_LF_MIN_SPEED_MM_S`，以最小速度为准。

通过 SPEED 模式发布：

```text
Controlqueue: SPEED
value1 = 目标速度（mm/s）
value2 = 目标 yaw rate（°/s）
```

效果：

- 偏差为 0：速度最大（受 30π→40π cm 动态上限限制）、转向为 0；
- 共模或差模偏差增大：速度下降、转向角速度增大；
- 当前只实现 P，参数在 `control_task.c` 顶部；
- 前/后只有一排有效时，用有效的一排作为 common，diff 置 0；两排都无效时发布 `SPEED(0,0)`；
- 巡线 yaw rate 上限为 `CONTROL_LF_MAX_RATE_DPS = 45°/s`；
- 巡线最大前进速度还会按 5.4 节的 30π→40π cm 行程动态降到 50%。

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
- 状态机 4 对齐成功后发布一次 `SPEED(0,0)` 停止转向，进入 `STRAIGHT_2`；
- 状态机 5 对齐成功后进入第一次右转；
- **没有超时停车**：对齐会持续尝试，直到连续 10 帧成功。

## 6. 状态机 5：四次 zigzag + 巡线

### 6.1 进入与周期

1. 进入 `WAIT_START`：发布一次 `POSITION(0,0)` 保持原地，等待 1 s；
2. 进入 `ALIGN`：用 5.5 节的对齐算法对齐一次；
3. 对齐成功后开始第 1 个周期；每个周期为：

```text
右转 atan(1.25) -> 直行 130 cm
  途中遇线 -> 巡线 -> 前轮丢线 -> 后退 5 cm -> 左转 atan(1.25)
  到点未遇线 -> 左转 -> SPEED 找线 -> 巡线 -> 前轮丢线 -> 后退 5 cm -> 左转
直行 130 cm
  途中遇线 -> 巡线 -> 前轮丢线
  到点未遇线 -> 右转 -> SPEED 找线 -> 巡线 -> 前轮丢线
```

4. 第二次巡线丢线表示一个周期结束；`s_sm5_cycle_index` 加 1；
5. 若小于 4，重新从“右转”开始下一个周期；等于 4 后发布 `POSITION(0,0)` 停车。

> 对齐只在启动时执行一次；每个周期结束后的下一个周期从当前巡线航向直接右转。

### 6.2 阶段表

| 阶段 | 行为 | 退出条件 |
| --- | --- | --- |
| `WAIT_START` | `POSITION(0,0)` 保持，1 s | 计时到 |
| `ALIGN` | `SPEED(0, yaw rate)` 前后中心对齐 | 连续 10 帧在容差内 |
| `TURN_RIGHT` | `POSITION(0, -513)` 右转 | IMU yaw 连续 3 帧在目标 ±2° |
| `STRAIGHT_1` | `POSITION(1300, heading)` | 黑线出现上升沿，或编码器到点（<3 cm） |
| `TURN_LEFT_SEARCH` | 在当前位置 `POSITION(0, heading+513)` 左转 | IMU yaw 连续 3 帧在目标 ±2° |
| `SEARCH_SPEED_1` | `SPEED(250, 0)` 直行找线 | 前轮见线 |
| `FOLLOW_1` | 巡线 | 前轮丢线下降沿 |
| `REVERSE` | `POSITION(-50, heading)` 后退 5 cm | 编码器到点（1 cm 容差） |
| `TURN_LEFT_2` | `POSITION(0, heading+513)` 左转 | IMU yaw 连续 3 帧在目标 ±2° |
| `STRAIGHT_2` | `POSITION(1300, heading)` | 黑线出现，或编码器到点（<3 cm） |
| `TURN_RIGHT_SEARCH` | `POSITION(0, heading-513)` 右转 | IMU yaw 连续 3 帧在目标 ±2° |
| `SEARCH_SPEED_2` | `SPEED(250, 0)` 直行找线 | 前轮见线 |
| `FOLLOW_2` | 巡线 | 前轮丢线下降沿 |
| `STOPPING` | `POSITION(0,0)` | 命令入队成功 |
| `DONE` | 保持停车 | — |

- 直行阶段先检查黑线，再检查编码器到点：同周期两者都满足时优先进入巡线；
- 前轮黑线每次出现/消失都会触发 1 s 蜂鸣器频率扫描（与状态机 4 相同）；
- 每次进入 `FOLLOW_1` / `FOLLOW_2` 都会重新记录巡线共模起点，因此四个周期的
  30π→40π cm 降速都从各自巡线起点独立计算；
- 找线阶段使用 `SPEED(250,0)`：速度固定、yaw rate 为 0；只要前轮见到线就进入巡线；
- 若找线阶段一直见不到线，小车会一直直行（没有超时）。

### 6.3 转角

`atan(1.25) = 51.34019175°`：

- Controlqueue 角度单位为 0.1°，使用 `CONTROL_SM5_TURN_0P1DEG = 513`（51.3°，
  最接近 atan(1.25) 的 0.1° 值）；
- 转角完成判断使用 `CONTROL_SM5_TURN_TOLERANCE_DEG = 2°`，
  连续 `CONTROL_SM5_TURN_SETTLE_FRAMES = 3` 帧满足才进入下一阶段；
- 约定逆时针为正：右转 `value2 = -513`，左转 `value2 = +513`；
- 转角通过 `POSITION(0, value2)` 下发，位置控制器同时保持共模位置不变；
- PID 只在从 SPEED 第一次进入 POSITION 时把当前 yaw 锁为 0；controlTask 在
  第一条 POSITION 命令入队后记录该零点，后续绝对目标为：

  ```text
  target_yaw = wrap180(yaw_zero + heading_0p1deg * 0.1°)
  ```

- 转角阶段等实际 yaw 到达 `target_yaw`（误差 ≤2° 连续 3 帧）再进入下一阶段；这样即使直行
  结束时还有航向残差，也会等到与 PID 相同的绝对目标，不会死等；
- 若实车左右转方向相反，检查 IMU 安装方向/`IMU_YAW_SIGN`，不要只改单个宏。

### 6.4 130 cm 编码器到点

每次 `POSITION(1300, heading)` 命令入队成功后：

```c
s_sm5_segment_start_m  = (左编码器累计 + 右编码器累计) / 2 * ENCODER_METERS_PER_COUNT;
s_sm5_segment_target_m = s_sm5_segment_start_m + 1.30f;
s_sm5_segment_tolerance_m = 0.03f;   // 3 cm
```

每个 20 ms 控制周期读取一次当前共模位置并比较：

```text
|current_m - target_m| <= 0.03 m  -> 认为到达 130 cm 终点
```

- 编码器累计值由 PID 中环 50 Hz 的 `Encoder_Update()` 维护，
  controlTask 只通过 `Encoder_GetLeftTotal()` / `Encoder_GetRightTotal()` 读取，
  不调用 `Encoder_Update()`；
- 目标值是“起点 + 130 cm”的编码器共模位置，与 PID 位置环使用同一套轮径/CPR 换算；
- 后退 5 cm 使用同一方法，目标为 `start - 0.05 m`，容差
  `CONTROL_SM5_REVERSE_TOLERANCE_M = 0.01 m`；
- 由于命令入队到 PID 真正锁存位置起点有最多一个控制周期延迟，3 cm 容差用于吸收
  这段延迟和停车误差；若实车到点误差偏大，应检查轮径标定和位置环参数；
- 编码器共模值沿车轮轨迹累计，转向修正会带来少量直线位移差；本状态机按题目要求
  使用编码器值判断到点，而不是外部测距。

### 6.5 yaw 零点与 `position_yaw_valid`

PID 只在**从其他模式第一次进入 POSITION** 时把当前 yaw 锁为 0；
已经在 POSITION 时只覆盖目标角，零点保持不变。controlTask 用
`s_sm5_position_yaw_valid` 跟踪这一点：

- 对齐、巡线、找线都使用 SPEED，进入这些阶段时把标志清 0；
- 之后第一条 POSITION（后退或转角）会把当前航向作为新零点，目标角写相对值；
- 同一段 POSITION 内的转角/直行保持零点，目标角累加/保持；
- 例：对齐后右转 -513（0.1° 单位）；直行保持 -513；到点左转 +513 回到 0；
  找线后进入 SPEED；后退时 POSITION 重新锁零；左转 +513；直行保持 +513；
  到点右转 -513 回到 0。

### 6.6 起点与异常假设

- 启动对齐需要前、后两排都看到线；看不到时会一直等待，不会超时；
- 直行阶段只认黑线**上升沿**：如果出发时已经压线，需要先离开再压线才会进入巡线；
  找线阶段同时接受“当前已见线”和“上升沿”；
- 转角/到点/找线都没有超时处理；若 PID、IMU、编码器或场地不满足条件，
  状态机会停在当前阶段或持续直行，调试时应先看 `s_sm5_state`、`s_sm5_cycle_index`；
- 编码器方向必须满足“前进时共模位置增大”，否则 130 cm 目标方向会相反；
- `CONTROL_SM5_SEARCH_SPEED_MM_S = 250` 只是初始值，按实车抓地力和场地调整。

## 7. 蜂鸣器 / LED

- 引脚：**PF7 / BEEP_LED**，TIM11_CH1，AF3；
- 同一引脚同时驱动蜂鸣器和 LED；
- `controlTask` 在初始化时把 TIM11 设为 `PSC=51`，
  计数时钟 = 168 MHz / 52 = **3.230769 MHz**；
- 起始频率 **2500 Hz**，每个 controlTask 周期减 **50 Hz**，最低 **50 Hz**；
- `ARR = 计数时钟 / 频率 - 1`，`CCR = (ARR+1)/2`，保持 50% 占空比；
- 1 s 单次 FreeRTOS 软件定时器 `osTimerOnce` 关闭 PWM；
- 2500 Hz 需要 PSC=51，PSC=0 时 16 位 ARR 最低只能到约 2563 Hz。

## 8. 主要参数

| 宏 | 默认值 | 说明 |
| --- | --- | --- |
| `CONTROL_SM4_FORWARD_MM` | 1100 | 状态机 4 每段直行 110 cm |
| `CONTROL_SM4_REVERSE_MM` | -50 | 状态机 4 后退 5 cm |
| `CONTROL_SM4_REVERSE_WAIT_MS` | 600 | 状态机 4 后退等待时间 |
| `CONTROL_SM5_START_WAIT_MS` | 1000 | 状态机 5 启动保持 1 s |
| `CONTROL_SM5_TURN_0P1DEG` | 513 | 转角 51.3°（Controlqueue 0.1° 单位） |
| `CONTROL_SM5_STRAIGHT_MM` | 1300 | 状态机 5 每段直行 130 cm |
| `CONTROL_SM5_REVERSE_MM` | -50 | 状态机 5 后退 5 cm |
| `CONTROL_SM5_ENDPOINT_TOLERANCE_M` | 0.03 | 130 cm 到点容差 3 cm |
| `CONTROL_SM5_REVERSE_TOLERANCE_M` | 0.01 | 后退 5 cm 到点容差 1 cm |
| `CONTROL_SM5_SEARCH_SPEED_MM_S` | 250 | 速度模式找线前进速度 |
| `CONTROL_SM5_TURN_TOLERANCE_DEG` | 2.0 | 转角完成 IMU 容差 |
| `CONTROL_SM5_TURN_SETTLE_FRAMES` | 3 | 转角容差内连续帧数 |
| `CONTROL_SM5_CYCLE_COUNT` | 4 | 连续执行周期数 |
| `CONTROL_LF_BASE_SPEED_MM_S` | 400 | 巡线零误差基础速度 |
| `CONTROL_LF_MIN_SPEED_MM_S` | 80 | 巡线最小前进速度 |
| `CONTROL_LF_MAX_SPEED_MM_S` | 400 | 巡线基础最大前进速度（末段动态降到 50%） |
| `CONTROL_LF_SLOWDOWN_START_M` | 30π cm（0.9425 m） | 开始降速的共模行程 |
| `CONTROL_LF_SLOWDOWN_END_M` | 40π cm（1.2566 m） | 降到 50% 的共模行程 |
| `CONTROL_LF_SLOWDOWN_MIN_SCALE` | 0.5 | 末段最大速度比例 |
| `CONTROL_LF_SPEED_KP` | 8.0 | 速度反比例 P，mm/s per mm |
| `CONTROL_LF_YAW_KP` | 6.0 | 巡线转向 P，°/s per mm |
| `CONTROL_LF_MAX_RATE_DPS` | 45 | 巡线 yaw rate 限幅 ±45°/s |
| `CONTROL_LF_STEER_SIGN` | -1.0 | 实车转向反了改 +1 |
| `CONTROL_LF_FRONT_POS_SIGN` / `REAR_POS_SIGN` | +1.0 | 某排安装方向相反时改 -1 |
| `CONTROL_LF_COMMON_TARGET_MM` | 0.0 | 横向中心目标 |
| `CONTROL_LF_DIFF_TARGET_MM` | 0.0 | 方向偏差目标 |
| `alinetarget`（全局变量） | 0.0 | 前-后中心目标差，单位=光电间距个数 |
| `CONTROL_ALIGN_KP` | 8.0 | 对齐 P，°/s per mm |
| `CONTROL_ALIGN_MAX_RATE_DPS` | 60 | 对齐 yaw rate 限幅（未改成 45） |
| `CONTROL_ALIGN_STEER_SIGN` | -1.0 | 对齐转向反了改 +1 |
| `CONTROL_ALIGN_TOLERANCE_MM` | 2.0 | 对齐成功容差 |
| `CONTROL_ALIGN_SETTLE_FRAMES` | 10 | 连续成功帧数 |
| `LINE_SENSOR_PITCH_MM` | 11.5 | 光电间距，实测 11.5 mm |
| `CONTROL_BUZZER_START_FREQ_HZ` | 2500 | 蜂鸣器起始频率 |
| `CONTROL_BUZZER_FREQ_STEP_HZ` | 50 | 每 20 ms 减小量 |
| `CONTROL_BUZZER_MIN_FREQ_HZ` | 50 | 最低频率 |
| `CONTROL_BUZZER_TIM_PRESCALER` | 51 | TIM11 预分频 |

## 9. 实车调试清单

1. **黑线有效电平**：确认 `robot_config.h` 的 `LINE_SENSOR_ACTIVE_LEVEL`（默认 0 = 黑线低电平）；
2. **传感器排列方向**：若某排 x1..x8 顺序相反，改 `CONTROL_LF_FRONT_POS_SIGN` / `REAR_POS_SIGN`；
3. **巡线转向方向**：若车往错误方向转，改 `CONTROL_LF_STEER_SIGN`；
4. **对齐转向方向**：若对齐越转越偏，改 `CONTROL_ALIGN_STEER_SIGN`；
5. **对齐目标差**：在调试器里修改全局变量 `alinetarget`，单位是“光电间距个数”，建议用 0、1、2 等整数；
6. **状态机 4 后退 5 cm**：当前用固定 600 ms 等待，按实际后退速度调 `CONTROL_SM4_REVERSE_WAIT_MS`；
7. **状态机 5 编码器到点**：确认前进时共模位置增大（`ENCODER_LEFT_SIGN` / `ENCODER_RIGHT_SIGN`），
   并用卷尺核对 130 cm 实际停车位置；误差大时检查 `WHEEL_DIAMETER_M`、`ENCODER_COUNTS_PER_REV`
   和位置环参数；
8. **状态机 5 转角方向**：先在空地上单步观察右转是否真的向右；方向相反时检查 IMU 安装方向/
   `IMU_YAW_SIGN`，不要只改一个符号宏；
9. **状态机 5 找线速度**：`CONTROL_SM5_SEARCH_SPEED_MM_S` 默认 250 mm/s，按抓地力和场地调整；
   找线没有超时，长时间见不到线会一直直行；
10. **状态机 5 起点**：启动对齐要求前、后两排都压线；若起点没有线，会停在 `ALIGN`；
11. **状态机 5 调试变量**：`s_sm5_state`、`s_sm5_cycle_index`、`s_sm5_heading_0p1deg`、
    `s_sm5_yaw_zero_deg`、`s_sm5_target_yaw_deg`、`s_sm5_segment_target_m`、
    `s_sm5_segment_tolerance_m`；巡线降速可观察 `s_lf_start_position_m`、`s_lf_start_valid`；
12. **巡线末段降速**：每次开始巡线记录共模起点；30π cm≈94.25 cm 开始降速，
    40π cm≈125.66 cm 最大速度降到 50%；若降速点不对，先检查编码器共模标定；
13. **巡线 P 参数**：先低速（降低 `CONTROL_LF_BASE_SPEED_MM_S`），再逐步增大 `CONTROL_LF_YAW_KP`；
14. **PID 内环**：`pid_task.h` 的 PID 参数仍需要先整定，否则上面所有目标都不会真正跟踪；
15. **Controlqueue 深度 1**：巡线目标 50 Hz 发布，个别周期可能因队列满被丢弃，下一周期会重新发布；
16. **状态机 5 周期数**：4 个周期完成后 `STOPPING` 发布 `POSITION(0,0)`，若需要重复更多次改
    `CONTROL_SM5_CYCLE_COUNT`。
