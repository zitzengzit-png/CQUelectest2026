# 无线调参（DEBUG）说明

本文档描述当前 `controlTask` 的无线调参链路、USART6 协议、队列和 CubeMX 配置要求。

## 1. 总体结构

```text
USART6 RX DMA2 Stream1（固定 64 B）
        ↓
paramtocontroltask（深度 1，64 B，覆盖）
        ↓
controlTask DEBUG：CRC/参数/目标校验
        ├─ 参数 → paramQueue（深度 1，68 B，覆盖）
        │           ↓
        │         PID 10 Hz：位置外环前应用
        │           ↓
        │         param_status/version 写入 dataToLCDQueue
        │
        └─ 控制目标 → Controlqueue（深度 1，6 B，覆盖）→ PID 50 Hz
```

- 调参数据不进 `dataToLCDQueue`；
- `dataToLCDQueue` 只回传遥测和参数应用结果；
- 所有队列深度为 **1**，均为最新值覆盖语义。

## 2. 状态和按键

| 值 | 顶层状态 | 说明 |
|---:|---|---|
| 0 | IDLE | 占位 |
| 1 | DEBUG | 无线调参 |
| 2 | HOLD | 原地停止 |
| 3 | RUN | 路径测试 |
| 4 | SM4 | 巡线/对齐 |
| 5 | SM5 | 四次 zigzag |
| 6 | STOP | 占位 |

- **KEY0 = PE4**：GPIO 输入、上拉、低有效，controlTask 每 20 ms 轮询；
  - 在 HOLD/STOP 按 KEY0 → DEBUG；
  - 在 DEBUG 按 KEY0 → HOLD。
- **KEY_UP = PA0**：EXTI0 上升沿，只循环 `HOLD → RUN → SM4 → SM5 → HOLD`；
- DEBUG 不进入 KEY_UP 循环，避免进入调参时误启动运动状态；
- 进入 DEBUG 后发一次 `ATTITUDE(0,0)`，之后没有新帧不重复发送；
- 退出 DEBUG 时清空两个 param 队列，并返回 HOLD。

> KEY0 当前假设低有效。实车若为高有效，需要同时修改
> `CONTROL_KEY0_ACTIVE_LEVEL` 和 CubeMX 的 PE4 Pull 配置。

## 3. 队列

| 队列 | 生产者 → 消费者 | 深度 | Itemsize | 覆盖方式 |
|---|---|---:|---:|---|
| `paramtocontroltask` | USART6 DMA ISR → controlTask | 1 | 64 B | `xQueueOverwriteFromISR` |
| `paramQueue` | controlTask → PID | 1 | 68 B | `xQueueOverwrite` |
| `Controlqueue` | controlTask → PID | 1 | 6 B | 丢弃旧项后 `osMessageQueuePut` |
| `dataToLCDQueue` | PID → displayTask | 1 | 80 B | 丢弃旧项后 `osMessageQueuePut` |

- `paramtocontroltask` 和 `paramQueue` 是 CubeMX 创建的 CMSIS 队列，代码通过其
  `QueueHandle_t` 使用原生覆盖 API；
- 深度 1 覆盖意味着：消费者没及时取走时，旧值直接丢弃；
- PC 端必须根据 `seq` / `param_version` 判断命令是否生效，不能假设每条中间命令都保留。

## 4. USART6

| 项目 | 配置 |
|---|---|
| 外设 | USART6 |
| TX / RX | PG14 / PC7 |
| 波特率 | 115200，8N1 |
| TX DMA | DMA2 Stream6 / Channel5 / Normal / Byte |
| RX DMA | DMA2 Stream1 / Channel5 / Normal / Byte |
| NVIC | USART6、DMA2 Stream1、DMA2 Stream6 |

### 4.1 遥测：TX 86 B

```text
偏移  长度  内容
0     2     AA 55
2     1     len = 80
3     1     ver = 2
4     80    LCD_Data_t（小端）
84    2     CRC16-CCITT，低字节在前
```

CRC 覆盖帧偏移 0..83。

`LCD_Data_t` 关键字段（payload 偏移）：

| 偏移 | 字段 | 类型 |
|---:|---|---|
| 4 | `timestamp_ms` | u32 |
| 8 | `imu_age_ms` | u32 |
| 68 | `imu_timestamp_ms` | u32 |
| 72 | `param_status` | u16 |
| 74 | `param_version` | u16 |
| 76 | `param_seq` | u16 |
| 78 | `param_id` | u16 |

对应到 86 B 帧的偏移：`imu_timestamp_ms=72`，`param_status=76`，
`param_version=78`，`param_seq=80`，`param_id=82`。

### 4.2 调参：RX 固定 64 B

```text
偏移  长度  内容
0     2     A5 5A
2     1     ver = 1
3     1     type = 1
4     2     seq（u16 小端）
6     1     control_mode
7     1     param_count（0..8）
8     2     target1（i16 小端）
10    2     target2（i16 小端）
12    40    8 × { id:u8, value:float32 小端 }
52    10    reserved
62    2     CRC16-CCITT 小端，覆盖 0..61
```

`control_mode`：

```text
0   STOP
1   POSITION
2   SPEED
3   ATTITUDE
FF  只改参数，不改控制目标
```

目标编码沿用 `Controlqueue`：

```text
POSITION: target1 = mm,      target2 = 0.1° 相对 yaw
SPEED:    target1 = mm/s,    target2 = 1°/s yaw rate
ATTITUDE: target1 = 0.1°,    target2 = 1°/s yaw rate
```

## 5. 参数 ID 和状态

| ID | 参数 | ID | 参数 |
|---:|---|---:|---|
| 1 | kp_distance | 9 | max_rate |
| 2 | kd_distance | 10 | kp_pitch |
| 3 | max_speed | 11 | kd_pitch |
| 4 | kp_speed | 12 | kp_yaw_rate |
| 5 | kd_speed | 13 | ki_yaw_rate |
| 6 | max_pitch | 14 | max_pwm |
| 7 | kp_angle | 15 | max_diff_pwm |
| 8 | kd_angle | 16 | max_yaw_integral |

一帧最多 8 个参数；16 个参数需要两帧。范围检查在
`control_task.c` 的 `TuneLink_ParamIsValid()`。

`param_status`：

```text
0 OK
1 BAD_SYNC
2 BAD_VERSION
3 BAD_TYPE
4 BAD_COUNT
5 BAD_CRC
6 BAD_MODE
7 BAD_TARGET
8 BAD_PARAM_ID
9 BAD_PARAM_RANGE
10 BAD_STATE
11 QUEUE_BUSY
```

## 6. PID 应用规则

- `PIDTask_UpdateTuningQueue()` 每 25 个 250 Hz tick 执行一次，即 **10 Hz**；
- 在 `PIDTask_UpdateControlQueue()` 和位置外环之前执行；
- 收到有效 `paramQueue` 消息后：
  1. 再次检查参数值；
  2. 写入对应的运行时 `s_*_params`；
  3. 清位置环微分历史；
  4. `SpeedController_Reset()`；
  5. `YawAngleController_Reset()`；
  6. `AttitudeController_Reset()`；
  7. `param_version++`，并更新 `param_status/seq/id`；
- NACK 消息只更新状态，不修改参数、不增加版本。

遥测帧在 `PIDTask_UpdateDisplayQueue()` 中复制这些字段，因此 PC 可在下一帧
（最多约 20 ms 后）看到结果；参数应用本身最多约 100 ms 延迟。

## 7. CubeMX 必须保持的配置

以后重新生成代码前，确认：

1. **PE4**
   - `GPIO_Input`
   - `GPIO_PULLUP`
   - User Label = `KEY0`
   - **不要配置 EXTI4**（KEY0 是轮询）
2. **PA0**
   - `GPIO_EXTI0`
   - Rising edge
   - Pull-down
   - Label = `KEY_UP`
3. **USART6**
   - 115200，8 data bits，no parity，1 stop
   - PG14 TX / PC7 RX
   - TX DMA2 Stream6 / Channel5 / Normal / Byte
   - RX DMA2 Stream1 / Channel5 / Normal / Byte
   - USART6、DMA2 Stream1、DMA2 Stream6 中断开启
4. **FreeRTOS 队列**
   - `Controlqueue`：深度 1，itemsize 6
   - `dataToLCDQueue`：深度 1，itemsize 80
   - `paramQueue`：深度 1，itemsize 68
   - `paramtocontroltask`：深度 1，itemsize 64
5. **不需要新增任务**
   - RX 由 `paramtocontroltask` 交给 controlTask；
   - 参数由 `paramQueue` 交给 PID；
   - 现有 controlTask/PID/displayTask 足够。

## 8. 已知限制和安全注意

1. USART6 RX 是**固定 64 B 帧**，PC 必须一次发满 64 B；
2. USART6 TX 是 86 B / 50 Hz，RX 与 TX 全双工，互不占用同一 DMA Stream；
3. KEY0 默认低有效；如果实车按键高有效，改 `CONTROL_KEY0_ACTIVE_LEVEL` 和上拉/下拉；
4. 队列全部是覆盖语义，PC 连续发多条参数时中间帧可能丢失，必须等
   `param_status/param_version` 或重发；
5. `Controlqueue` 覆盖会丢 HOLD/STOP 旧命令，这是当前设计接受的行为；
6. 参数和目标在同一帧时，参数走 10 Hz，目标走 50 Hz，目标可能比参数最多早约 100 ms
   生效；调参时建议先保持姿态、再下发运动目标；
7. `pid_task.c` 的 `PID_ATT_TARGET_PITCH_MAX_DEG`、`PID_YAW_MAX_RATE_DPS` 和
   `PID_ATT_MAX_PWM` 仍是编译期宏；不要在 PC 上调超出实际生效范围的“假参数”；
8. 当前只做了编译验证，真实 BT27/手机/实车时序需要上车验证。

## 9. PC 端解析建议

### 遥测 CRC

```python
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc
```

### 解析 86 B 遥测

```python
if len(frame) == 86 and frame[0:2] == b"\xAA\x55" and frame[2] == 80 and frame[3] == 2:
    if (frame[84] | (frame[85] << 8)) == crc16_ccitt(frame[:84]):
        payload = frame[4:84]
        seq = int.from_bytes(payload[0:4], "little")
        param_status = int.from_bytes(payload[72:74], "little")
        param_version = int.from_bytes(payload[74:76], "little")
        param_seq = int.from_bytes(payload[76:78], "little")
        param_id = int.from_bytes(payload[78:80], "little")
```

### 生成 64 B 调参帧

```python
import struct

def build_tune_frame(seq, mode, target1, target2, params):
    # params: [(id, value), ...], 最多 8 个
    frame = bytearray(64)
    frame[0:2] = b"\xA5\x5A"
    frame[2] = 1       # version
    frame[3] = 1       # type
    struct.pack_into("<H", frame, 4, seq)
    frame[6] = mode
    frame[7] = len(params)
    struct.pack_into("<h", frame, 8, target1)
    struct.pack_into("<h", frame, 10, target2)
    for i, (pid, value) in enumerate(params):
        frame[12 + i * 5] = pid
        struct.pack_into("<f", frame, 13 + i * 5, value)
    struct.pack_into("<H", frame, 62, crc16_ccitt(frame[:62]))
    return bytes(frame)
```

## 10. 当前验证状态

```text
Keil MDK clean rebuild：0 Error(s), 0 Warning(s)
Code=40688  RO-data=1264  RW-data=560  ZI-data=23000
```

未进行真实蓝牙收包、实车电机和参数闭环验证。

