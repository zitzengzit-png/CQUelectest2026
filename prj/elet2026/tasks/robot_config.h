/**
  ******************************************************************************
  * @file    robot_config.h
  * @brief   全车可调参数集中配置：任务频率、PID 参数、IMU、编码器、OLED。
  ******************************************************************************
  * @note    调参时优先只改这个文件；其他模块通过包含本文件取得参数。
  *          PID 内部长度单位为 m，速度单位为 m/s，角度/角速度仍为 deg、deg/s；
  *          Controlqueue value1 使用 mm / (mm/s) 编码，进入 PID 后换算为 m / (m/s)。
  *          修改频率后必须保持串级比例：位置 10 Hz / 速度 50 Hz / 内环 250 Hz
  *          约 1:5:25；内环不能超过 IMU 实际上报率。
  ******************************************************************************
  */

#ifndef __ROBOT_CONFIG_H
#define __ROBOT_CONFIG_H

/* ========================================================================== */
/* 1. 任务与采样频率                                                          */
/* ========================================================================== */

/* PID 基础任务：250 Hz，4 ms；由 IMU DMA 完成信号量驱动 */
#define PID_TASK_PERIOD_MS            4U
#define PID_INNER_LOOP_HZ             (1000U / PID_TASK_PERIOD_MS)
#define PID_INNER_DT_S                ((float)PID_TASK_PERIOD_MS / 1000.0f)

/* 速度/Pitch 中环：50 Hz，20 ms */
#define PID_MIDDLE_LOOP_HZ            50U
#define PID_MIDDLE_PERIOD_MS          (1000U / PID_MIDDLE_LOOP_HZ)
#define PID_MIDDLE_DT_S               ((float)PID_MIDDLE_PERIOD_MS / 1000.0f)
#define PID_MIDDLE_TICK_DIV           (PID_INNER_LOOP_HZ / PID_MIDDLE_LOOP_HZ)

/* 位置外环：10 Hz，100 ms */
#define PID_OUTER_LOOP_HZ             10U
#define PID_OUTER_PERIOD_MS           (1000U / PID_OUTER_LOOP_HZ)
#define PID_OUTER_DT_S                ((float)PID_OUTER_PERIOD_MS / 1000.0f)
#define PID_OUTER_TICK_DIV            (PID_INNER_LOOP_HZ / PID_OUTER_LOOP_HZ)

/* 各环 dt 上限，防止任务停顿后 D/I 项异常放大 */
#define PID_INNER_DT_MAX_S            0.020f
#define PID_MIDDLE_DT_MAX_S           0.100f
#define PID_OUTER_DT_MAX_S            0.500f

/* DWT 32 位计数器回绕保护 */
#define PID_DT_OVERFLOW_GUARD_MS      20000U

/* Controlqueue 轮询：50 Hz，20 ms，即每 5 个 250 Hz tick 一次 */
#define PID_CONTROL_QUEUE_LOOP_HZ     50U
#define PID_CONTROL_QUEUE_PERIOD_MS   (1000U / PID_CONTROL_QUEUE_LOOP_HZ)
#define PID_CONTROL_QUEUE_TICK_DIV    (PID_INNER_LOOP_HZ / PID_CONTROL_QUEUE_LOOP_HZ)

/* 主控任务：50 Hz，20 ms */
#define CONTROL_TASK_PERIOD_MS        20U

/* OLED 名义刷新周期；displayTask 已改为队列事件驱动，不再按此周期轮询 */
#define DISPLAY_TASK_PERIOD_MS        50U

/* PID 向显示/蓝牙发送快照：50 Hz，每 5 个 4 ms PID tick 一次 */
#define PID_DISPLAY_UPDATE_HZ         50U

/* 显示任务频率统计窗口：8 个队列快照，50 Hz 下约 160 ms */
#define DISPLAY_FREQ_WINDOW_SAMPLES   8U

/* 显示刷新 DMA 超时（400 kHz 全屏约 23 ms，留余量；当前仅用于 OLED 初始化） */
#define DISPLAY_REFRESH_TIMEOUT_MS    40U

/* USART6 遥测发送超时：DMA 完成/错误回调丢失时强制恢复 */
#define DISPLAY_UART6_TX_TIMEOUT_MS   100U

/* 显示布局：5x7 字体，21 列 x 8 行 */
#define DISPLAY_LINE_CHARS            21U
#define DISPLAY_LINE_BUF_SIZE         (DISPLAY_LINE_CHARS + 1U)
#define DISPLAY_LINE_COUNT            8U

/* IMU 主动上报率（必须与 IMU_RETURNRATE_VALUE 档位一致，仅作说明） */
#define IMU_OUTPUT_RATE_HZ            250U

/* 编码器反馈更新率：跟随 PID 中环 */
#define ENCODER_UPDATE_RATE_HZ        PID_MIDDLE_LOOP_HZ

/* ========================================================================== */
/* 2. PID 控制参数                                                            */
/* ========================================================================== */

/* 位置外环：距离误差（m） -> 目标速度（m/s）
 * 初值来源：matlab/autotune_pid.m 自动整定的结果（12 V、总延迟 10 ms、
 *           Jp=0.0031、死区 170），实车再复核。
 * 1.44 相比 0.7：位置阶跃上升 2.55 s -> 0.75 s，稳态误差 -1.55 -> 0.14 cm；
 * 代价是 0.30 m 阶跃 3.2% 超调（≈0.9 cm）。若实车觉得冲得厉害，降到 1.2。 */
#define PID_POS_KP_DISTANCE           1.44f
#define PID_POS_KD_DISTANCE           0.0f
#define PID_POS_MAX_SPEED_MPS         0.5f

/* 速度中环：速度误差（m/s） -> 目标 Pitch（deg）
 * 初值来源：balance_model 的 kp_speed 扫描（8/12/19.5），12 过冲最小 */
#define PID_SPEED_KP                  12.0f
#define PID_SPEED_KD                  0.0f
#define PID_SPEED_MAX_PITCH_DEG       10.0f

/* yaw 角度环：角度误差（deg） -> 目标 yaw rate（deg/s）
 * 初值来源：角度环比速率环慢 3~5 倍 */
#define PID_YAW_ANGLE_KP              3.0f
#define PID_YAW_ANGLE_KD              0.0f

/* yaw rate 内环：角速度误差（deg/s） -> 差模 PWM
 * 初值来源：balance_model 的偏航 PI 设计（tau_yaw≈80 ms，PM=90°） */
#define PID_YAW_KP                    18.5f
#define PID_YAW_KI                    230.0f

/* Pitch 内环：Pitch 误差（deg） + pitch_rate（deg/s） -> 共模 PWM
 * 初值来源：matlab/autotune_pid.m（在 balance_model 三环模型上做 5 工况自动整定，
 *           坐标模式搜索，J 41.6 -> 10.1）；相位裕度 @10 ms 延迟 40.7° -> 47.4°。
 * 实车若出现高频抖，先降 Kd；若回正无力，先升 Kp。 */
#define PID_ATT_KP_PITCH              149.0f
#define PID_ATT_KD_PITCH              10.30f

/*
 * 速度前馈（抵消电机反电动势）：common += PID_ATT_FF_SPEED * current_speed
 * 单位：PWM 计数 per (m/s)。由 balance_model 的 k_ff = (tau_stall/v_nl)/Kt 得出。
 * 没有这一项时，纯 PD 在带速度的状态下闭环会发散（模型给出 +4.5 /s 的右半平面极点）。
 * 电池电压变化时 k_ff 按 1/Kt 变化（12 V → 7.4 V 约 ×1.6），需要时按实测修正。
 */
#define PID_ATT_FF_SPEED              6207.0f

/*
 * 姿态延迟补偿：内环使用 pitch + pitch_rate * PID_DELAY_COMP_S。
 * 总延迟 10 ms（IMU 内部 2 + 传输/运算 6 + 编码器 2）；先用 10 ms 标称值，
 * 实测 IMU 延迟后再改。置 0 表示不补偿。
 */
#define PID_DELAY_COMP_S              0.010f

/* 电机驱动 STBY：TB6612 类驱动通常高电平使能，低电平待机 */
#define MOTOR_STBY_ENABLE_LEVEL       1U
#define MOTOR_STBY_STOP_LEVEL         0U

/*
 * 电机静摩擦前馈补偿（平滑符号函数）：输出 PWM + MOTOR_DEADBAND_PWM * tanh(PWM / eps)。
 *
 * 死区来自减速箱静摩擦：小于该占空比时轮子不转，PD 在 0 附近来回穿零会形成
 * 极限环。用"加"而不是"抬到最小值"：摩擦是恒定阻力矩，加性前馈才能把它抵消掉，
 * 剩余部分才是净转矩（"抬到 170"会让小指令刚好被摩擦吃掉、轮子仍不动）。
 *
 * 用 tanh 而不是 sign：sign 在 0 处跳变，指令穿零时输出在 ±170 之间来回跳（嗒嗒抖）。
 * tanh 让补偿在 |PWM| ≈ eps 附近平滑过渡，|PWM| >> eps 时几乎等于 ±MOTOR_DEADBAND_PWM。
 * PWM == 0 时 tanh(0) = 0，输出仍为 0，短路刹车不受影响。
 *
 * 两个参数都要现场整定：
 *   MOTOR_DEADBAND_PWM    —— 轮子刚开始转动时的 |output_*|（架空、ATTITUDE(0,0)、
 *                            target_pitch 从 0 缓慢增加），实测前先用 170（≈4%）。
 *   MOTOR_DEADBAND_SMOOTH_EPS —— 平滑区宽度（PWM 计数）。越小越接近硬 sign（补偿更
 *                            干脆、但零交叉抖）；越大越柔和（不易抖、但小指令补偿不足）。
 */
#define MOTOR_DEADBAND_COMP_ENABLE    1U
#define MOTOR_DEADBAND_PWM            170.0f
#define MOTOR_DEADBAND_SMOOTH_EPS     20.0f

/* 目标与输出限幅 */
#define PID_ATT_TARGET_PITCH_MAX_DEG  30.0f
#define PID_YAW_MAX_RATE_DPS          180.0f
#define PID_ATT_MAX_PWM               4199.0f

/*
 * 差模（yaw）输出上限：TIM2 ARR = 4199 时，2100 约等于 50% 占空比。
 *
 * 左右轮实际输出是 common ± diff，若 |common| + |diff| > 4199，
 * 必然有一侧被单独削掉，等效的共模和差模都会损失
 * (|common| + |diff| - 4199) / 2 —— 即“共模被差模抢走”。
 *
 * 因此差模不再用满量程 4199，而是留出约一半给共模（平衡）：
 * 正常巡线 / 原地转角所需差模只有几百 PWM，2100 足够；
 * 若实车确实需要更大转向权限，再按“优先保共模”的方式修改输出合成，
 * 不要把这里改回 4199。
 */
#define PID_ATT_MAX_DIFF_PWM          2100.0f

/*
 * yaw rate 积分项（PI 的 I）限幅。
 *
 *   s_yaw_rate_integral 的单位是 deg（角速度误差对时间的积分），
 *   ki_yaw_rate 的单位是 PWM/deg，所以真正需要限幅的是 ki * I（PWM），
 *   而不是 I 本身（旧实现把 I 限到 4199 deg，等于没有限幅）。
 *
 * PID_YAW_INTEGRAL_AUTHORITY：积分项最多占差模上限的比例（0~1）；
 * PID_YAW_MAX_INTEGRAL     ：积分项 PWM 贡献的绝对上限，与上一项取小；
 *                            设为 0 表示关闭积分。
 * 两者共同保证积分不会顶满差模输出，也保证饱和后能快速退饱和。
 */
#define PID_YAW_INTEGRAL_AUTHORITY    0.5f
#define PID_YAW_MAX_INTEGRAL          4199.0f

/* Controlqueue 指令单位 */
#define PID_CONTROL_YAW_UNIT_DEG       0.1f
#define PID_CONTROL_YAW_RATE_UNIT_DPS  1.0f
#define PID_CONTROL_PITCH_UNIT_DEG     0.1f

/*
 * Controlqueue 是 int16 队列，长度无法直接放 float 米。
 * value1 按毫米/mm/s 编码，PID 收到后立即换算为 m / m/s：
 *   位置模式 value1 = 毫米
 *   速度模式 value1 = 毫米/秒
 */
#define PID_CONTROL_DISTANCE_UNIT_M     0.001f
#define PID_CONTROL_SPEED_UNIT_MPS      0.001f

/* Controlqueue 模式编码：最高两位 */
#define CONTROL_MODE_STOP             0x0U
#define CONTROL_MODE_POSITION         0x1U
#define CONTROL_MODE_SPEED            0x2U
#define CONTROL_MODE_ATTITUDE         0x3U

/* ========================================================================== */
/* 3. IMU / 编码器 / OLED 硬件参数                                            */
/* ========================================================================== */

/* ATK-MS901M UART 与上报内容 */
#define IMU_UART_BAUD                 921600U
#define IMU_BAUD_FALLBACK             115200U
#define IMU_DMA_BYTES                 28U
#define IMU_RETURNSET_VALUE           0x05U  /* 0x01 姿态 + 0x03 陀螺/加速度 */
#define IMU_RETURNRATE_VALUE          0x00U  /* 0x00=250Hz；0x01=200Hz；... */
#define IMU_ALG_VALUE                 0x01U  /* 0x01=九轴，0x00=六轴 */

/* 模块内部传感器量程/带宽（默认值；修改后需重启模块） */
#define IMU_ACC_FSR_G                 4.0f
#define IMU_GYRO_FSR_DPS              2000.0f

/* 解析缓冲与初始化超时 */
#define IMU_STREAM_BUF_SIZE           256U
#define IMU_READY_TIMEOUT_MS          500U

/* IMU 安装方向与轴映射：实车确认后修改 */
#define IMU_PITCH_SIGN                1.0f
#define IMU_YAW_SIGN                  1.0f
#define IMU_PITCH_RATE_SIGN           1.0f
#define IMU_YAW_RATE_SIGN             1.0f
#define IMU_PITCH_GYRO_AXIS           1     /* 0=Gx, 1=Gy, 2=Gz */

/* TIM3/TIM4 正交编码器 */
#define ENCODER_QUADRATURE_FACTOR     4U
#define ENCODER_COUNTS_PER_REV        1540.0f  /* 25GA370 编码器 385 线/轮圈 × 4，四倍频后的计数/圈 */
#define ENCODER_INIT_COUNT            6554U
#define ENCODER_LEFT_SIGN             (+1)
#define ENCODER_RIGHT_SIGN            (+1)
#define ENCODER_NOMINAL_DT_S          0.020f
#define ENCODER_DT_MAX_S              0.100f
#define ENCODER_UPDATE_GAP_MS         200U

/*
 * 轮径与轮距（实车实测，2026-12）：
 *   25GA370 减速电机 + 76 mm 轮，左右轮中心距 170 mm。
 *   轮径用于脉冲 -> 米；轮距当前共模控制不使用，预留给差速/里程计。
 *   每圈计数 = 1540，故每个 4 倍频计数 = π × 76 mm / 1540 ≈ 0.1550 mm。
 */
#define WHEEL_DIAMETER_M              0.076f
#define WHEEL_TRACK_M                 0.170f
#define ENCODER_METERS_PER_COUNT      \
  (3.14159265358979323846f * WHEEL_DIAMETER_M / ENCODER_COUNTS_PER_REV)

/*
 * 实车几何（实测，2026-12，供巡线/对齐/文档换算使用，当前控制器只用
 * LINE_SENSOR_PITCH_MM）：
 *   - 前、后光电排各距车轴 50 mm（两排间距 100 mm）；
 *   - 25GA370 电机轴线即车轴，整车重心在轴线正上方 40 mm。
 * 说明：巡线的方向偏差按 ψ ≈ diff_mm / LINE_SENSOR_ROW_SPACING_MM 换算。
 */
#define LINE_SENSOR_ROW_TO_AXLE_MM    50.0f
#define LINE_SENSOR_ROW_SPACING_MM    100.0f
#define CG_HEIGHT_ABOVE_AXLE_M        0.040f

/*
 * 八路红外巡线传感器：前 F1-F8，后 B1-B8，均为 GPIO 输入。
 * 数组下标 0..7 对应模块的 x1..x8。
 * 压线有效电平：模块常见为低电平有效（0），实车可用单根黑线逐路确认；
 * 若探头压黑线时读到 1，则把 LINE_SENSOR_ACTIVE_LEVEL 改为 1。
 */
#define LINE_SENSOR_COUNT             8U
#define LINE_SENSOR_ACTIVE_LEVEL      0U
#define LINE_SENSOR_PITCH_MM          11.5f   /* 相邻探头中心距，实测 11.5 mm */

/* OLED 0.96 寸 128x64，I2C3 */
#define OLED_WIDTH                    128U
#define OLED_HEIGHT                   64U
#define OLED_PAGE_COUNT               8U
#define OLED_I2C_ADDR_DEFAULT         0x78U
#define OLED_I2C_ADDR_ALT             0x7AU

/* 显示状态标志与 IMU 停更阈值 */
#define LCD_MODE_STOP                 0U
#define LCD_MODE_POSITION             1U
#define LCD_MODE_SPEED                2U
#define LCD_MODE_ATTITUDE             3U

#define LCD_FLAG_IMU_READY            (1U << 0)
#define LCD_FLAG_IMU_VALID            (1U << 1)
#define LCD_FLAG_IMU_STALE            (1U << 2)
#define LCD_FLAG_QUEUE_DROP           (1U << 3)

#define LCD_IMU_STALE_MS              100U

/* dataToLCDQueue 元素大小；必须与 CubeMX 中 dataToLCDQueue itemsize 一致 */
#define LCD_QUEUE_ITEM_SIZE           80U

#endif /* __ROBOT_CONFIG_H */

/************************ (C) COPYRIGHT *****END OF FILE***********************/


