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

/* OLED 显示任务：20 Hz，50 ms */
#define DISPLAY_TASK_PERIOD_MS        50U

/* PID 向显示任务发送快照：平均 20 Hz */
#define PID_DISPLAY_UPDATE_HZ         20U

/* 显示任务频率统计窗口：8 个 50 ms 周期，约 400 ms */
#define DISPLAY_FREQ_WINDOW_SAMPLES   8U

/* 显示刷新 DMA 超时（400 kHz 全屏约 23 ms，留余量） */
#define DISPLAY_REFRESH_TIMEOUT_MS    40U

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

/* 位置外环：距离误差（m） -> 目标速度（m/s） */
#define PID_POS_KP_DISTANCE           0.0f
#define PID_POS_KD_DISTANCE           0.0f
#define PID_POS_MAX_SPEED_MPS         0.5f

/* 速度中环：速度误差（m/s） -> 目标 Pitch（deg） */
#define PID_SPEED_KP                  0.0f
#define PID_SPEED_KD                  0.0f
#define PID_SPEED_MAX_PITCH_DEG       10.0f

/* yaw 角度环：角度误差（deg） -> 目标 yaw rate（deg/s） */
#define PID_YAW_ANGLE_KP              0.0f
#define PID_YAW_ANGLE_KD              0.0f

/* yaw rate 内环：角速度误差（deg/s） -> 差模 PWM */
#define PID_YAW_KP                    0.0f
#define PID_YAW_KI                    0.0f

/* Pitch 内环：Pitch 误差（deg） + pitch_rate（deg/s） -> 共模 PWM */
#define PID_ATT_KP_PITCH              0.0f
#define PID_ATT_KD_PITCH              0.0f

/* 电机驱动 STBY：TB6612 类驱动通常高电平使能，低电平待机 */
#define MOTOR_STBY_ENABLE_LEVEL       1U
#define MOTOR_STBY_STOP_LEVEL         0U

/* 目标与输出限幅 */
#define PID_ATT_TARGET_PITCH_MAX_DEG  30.0f
#define PID_YAW_MAX_RATE_DPS          180.0f
#define PID_ATT_MAX_PWM               4199.0f
#define PID_ATT_MAX_DIFF_PWM          4199.0f
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
#define ENCODER_COUNTS_PER_REV        1540.0f  /* 385 × 4，四倍频后的计数/圈 */
#define ENCODER_INIT_COUNT            6554U
#define ENCODER_LEFT_SIGN             (+1)
#define ENCODER_RIGHT_SIGN            (+1)
#define ENCODER_NOMINAL_DT_S          0.020f
#define ENCODER_DT_MAX_S              0.100f
#define ENCODER_UPDATE_GAP_MS         200U

/*
 * 轮径与轮距：轮径用于脉冲 -> 米，轮距为左右轮中心距（m）。
 * WHEEL_DIAMETER_M 先按 65 mm 轮子占位，实车必须测量后修改。
 * WHEEL_TRACK_M 当前共模控制不使用，预留给后续差速/里程计。
 */
#define WHEEL_DIAMETER_M              0.065f
#define WHEEL_TRACK_M                 0.150f
#define ENCODER_METERS_PER_COUNT      \
  (3.14159265358979323846f * WHEEL_DIAMETER_M / ENCODER_COUNTS_PER_REV)

/*
 * 八路红外巡线传感器：前 F1-F8，后 B1-B8，均为 GPIO 输入。
 * 数组下标 0..7 对应模块的 x1..x8。
 * 压线有效电平：模块常见为低电平有效（0），实车可用单根黑线逐路确认；
 * 若探头压黑线时读到 1，则把 LINE_SENSOR_ACTIVE_LEVEL 改为 1。
 */
#define LINE_SENSOR_COUNT             8U
#define LINE_SENSOR_ACTIVE_LEVEL      0U
#define LINE_SENSOR_PITCH_MM          11.5f   /* 相邻探头中心距，按 PCB 估算 */

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
#define LCD_QUEUE_ITEM_SIZE           68U

#endif /* __ROBOT_CONFIG_H */

/************************ (C) COPYRIGHT *****END OF FILE***********************/


