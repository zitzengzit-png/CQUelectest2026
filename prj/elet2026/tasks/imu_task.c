/**
  ******************************************************************************
  * @file    imu_task.c
  * @brief   IMU 任务：MPU6050 DMA 采集、卡尔曼滤波与双缓冲发布。
  ******************************************************************************
  * @note    数据流：
  *          TIM7 2ms -> IMU_OnTimer7() -> HAL_I2C_Mem_Read_DMA()
  *          -> HAL_I2C_MemRxCpltCallback() 拷贝原始数据 + 释放信号量
  *          -> IMUTask 卡尔曼解算 -> 双缓冲 -> kalmaneddataQueue（指针）
  *          -> PID 任务从队列取指针并拷贝。
  *
  * @note    滤波算法：TKJ Electronics 的二状态卡尔曼（angle + gyro bias），
  *          参考 https://github.com/TKJElectronics/KalmanFilter 与
  *          https://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
  *          上电先在 IMU_Init() 中静止校准陀螺零偏，再启动 TIM7。
  ******************************************************************************
  */

/* 头文件包含 ----------------------------------------------------------------*/
#include <math.h>
#include <string.h>
#include "main.h"
#include "dwt_time.h"
#include "imu_task.h"
#include "i2c.h"

/* 私有宏定义 ----------------------------------------------------------------*/
#define MPU6050_ADDR              (0x68U << 1)   /* AD0=GND，HAL 8 位地址 */

#define MPU6050_REG_SMPLRT_DIV    0x19U
#define MPU6050_REG_CONFIG        0x1AU
#define MPU6050_REG_GYRO_CONFIG   0x1BU
#define MPU6050_REG_ACCEL_CONFIG  0x1CU
#define MPU6050_REG_ACCEL_XOUT_H  0x3BU
#define MPU6050_REG_PWR_MGMT_1    0x6BU
#define MPU6050_REG_WHO_AM_I      0x75U

#define MPU6050_WHO_AM_I_VALUE    0x68U

/* 与 IMU_Init() 中的量程配置保持一致 */
#define IMU_ACCEL_LSB_PER_G       16384.0f  /* ±2g */
#define IMU_GYRO_LSB_PER_DPS      32.8f     /* ±1000 dps */
#define IMU_RAD_TO_DEG            57.2957795f
#define IMU_SAMPLE_DT_S           0.002f    /* TIM7 2 ms，DWT 回退值 */

/*
 * 卡尔曼参数（角度单位统一为 deg、角速度 deg/s，因此 Q/R 也是 deg 体系）：
 * Q_angle/Q_bias/R_measure 采用 TKJ Electronics 二状态卡尔曼的经典默认值，
 * 该组值同样被 STM32 HAL 版 MPU6050 驱动（leech001/MPU6050）沿用。
 * R 不再固定：IMU_MeasurementNoise() 会按加速度模长偏离 1g 的程度放大 R。
 */
#define IMU_KALMAN_Q_ANGLE        0.001f
#define IMU_KALMAN_Q_BIAS         0.003f
#define IMU_KALMAN_R_MEASURE      0.03f     /* deg^2，约 0.17 deg 标准差 */
#define IMU_KALMAN_R_MOTION_GAIN  20.0f     /* |a|-1g 每 1g 放大 20 倍 */
#define IMU_KALMAN_R_SCALE_MAX    20.0f     /* R 最大 0.6 deg^2 */
#define IMU_KALMAN_ACC_DEADBAND   0.05f     /* 1g 附近 5% 内不放大 R */
#define IMU_KALMAN_P0_ANGLE       1.0f      /* 首帧角度来自含噪加速度计，不能设 0 */
#define IMU_KALMAN_P0_BIAS        0.01f     /* 已完成静止零偏校准 */
#define IMU_KALMAN_P0_BIAS_RAW    1.0f      /* 未校准，允许滤波器在线学习 */
#define IMU_KALMAN_BIAS_LIMIT     30.0f     /* deg/s，MPU6050 初始零偏最大约 ±20 dps */
#define IMU_KALMAN_DT_MAX         0.020f    /* 丢帧时 dt 最多按 20 ms 计 */
#define IMU_KALMAN_DT_GUARD_TICKS 10000U    /* 20 s / 2 ms，DWT 32 位回绕保护 */

/* 上电静止陀螺零偏校准：500 点 × 约 2 ms ≈ 1 s，峰峰值 < 5 dps 视为静止 */
#define IMU_GYRO_CALIB_SAMPLES    500U
#define IMU_GYRO_CALIB_RANGE_DPS  5.0f
#define IMU_GYRO_CALIB_RETRIES    3U

/* 安装方向：默认 pitch 绕模块 Y 轴、X 轴朝前、Z 轴朝上；
 * 仅前后装反（绕 Z 转 180°）时改为 -1.0f；换轴需同时改公式和陀螺轴 */
#define IMU_PITCH_SIGN            (1.0f)

/* 私有类型定义 --------------------------------------------------------------*/
/* 二状态卡尔曼：angle + gyro bias */
typedef struct
{
  float angle;
  float bias;
  float P[2][2];
  float Q_angle;
  float Q_bias;
  float R_measure;
} Kalman_t;

/* 私有变量 ------------------------------------------------------------------*/
static uint8_t s_imu_raw[14];        /* DMA 目标缓冲 */
static uint8_t s_imu_ready[14];      /* DMA 完成后的快照 */
static volatile uint8_t s_imu_busy = 0U;
static volatile uint32_t s_tim7_ticks = 0U;      /* TIM7 触发计数（2 ms/格） */
static volatile uint32_t s_imu_frame_tick = 0U;  /* 当前 DMA 帧启动时的计数 */
static volatile uint32_t s_imu_ready_tick = 0U;  /* 已完成帧启动时的计数 */
static uint32_t s_imu_last_tick = 0U;            /* 上次解算帧的计数 */

/* DWT 时间戳：在 DMA 启动时记录，随完成帧一起带到 IMU 任务 */
static volatile uint32_t s_imu_frame_cycles = 0U;
static volatile uint32_t s_imu_ready_cycles = 0U;
static uint32_t s_imu_last_cycles = 0U;

static float s_gyro_bias = 0.0f;       /* 上电静止校准得到的零偏，deg/s */
static uint8_t s_gyro_calibrated = 0U; /* 1 = 零偏校准成功 */
static volatile uint8_t s_imu_ready_flag = 0U; /* 1 = 已发布首帧 */

static Kalman_t s_kalman;
static uint8_t s_kalman_initialized = 0U;

static KalmanData_t s_kalman_buf[2]; /* 双缓冲 */
static uint8_t s_kalman_write_idx = 0U;

static uint32_t s_imu_error_count = 0U;

/* 私有函数声明 --------------------------------------------------------------*/
static HAL_StatusTypeDef MPU_WriteReg(uint8_t reg, uint8_t data);
static HAL_StatusTypeDef MPU_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len);

static uint8_t IMU_CalibrateGyro(void);
static void Kalman_Init(Kalman_t *k);
static float Kalman_Update(Kalman_t *k, float new_angle, float new_rate, float dt,
                           float r_measure);
static float IMU_MeasurementNoise(float accel_norm);

static void IMU_Process(void);
static void IMU_Publish(KalmanData_t *data);

/* ========================================================================== */
/*                              以下为函数实现                                */
/* ========================================================================== */

/*
 * 功能：IMU 任务入口，等待 DMA 完成信号后执行一次解算
 * 输入：argument —— 任务参数（未使用）
 * 输出：无
 */
void StartIMUTask(void *argument)
{
  (void)argument;

  /* 清掉 CubeMX 可能配置的初始计数；建议把 CubeMX 初始计数改为 0 */
  (void)osSemaphoreAcquire(IMUtaskBinarySemHandle, 0U);

  /* 滤波器在 IMU_Init() 中完成零偏校准后再初始化，避免这里覆盖校准值 */
  s_kalman_initialized = 0U;

  for (;;)
  {
    if (osSemaphoreAcquire(IMUtaskBinarySemHandle, osWaitForever) == osOK)
    {
      IMU_Process();
    }
  }
}

/*
 * 功能：查询 IMU 是否已成功发布首帧数据
 * 输入：无
 * 输出：1 = 首帧数据已发布；0 = 尚未就绪
 */
uint8_t IMU_IsReady(void)
{
  return s_imu_ready_flag;
}

/*
 * 功能：MPU6050 初始化（阻塞 I2C，必须在 TIM7 启动前调用）
 *       配置完成后做约 1 s 静止陀螺零偏校准，再初始化卡尔曼滤波器。
 * 输入：无
 * 输出：1 = I2C 配置与 WHO_AM_I 成功（零偏校准失败时仍为 1，滤波器会在线估计零偏）；
 *       0 = I2C 读写失败或 WHO_AM_I 校验失败
 */
uint8_t IMU_Init(void)
{
  uint8_t who_am_i = 0U;
  uint8_t attempt;

  /* TIM7 启动前先初始化共享 DWT 时钟，IMU/PID 都使用同一时间基准 */
  DWTTime_Init();

  s_gyro_bias = 0.0f;
  s_gyro_calibrated = 0U;
  s_imu_ready_flag = 0U;

  HAL_Delay(100U);

  if (MPU_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x01U) != HAL_OK)   /* 唤醒 + PLL X 陀螺 */
  {
    return 0U;
  }
  if (MPU_WriteReg(MPU6050_REG_SMPLRT_DIV, 0x01U) != HAL_OK)   /* 500 Hz */
  {
    return 0U;
  }
  if (MPU_WriteReg(MPU6050_REG_CONFIG, 0x02U) != HAL_OK)       /* DLPF 94/98 Hz */
  {
    return 0U;
  }
  if (MPU_WriteReg(MPU6050_REG_GYRO_CONFIG, 0x10U) != HAL_OK)  /* ±1000 dps */
  {
    return 0U;
  }
  if (MPU_WriteReg(MPU6050_REG_ACCEL_CONFIG, 0x00U) != HAL_OK) /* ±2g */
  {
    return 0U;
  }

  HAL_Delay(50U);

  if (MPU_ReadRegs(MPU6050_REG_WHO_AM_I, &who_am_i, 1U) != HAL_OK)
  {
    return 0U;
  }
  if (who_am_i != MPU6050_WHO_AM_I_VALUE)
  {
    return 0U;
  }

  /*
   * 上电静止零偏校准（500 点 × 2 ms ≈ 1 s）。
   * 若小车被碰动，峰峰值会超过门限，函数返回 0 并自动重试；
   * 全部重试失败时 bias 保持 0、P0_bias 取未校准值，由滤波器在线学习。
   */
  for (attempt = 0U; attempt < IMU_GYRO_CALIB_RETRIES; attempt++)
  {
    if (IMU_CalibrateGyro() != 0U)
    {
      break;
    }
    HAL_Delay(200U);
  }

  Kalman_Init(&s_kalman);
  s_kalman_initialized = 0U;

  return 1U;
}

/*
 * 功能：TIM7 每 2 ms 调用一次，启动 I2C DMA 读取 14 字节原始数据
 * 输入：无
 * 输出：无
 */
void IMU_OnTimer7(void)
{
  /* 每个 2 ms 触发都计数，供解算任务计算相邻成功帧的真实间隔 */
  s_tim7_ticks++;

  if (s_imu_busy != 0U)
  {
    return;
  }

  if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
  {
    return;
  }

  s_imu_busy = 1U;
  s_imu_frame_tick = s_tim7_ticks;
  /* 记录本帧 DMA 启动时刻，供 IMU 任务计算实际采样间隔 */
  s_imu_frame_cycles = DWTTime_GetCycles();
  if (HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H,
                           I2C_MEMADD_SIZE_8BIT, s_imu_raw, 14U) != HAL_OK)
  {
    s_imu_busy = 0U;
  }
}

/*
 * 功能：I2C 内存读 DMA 完成回调：保存一帧原始数据并释放信号量
 * 输入：hi2c —— I2C 句柄
 * 输出：无
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    (void)memcpy(s_imu_ready, s_imu_raw, sizeof(s_imu_ready));
    s_imu_ready_tick = s_imu_frame_tick;
    s_imu_ready_cycles = s_imu_frame_cycles;
    s_imu_busy = 0U;
    (void)osSemaphoreRelease(IMUtaskBinarySemHandle);
  }
}

/*
 * 功能：I2C 错误回调：清除忙标志并计数（总线恢复可后续补充）
 * 输入：hi2c —— I2C 句柄
 * 输出：无
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    s_imu_busy = 0U;
    s_imu_error_count++;
  }
}


/*
 * 功能：向 MPU6050 写一个寄存器（阻塞 I2C）
 * 输入：reg —— 寄存器地址；data —— 写入数据
 * 输出：无
 */
static HAL_StatusTypeDef MPU_WriteReg(uint8_t reg, uint8_t data)
{
  return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                           &data, 1U, HAL_MAX_DELAY);
}

/*
 * 功能：从 MPU6050 连续读寄存器（阻塞 I2C）
 * 输入：reg —— 起始寄存器；data —— 输出缓冲；len —— 字节数
 * 输出：无
 */
static HAL_StatusTypeDef MPU_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len)
{
  return HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                          data, len, HAL_MAX_DELAY);
}

/*
 * 功能：上电静止陀螺零偏校准（取 pitch 轴角速度均值）
 * 输入：无
 * 输出：1 = 校准成功；0 = 采样期间明显移动，校准失败
 */
static uint8_t IMU_CalibrateGyro(void)
{
  uint8_t raw[14];
  uint16_t i;
  int16_t gyro_raw;
  float rate;
  float sum = 0.0f;
  float min_rate = 0.0f;
  float max_rate = 0.0f;

  s_gyro_bias = 0.0f;
  s_gyro_calibrated = 0U;

  for (i = 0U; i < IMU_GYRO_CALIB_SAMPLES; i++)
  {
    if (MPU_ReadRegs(MPU6050_REG_ACCEL_XOUT_H, raw, 14U) != HAL_OK)
    {
      return 0U;
    }
    gyro_raw = (int16_t)(((uint16_t)raw[10] << 8) | (uint16_t)raw[11]);
    /* 与 IMU_Process() 中的 gyro_pitch_rate 保持同一符号 */
    rate = IMU_PITCH_SIGN * ((float)gyro_raw / IMU_GYRO_LSB_PER_DPS);

    if (i == 0U)
    {
      min_rate = rate;
      max_rate = rate;
    }
    else
    {
      if (rate < min_rate)
      {
        min_rate = rate;
      }
      if (rate > max_rate)
      {
        max_rate = rate;
      }
    }

    sum += rate;
    HAL_Delay(2U);
  }

  /* 峰峰值过大说明上电时小车被移动，本次均值不可信 */
  if ((max_rate - min_rate) > IMU_GYRO_CALIB_RANGE_DPS)
  {
    return 0U;
  }

  s_gyro_bias = sum / (float)IMU_GYRO_CALIB_SAMPLES;
  if (s_gyro_bias > IMU_KALMAN_BIAS_LIMIT)
  {
    s_gyro_bias = IMU_KALMAN_BIAS_LIMIT;
  }
  else if (s_gyro_bias < -IMU_KALMAN_BIAS_LIMIT)
  {
    s_gyro_bias = -IMU_KALMAN_BIAS_LIMIT;
  }

  s_gyro_calibrated = 1U;
  return 1U;
}

/*
 * 功能：卡尔曼滤波器初始化
 * 输入：k —— 滤波器实例
 * 输出：无
 */
static void Kalman_Init(Kalman_t *k)
{
  k->angle = 0.0f;
  k->bias = s_gyro_bias;
  k->P[0][0] = IMU_KALMAN_P0_ANGLE;
  k->P[0][1] = 0.0f;
  k->P[1][0] = 0.0f;
  k->P[1][1] = (s_gyro_calibrated != 0U) ? IMU_KALMAN_P0_BIAS
                                         : IMU_KALMAN_P0_BIAS_RAW;

  k->Q_angle = IMU_KALMAN_Q_ANGLE;
  k->Q_bias = IMU_KALMAN_Q_BIAS;
  k->R_measure = IMU_KALMAN_R_MEASURE;
}

/*
 * 功能：二状态卡尔曼滤波（角度 + 陀螺零偏）
 * 输入：k —— 滤波器实例；new_angle —— 加速度计角度（deg）；
 *       new_rate —— 陀螺角速度（deg/s）；dt —— 实际采样周期（s）；
 *       r_measure —— 本次测量噪声（deg^2，按加速度模长自适应）
 * 输出：滤波后的角度（deg）
 */
static float Kalman_Update(Kalman_t *k, float new_angle, float new_rate, float dt,
                           float r_measure)
{
  float rate = new_rate - k->bias;
  float S;
  float K0;
  float K1;
  float y;
  float P00;
  float P01;

  k->angle += dt * rate;

  k->P[0][0] += dt * (dt * k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
  k->P[0][1] -= dt * k->P[1][1];
  k->P[1][0] -= dt * k->P[1][1];
  k->P[1][1] += k->Q_bias * dt;

  S = k->P[0][0] + r_measure;
  K0 = k->P[0][0] / S;
  K1 = k->P[1][0] / S;

  y = new_angle - k->angle;
  k->angle += K0 * y;
  k->bias += K1 * y;

  /* MPU6050 数据手册给出的初始零偏约 ±20 dps，限幅防止滤波器发散 */
  if (k->bias > IMU_KALMAN_BIAS_LIMIT)
  {
    k->bias = IMU_KALMAN_BIAS_LIMIT;
  }
  else if (k->bias < -IMU_KALMAN_BIAS_LIMIT)
  {
    k->bias = -IMU_KALMAN_BIAS_LIMIT;
  }

  P00 = k->P[0][0];
  P01 = k->P[0][1];
  k->P[0][0] -= K0 * P00;
  k->P[0][1] -= K0 * P01;
  k->P[1][0] -= K1 * P00;
  k->P[1][1] -= K1 * P01;

  return k->angle;
}

/*
 * 功能：按加速度模长计算测量噪声 R
 * 输入：accel_norm —— sqrt(ax^2+ay^2+az^2)，单位 g
 * 输出：本次测量噪声，deg^2
 */
static float IMU_MeasurementNoise(float accel_norm)
{
  float deviation;
  float scale;

  /* 模长异常（含 NaN）时完全不信加速度计，只靠陀螺积分 */
  if (!(accel_norm > 0.0f) || (accel_norm > 4.0f))
  {
    return IMU_KALMAN_R_MEASURE * IMU_KALMAN_R_SCALE_MAX;
  }

  deviation = fabsf(accel_norm - 1.0f);
  if (deviation < IMU_KALMAN_ACC_DEADBAND)
  {
    deviation = 0.0f;
  }

  scale = 1.0f + (IMU_KALMAN_R_MOTION_GAIN * deviation);
  if (scale > IMU_KALMAN_R_SCALE_MAX)
  {
    scale = IMU_KALMAN_R_SCALE_MAX;
  }

  return IMU_KALMAN_R_MEASURE * scale;
}

/*
 * 功能：解析一帧原始数据，执行卡尔曼滤波并发布结果
 * 输入：无（使用 s_imu_ready）
 * 输出：无
 */
static void IMU_Process(void)
{
  int16_t ax_raw;
  int16_t ay_raw;
  int16_t az_raw;
  int16_t gy_raw;
  float ax_g;
  float ay_g;
  float az_g;
  float gy_dps;
  float accel_norm;
  float accel_pitch;
  float gyro_pitch_rate;
  float r_measure;
  float dt;
  float pitch;
  uint32_t frame_tick;
  uint32_t delta_ticks;
  uint32_t frame_cycles;
  uint32_t delta_cycles;
  KalmanData_t *out;

  ax_raw = (int16_t)(((uint16_t)s_imu_ready[0] << 8) | (uint16_t)s_imu_ready[1]);
  ay_raw = (int16_t)(((uint16_t)s_imu_ready[2] << 8) | (uint16_t)s_imu_ready[3]);
  az_raw = (int16_t)(((uint16_t)s_imu_ready[4] << 8) | (uint16_t)s_imu_ready[5]);
  gy_raw = (int16_t)(((uint16_t)s_imu_ready[10] << 8) | (uint16_t)s_imu_ready[11]);

  ax_g = (float)ax_raw / IMU_ACCEL_LSB_PER_G;
  ay_g = (float)ay_raw / IMU_ACCEL_LSB_PER_G;
  az_g = (float)az_raw / IMU_ACCEL_LSB_PER_G;
  gy_dps = (float)gy_raw / IMU_GYRO_LSB_PER_DPS;
  accel_norm = sqrtf((ax_g * ax_g) + (ay_g * ay_g) + (az_g * az_g));

  /*
   * 安装方向：默认 pitch 绕模块 Y 轴、X 轴朝前、Z 轴朝上。
   * 若只是前后装反（绕 Z 转 180°），把 IMU_PITCH_SIGN 改为 -1.0f，
   * pitch 与 pitch_rate 会同时取反；若绕 X/Y 翻转或改绕其他轴，
   * 必须同时修改上面的加速度计公式和下面的陀螺轴，不能只改符号。
   */
  accel_pitch = IMU_PITCH_SIGN * atan2f(-ax_g, sqrtf((ay_g * ay_g) + (az_g * az_g)))
                * IMU_RAD_TO_DEG;
  gyro_pitch_rate = IMU_PITCH_SIGN * gy_dps;

  if (s_kalman_initialized == 0U)
  {
    /* 首帧只用加速度计定初值，并保留上电校准得到的零偏 */
    s_kalman.angle = accel_pitch;
    s_kalman.bias = s_gyro_bias;
    s_kalman.P[0][0] = IMU_KALMAN_P0_ANGLE;
    s_kalman.P[0][1] = 0.0f;
    s_kalman.P[1][0] = 0.0f;
    s_kalman.P[1][1] = (s_gyro_calibrated != 0U) ? IMU_KALMAN_P0_BIAS
                                                 : IMU_KALMAN_P0_BIAS_RAW;
    s_kalman_initialized = 1U;
    s_imu_last_tick = s_imu_ready_tick;
    s_imu_last_cycles = s_imu_ready_cycles;
    pitch = accel_pitch;
  }
  else
  {
    /*
     * TIM7 2 ms 计数仍保留：
     *   - 用于检测相邻成功帧是否超过 20 s（DWT 32 位回绕保护）；
     *   - DWT 不可用时作为回退时间基准。
     */
    frame_tick = s_imu_ready_tick;
    delta_ticks = frame_tick - s_imu_last_tick;
    s_imu_last_tick = frame_tick;
    if (delta_ticks == 0U)
    {
      delta_ticks = 1U;
    }

    if (delta_ticks > IMU_KALMAN_DT_GUARD_TICKS)
    {
      /* 长时间没有成功帧，DWT 差值可能回绕，直接重置时间戳 */
      s_imu_last_cycles = s_imu_ready_cycles;
      dt = IMU_KALMAN_DT_MAX;
    }
    else if ((DWTTime_IsReady() != 0U) && (SystemCoreClock != 0U))
    {
      /* 用 DMA 启动时刻的 DWT 时间戳计算实际采样间隔 */
      frame_cycles = s_imu_ready_cycles;
      delta_cycles = frame_cycles - s_imu_last_cycles;
      s_imu_last_cycles = frame_cycles;

      if (delta_cycles == 0U)
      {
        dt = IMU_SAMPLE_DT_S;
      }
      else
      {
        dt = (float)delta_cycles / (float)SystemCoreClock;
      }
    }
    else
    {
      /* DWT 不可用：退回 TIM7 2 ms 计数 */
      s_imu_last_cycles = s_imu_ready_cycles;
      dt = IMU_SAMPLE_DT_S * (float)delta_ticks;
    }

    if (dt > IMU_KALMAN_DT_MAX)
    {
      dt = IMU_KALMAN_DT_MAX;
    }

    r_measure = IMU_MeasurementNoise(accel_norm);
    pitch = Kalman_Update(&s_kalman, accel_pitch, gyro_pitch_rate, dt, r_measure);
  }

  out = &s_kalman_buf[s_kalman_write_idx];
  out->pitch = pitch;
  out->pitch_rate = gyro_pitch_rate - s_kalman.bias;
  out->gyro_bias = s_kalman.bias;
  out->timestamp = HAL_GetTick();

  IMU_Publish(out);
  s_kalman_write_idx ^= 1U;
}

/*
 * 功能：把解算结果指针覆盖写入 kalmaneddataQueue（深度 1）
 * 输入：data —— 指向已填充的双缓冲之一
 * 输出：无
 */
static void IMU_Publish(KalmanData_t *data)
{
  (void)osMessageQueueReset(kalmaneddataQueueHandle);
  if (osMessageQueuePut(kalmaneddataQueueHandle, &data, 0U, 0U) == osOK)
  {
    /* 只有数据真正进入队列，才允许 PID 解除启动等待 */
    s_imu_ready_flag = 1U;
  }
}

/************************ (C) COPYRIGHT *****END OF FILE***********************/

