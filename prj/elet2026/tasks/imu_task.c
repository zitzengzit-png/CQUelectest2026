/**
  ******************************************************************************
  * @file    imu_task.c
  * @brief   ATK-MS901M 九轴 IMU：USART3 + DMA 接收、帧解析。
  ******************************************************************************
  * @note    数据流：
  *          USART3 RX DMA 每收满 IMU_DMA_BYTES 字节
  *          -> HAL_UART_RxCpltCallback()
  *          -> 拷贝到 s_imu_snapshot + 释放 IMUdatadoneBinarySem
  *          -> PID 任务 IMU_Parse() 解析 0x55 0x55 帧
  *          -> IMU_GetData() 给 PID 控制使用。
  *
  *          模块配置：921600、250 Hz、姿态 + 陀螺/加速度 + 磁力计、九轴算法。
  ******************************************************************************
  */

/* 头文件包含 ----------------------------------------------------------------*/
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "usart.h"
#include "imu_task.h"

/* 私有宏定义 ----------------------------------------------------------------*/
/* 协议固定常量；可调参数（波特率、上报率、量程、轴映射等）见 robot_config.h */
#define IMU_UPLOAD_HEADER0       0x55U
#define IMU_UPLOAD_HEADER1       0x55U
#define IMU_CMD_HEADER0          0x55U
#define IMU_CMD_HEADER1          0xAFU

#define IMU_REG_SAVE             0x00U
#define IMU_REG_BAUD             0x07U
#define IMU_REG_RETURNSET        0x08U
#define IMU_REG_RETURNRATE       0x0AU
#define IMU_REG_ALG              0x0BU

#define IMU_FRAME_ID_ATTITUDE    0x01U
#define IMU_FRAME_ID_GYRO_ACC    0x03U
#define IMU_FRAME_ID_MAG         0x04U

#define IMU_FRAME_ATTITUDE_LEN   6U
#define IMU_FRAME_GYRO_ACC_LEN   12U
#define IMU_FRAME_MAG_LEN        8U

/* 私有变量 ------------------------------------------------------------------*/
static uint8_t s_imu_dma_buf[IMU_DMA_BYTES];
static uint8_t s_imu_snapshot[IMU_DMA_BYTES];
static volatile uint16_t s_imu_snapshot_len = 0U;

static uint8_t s_imu_stream[IMU_STREAM_BUF_SIZE];
static uint16_t s_imu_stream_len = 0U;

static IMU_Data_t s_imu_data;
static uint8_t s_have_attitude = 0U;
static uint8_t s_have_gyro = 0U;
static uint8_t s_imu_boot_delay_done = 0U;
static volatile uint8_t s_imu_ready = 0U;
static volatile uint32_t s_imu_error_count = 0U;

/* 私有函数声明 --------------------------------------------------------------*/
static void IMU_ResetParser(void);
static uint8_t IMU_StartDma(void);
static uint8_t IMU_TryStart(void);
static uint8_t IMU_SetUartBaud(uint32_t baud);
static void IMU_SendFrame(uint8_t id, uint8_t data);
static void IMU_SendConfig(void);
static uint16_t IMU_ReadU16(const uint8_t *data);
static int16_t IMU_ReadS16(const uint8_t *data);
static void IMU_ParseFrame(const uint8_t *frame, uint8_t data_len);
static float IMU_GetPitchRate(float gx, float gy, float gz);
static void IMU_AppendBytes(const uint8_t *data, uint16_t len);

/* ========================================================================== */
/*                              以下为函数实现                                */
/* ========================================================================== */

/*
 * 功能：初始化 USART3 接收 DMA，并向 ATK-MS901M 写入上报配置
 * 输入：无
 * 输出：1 = 已收到有效姿态/陀螺帧；0 = 超时或 DMA 启动失败
 */
uint8_t IMU_Init(void)
{
  uint8_t attempt;

  s_imu_error_count = 0U;

  /*
   * 模块上电后默认会做陀螺零偏自动校准，需要保持静止约 1 s。
   * 只在第一次初始化时等待，后续失败重试不再重复等待。
   */
  if (s_imu_boot_delay_done == 0U)
  {
    (void)osDelay(1000U);
    s_imu_boot_delay_done = 1U;
  }

  /* 第一次按 921600 尝试：模块已用上位机保存过 921600 时会直接成功 */
  if (IMU_TryStart() != 0U)
  {
    return 1U;
  }

  /*
   * 回退到 115200：模块仍处于出厂默认波特率时，只发送“改波特率 + 保存”。
   * 注意：手册要求波特率修改后模块重启才生效。若模块没有立即切换，
   * 需要给模块重新上电一次；之后 IMU_Init() 的第一次 921600 尝试即可成功。
   */
  if (IMU_SetUartBaud(IMU_BAUD_FALLBACK) == 0U)
  {
    return 0U;
  }

  IMU_SendFrame(IMU_REG_BAUD, 0x00U); /* 0x00 = 921600 bps */
  (void)osDelay(5U);
  IMU_SendFrame(IMU_REG_SAVE, 0x00U);
  (void)osDelay(100U);

  if (IMU_SetUartBaud(IMU_UART_BAUD) == 0U)
  {
    return 0U;
  }

  for (attempt = 0U; attempt < 2U; attempt++)
  {
    if (IMU_TryStart() != 0U)
    {
      return 1U;
    }
    (void)osDelay(200U);
  }

  return 0U;
}

/*
 * 功能：清空解析器状态，开始一次新的接收会话
 * 输入：无
 * 输出：无
 */
static void IMU_ResetParser(void)
{
  taskENTER_CRITICAL();
  s_imu_snapshot_len = 0U;
  taskEXIT_CRITICAL();

  s_imu_stream_len = 0U;
  (void)memset(&s_imu_data, 0, sizeof(s_imu_data));
  s_have_attitude = 0U;
  s_have_gyro = 0U;
  s_imu_ready = 0U;
}

/*
 * 功能：启动一次 USART3 RX DMA 接收
 * 输入：无
 * 输出：1 = 启动成功；0 = 启动失败
 */
static uint8_t IMU_StartDma(void)
{
  (void)HAL_UART_AbortReceive(&huart3);
  __HAL_UART_CLEAR_OREFLAG(&huart3);

  if (HAL_UART_Receive_DMA(&huart3, s_imu_dma_buf, IMU_DMA_BYTES) != HAL_OK)
  {
    s_imu_error_count++;
    return 0U;
  }

  return 1U;
}

/*
 * 功能：按当前 UART 波特率发送配置并等待首帧有效数据
 * 输入：无
 * 输出：1 = 已收到有效数据；0 = 超时
 */
static uint8_t IMU_TryStart(void)
{
  uint32_t elapsed_ms;

  IMU_ResetParser();
  IMU_SendConfig();

  if (IMU_StartDma() == 0U)
  {
    return 0U;
  }

  for (elapsed_ms = 0U; elapsed_ms < IMU_READY_TIMEOUT_MS; elapsed_ms += 10U)
  {
    if (s_imu_ready != 0U)
    {
      return 1U;
    }
    (void)osDelay(10U);
  }

  (void)HAL_UART_AbortReceive(&huart3);
  return 0U;
}

/*
 * 功能：修改 USART3 波特率并重新初始化
 * 输入：baud —— 目标波特率
 * 输出：1 = 成功；0 = 失败
 */
static uint8_t IMU_SetUartBaud(uint32_t baud)
{
  (void)HAL_UART_AbortReceive(&huart3);
  huart3.Init.BaudRate = baud;

  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    s_imu_error_count++;
    return 0U;
  }

  __HAL_UART_CLEAR_OREFLAG(&huart3);
  return 1U;
}

/*
 * 功能：发送一条 ATK-MS901M 单字节寄存器写命令
 * 输入：id —— 寄存器帧 ID；data —— 写入值
 * 输出：无
 */
static void IMU_SendFrame(uint8_t id, uint8_t data)
{
  uint8_t frame[6];

  frame[0] = IMU_CMD_HEADER0;
  frame[1] = IMU_CMD_HEADER1;
  frame[2] = id;
  frame[3] = 0x01U;
  frame[4] = data;
  frame[5] = (uint8_t)(frame[0] + frame[1] + frame[2] + frame[3] + frame[4]);

  (void)HAL_UART_Transmit(&huart3, frame, (uint16_t)sizeof(frame), 100U);
}

/*
 * 功能：写入 250 Hz、两帧上报内容和九轴算法，并保存到模块 Flash
 * 输入：无
 * 输出：无
 *
 * 说明：量程和带宽沿用模块默认值（陀螺 ±2000 dps、加速度 ±4 g）。
 *       若模块被改过，必须同步修改 IMU_GYRO_FSR_DPS / IMU_ACC_FSR_G，
 *       或先用上位机恢复默认值。ALG 修改后模块需要重启才生效。
 */
static void IMU_SendConfig(void)
{
  IMU_SendFrame(IMU_REG_RETURNRATE, IMU_RETURNRATE_VALUE);
  (void)osDelay(2U);

  IMU_SendFrame(IMU_REG_RETURNSET, IMU_RETURNSET_VALUE);
  (void)osDelay(2U);

  IMU_SendFrame(IMU_REG_ALG, IMU_ALG_VALUE);
  (void)osDelay(2U);

  IMU_SendFrame(IMU_REG_SAVE, 0x00U);
  (void)osDelay(2U);
}


/*
 * 功能：读取小端无符号 16 位
 * 输入：data —— 两字节数据
 * 输出：转换后的 uint16_t
 */
static uint16_t IMU_ReadU16(const uint8_t *data)
{
  return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

/*
 * 功能：读取小端有符号 16 位
 * 输入：data —— 两字节数据
 * 输出：转换后的 int16_t
 */
static int16_t IMU_ReadS16(const uint8_t *data)
{
  return (int16_t)IMU_ReadU16(data);
}

/*
 * 功能：按帧 ID 解析一帧有效数据
 * 输入：frame —— 完整帧起始地址；data_len —— 数据段长度
 * 输出：无（更新 s_imu_data）
 */
static void IMU_ParseFrame(const uint8_t *frame, uint8_t data_len)
{
  const uint8_t *data = &frame[4];
  float gx;
  float gy;
  float gz;

  switch (frame[2])
  {
    case IMU_FRAME_ID_ATTITUDE:
      if (data_len == IMU_FRAME_ATTITUDE_LEN)
      {
        s_imu_data.roll_deg = ((float)IMU_ReadS16(&data[0]) / 32768.0f) * 180.0f;
        s_imu_data.pitch_deg = IMU_PITCH_SIGN *
            (((float)IMU_ReadS16(&data[2]) / 32768.0f) * 180.0f);
        s_imu_data.yaw_deg = IMU_YAW_SIGN *
            (((float)IMU_ReadS16(&data[4]) / 32768.0f) * 180.0f);
        s_imu_data.timestamp_ms = HAL_GetTick();
        s_have_attitude = 1U;
      }
      break;

    case IMU_FRAME_ID_GYRO_ACC:
      if (data_len == IMU_FRAME_GYRO_ACC_LEN)
      {
        s_imu_data.acc_x_g = ((float)IMU_ReadS16(&data[0]) / 32768.0f) * IMU_ACC_FSR_G;
        s_imu_data.acc_y_g = ((float)IMU_ReadS16(&data[2]) / 32768.0f) * IMU_ACC_FSR_G;
        s_imu_data.acc_z_g = ((float)IMU_ReadS16(&data[4]) / 32768.0f) * IMU_ACC_FSR_G;

        gx = ((float)IMU_ReadS16(&data[6]) / 32768.0f) * IMU_GYRO_FSR_DPS;
        gy = ((float)IMU_ReadS16(&data[8]) / 32768.0f) * IMU_GYRO_FSR_DPS;
        gz = ((float)IMU_ReadS16(&data[10]) / 32768.0f) * IMU_GYRO_FSR_DPS;

        s_imu_data.gyro_x_dps = gx;
        s_imu_data.gyro_y_dps = gy;
        s_imu_data.gyro_z_dps = gz;
        s_imu_data.pitch_rate_dps = IMU_GetPitchRate(gx, gy, gz);
        s_imu_data.yaw_rate_dps = IMU_YAW_RATE_SIGN * gz;
        s_have_gyro = 1U;
      }
      break;

    case IMU_FRAME_ID_MAG:
      if (data_len == IMU_FRAME_MAG_LEN)
      {
        s_imu_data.mag_x = IMU_ReadS16(&data[0]);
        s_imu_data.mag_y = IMU_ReadS16(&data[2]);
        s_imu_data.mag_z = IMU_ReadS16(&data[4]);
        s_imu_data.temperature_c = (float)IMU_ReadS16(&data[6]) / 100.0f;
      }
      break;

    default:
      break;
  }
}

/*
 * 功能：按安装方向选择 pitch 角速度所在陀螺轴
 * 输入：gx / gy / gz —— 三轴角速度，deg/s
 * 输出：pitch 角速度，deg/s
 */
static float IMU_GetPitchRate(float gx, float gy, float gz)
{
#if (IMU_PITCH_GYRO_AXIS == 0)
  return IMU_PITCH_RATE_SIGN * gx;
#elif (IMU_PITCH_GYRO_AXIS == 1)
  return IMU_PITCH_RATE_SIGN * gy;
#elif (IMU_PITCH_GYRO_AXIS == 2)
  return IMU_PITCH_RATE_SIGN * gz;
#else
#error "IMU_PITCH_GYRO_AXIS must be 0, 1 or 2"
  return 0.0f;
#endif
}

/*
 * 功能：把新收到的字节追加到解析缓冲区，必要时丢弃旧数据
 * 输入：data —— 新数据；len —— 字节数
 * 输出：无
 */
static void IMU_AppendBytes(const uint8_t *data, uint16_t len)
{
  if (len > IMU_STREAM_BUF_SIZE)
  {
    len = IMU_STREAM_BUF_SIZE;
  }

  if (len > (uint16_t)(IMU_STREAM_BUF_SIZE - s_imu_stream_len))
  {
    s_imu_stream_len = 0U;
  }

  (void)memcpy(&s_imu_stream[s_imu_stream_len], data, len);
  s_imu_stream_len = (uint16_t)(s_imu_stream_len + len);
}


/*
 * 功能：解析 DMA 完成中断保存的一批原始字节
 * 输入：无
 * 输出：1 = 本次至少解析出一帧；0 = 没有新数据或不完整
 */
uint8_t IMU_Parse(void)
{
  uint8_t chunk[IMU_DMA_BYTES];
  uint16_t chunk_len;
  uint16_t data_len;
  uint16_t frame_len;
  uint16_t i;
  uint8_t sum;
  uint8_t parsed = 0U;

  taskENTER_CRITICAL();
  chunk_len = s_imu_snapshot_len;
  if (chunk_len > (uint16_t)sizeof(chunk))
  {
    chunk_len = (uint16_t)sizeof(chunk);
  }
  if (chunk_len > 0U)
  {
    (void)memcpy(chunk, s_imu_snapshot, chunk_len);
    s_imu_snapshot_len = 0U;
  }
  taskEXIT_CRITICAL();

  if (chunk_len == 0U)
  {
    return 0U;
  }

  IMU_AppendBytes(chunk, chunk_len);

  while (s_imu_stream_len >= 5U)
  {
    if ((s_imu_stream[0] != IMU_UPLOAD_HEADER0) ||
        (s_imu_stream[1] != IMU_UPLOAD_HEADER1))
    {
      (void)memmove(s_imu_stream, &s_imu_stream[1],
                    (size_t)(s_imu_stream_len - 1U));
      s_imu_stream_len--;
      continue;
    }

    data_len = s_imu_stream[3];
    frame_len = (uint16_t)(data_len + 5U);

    if (frame_len > IMU_STREAM_BUF_SIZE)
    {
      (void)memmove(s_imu_stream, &s_imu_stream[1],
                    (size_t)(s_imu_stream_len - 1U));
      s_imu_stream_len--;
      continue;
    }

    if (s_imu_stream_len < frame_len)
    {
      break;
    }

    sum = 0U;
    for (i = 0U; i < (uint16_t)(frame_len - 1U); i++)
    {
      sum = (uint8_t)(sum + s_imu_stream[i]);
    }

    if (sum != s_imu_stream[frame_len - 1U])
    {
      (void)memmove(s_imu_stream, &s_imu_stream[1],
                    (size_t)(s_imu_stream_len - 1U));
      s_imu_stream_len--;
      continue;
    }

    IMU_ParseFrame(s_imu_stream, (uint8_t)data_len);
    parsed = 1U;

    if (s_imu_stream_len > frame_len)
    {
      (void)memmove(s_imu_stream, &s_imu_stream[frame_len],
                    (size_t)(s_imu_stream_len - frame_len));
    }
    s_imu_stream_len = (uint16_t)(s_imu_stream_len - frame_len);
  }

  s_imu_data.valid = ((s_have_attitude != 0U) && (s_have_gyro != 0U)) ? 1U : 0U;
  if (s_imu_data.valid != 0U)
  {
    s_imu_ready = 1U;
  }

  return parsed;
}

/*
 * 功能：读取最新解析结果
 * 输入：data —— 输出结构体
 * 输出：无
 */
void IMU_GetData(IMU_Data_t *data)
{
  if (data != NULL)
  {
    *data = s_imu_data;
  }
}

/*
 * 功能：查询是否已收到姿态和陀螺数据
 * 输入：无
 * 输出：1 = 已就绪；0 = 未就绪
 */
uint8_t IMU_IsReady(void)
{
  return s_imu_ready;
}

/*
 * 功能：读取 UART/DMA 错误计数
 * 输入：无
 * 输出：错误计数
 */
uint32_t IMU_GetErrorCount(void)
{
  return s_imu_error_count;
}

/*
 * 功能：USART3 RX DMA 完成回调：保存一批原始数据、释放信号量并重新挂起 DMA
 * 输入：huart —— UART 句柄
 * 输出：无
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    (void)memcpy(s_imu_snapshot, s_imu_dma_buf, IMU_DMA_BYTES);
    s_imu_snapshot_len = IMU_DMA_BYTES;

    if (IMUdatadoneBinarySemHandle != NULL)
    {
      (void)osSemaphoreRelease(IMUdatadoneBinarySemHandle);
    }

    if (HAL_UART_Receive_DMA(&huart3, s_imu_dma_buf, IMU_DMA_BYTES) != HAL_OK)
    {
      s_imu_error_count++;
      (void)HAL_UART_AbortReceive(&huart3);
      if (HAL_UART_Receive_DMA(&huart3, s_imu_dma_buf, IMU_DMA_BYTES) != HAL_OK)
      {
        s_imu_error_count++;
      }
    }
  }
}

/*
 * 功能：USART3 错误回调：中止当前接收并重新挂起 DMA
 * 输入：huart —— UART 句柄
 * 输出：无
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    s_imu_error_count++;
    (void)HAL_UART_AbortReceive(&huart3);

    if (HAL_UART_Receive_DMA(&huart3, s_imu_dma_buf, IMU_DMA_BYTES) != HAL_OK)
    {
      s_imu_error_count++;
    }
  }
}

/************************ (C) COPYRIGHT *****END OF FILE***********************/


