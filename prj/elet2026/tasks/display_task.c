/**
  ******************************************************************************
  * @file    display_task.c
  * @brief   事件驱动 debugTask：50 Hz PID 快照 -> USART6 / OLED DMA 输出。
  ******************************************************************************
  * @note    显示任务优先级 8，低于 controlTask(13) 和 PID(28)，
  *          只在 PID/control 空闲时运行，不参与控制。
  *          队列阻塞等待快照；USART6 与 OLED 各自检查 DMA 就绪，忙则跳过本帧。
  ******************************************************************************
  */

/* 头文件包含 ----------------------------------------------------------------*/
#include <string.h>
#include "FreeRTOS.h"
#include "main.h"
#include "usart.h"
#include "cmsis_os.h"
#include "display_task.h"
#include "oled.h"
#include "dwt_time.h"
#include "pid_task.h"
#include "control_task.h"

/* 私有宏定义 ----------------------------------------------------------------*/
/* DISPLAY_LINE_*、DISPLAY_UART6_TX_TIMEOUT_MS、DISPLAY_FREQ_WINDOW_SAMPLES
 * 均来自 robot_config.h。 */

/* USART6 遥测帧：AA 55 | len=80 | ver=2 | LCD_Data_t(80 B) | CRC16-CCITT */
#define DISPLAY_BT_SYNC0             0xAAU
#define DISPLAY_BT_SYNC1             0x55U
#define DISPLAY_BT_VERSION           0x02U
#define DISPLAY_BT_HEADER_SIZE       4U
#define DISPLAY_BT_CRC_SIZE          2U
#define DISPLAY_BT_PAYLOAD_OFFSET    DISPLAY_BT_HEADER_SIZE
#define DISPLAY_BT_CRC_OFFSET        (DISPLAY_BT_PAYLOAD_OFFSET + LCD_QUEUE_ITEM_SIZE)
#define DISPLAY_BT_FRAME_SIZE        (DISPLAY_BT_CRC_OFFSET + DISPLAY_BT_CRC_SIZE)
#define DISPLAY_BT_CRC_POLY          0x1021U
#define DISPLAY_BT_CRC_INIT          0xFFFFU

typedef char DisplayTask_BtFrameSizeCheck[
    (DISPLAY_BT_FRAME_SIZE == 86U) ? 1 : -1];

/* 私有变量 ------------------------------------------------------------------*/
static LCD_Data_t s_lcd_data;
static uint8_t s_oled_available = 0U;
static uint8_t s_uart6_frame[DISPLAY_BT_FRAME_SIZE];
static volatile uint8_t s_uart6_tx_ready = 1U;
static volatile uint32_t s_uart6_error_count = 0U;
static uint32_t s_uart6_start_tick = 0U;

/* 两次 displayTask 刷新之间的实际运行频率采样 */
static uint32_t s_last_pid_count = 0U;
static uint32_t s_last_control_count = 0U;
static uint32_t s_last_dwt_cycles = 0U;
static uint32_t s_pid_freq_hz = 0U;
static uint32_t s_control_freq_hz = 0U;
static uint8_t s_freq_initialized = 0U;
static uint8_t s_freq_valid = 0U;
static uint8_t s_freq_window_count = 0U;

/* 私有函数声明 --------------------------------------------------------------*/
static void DisplayTask_PutChar(char *line, uint8_t *pos, char ch);
static void DisplayTask_PutString(char *line, uint8_t *pos, const char *text);
static void DisplayTask_PutUnsigned(char *line, uint8_t *pos,
                                    uint32_t value, uint8_t width);
static void DisplayTask_PutSigned(char *line, uint8_t *pos,
                                  int32_t value, uint8_t width);
static void DisplayTask_PutFixed1(char *line, uint8_t *pos,
                                  int16_t value_0p1, uint8_t width);
static const char *DisplayTask_ModeString(uint8_t mode);
static void DisplayTask_UpdateFrequencies(void);
static void DisplayTask_Render(const LCD_Data_t *data);
static void DisplayTask_RenderWaiting(void);
static uint16_t DisplayTask_Crc16(const uint8_t *data, uint16_t length);
static void DisplayTask_BuildUart6Frame(const LCD_Data_t *data);
static void DisplayTask_StartUart6Dma(const LCD_Data_t *data);
static void DisplayTask_StartOled(const LCD_Data_t *data);
static void DisplayTask_UpdateUart6Watchdog(void);

/* ========================================================================== */
/*                              以下为函数实现                                */
/* ========================================================================== */

/*
 * 功能：向行缓冲区追加一个字符（最多 21 个）
 * 输入：line —— 行缓冲区；pos —— 当前写入位置；ch —— 字符
 * 输出：无
 */
static void DisplayTask_PutChar(char *line, uint8_t *pos, char ch)
{
  if (*pos < DISPLAY_LINE_CHARS)
  {
    line[*pos] = ch;
    (*pos)++;
    line[*pos] = '\0';
  }
}

/*
 * 功能：向行缓冲区追加字符串
 * 输入：line —— 行缓冲区；pos —— 当前写入位置；text —— 字符串
 * 输出：无
 */
static void DisplayTask_PutString(char *line, uint8_t *pos, const char *text)
{
  if (text == NULL)
  {
    return;
  }

  while ((*text != '\0') && (*pos < DISPLAY_LINE_CHARS))
  {
    line[*pos] = *text;
    (*pos)++;
    text++;
  }
  line[*pos] = '\0';
}

/*
 * 功能：追加定宽无符号十进制数，左侧补 0
 * 输入：line/pos —— 行缓冲区和位置；value —— 数值；width —— 总宽度
 * 输出：无
 */
static void DisplayTask_PutUnsigned(char *line, uint8_t *pos,
                                    uint32_t value, uint8_t width)
{
  char tmp[10];
  uint8_t n = 0U;
  uint8_t i;
  uint32_t limit = 1U;

  if (width == 0U)
  {
    return;
  }
  if (width > 9U)
  {
    width = 9U;
  }

  for (i = 0U; i < width; i++)
  {
    limit *= 10U;
  }
  if (value >= limit)
  {
    value = limit - 1U;
  }

  do
  {
    tmp[n] = (char)('0' + (value % 10U));
    n++;
    value /= 10U;
  } while ((value != 0U) && (n < sizeof(tmp)));

  while (n < width)
  {
    tmp[n] = '0';
    n++;
  }

  while (n > 0U)
  {
    n--;
    DisplayTask_PutChar(line, pos, tmp[n]);
  }
}

/*
 * 功能：追加定宽有符号十进制数，签号占 1 位，剩余位左侧补 0
 * 输入：line/pos —— 行缓冲区和位置；value —— 数值；width —— 总宽度
 * 输出：无
 */
static void DisplayTask_PutSigned(char *line, uint8_t *pos,
                                  int32_t value, uint8_t width)
{
  char tmp[10];
  uint8_t n = 0U;
  uint8_t i;
  uint32_t abs_value;
  uint32_t limit = 1U;
  char sign;

  if (width < 2U)
  {
    return;
  }
  if (width > 9U)
  {
    width = 9U;
  }

  for (i = 0U; i < (uint8_t)(width - 1U); i++)
  {
    limit *= 10U;
  }

  if (value < 0)
  {
    sign = '-';
    abs_value = (uint32_t)(-(value + 1)) + 1U;
  }
  else
  {
    sign = '+';
    abs_value = (uint32_t)value;
  }

  if (abs_value >= limit)
  {
    abs_value = limit - 1U;
  }

  do
  {
    tmp[n] = (char)('0' + (abs_value % 10U));
    n++;
    abs_value /= 10U;
  } while ((abs_value != 0U) && (n < sizeof(tmp)));

  while (n < (uint8_t)(width - 1U))
  {
    tmp[n] = '0';
    n++;
  }

  DisplayTask_PutChar(line, pos, sign);
  while (n > 0U)
  {
    n--;
    DisplayTask_PutChar(line, pos, tmp[n]);
  }
}

/*
 * 功能：追加 0.1 单位定点数，格式如 +012.3；宽度包含符号和小数点
 * 输入：line/pos —— 行缓冲区和位置；value_0p1 —— 0.1 单位数值；width —— 总宽度
 * 输出：无
 */
static void DisplayTask_PutFixed1(char *line, uint8_t *pos,
                                  int16_t value_0p1, uint8_t width)
{
  char tmp[10];
  uint8_t n = 0U;
  uint8_t i;
  uint8_t int_digits;
  uint32_t abs_value;
  uint32_t limit = 1U;
  uint32_t int_part;
  char sign;

  if (width < 4U)
  {
    return;
  }
  if (width > 9U)
  {
    width = 9U;
  }

  int_digits = (uint8_t)(width - 3U); /* 符号 + 整数 + '.' + 1 位小数 */
  for (i = 0U; i < int_digits; i++)
  {
    limit *= 10U;
  }

  if (value_0p1 < 0)
  {
    sign = '-';
    abs_value = (uint32_t)(-((int32_t)value_0p1));
  }
  else
  {
    sign = '+';
    abs_value = (uint32_t)value_0p1;
  }

  if (abs_value >= (limit * 10U))
  {
    abs_value = (limit * 10U) - 1U;
  }

  int_part = abs_value / 10U;
  do
  {
    tmp[n] = (char)('0' + (int_part % 10U));
    n++;
    int_part /= 10U;
  } while ((int_part != 0U) && (n < sizeof(tmp)));

  while (n < int_digits)
  {
    tmp[n] = '0';
    n++;
  }

  DisplayTask_PutChar(line, pos, sign);
  while (n > 0U)
  {
    n--;
    DisplayTask_PutChar(line, pos, tmp[n]);
  }
  DisplayTask_PutChar(line, pos, '.');
  DisplayTask_PutChar(line, pos, (char)('0' + (abs_value % 10U)));
}

/*
 * 功能：把 PID 模式映射成 3 字符显示
 * 输入：mode —— LCD_MODE_*
 * 输出：模式字符串
 */
static const char *DisplayTask_ModeString(uint8_t mode)
{
  switch (mode)
  {
    case LCD_MODE_POSITION:
      return "POS";
    case LCD_MODE_SPEED:
      return "SPD";
    case LCD_MODE_ATTITUDE:
      return "ATT";
    case LCD_MODE_STOP:
    default:
      return "STP";
  }
}

/*
 * 功能：用统计窗口两端的 DWT 时间和运行计数差计算实际频率
 * 输入：无
 * 输出：无（更新 s_pid_freq_hz / s_control_freq_hz）
 *
 * 统计窗口为 DISPLAY_FREQ_WINDOW_SAMPLES 个队列快照（50 Hz 下约 160 ms）；
 * 频率 = 计数差 / (DWT 周期差 / SystemCoreClock)，结果四舍五入到整数 Hz。
 */
static void DisplayTask_UpdateFrequencies(void)
{
  uint32_t now_cycles;
  uint32_t pid_count;
  uint32_t control_count;
  uint32_t delta_cycles;
  uint64_t pid_freq;
  uint64_t control_freq;

  if (DWTTime_IsReady() == 0U)
  {
    return;
  }

  now_cycles = DWTTime_GetCycles();
  pid_count = PIDTask_GetRunCount();
  control_count = ControlTask_GetRunCount();

  if (s_freq_initialized == 0U)
  {
    s_last_dwt_cycles = now_cycles;
    s_last_pid_count = pid_count;
    s_last_control_count = control_count;
    s_freq_window_count = 0U;
    s_freq_initialized = 1U;
    return;
  }

  s_freq_window_count++;
  if (s_freq_window_count < DISPLAY_FREQ_WINDOW_SAMPLES)
  {
    return;
  }

  delta_cycles = now_cycles - s_last_dwt_cycles;
  if (delta_cycles == 0U)
  {
    return;
  }

  pid_freq = ((uint64_t)(pid_count - s_last_pid_count) *
              (uint64_t)SystemCoreClock +
              ((uint64_t)delta_cycles / 2U)) / (uint64_t)delta_cycles;

  control_freq = ((uint64_t)(control_count - s_last_control_count) *
                  (uint64_t)SystemCoreClock +
                  ((uint64_t)delta_cycles / 2U)) / (uint64_t)delta_cycles;

  s_pid_freq_hz = (uint32_t)pid_freq;
  s_control_freq_hz = (uint32_t)control_freq;

  s_last_dwt_cycles = now_cycles;
  s_last_pid_count = pid_count;
  s_last_control_count = control_count;
  s_freq_window_count = 0U;
  s_freq_valid = 1U;
}

/*
 * 功能：把一份 PID 快照渲染到 OLED 显存（不启动 DMA）
 * 输入：data —— 显示快照
 * 输出：无
 */
static void DisplayTask_Render(const LCD_Data_t *data)
{
  char line[DISPLAY_LINE_BUF_SIZE];
  uint8_t pos;
  uint32_t error_count;
  uint8_t imu_ok;

  if (data == NULL)
  {
    return;
  }

  OLED_Clear();

  /* 第 0 行：模式 / 运行状态 / PID 实际频率 / controlTask 实际频率 / 错误数 */
  line[0] = '\0';
  pos = 0U;
  DisplayTask_PutString(line, &pos, DisplayTask_ModeString(data->mode));
  DisplayTask_PutChar(line, &pos, ' ');
  DisplayTask_PutString(line, &pos,
                        (data->mode == LCD_MODE_STOP) ? "STP" : "RUN");
  DisplayTask_PutChar(line, &pos, ' ');

  if (s_freq_valid != 0U)
  {
    DisplayTask_PutChar(line, &pos, 'P');
    DisplayTask_PutUnsigned(line, &pos, s_pid_freq_hz, 3U);
    DisplayTask_PutChar(line, &pos, ' ');
    DisplayTask_PutChar(line, &pos, 'C');
    DisplayTask_PutUnsigned(line, &pos, s_control_freq_hz, 3U);
  }
  else
  {
    DisplayTask_PutString(line, &pos, "P--- C---");
  }

  DisplayTask_PutChar(line, &pos, ' ');
  DisplayTask_PutChar(line, &pos, 'E');
  error_count = data->imu_error_count + OLED_GetErrorCount() +
                data->queue_drop_count + s_uart6_error_count;
  DisplayTask_PutUnsigned(line, &pos, error_count, 1U);

  imu_ok = (((data->flags & LCD_FLAG_IMU_READY) != 0U) &&
            ((data->flags & LCD_FLAG_IMU_STALE) == 0U)) ? 1U : 0U;
  if (imu_ok == 0U)
  {
    DisplayTask_PutChar(line, &pos, '!');
  }
  OLED_ShowString(0U, 0U, line);

  /* 第 1 行：当前 yaw / 目标 yaw（0.1°） */
  line[0] = '\0';
  pos = 0U;
  DisplayTask_PutString(line, &pos, "YAW");
  DisplayTask_PutFixed1(line, &pos, data->current_yaw_0p1deg, 6U);
  DisplayTask_PutChar(line, &pos, ' ');
  DisplayTask_PutString(line, &pos, "YTG");
  DisplayTask_PutFixed1(line, &pos, data->target_yaw_0p1deg, 6U);
  OLED_ShowString(0U, 1U, line);

  /* 第 2 行：当前 pitch / 目标 pitch（0.1°） */
  line[0] = '\0';
  pos = 0U;
  DisplayTask_PutString(line, &pos, "PIT");
  DisplayTask_PutFixed1(line, &pos, data->current_pitch_0p1deg, 6U);
  DisplayTask_PutChar(line, &pos, ' ');
  DisplayTask_PutString(line, &pos, "PT");
  DisplayTask_PutFixed1(line, &pos, data->target_pitch_0p1deg, 6U);
  OLED_ShowString(0U, 2U, line);

  /* 第 3 行：yaw rate / pitch rate（0.1°/s） */
  line[0] = '\0';
  pos = 0U;
  DisplayTask_PutString(line, &pos, "YR");
  DisplayTask_PutFixed1(line, &pos, data->current_yaw_rate_0p1dps, 6U);
  DisplayTask_PutChar(line, &pos, ' ');
  DisplayTask_PutString(line, &pos, "PR");
  DisplayTask_PutFixed1(line, &pos, data->current_pitch_rate_0p1dps, 6U);
  OLED_ShowString(0U, 3U, line);

  /* 第 4 行：当前距离 / 目标距离（mm） */
  line[0] = '\0';
  pos = 0U;
  DisplayTask_PutString(line, &pos, "DST");
  DisplayTask_PutSigned(line, &pos, data->current_distance, 7U);
  DisplayTask_PutChar(line, &pos, '/');
  DisplayTask_PutSigned(line, &pos, data->target_distance, 7U);
  OLED_ShowString(0U, 4U, line);

  /* 第 5 行：当前速度 / 目标速度（mm/s） */
  line[0] = '\0';
  pos = 0U;
  DisplayTask_PutString(line, &pos, "SPD");
  DisplayTask_PutSigned(line, &pos, data->current_speed, 7U);
  DisplayTask_PutChar(line, &pos, '/');
  DisplayTask_PutSigned(line, &pos, data->target_speed, 7U);
  OLED_ShowString(0U, 5U, line);

  /* 第 6 行：左右 PWM */
  line[0] = '\0';
  pos = 0U;
  DisplayTask_PutString(line, &pos, "OUT L");
  DisplayTask_PutSigned(line, &pos, data->output_left, 5U);
  DisplayTask_PutString(line, &pos, " R");
  DisplayTask_PutSigned(line, &pos, data->output_right, 5U);
  OLED_ShowString(0U, 6U, line);

  /* 第 7 行：左右编码器累计（4x 计数） */
  line[0] = '\0';
  pos = 0U;
  DisplayTask_PutChar(line, &pos, 'L');
  DisplayTask_PutSigned(line, &pos, data->encoder_left, 7U);
  DisplayTask_PutString(line, &pos, " R");
  DisplayTask_PutSigned(line, &pos, data->encoder_right, 7U);
  OLED_ShowString(0U, 7U, line);
}

/*
 * 功能：还没有收到 PID 快照时的占位画面
 * 输入：无
 * 输出：无
 */
static void DisplayTask_RenderWaiting(void)
{
  OLED_Clear();
  OLED_ShowString(0U, 0U, "WAIT PID DATA");
  OLED_ShowString(0U, 2U, "OLED DMA READY");
  OLED_ShowString(0U, 4U, "I2C3 400K DMA");
}

/*
 * 功能：CRC16-CCITT（初值 0xFFFF，多项式 0x1021）
 * 输入：data/length —— 待校验数据
 * 输出：CRC
 */
static uint16_t DisplayTask_Crc16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = DISPLAY_BT_CRC_INIT;
  uint8_t bit;

  while (length > 0U)
  {
    crc ^= (uint16_t)(*data) << 8;
    data++;
    length--;

    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)((crc << 1) ^ DISPLAY_BT_CRC_POLY);
      }
      else
      {
        crc = (uint16_t)(crc << 1);
      }
    }
  }

  return crc;
}

/*
 * 功能：把 LCD_Data_t 打包成 86 B USART6 遥测帧
 * 输入：data —— PID 快照
 * 输出：无（写入 s_uart6_frame）
 */
static void DisplayTask_BuildUart6Frame(const LCD_Data_t *data)
{
  uint16_t crc;

  s_uart6_frame[0] = DISPLAY_BT_SYNC0;
  s_uart6_frame[1] = DISPLAY_BT_SYNC1;
  s_uart6_frame[2] = (uint8_t)sizeof(LCD_Data_t);
  s_uart6_frame[3] = DISPLAY_BT_VERSION;

  (void)memcpy(&s_uart6_frame[DISPLAY_BT_PAYLOAD_OFFSET],
               data,
               sizeof(LCD_Data_t));

  crc = DisplayTask_Crc16(s_uart6_frame,
                          (uint16_t)DISPLAY_BT_CRC_OFFSET);
  s_uart6_frame[DISPLAY_BT_CRC_OFFSET] = (uint8_t)(crc & 0xFFU);
  s_uart6_frame[DISPLAY_BT_CRC_OFFSET + 1U] = (uint8_t)(crc >> 8);
}

/*
 * 功能：USART6 DMA 空闲时启动一帧遥测；忙则跳过本帧
 * 输入：data —— PID 快照
 * 输出：无
 */
static void DisplayTask_StartUart6Dma(const LCD_Data_t *data)
{
  HAL_StatusTypeDef status;

  if (s_uart6_tx_ready == 0U)
  {
    return;
  }

  /* 先置忙，再启动 DMA；完成/错误回调恢复为就绪 */
  s_uart6_tx_ready = 0U;
  s_uart6_start_tick = HAL_GetTick();
  DisplayTask_BuildUart6Frame(data);

  status = HAL_UART_Transmit_DMA(&huart6,
                                 s_uart6_frame,
                                 (uint16_t)sizeof(s_uart6_frame));
  if (status != HAL_OK)
  {
    s_uart6_error_count++;

    /* 真正 BUSY 时等待完成回调；其他错误恢复许可，下一帧重试 */
    if (status != HAL_BUSY)
    {
      s_uart6_tx_ready = 1U;
    }
  }
}

/*
 * 功能：USART6 TX DMA 超时保护
 * 输入：无
 * 输出：无
 */
static void DisplayTask_UpdateUart6Watchdog(void)
{
  if ((s_uart6_tx_ready == 0U) &&
      ((HAL_GetTick() - s_uart6_start_tick) > DISPLAY_UART6_TX_TIMEOUT_MS))
  {
    (void)HAL_UART_AbortTransmit(&huart6);
    s_uart6_error_count++;
    s_uart6_tx_ready = 1U;
  }
}

/*
 * 功能：OLED DMA 空闲时渲染并启动整屏刷新；忙则跳过本帧
 * 输入：data —— PID 快照
 * 输出：无
 */
static void DisplayTask_StartOled(const LCD_Data_t *data)
{
  if ((s_oled_available == 0U) || (OLED_IsReady() == 0U))
  {
    return;
  }

  DisplayTask_Render(data);
  (void)OLED_Refresh_DMA();
}

/*
 * 功能：USART6 TX DMA 完成回调
 * 输入：huart —— UART 句柄
 * 输出：无
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART6))
  {
    s_uart6_tx_ready = 1U;
  }
}

/*
 * 功能：USART6 TX 错误钩子，由 HAL_UART_ErrorCallback() 调用
 * 输入：无
 * 输出：无
 */
void DisplayTask_OnUart6Error(void)
{
  s_uart6_error_count++;
  s_uart6_tx_ready = 1U;
}

/*
 * 功能：事件驱动 debugTask 入口
 * 输入：argument —— 未使用
 * 输出：无
 */
void StartdisplayTask(void *argument)
{
  (void)argument;

  (void)memset(&s_lcd_data, 0, sizeof(s_lcd_data));

  /* 4 针模块上电稳定后尝试初始化；OLED 缺失不能阻塞 USART6 遥测 */
  (void)osDelay(200U);
  if (OLED_Init() != 0U)
  {
    s_oled_available = 1U;

    /* 先显示等待画面；DMA 忙时首帧 OLED 会被跳过，USART6 不受影响 */
    DisplayTask_RenderWaiting();
    (void)OLED_Refresh_DMA();
  }

  for (;;)
  {
    if (dataToLCDQueueHandle == NULL)
    {
      (void)osDelay(100U);
      continue;
    }

    if (osMessageQueueGetMsgSize(dataToLCDQueueHandle) !=
        (uint32_t)sizeof(LCD_Data_t))
    {
      (void)osDelay(100U);
      continue;
    }

    /* 队列驱动：没有新快照就阻塞，不空转 */
    if (osMessageQueueGet(dataToLCDQueueHandle,
                          &s_lcd_data,
                          NULL,
                          osWaitForever) != osOK)
    {
      continue;
    }

    DisplayTask_UpdateFrequencies();
    DisplayTask_UpdateUart6Watchdog();
    DisplayTask_StartUart6Dma(&s_lcd_data);
    DisplayTask_StartOled(&s_lcd_data);
  }
}

/************************ (C) COPYRIGHT *****END OF FILE***********************/




