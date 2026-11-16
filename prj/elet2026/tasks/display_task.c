/**
  ******************************************************************************
  * @file    display_task.c
  * @brief   20 Hz OLED 调试显示任务：消费 PID 的 dataToLCDQueue 快照。
  ******************************************************************************
  * @note    显示任务优先级 8，低于 controlTask(13) 和 PID(28)，
  *          只在 PID/control 空闲时运行，不参与控制。
  ******************************************************************************
  */

/* 头文件包含 ----------------------------------------------------------------*/
#include <string.h>
#include "FreeRTOS.h"
#include "main.h"
#include "cmsis_os.h"
#include "display_task.h"
#include "oled.h"

/* 私有宏定义 ----------------------------------------------------------------*/
#define DISPLAY_LINE_CHARS        21U
#define DISPLAY_LINE_BUF_SIZE     (DISPLAY_LINE_CHARS + 1U)
#define DISPLAY_LINE_COUNT        8U
#define DISPLAY_REFRESH_TIMEOUT_MS 40U

/* 私有变量 ------------------------------------------------------------------*/
static LCD_Data_t s_lcd_data;
static uint8_t s_have_data = 0U;

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
static uint8_t DisplayTask_CopySnapshot(const LCD_QueueItem_t *item,
                                        LCD_Data_t *dest);
static void DisplayTask_Render(const LCD_Data_t *data);
static void DisplayTask_RenderWaiting(void);

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
 * 功能：校验并拷贝 PID 双缓冲快照
 * 输入：item —— 队列项；dest —— 目标快照
 * 输出：1 = 拷贝成功；0 = 正在写或序号不一致
 */
static uint8_t DisplayTask_CopySnapshot(const LCD_QueueItem_t *item,
                                        LCD_Data_t *dest)
{
  uint32_t seq_first;
  uint32_t seq_last;

  if ((item == NULL) || (item->snapshot == NULL) || (dest == NULL))
  {
    return 0U;
  }

  seq_first = item->snapshot->sequence;
  if (((seq_first & 1U) != 0U) || (seq_first != item->sequence))
  {
    return 0U;
  }

  __DMB();
  (void)memcpy(dest, item->snapshot, sizeof(*dest));
  __DMB();

  seq_last = item->snapshot->sequence;
  if ((seq_first != seq_last) || ((seq_last & 1U) != 0U))
  {
    return 0U;
  }

  return 1U;
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

  /* 第 0 行：模式 / 运行状态 / IMU / 错误数 / 循环耗时 */
  line[0] = '\0';
  pos = 0U;
  DisplayTask_PutString(line, &pos, DisplayTask_ModeString(data->mode));
  DisplayTask_PutChar(line, &pos, ' ');
  DisplayTask_PutString(line, &pos,
                        (data->mode == LCD_MODE_STOP) ? "STP" : "RUN");
  DisplayTask_PutChar(line, &pos, ' ');

  imu_ok = (((data->flags & LCD_FLAG_IMU_READY) != 0U) &&
            ((data->flags & LCD_FLAG_IMU_STALE) == 0U)) ? 1U : 0U;
  DisplayTask_PutString(line, &pos, (imu_ok != 0U) ? "OK" : "--");
  DisplayTask_PutChar(line, &pos, ' ');
  DisplayTask_PutChar(line, &pos, 'E');

  error_count = data->imu_error_count + OLED_GetErrorCount() +
                data->queue_drop_count;
  DisplayTask_PutUnsigned(line, &pos, error_count, 2U);
  DisplayTask_PutChar(line, &pos, ' ');
  DisplayTask_PutString(line, &pos, "LP");
  DisplayTask_PutUnsigned(line, &pos, data->loop_time_us, 4U);
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

  /* 第 4 行：当前距离 / 目标距离（4x 编码器脉冲） */
  line[0] = '\0';
  pos = 0U;
  DisplayTask_PutString(line, &pos, "DST");
  DisplayTask_PutSigned(line, &pos, data->current_distance, 7U);
  DisplayTask_PutChar(line, &pos, '/');
  DisplayTask_PutSigned(line, &pos, data->target_distance, 7U);
  OLED_ShowString(0U, 4U, line);

  /* 第 5 行：当前速度 / 目标速度（4x 编码器脉冲/s） */
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
  OLED_ShowString(0U, 2U, "OLED 20HZ");
  OLED_ShowString(0U, 4U, "I2C1 400K DMA");
}

/*
 * 功能：20 Hz 显示任务入口
 * 输入：argument —— 未使用
 * 输出：无
 */
void StartdisplayTask(void *argument)
{
  LCD_QueueItem_t item;
  uint32_t wake_tick;

  (void)argument;

  (void)memset(&s_lcd_data, 0, sizeof(s_lcd_data));
  s_have_data = 0U;

  /* 4 针模块上电稳定后再初始化；失败每秒重试 */
  (void)osDelay(200U);
  while (OLED_Init() == 0U)
  {
    (void)osDelay(1000U);
  }

  wake_tick = osKernelGetTickCount();

  for (;;)
  {
    if (dataToLCDQueueHandle != NULL)
    {
      if (osMessageQueueGetMsgSize(dataToLCDQueueHandle) ==
          (uint32_t)sizeof(LCD_QueueItem_t))
      {
        if (osMessageQueueGet(dataToLCDQueueHandle, &item, NULL, 0U) == osOK)
        {
          if (DisplayTask_CopySnapshot(&item, &s_lcd_data) != 0U)
          {
            s_have_data = 1U;
          }
        }
      }
    }

    if (s_have_data != 0U)
    {
      DisplayTask_Render(&s_lcd_data);
    }
    else
    {
      DisplayTask_RenderWaiting();
    }

    if (OLED_Refresh_DMA() != 0U)
    {
      (void)OLED_WaitRefresh(DISPLAY_REFRESH_TIMEOUT_MS);
    }

    wake_tick += DISPLAY_TASK_PERIOD_TICKS;
    (void)osDelayUntil(wake_tick);
  }
}

/************************ (C) COPYRIGHT *****END OF FILE***********************/



