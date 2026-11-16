/**
  ******************************************************************************
  * @file    display_task.h
  * @brief   20 Hz OLED 调试显示任务与 PID->LCD 快照接口。
  ******************************************************************************
  * @note    CubeMX 生成的 dataToLCDQueue 元素大小是 8 字节，无法直接容纳
  *          完整 LCD_Data_t。因此队列项只放：
  *            - 指向 PID 静态双缓冲快照的指针（4 字节）
  *            - 快照序号（4 字节）
  *          合计 8 字节。PID 交替写两块快照，显示任务拷贝后校验序号，
  *          避免显示任务读到写了一半的数据。
  ******************************************************************************
  */

#ifndef __DISPLAY_TASK_H
#define __DISPLAY_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "cmsis_os.h"

/* 显示刷新周期：20 Hz，50 ms */
#define DISPLAY_TASK_PERIOD_MS        50U
#define DISPLAY_TASK_PERIOD_TICKS     pdMS_TO_TICKS(DISPLAY_TASK_PERIOD_MS)

/* CubeMX 中 dataToLCDQueue 的实际元素大小：1 个元素 × 8 字节 */
#define LCD_QUEUE_ITEM_SIZE           8U

/* PID_State_t 的数值映射，必须与 pid_task.h 的枚举顺序一致 */
#define LCD_MODE_STOP                 0U
#define LCD_MODE_POSITION             1U
#define LCD_MODE_SPEED                2U
#define LCD_MODE_ATTITUDE             3U

/* LCD_Data_t.flags */
#define LCD_FLAG_IMU_READY            (1U << 0)
#define LCD_FLAG_IMU_VALID            (1U << 1)
#define LCD_FLAG_IMU_STALE            (1U << 2)
#define LCD_FLAG_QUEUE_DROP           (1U << 3)

/* 超过该时间没有新的 IMU 姿态帧就认为 IMU 已停更 */
#define LCD_IMU_STALE_MS              100U

/*
 * PID 侧生成的显示快照。
 *
 * 角度/角速度使用 0.1 单位，避免在显示任务里做浮点 printf：
 *   target_pitch_0p1deg      目标 Pitch，0.1°
 *   current_pitch_0p1deg     当前 Pitch，0.1°
 *   target_yaw_0p1deg        目标 yaw，0.1°
 *   current_yaw_0p1deg       当前 yaw，0.1°
 *   target_yaw_rate_0p1dps   目标 yaw rate，0.1°/s
 *   current_yaw_rate_0p1dps  当前 yaw rate，0.1°/s
 *   current_pitch_rate_0p1dps 当前 Pitch rate，0.1°/s
 */
typedef struct
{
  uint32_t sequence;          /* 偶数 = 稳定；奇数 = PID 正在写 */
  uint32_t timestamp_ms;      /* 快照生成时刻 */
  uint32_t imu_age_ms;        /* 距最近一帧 IMU 姿态数据的毫秒数 */
  uint32_t imu_error_count;   /* IMU UART/DMA 错误计数 */
  uint32_t queue_drop_count;  /* PID 发送队列失败的次数 */

  int32_t target_distance;    /* 目标共模位移，4x 编码器脉冲 */
  int32_t current_distance;   /* 相对本次位置起点的共模位移，脉冲 */
  int32_t target_speed;       /* 当前模式下的目标速度，脉冲/s */
  int32_t current_speed;      /* 当前共模速度，脉冲/s */
  int32_t encoder_left;       /* 左编码器累计，4x 计数 */
  int32_t encoder_right;      /* 右编码器累计，4x 计数 */

  int16_t target_pitch_0p1deg;
  int16_t current_pitch_0p1deg;
  int16_t target_yaw_0p1deg;
  int16_t current_yaw_0p1deg;
  int16_t target_yaw_rate_0p1dps;
  int16_t current_yaw_rate_0p1dps;
  int16_t current_pitch_rate_0p1dps;
  int16_t output_left;        /* 左轮有符号 PWM，-1000 ~ +1000 */
  int16_t output_right;       /* 右轮有符号 PWM，-1000 ~ +1000 */

  uint16_t loop_time_us;      /* 最近一次 PID 250 Hz 循环耗时 */
  uint8_t mode;               /* LCD_MODE_* */
  uint8_t flags;              /* LCD_FLAG_* */
  uint8_t imu_ready;          /* IMU_IsReady() */
  uint8_t imu_valid;          /* IMU_Data_t.valid */
} LCD_Data_t;

/* 8 字节队列项：4 字节指针 + 4 字节序号 */
typedef struct
{
  const LCD_Data_t *snapshot;
  uint32_t sequence;
} LCD_QueueItem_t;

/* 编译期确认队列项与 CubeMX 的 itemsize 一致；不一致会在这里报错 */
typedef char LCD_QueueItemSizeCheck[
    (sizeof(LCD_QueueItem_t) == LCD_QUEUE_ITEM_SIZE) ? 1 : -1];

/* CubeMX 生成的消息队列句柄（定义在 freertos.c） */
extern osMessageQueueId_t dataToLCDQueueHandle;

/*
 * 功能：20 Hz OLED 显示任务入口（CubeMX 生成的入口名不要改）
 * 输入：argument —— 未使用
 * 输出：无
 */
void StartdisplayTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __DISPLAY_TASK_H */

/************************ (C) COPYRIGHT *****END OF FILE***********************/
