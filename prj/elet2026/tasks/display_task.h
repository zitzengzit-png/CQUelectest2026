/**
  ******************************************************************************
  * @file    display_task.h
  * @brief   事件驱动 debugTask：50 Hz PID 快照 -> OLED / USART6 DMA 输出。
  ******************************************************************************
  * @note    CubeMX 生成的 dataToLCDQueue 元素大小是 80 字节，正好等于
  *          sizeof(LCD_Data_t)。PID 每次把完整快照按值放入队列，显示任务
  *          取到的是独立副本，不需要指针、双缓冲或序号校验。
  *
  *          USART6 遥测帧（86 B，小端）：
  *            AA 55 | len=80 | ver=2 | LCD_Data_t(80 B) | CRC16-CCITT lo hi
  *          CRC 覆盖 AA 到 payload 末尾，初值 0xFFFF，多项式 0x1021。
  ******************************************************************************
  */

#ifndef __DISPLAY_TASK_H
#define __DISPLAY_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "cmsis_os.h"
#include "robot_config.h"

/*
 * LCD_MODE_* / LCD_FLAG_* / LCD_QUEUE_ITEM_SIZE / LCD_IMU_STALE_MS
 * 均在 robot_config.h 中定义。displayTask 由 dataToLCDQueue 事件驱动，
 * 不再使用固定周期。
 */

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
  uint32_t sequence;          /* PID 发送序号，仅供观察，不再用于防撕裂 */
  uint32_t timestamp_ms;      /* 快照生成时刻 */
  uint32_t imu_age_ms;        /* 距最近一帧 IMU 姿态数据的毫秒数 */
  uint32_t imu_error_count;   /* IMU UART/DMA 错误计数 */
  uint32_t queue_drop_count;  /* PID 发送队列失败的次数 */

  int32_t target_distance;    /* 目标共模位移，mm */
  int32_t current_distance;   /* 相对本次位置起点的共模位移，mm */
  int32_t target_speed;       /* 当前模式下的目标速度，mm/s */
  int32_t current_speed;      /* 当前共模速度，mm/s */
  int32_t encoder_left;       /* 左编码器累计，4x 计数 */
  int32_t encoder_right;      /* 右编码器累计，4x 计数 */

  int16_t target_pitch_0p1deg;
  int16_t current_pitch_0p1deg;
  int16_t target_yaw_0p1deg;
  int16_t current_yaw_0p1deg;
  int16_t target_yaw_rate_0p1dps;
  int16_t current_yaw_rate_0p1dps;
  int16_t current_pitch_rate_0p1dps;
  int16_t output_left;        /* 左轮有符号 PWM，±PID_ATT_MAX_PWM */
  int16_t output_right;       /* 右轮有符号 PWM，±PID_ATT_MAX_PWM */

  uint16_t loop_time_us;      /* 最近一次 PID 250 Hz 循环耗时 */
  uint8_t mode;               /* LCD_MODE_* */
  uint8_t flags;              /* LCD_FLAG_* */
  uint8_t imu_ready;          /* IMU_IsReady() */
  uint8_t imu_valid;          /* IMU_Data_t.valid */

  uint32_t imu_timestamp_ms;  /* 最近一帧 IMU 姿态数据的 USART3 RX DMA 完成时刻 */

  uint16_t param_status;      /* 0=成功，非0=TUNE_STATUS_* */
  uint16_t param_version;     /* 成功应用参数后递增 */
  uint16_t param_seq;         /* 最近一次调参帧序号 */
  uint16_t param_id;          /* 最近一次参数 ID */
} LCD_Data_t;

/* 编译期确认完整快照与 CubeMX 的 itemsize 一致；不一致会在这里报错 */
typedef char LCD_QueueItemSizeCheck[
    (sizeof(LCD_Data_t) == LCD_QUEUE_ITEM_SIZE) ? 1 : -1];

/* 电脑端解析依赖这些偏移 */
typedef char LCD_ImuTimestampOffsetCheck[
    (offsetof(LCD_Data_t, imu_timestamp_ms) == 68U) ? 1 : -1];
typedef char LCD_ParamStatusOffsetCheck[
    (offsetof(LCD_Data_t, param_status) == 72U) ? 1 : -1];
typedef char LCD_ParamVersionOffsetCheck[
    (offsetof(LCD_Data_t, param_version) == 74U) ? 1 : -1];
typedef char LCD_ParamSeqOffsetCheck[
    (offsetof(LCD_Data_t, param_seq) == 76U) ? 1 : -1];
typedef char LCD_ParamIdOffsetCheck[
    (offsetof(LCD_Data_t, param_id) == 78U) ? 1 : -1];

/* CubeMX 生成的消息队列句柄（定义在 freertos.c） */
extern osMessageQueueId_t dataToLCDQueueHandle;

/*
 * 功能：事件驱动 debugTask 入口（CubeMX 生成的入口名不要改）
 * 输入：argument —— 未使用
 * 输出：无
 */
void StartdisplayTask(void *argument);

/*
 * 功能：USART6 TX DMA 错误钩子，由 HAL_UART_ErrorCallback() 调用
 * 输入：无
 * 输出：无
 */
void DisplayTask_OnUart6Error(void);

#ifdef __cplusplus
}
#endif

#endif /* __DISPLAY_TASK_H */

/************************ (C) COPYRIGHT *****END OF FILE***********************/
