/**
  ******************************************************************************
  * @file    tune_link.h
  * @brief   无线调参帧、参数 ID 与两级覆盖队列接口。
  ******************************************************************************
  * @note    USART6 RX 固定 64 B 帧：
  *            [0] 0xA5
  *            [1] 0x5A
  *            [2] version = 1
  *            [3] type = 1
  *            [4..5] seq（小端）
  *            [6] control_mode（0=STOP,1=POSITION,2=SPEED,3=ATTITUDE,0xFF=不改目标）
  *            [7] param_count（0..8）
  *            [8..9] target1（int16 小端）
  *            [10..11] target2（int16 小端）
  *            [12..51] 8 × { id:u8, value:float32 小端 } = 40 B
  *            [52..61] reserved
  *            [62..63] CRC16-CCITT 小端，覆盖 0..61
  ******************************************************************************
  */

#ifndef __TUNE_LINK_H
#define __TUNE_LINK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define TUNE_FRAME_SIZE             64U
#define TUNE_MAX_PARAMS             8U
#define TUNE_FRAME_VERSION          1U
#define TUNE_FRAME_TYPE_SET         1U

/* 参数 ID：与电脑端必须一致 */
#define TUNE_PARAM_KP_DISTANCE      1U
#define TUNE_PARAM_KD_DISTANCE      2U
#define TUNE_PARAM_MAX_SPEED        3U
#define TUNE_PARAM_KP_SPEED         4U
#define TUNE_PARAM_KD_SPEED         5U
#define TUNE_PARAM_MAX_PITCH        6U
#define TUNE_PARAM_KP_ANGLE         7U
#define TUNE_PARAM_KD_ANGLE         8U
#define TUNE_PARAM_MAX_RATE         9U
#define TUNE_PARAM_KP_PITCH         10U
#define TUNE_PARAM_KD_PITCH         11U
#define TUNE_PARAM_KP_YAW_RATE      12U
#define TUNE_PARAM_KI_YAW_RATE      13U
#define TUNE_PARAM_MAX_PWM          14U
#define TUNE_PARAM_MAX_DIFF_PWM     15U
#define TUNE_PARAM_MAX_YAW_INTEGRAL 16U

/* 调参结果状态 */
#define TUNE_STATUS_OK              0U
#define TUNE_STATUS_BAD_SYNC        1U
#define TUNE_STATUS_BAD_VERSION     2U
#define TUNE_STATUS_BAD_TYPE        3U
#define TUNE_STATUS_BAD_COUNT       4U
#define TUNE_STATUS_BAD_CRC         5U
#define TUNE_STATUS_BAD_MODE        6U
#define TUNE_STATUS_BAD_TARGET      7U
#define TUNE_STATUS_BAD_PARAM_ID    8U
#define TUNE_STATUS_BAD_PARAM_RANGE 9U
#define TUNE_STATUS_BAD_STATE       10U
#define TUNE_STATUS_QUEUE_BUSY      11U

typedef struct
{
  uint8_t bytes[TUNE_FRAME_SIZE];
} TuneFrame_t;

typedef struct
{
  uint8_t id;
  float value;
} TuneParamValue_t;

/* controlTask 解析后的结果 */
typedef struct
{
  uint16_t seq;
  uint8_t control_mode;
  int16_t target1;
  int16_t target2;
  uint8_t count;
  uint8_t status;
  TuneParamValue_t params[TUNE_MAX_PARAMS];
} TuneParsed_t;

/* controlTask -> PID 的已校验参数消息 */
typedef struct
{
  uint16_t seq;
  uint8_t status;
  uint8_t count;
  TuneParamValue_t params[TUNE_MAX_PARAMS];
} TunePidMsg_t;

/* 与 CubeMX paramQueue itemsize 对齐 */
typedef char TunePidMsgSizeCheck[(sizeof(TunePidMsg_t) == 68U) ? 1 : -1];
typedef char TuneFrameSizeCheck[(sizeof(TuneFrame_t) == 64U) ? 1 : -1];

/*
 * 功能：创建覆盖队列并启动 USART6 RX DMA
 * 输出：1 = 成功；0 = 失败
 */
uint8_t TuneLink_Init(void);

/* USART6 RX DMA 完成/错误回调，由 imu_task.c 的 HAL 回调调用 */
void TuneLink_OnUsart6RxComplete(void);
void TuneLink_OnUsart6Error(void);

/* controlTask 侧：取最新原始帧（非阻塞） */
uint8_t TuneLink_TryReadFrame(TuneFrame_t *frame);

/* controlTask 侧：覆盖写入 PID 参数消息 */
uint8_t TuneLink_SendPidMessage(const TunePidMsg_t *message);

/* PID 侧：取最新参数消息（非阻塞） */
uint8_t TuneLink_ReceivePidMessage(TunePidMsg_t *message);

/* 清空两级调参队列 */
void TuneLink_ClearQueues(void);

/* 64 B 帧解析/校验，输出到 parsed */
uint8_t TuneLink_ParseFrame(const TuneFrame_t *frame, TuneParsed_t *parsed);

/* 参数 ID + 值的范围校验 */
uint8_t TuneLink_ParamIsValid(uint8_t id, float value);

#ifdef __cplusplus
}
#endif

#endif /* __TUNE_LINK_H */
