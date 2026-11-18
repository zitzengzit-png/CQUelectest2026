/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   TIM3/TIM4 正交编码器读取：共模位置/速度送 PID 任务。
  ******************************************************************************
  */

/* 头文件包含 ----------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "dwt_time.h"
#include "cmsis_os.h"
#include "encoder.h"

/* 私有变量 ------------------------------------------------------------------*/
static uint16_t s_last_left = ENCODER_INIT_COUNT;
static uint16_t s_last_right = ENCODER_INIT_COUNT;
static int32_t s_left_total = 0;
static int32_t s_right_total = 0;
static uint32_t s_last_cycles = 0U;
static uint32_t s_last_update_tick = 0U;
static uint8_t s_encoder_initialized = 0U;
static uint8_t s_encoder_first_update = 1U;

/* ENCODER_UPDATE_GAP_MS 来自 robot_config.h */

/* ========================================================================== */
/*                              以下为函数实现                                */
/* ========================================================================== */

/*
 * 功能：设置 TIM3/TIM4 初始 CNT 并启动编码器接口
 * 输入：无
 * 输出：1 = 两路都启动成功；0 = 启动失败
 */
uint8_t Encoder_Init(void)
{
  s_encoder_initialized = 0U;
  s_left_total = 0;
  s_right_total = 0;
  s_last_left = ENCODER_INIT_COUNT;
  s_last_right = ENCODER_INIT_COUNT;
  s_last_cycles = 0U;
  s_last_update_tick = 0U;
  s_encoder_first_update = 1U;

  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
  {
    return 0U;
  }

  if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK)
  {
    return 0U;
  }

  /*
   * 16 位总计数 65536，初始值取 1/10（6554）。
   * 软件只用 int16_t 差值累计，所以该值只影响回绕余量，不影响方向判断。
   */
  __HAL_TIM_SET_COUNTER(&htim3, ENCODER_INIT_COUNT);
  __HAL_TIM_SET_COUNTER(&htim4, ENCODER_INIT_COUNT);
  s_last_left = ENCODER_INIT_COUNT;
  s_last_right = ENCODER_INIT_COUNT;

  s_encoder_initialized = 1U;

  return 1U;
}

/*
 * 功能：读取左右编码器增量，累计共模位置/速度并转换为米、米/秒
 * 输入：position_m —— 输出共模位置（m）；
 *       speed_mps —— 输出共模速度（m/s）
 * 输出：无
 *
 * 说明：
 *   - 右编码器 TIM3 = PA6/PA7；
 *   - 左编码器 TIM4 = PD12/PD13；
 *   - int16_t 差值自动处理 16 位 CNT 的 0/65535 回绕；
 *   - 共模位置/速度取左右平均；
 *   - 原始计数 × ENCODER_METERS_PER_COUNT 转为米。
 */
void Encoder_Update(float *position_m, float *speed_mps)
{
  uint16_t now_left;
  uint16_t now_right;
  int16_t delta_left;
  int16_t delta_right;
  int32_t signed_left;
  int32_t signed_right;
  uint32_t now_cycles;
  uint32_t now_tick;
  float dt;
  float common_position;
  float common_speed;

  if (s_encoder_initialized == 0U)
  {
    return;
  }

  now_left = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
  now_right = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

  delta_left = (int16_t)(now_left - s_last_left);
  delta_right = (int16_t)(now_right - s_last_right);

  s_last_left = now_left;
  s_last_right = now_right;

  signed_left = (int32_t)delta_left * ENCODER_LEFT_SIGN;
  signed_right = (int32_t)delta_right * ENCODER_RIGHT_SIGN;

  s_left_total += signed_left;
  s_right_total += signed_right;

  now_cycles = DWTTime_GetCycles();
  now_tick = osKernelGetTickCount();

  if ((s_encoder_first_update != 0U) ||
      ((now_tick - s_last_update_tick) > ENCODER_UPDATE_GAP_MS))
  {
    /* 第一次更新或长时间停顿：只建立时间戳，不把长间隔当 dt */
    s_encoder_first_update = 0U;
    s_last_cycles = now_cycles;
    dt = ENCODER_NOMINAL_DT_S;
  }
  else
  {
    dt = DWTTime_CalculateDt(now_cycles, &s_last_cycles,
                             ENCODER_NOMINAL_DT_S, ENCODER_DT_MAX_S);
    if (dt <= 0.0f)
    {
      dt = ENCODER_NOMINAL_DT_S;
    }
  }

  s_last_update_tick = now_tick;

  /* 共模：左右平均；差模不参与当前控制；计数转米 */
  common_position = ((float)s_left_total + (float)s_right_total) *
                    0.5f * ENCODER_METERS_PER_COUNT;
  common_speed = ((float)signed_left + (float)signed_right) *
                 0.5f * ENCODER_METERS_PER_COUNT / dt;

  if (position_m != NULL)
  {
    *position_m = common_position;
  }

  if (speed_mps != NULL)
  {
    *speed_mps = common_speed;
  }
}

/*
 * 功能：读取左编码器累计脉冲（调试用）
 * 输入：无
 * 输出：左累计脉冲
 */
int32_t Encoder_GetLeftTotal(void)
{
  return s_left_total;
}

/*
 * 功能：读取右编码器累计脉冲（调试用）
 * 输入：无
 * 输出：右累计脉冲
 */
int32_t Encoder_GetRightTotal(void)
{
  return s_right_total;
}

/************************ (C) COPYRIGHT *****END OF FILE***********************/
