/**
  ******************************************************************************
  * @file    dwt_time.c
  * @brief   Cortex-M4 DWT CYCCNT 高精度时间戳实现。
  ******************************************************************************
  */

/* 头文件包含 ----------------------------------------------------------------*/
#include <stddef.h>
#include "main.h"
#include "dwt_time.h"

/* 私有变量 ------------------------------------------------------------------*/
static uint8_t s_dwt_initialized = 0U;
static uint8_t s_dwt_ready = 0U;

/*
 * 功能：初始化 DWT 周期计数器（幂等，只初始化一次）
 * 输入：无
 * 输出：无
 */
void DWTTime_Init(void)
{
  if (s_dwt_initialized != 0U)
  {
    return;
  }

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  s_dwt_ready = ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) == 0U) ? 1U : 0U;
  s_dwt_initialized = 1U;
}

/*
 * 功能：查询 DWT 周期计数器是否可用
 * 输入：无
 * 输出：1 = 可用；0 = 不可用
 */
uint8_t DWTTime_IsReady(void)
{
  return s_dwt_ready;
}

/*
 * 功能：读取当前 DWT 周期计数
 * 输入：无
 * 输出：当前 CYCCNT；DWT 不可用时返回 0
 */
uint32_t DWTTime_GetCycles(void)
{
  if (s_dwt_ready == 0U)
  {
    return 0U;
  }

  return DWT->CYCCNT;
}

/*
 * 功能：用两个 DWT 时间戳计算实际 dt
 * 输入：now —— 当前 CYCCNT；
 *       last_cycles —— 上一次时间戳，函数内更新；
 *       nominal_dt —— DWT 不可用或时间戳间隔为 0 时的标称 dt；
 *       max_dt —— dt 上限
 * 输出：实际 dt（s）
 */
float DWTTime_CalculateDt(uint32_t now, uint32_t *last_cycles,
                          float nominal_dt, float max_dt)
{
  uint32_t cycles;
  float dt;

  if ((s_dwt_ready == 0U) || (last_cycles == NULL) ||
      (SystemCoreClock == 0U))
  {
    return nominal_dt;
  }

  cycles = now - *last_cycles;
  *last_cycles = now;

  if (cycles == 0U)
  {
    return nominal_dt;
  }

  dt = (float)cycles / (float)SystemCoreClock;

  if (dt > max_dt)
  {
    dt = max_dt;
  }

  return dt;
}

/************************ (C) COPYRIGHT *****END OF FILE***********************/
