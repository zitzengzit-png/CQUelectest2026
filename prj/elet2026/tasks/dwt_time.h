/**
  ******************************************************************************
  * @file    dwt_time.h
  * @brief   基于 Cortex-M4 DWT CYCCNT 的高精度时间戳。
  ******************************************************************************
  */

#ifndef __DWT_TIME_H
#define __DWT_TIME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * 功能：初始化 DWT 周期计数器（多次调用只初始化一次）
 * 输入：无
 * 输出：无
 */
void DWTTime_Init(void);

/*
 * 功能：查询 DWT 周期计数器是否可用
 * 输入：无
 * 输出：1 = 可用；0 = 不可用
 */
uint8_t DWTTime_IsReady(void);

/*
 * 功能：读取当前 DWT 周期计数
 * 输入：无
 * 输出：当前 CYCCNT；DWT 不可用时返回 0
 */
uint32_t DWTTime_GetCycles(void);

/*
 * 功能：用两个 DWT 时间戳计算实际 dt
 * 输入：now —— 当前 CYCCNT；
 *       last_cycles —— 上一次时间戳，函数内更新；
 *       nominal_dt —— DWT 不可用或时间戳间隔为 0 时的标称 dt；
 *       max_dt —— dt 上限
 * 输出：实际 dt（s）
 */
float DWTTime_CalculateDt(uint32_t now, uint32_t *last_cycles,
                          float nominal_dt, float max_dt);

#ifdef __cplusplus
}
#endif

#endif /* __DWT_TIME_H */

/************************ (C) COPYRIGHT *****END OF FILE***********************/
