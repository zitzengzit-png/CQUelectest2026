/**
  ******************************************************************************
  * @file    control_task.h
  * @brief   主控任务：赛题流程与 Controlqueue 命令生成（当前为骨架）。
  ******************************************************************************
  * @note    CubeMX 生成的入口名为 StartcontrolTask（小写 c），不要改名。
  ******************************************************************************
  */

#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "cmsis_os.h"

/* 主控任务周期（ms）：50 Hz，20 ms */
#define CONTROL_TASK_PERIOD_MS    20U

/*
 * 功能：主控任务入口（赛题流程、路径状态机、Controlqueue 命令）
 * 输入：argument —— 任务参数（未使用）
 * 输出：无
 */
void StartcontrolTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_TASK_H */

/************************ (C) COPYRIGHT *****END OF FILE***********************/
