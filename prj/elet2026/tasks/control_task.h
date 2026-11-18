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
#include "robot_config.h"

/*
 * 主控任务周期在 robot_config.h 的 CONTROL_TASK_PERIOD_MS 中配置。
 */

/*
 * 功能：主控任务入口（赛题流程、路径状态机、Controlqueue 命令）
 * 输入：argument —— 任务参数（未使用）
 * 输出：无
 */
void StartcontrolTask(void *argument);

/* 调试计数：controlTask 实际循环次数（displayTask 读取） */
uint32_t ControlTask_GetRunCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_TASK_H */

/************************ (C) COPYRIGHT *****END OF FILE***********************/
