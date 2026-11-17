/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   主控任务骨架：状态机与 Controlqueue 命令预留。
  ******************************************************************************
  * @note    当前只建立任务结构和状态机，不实现具体赛题路径。
  *          任务周期 20 ms（50 Hz），与 PID 的 Controlqueue 轮询频率一致。
  *          后续在这里实现：
  *            - 一键启动/按键；
  *            - A/B/C/D 路径与到点声光提示；
  *            - 通过 Controlqueue 发送位置/速度命令；
  *            - 4 圈计时与停车。
  ******************************************************************************
  */

/* 头文件包含 ----------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "main.h"
#include "cmsis_os.h"
#include "control_task.h"

/* 私有类型定义 --------------------------------------------------------------*/
typedef enum
{
  CONTROL_STATE_IDLE = 0,  /* 空闲：等待启动条件 */
  CONTROL_STATE_RUN,       /* 运行：执行当前赛题路径 */
  CONTROL_STATE_STOP       /* 停止：输出停车命令并等待复位 */
} Control_State_t;

/* 私有变量 ------------------------------------------------------------------*/
static Control_State_t s_control_state = CONTROL_STATE_IDLE;
static uint32_t s_control_wake_tick = 0U;

/* 私有函数声明 --------------------------------------------------------------*/
static void ControlTask_Init(void);
static void ControlTask_Update(void);
static void ControlTask_StateIdle(void);
static void ControlTask_StateRun(void);
static void ControlTask_StateStop(void);
static void ControlTask_DelayUntil(void);

/* ========================================================================== */
/*                              以下为函数实现                                */
/* ========================================================================== */

/*
 * 功能：主控任务入口，按固定周期运行状态机
 * 输入：argument —— 任务参数（未使用）
 * 输出：无
 */
void StartcontrolTask(void *argument)
{
  (void)argument;

  ControlTask_Init();

  for (;;)
  {
    ControlTask_Update();

    switch (s_control_state)
    {
      case CONTROL_STATE_RUN:
        ControlTask_StateRun();
        break;

      case CONTROL_STATE_STOP:
        ControlTask_StateStop();
        break;

      case CONTROL_STATE_IDLE:
      default:
        ControlTask_StateIdle();
        break;
    }

    ControlTask_DelayUntil();
  }
}

/*
 * 功能：主控任务一次性初始化
 * 输入：无
 * 输出：无
 */
static void ControlTask_Init(void)
{
  s_control_state = CONTROL_STATE_IDLE;
  s_control_wake_tick = osKernelGetTickCount();
}

/*
 * 功能：主控任务的周期更新入口（当前为空，后续放按键/路径状态更新）
 * 输入：无
 * 输出：无
 */
static void ControlTask_Update(void)
{
  /* TODO: 读取按键/启动条件，更新 s_control_state 和目标。 */
}

/*
 * 功能：空闲状态处理（当前为空）
 * 输入：无
 * 输出：无
 */
static void ControlTask_StateIdle(void)
{
  /* TODO: 等待一键启动；满足条件后切换到 CONTROL_STATE_RUN。 */
}

/*
 * 功能：运行状态处理（当前为空）
 * 输入：无
 * 输出：无
 */
static void ControlTask_StateRun(void)
{
  /* TODO: 按赛题路径发送 Controlqueue 命令，处理到点提示和计时。 */
}

/*
 * 功能：停止状态处理（当前为空）
 * 输入：无
 * 输出：无
 */
static void ControlTask_StateStop(void)
{
  /* TODO: 发送停止命令，等待复位/重新启动。 */
}

/*
 * 功能：延时到下一个主控任务节拍（绝对 tick，避免累计误差）
 * 输入：无
 * 输出：无
 */
static void ControlTask_DelayUntil(void)
{
  s_control_wake_tick += pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS);
  (void)osDelayUntil(s_control_wake_tick);
}

/************************ (C) COPYRIGHT *****END OF FILE***********************/
