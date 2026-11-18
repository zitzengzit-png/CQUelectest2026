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
#include "line_sensor.h"

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
static volatile uint32_t s_control_run_count = 0U;

/* 前/后 8 路传感器原始电平：0/1，下标 0..7 对应 x1..x8 */
static uint8_t s_front_sensor_levels[LINE_SENSOR_COUNT];
static uint8_t s_back_sensor_levels[LINE_SENSOR_COUNT];

/* 前/后巡线位掩码：1 表示检测到线，bit0 = x1，bit7 = x8 */
static uint8_t s_front_line_mask = 0U;
static uint8_t s_back_line_mask = 0U;

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
    s_control_run_count++;
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
  uint32_t i;

  s_control_state = CONTROL_STATE_IDLE;
  s_control_wake_tick = osKernelGetTickCount();

  for (i = 0U; i < LINE_SENSOR_COUNT; i++)
  {
    s_front_sensor_levels[i] = 0U;
    s_back_sensor_levels[i] = 0U;
  }

  s_front_line_mask = 0U;
  s_back_line_mask = 0U;

  /* 上电先建立一次有效采样，避免第一拍数组为全 0 */
  LineSensor_ReadLevels(s_front_sensor_levels, s_back_sensor_levels);
  s_front_line_mask = LineSensor_LevelsToMask(s_front_sensor_levels);
  s_back_line_mask = LineSensor_LevelsToMask(s_back_sensor_levels);
}

/*
 * 功能：主控任务的周期更新入口
 * 输入：无
 * 输出：无
 *
 * 每个 20 ms 周期刷新一次前/后 8 路传感器电平，供巡线状态机使用。
 */
static void ControlTask_Update(void)
{
  LineSensor_ReadLevels(s_front_sensor_levels, s_back_sensor_levels);

  s_front_line_mask = LineSensor_LevelsToMask(s_front_sensor_levels);
  s_back_line_mask = LineSensor_LevelsToMask(s_back_sensor_levels);

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

/*
 * 功能：读取 controlTask 实际循环次数
 * 输入：无
 * 输出：运行计数（32 位，displayTask 用差值计算频率）
 */
uint32_t ControlTask_GetRunCount(void)
{
  return s_control_run_count;
}

/*
 * 功能：获取最近一次 controlTask 采集的前 8 路传感器原始电平
 * 输入：无
 * 输出：指向 8 字节数组的只读指针，下标 0..7 对应 x1..x8
 */
const uint8_t *ControlTask_GetFrontSensorLevels(void)
{
  return s_front_sensor_levels;
}

/*
 * 功能：获取最近一次 controlTask 采集的后 8 路传感器原始电平
 * 输入：无
 * 输出：指向 8 字节数组的只读指针，下标 0..7 对应 x1..x8
 */
const uint8_t *ControlTask_GetBackSensorLevels(void)
{
  return s_back_sensor_levels;
}

/*
 * 功能：获取前 8 路巡线位掩码
 * 输入：无
 * 输出：bit0 = x1 ... bit7 = x8；1 表示检测到线
 */
uint8_t ControlTask_GetFrontLineMask(void)
{
  return s_front_line_mask;
}

/*
 * 功能：获取后 8 路巡线位掩码
 * 输入：无
 * 输出：bit0 = x1 ... bit7 = x8；1 表示检测到线
 */
uint8_t ControlTask_GetBackLineMask(void)
{
  return s_back_line_mask;
}

/************************ (C) COPYRIGHT *****END OF FILE***********************/
