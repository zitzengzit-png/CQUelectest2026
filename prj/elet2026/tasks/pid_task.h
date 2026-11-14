/**
  ******************************************************************************
  * @file    pid_task.h
  * @brief   PID 任务：类型、宏、参数结构体与函数声明。
  ******************************************************************************
  * @note    所有类型/宏/函数声明集中在本文件；
  *          pid_task.c 只保留变量定义与函数实现。
  ******************************************************************************
  */

#ifndef __PID_TASK_H
#define __PID_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "cmsis_os.h"

/* 任务周期 ------------------------------------------------------------------*/
/*
 * PID 基础任务 250 Hz（4 ms），三环不另建任务，而是在本任务内用计数器分频：
 *   内环 250 Hz：每个基础 tick 执行一次；
 *   中环  50 Hz：每 5 个基础 tick 执行一次；
 *   外环  10 Hz：每 25 个基础 tick 执行一次。
 */
#define PID_TASK_PERIOD_MS        4U
#define PID_INNER_LOOP_HZ         (1000U / PID_TASK_PERIOD_MS)
/* 标称 dt；正常运行时由 DWT 实测，DWT 不可用时才回退到这些值。 */
#define PID_INNER_DT_S            ((float)PID_TASK_PERIOD_MS / 1000.0f)

#define PID_MIDDLE_LOOP_HZ        50U
#define PID_MIDDLE_PERIOD_MS      (1000U / PID_MIDDLE_LOOP_HZ)
#define PID_MIDDLE_DT_S           ((float)PID_MIDDLE_PERIOD_MS / 1000.0f)
#define PID_MIDDLE_TICK_DIV       (PID_INNER_LOOP_HZ / PID_MIDDLE_LOOP_HZ)

#define PID_OUTER_LOOP_HZ         10U
#define PID_OUTER_PERIOD_MS       (1000U / PID_OUTER_LOOP_HZ)
#define PID_OUTER_DT_S            ((float)PID_OUTER_PERIOD_MS / 1000.0f)
#define PID_OUTER_TICK_DIV        (PID_INNER_LOOP_HZ / PID_OUTER_LOOP_HZ)

/* 实测 dt 限幅 --------------------------------------------------------------*/
/* 正常情况下使用 DWT 实测 dt；仅当任务长时间停顿、超过这些上限时截断。 */
#define PID_INNER_DT_MAX_S        0.020f
#define PID_MIDDLE_DT_MAX_S       0.100f
#define PID_OUTER_DT_MAX_S        0.500f

/*
 * DWT 32 位周期计数器回绕保护。
 * 168 MHz 下 2^32 个周期约 25.55 s；用 1 ms FreeRTOS tick 检测到
 * 任务循环间隔超过 20 s 时重置 DWT 时间戳，避免把回绕差值当成 dt。
 */
#define PID_DT_OVERFLOW_GUARD_MS  20000U

/* Controlqueue 轮询 ---------------------------------------------------------*/
/*
 * 每次任务循环先更新 IMU，再按 25 Hz 读取 Controlqueue，最后进入状态机。
 * 250 Hz 基础任务下，每 10 个 tick 读取一次（40 ms）。
 */
#define PID_CONTROL_QUEUE_LOOP_HZ    25U
#define PID_CONTROL_QUEUE_PERIOD_MS  (1000U / PID_CONTROL_QUEUE_LOOP_HZ)
#define PID_CONTROL_QUEUE_TICK_DIV   (PID_INNER_LOOP_HZ / PID_CONTROL_QUEUE_LOOP_HZ)

/* Controlqueue 模式编码（消息最高两位） -------------------------------------*/
#define CONTROL_MODE_STOP       0x0U
#define CONTROL_MODE_POSITION   0x1U
#define CONTROL_MODE_SPEED      0x2U

/* PID 参数（TODO: 实车整定） ------------------------------------------------*/
#define PID_POS_KP_DISTANCE        0.0f
#define PID_POS_KD_DISTANCE        0.0f
#define PID_POS_KP_DIFF_POSITION   0.0f
#define PID_POS_KD_DIFF_POSITION   0.0f
#define PID_SPEED_KP            0.0f
#define PID_SPEED_KD            0.0f
#define PID_ATT_KP_PITCH        0.0f
#define PID_ATT_KD_PITCH        0.0f
#define PID_ATT_KP_DIFF         0.0f
#define PID_ATT_KI_DIFF         0.0f

/* 输出限幅（TODO: 实车整定） ------------------------------------------------*/
#define PID_POS_MAX_SPEED_PULSES_S        50.0f
#define PID_POS_MAX_DIFF_SPEED_PULSES_S   200.0f
#define PID_SPEED_MAX_PITCH_DEG           10.0f
#define PID_SPEED_MAX_DIFF_SPEED_PULSES_S 200.0f
#define PID_ATT_MAX_PWM                   1000.0f
#define PID_ATT_MAX_DIFF_PWM              1000.0f
#define PID_ATT_MAX_DIFF_INTEGRAL         10000.0f

/* CubeMX 生成的队列句柄（freertos.c） --------------------------------------*/
extern osMessageQueueId_t ControlqueueHandle;

/* 位置控制器参数 ------------------------------------------------------------*/
typedef struct
{
  float kp_distance;         /* 共模位置误差 -> 目标速度，1/s */
  float kd_distance;         /* 共模位置误差微分 -> 目标速度，无量纲 */
  float kp_diff_position;    /* 差模位置误差 -> 目标差模速度，1/s */
  float kd_diff_position;    /* 差模位置误差微分 -> 目标差模速度，无量纲 */
  float max_speed;           /* 目标速度限幅，脉冲/s */
  float max_diff_speed;      /* 目标差模速度限幅，脉冲/s */
  int32_t encoder_start;        /* 位置模式入口共模编码器值 */
  int32_t encoder_current;      /* 当前共模编码器值 */
  int32_t encoder_diff_start;   /* 位置模式入口差模编码器值 */
  int32_t encoder_diff_current; /* 当前差模编码器值 */
} PositionControllerParams_t;

/* 速度/姿态控制器参数 --------------------------------------------------------*/
typedef struct
{
  float kp_speed;         /* 速度误差 -> 倾角，deg per 脉冲/s */
  float kd_speed;
  float max_pitch;        /* 目标倾角限幅，deg */
  float max_diff_speed;   /* 差模目标速度限幅，脉冲/s */
} SpeedControllerParams_t;

/* 姿态/差速内环控制器参数 ----------------------------------------------------*/
typedef struct
{
  float kp_pitch;         /* 倾角误差 -> 共模 PWM */
  float kd_pitch;         /* 俯仰角速度 -> 共模 PWM */
  float kp_diff;          /* 差速误差 -> 差模 PWM */
  float ki_diff;
  float max_pwm;          /* 共模/左右 PWM 限幅 */
  float max_diff_pwm;     /* 差模 PWM 限幅 */
  float max_diff_integral;/* 差速积分限幅 */
} AttitudeControllerParams_t;

/* PID 任务状态 --------------------------------------------------------------*/
typedef enum
{
  PID_STATE_STOP = 0,    /* 停止：输出关闭，等待 Controlqueue 命令 */
  PID_STATE_POSITION,    /* 位置控制模式：位置 -> 速度/姿态 -> 内环 */
  PID_STATE_SPEED        /* 速度控制模式：速度/姿态 -> 内环 */
} PID_State_t;

/* 任务输入（目标由 Controlqueue 写入，编码器反馈由 PIDTask_SetFeedback 写入） */
typedef struct
{
  float target_distance;       /* 目标共模位移，编码器脉冲 */
  float target_diff_position;  /* 目标差模旋转量，编码器脉冲 */
  float target_speed;          /* 目标速度，脉冲/s */
  float target_diff_speed;     /* 目标差模速度，脉冲/s（速度模式） */
  float current_position;      /* 当前共模位置，编码器脉冲 */
  float current_diff_position; /* 当前差模位置，编码器脉冲 */
  float current_speed;         /* 当前速度，脉冲/s */
  float current_pitch;      /* deg（IMU 读取后更新） */
  float current_pitch_rate; /* deg/s（IMU 读取后更新） */
  float current_diff_speed; /* 当前差模速度，脉冲/s */
} PIDTask_Input_t;

/* 硬件抽象 ----------------------------------------------------------------*/
/*
 * 功能：设置单侧电机 PWM 方向与比较值
 * 输入：left_wheel —— 1 = 左轮，0 = 右轮；
 *       forward —— 1 = 正转，0 = 反转；
 *       compare —— 写入比较寄存器的值
 * 输出：无
 */
void SETPWM(uint8_t left_wheel, uint8_t forward, uint16_t compare);

/* 函数声明 ------------------------------------------------------------------*/
/* 任务入口 */
void StartPIDTask(void *argument);

/*
 * 反馈接口（编码器数据由外部换算后写入）：
 * position_pulses       —— 共模位置累计值（脉冲）
 * diff_position_pulses  —— 差模位置累计值（脉冲）
 * speed_pulses_s        —— 共模速度（脉冲/s）
 * diff_speed_pulses_s   —— 差模速度（脉冲/s）
 *
 * 位置模式由 PositionController_Start() 锁存共模/差模起点；
 * 差模正方向必须与 Controlqueue 的 value2 正方向和 PWM 转向一致。
 */
void PIDTask_SetFeedback(float position_pulses, float diff_position_pulses,
                         float speed_pulses_s, float diff_speed_pulses_s);

/* 控制器函数 */
float PID_Clamp(float value, float limit);

void PositionController_Update(const PositionControllerParams_t *params,
                               float target_distance,
                               float target_diff_position,
                               float dt,
                               float *target_speed,
                               float *target_diff_speed);
void PositionController_Start(void);

void SpeedController_Update(const SpeedControllerParams_t *params,
                            float target_speed, float current_speed,
                            float target_diff_speed,
                            float dt,
                            float *target_pitch, float *diff_speed_command);
void SpeedController_Reset(void);

void AttitudeController_Update(const AttitudeControllerParams_t *params,
                               float target_pitch, float current_pitch,
                               float pitch_rate,
                               float target_diff_speed, float current_diff_speed,
                               float dt,
                               float *pwm_left, float *pwm_right);
void AttitudeController_Reset(void);

/* 任务内部函数 */
void PIDTask_Init(void);
void PIDTask_StateStop(void);
void PIDTask_StatePosition(void);
void PIDTask_StateSpeed(void);
void PIDTask_StartPositionMode(void);
void PIDTask_ProcessControlQueue(void);
void PIDTask_UpdateIMU(void);
void PIDTask_UpdateControlQueue(void);
void PIDTask_WriteOutput(float pwm_left, float pwm_right);

/* 实测 dt：DWT CYCCNT 时间戳 */
void PIDTask_InitHighResClock(void);
uint32_t PIDTask_GetTimestamp(void);
float PIDTask_CalculateDt(uint32_t now, uint32_t *last_cycles,
                          float nominal_dt, float max_dt);
void PIDTask_ResetLoopTimers(void);
void PIDTask_CheckTimestampOverflow(void);
void PIDTask_DelayUntil(void);

/* 多环调度：基础 250 Hz tick 由内置计数器分频 */
uint32_t PIDTask_NextLoopTick(void);
void PIDTask_ResetLoopCounter(void);
void PIDTask_PositionOuterLoop(void);
void PIDTask_PositionMiddleLoop(void);
void PIDTask_SpeedMiddleLoop(void);
void PIDTask_AttitudeInnerLoop(void);

#ifdef __cplusplus
}
#endif

#endif /* __PID_TASK_H */

/************************ (C) COPYRIGHT *****END OF FILE***********************/

