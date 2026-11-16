/**
  ******************************************************************************
  * @file    pid_task.h
  * @brief   PID 任务：类型、宏、参数结构体与函数声明。
  ******************************************************************************
  * @note    当前转向架构：
  *          - 编码器只提供共模位置/速度；
  *          - yaw 角度使用 ATK-MS901M 九轴相对角（进入模式时归零）；
  *          - 中环 yaw 角度 PD -> 目标 yaw rate -> 限幅；
  *          - 内环 yaw rate PI -> 差模 PWM；
  *          - 不使用编码器差模，不使用 yaw 2 状态 Kalman。
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
#define PID_CONTROL_QUEUE_LOOP_HZ    25U
#define PID_CONTROL_QUEUE_PERIOD_MS  (1000U / PID_CONTROL_QUEUE_LOOP_HZ)
#define PID_CONTROL_QUEUE_TICK_DIV   (PID_INNER_LOOP_HZ / PID_CONTROL_QUEUE_LOOP_HZ)

/* OLED 调试快照发送频率：20 Hz（队列元素 68 字节，见 display_task.h） */
#define PID_DISPLAY_UPDATE_HZ        20U

/* Controlqueue 模式编码（消息最高两位） -------------------------------------*/
#define CONTROL_MODE_STOP       0x0U
#define CONTROL_MODE_POSITION   0x1U
#define CONTROL_MODE_SPEED      0x2U
#define CONTROL_MODE_ATTITUDE   0x3U  /* 姿态控制：只跑内环，方便调参 */

/*
 * Controlqueue 的 value2 / value1 单位：
 *   位置模式 value2：目标 yaw 角，单位 0.1°，相对首次进入位置模式零点；
 *   速度模式 value2：目标 yaw rate，单位 1°/s，不使用 yaw 零点；
 *   姿态模式 value1：目标 Pitch 角，单位 0.1°；
 *   姿态模式 value2：目标 yaw rate，单位 1°/s。
 */
#define PID_CONTROL_YAW_UNIT_DEG       0.1f
#define PID_CONTROL_YAW_RATE_UNIT_DPS  1.0f
#define PID_CONTROL_PITCH_UNIT_DEG     0.1f

/* PID 参数（TODO: 实车整定） ------------------------------------------------*/
#define PID_POS_KP_DISTANCE        0.0f
#define PID_POS_KD_DISTANCE        0.0f
#define PID_SPEED_KP               0.0f
#define PID_SPEED_KD               0.0f

/* yaw 角度环：输入角误差（deg），输出目标 yaw rate（deg/s） */
#define PID_YAW_ANGLE_KP           0.0f   /* 1/s */
#define PID_YAW_ANGLE_KD           0.0f   /* s */

/* yaw rate 内环：输入角速度误差（deg/s），输出差模 PWM */
#define PID_YAW_KP                 0.0f
#define PID_YAW_KI                 0.0f

#define PID_ATT_KP_PITCH           0.0f
#define PID_ATT_KD_PITCH           0.0f

/* 输出限幅（TODO: 实车整定） ------------------------------------------------*/
#define PID_POS_MAX_SPEED_PULSES_S        50.0f
#define PID_SPEED_MAX_PITCH_DEG           10.0f
#define PID_ATT_TARGET_PITCH_MAX_DEG      30.0f  /* 姿态模式目标 Pitch 限幅 */
/*
 * TIM2 @ 84 MHz、Prescaler=0、ARR=4199 -> 20 kHz，共 4200 个计数。
 * PWM 满量程设为 4199，SETPWM() 会把 0~4199 映射到 0~ARR，
 * 从而用满 20 kHz 下的硬件占空比分辨率（约 0.024%）。
 */
#define PID_ATT_MAX_PWM                   4199.0f
#define PID_ATT_MAX_DIFF_PWM              4199.0f
#define PID_YAW_MAX_INTEGRAL              4199.0f
#define PID_YAW_MAX_RATE_DPS              180.0f  /* 中环输出目标 yaw rate 限幅 */

/* CubeMX 生成的队列句柄（freertos.c） --------------------------------------*/
extern osMessageQueueId_t ControlqueueHandle;

/* 位置控制器参数（只保留共模/直行） ------------------------------------------*/
typedef struct
{
  float kp_distance;      /* 共模位置误差 -> 目标速度，1/s */
  float kd_distance;      /* 共模位置误差微分 -> 目标速度，无量纲 */
  float max_speed;        /* 目标速度限幅，脉冲/s */
  int32_t encoder_start;   /* 位置模式入口共模编码器值 */
  int32_t encoder_current; /* 当前共模编码器值 */
} PositionControllerParams_t;

/* 速度/姿态控制器参数（只保留共模/直行） ------------------------------------*/
typedef struct
{
  float kp_speed;         /* 速度误差 -> 倾角，deg per 脉冲/s */
  float kd_speed;
  float max_pitch;        /* 目标倾角限幅，deg */
} SpeedControllerParams_t;

/* yaw 角度控制器参数（中环） -------------------------------------------------*/
typedef struct
{
  float kp_angle;         /* 角度误差 -> 目标 yaw rate，1/s */
  float kd_angle;         /* 角度误差微分 -> 目标 yaw rate，s */
  float max_rate_dps;     /* 目标 yaw rate 限幅，deg/s */
} YawAngleControllerParams_t;

/* 姿态/yaw rate 内环控制器参数 ----------------------------------------------*/
typedef struct
{
  float kp_pitch;         /* 倾角误差 -> 共模 PWM */
  float kd_pitch;         /* 俯仰角速度 -> 共模 PWM */
  float kp_yaw_rate;      /* yaw rate 误差 -> 差模 PWM，PWM per deg/s */
  float ki_yaw_rate;      /* yaw rate 积分 -> 差模 PWM */
  float max_pwm;          /* 共模/左右 PWM 限幅 */
  float max_diff_pwm;     /* 差模 PWM 限幅 */
  float max_yaw_integral; /* yaw rate 积分限幅 */
} AttitudeControllerParams_t;


/* PID 任务状态 --------------------------------------------------------------*/
typedef enum
{
  PID_STATE_STOP = 0,    /* 停止：输出关闭，等待 Controlqueue 命令 */
  PID_STATE_POSITION,    /* 位置控制模式：位置 -> 速度/姿态 -> 内环 */
  PID_STATE_SPEED,       /* 速度控制模式：速度/姿态 -> 内环 */
  PID_STATE_ATTITUDE     /* 姿态控制模式：只跑内环，方便调参 */
} PID_State_t;

/* 任务输入（目标由 Controlqueue 写入，编码器反馈由 PID 内部读取） */
typedef struct
{
  float target_distance;    /* 目标共模位移，编码器脉冲 */
  float target_speed;       /* 目标共模速度，脉冲/s */
  float target_pitch;       /* 姿态模式目标 Pitch，deg */
  float target_yaw_angle;   /* 位置模式目标 yaw 角，deg；相对首次进入位置模式的零点 */
  float target_yaw_rate;    /* 速度模式目标 yaw rate，deg/s；不使用 yaw 零点 */
  float current_position;   /* 当前共模位置，编码器脉冲 */
  float current_speed;      /* 当前共模速度，脉冲/s */
  float current_pitch;      /* deg（IMU 读取后更新） */
  float current_pitch_rate; /* deg/s（IMU 读取后更新） */
  float current_yaw_rate;   /* 当前 yaw 角速度，deg/s（IMU 读取后更新） */
  float current_yaw_angle;  /* 九轴绝对 yaw，deg（IMU 读取后更新） */
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

/* 控制器函数 */
float PID_Clamp(float value, float limit);

void PositionController_Update(const PositionControllerParams_t *params,
                               float target_distance,
                               float dt,
                               float *target_speed);
void PositionController_Start(void);

void SpeedController_Update(const SpeedControllerParams_t *params,
                            float target_speed, float current_speed,
                            float dt,
                            float *target_pitch);
void SpeedController_Reset(void);

void YawAngleController_Update(const YawAngleControllerParams_t *params,
                               float target_yaw_angle,
                               float current_yaw_angle,
                               float dt,
                               float *target_yaw_rate);
void YawAngleController_Reset(void);

void AttitudeController_Update(const AttitudeControllerParams_t *params,
                               float target_pitch, float current_pitch,
                               float pitch_rate,
                               float target_yaw_rate, float current_yaw_rate,
                               float dt,
                               float *pwm_left, float *pwm_right);
void AttitudeController_Reset(void);

/* 任务内部函数 */
void PIDTask_Init(void);
void PIDTask_StateStop(void);
void PIDTask_StatePosition(void);
void PIDTask_StateSpeed(void);
void PIDTask_StateAttitude(void);
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

