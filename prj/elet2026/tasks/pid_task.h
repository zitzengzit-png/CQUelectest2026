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
#include "robot_config.h"

/*
 * 本文件的频率、PID 增益、限幅、模式编码和指令单位全部在 robot_config.h 中定义。
 * 调参时只改 robot_config.h，不需要在多个头文件之间同步。
 */

/* CubeMX 生成的队列句柄（freertos.c） --------------------------------------*/
extern osMessageQueueId_t ControlqueueHandle;

/* 位置控制器参数（只保留共模/直行，长度单位 m） ------------------------------*/
typedef struct
{
  float kp_distance;      /* 共模位置误差(m) -> 目标速度(m/s)，1/s */
  float kd_distance;      /* 共模位置误差微分 -> 目标速度，无量纲 */
  float max_speed;        /* 目标速度限幅，m/s */
  float position_start_m;   /* 位置模式入口共模位置，m */
  float position_current_m; /* 当前共模位置，m */
} PositionControllerParams_t;

/* 速度/姿态控制器参数（只保留共模/直行，速度单位 m/s） ----------------------*/
typedef struct
{
  float kp_speed;         /* 速度误差(m/s) -> 倾角，deg per m/s */
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
  float ff_speed;         /* 速度前馈：PWM per (m/s)，抵消电机反电动势 */
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
  float target_distance;    /* 目标共模位移，m */
  float target_speed;       /* 目标共模速度，m/s */
  float target_pitch;       /* 姿态模式目标 Pitch，deg */
  float target_yaw_angle;   /* 位置模式目标 yaw 角，deg；相对首次进入位置模式的零点 */
  float target_yaw_rate;    /* 速度模式目标 yaw rate，deg/s；不使用 yaw 零点 */
  float current_position;   /* 当前共模位置，m */
  float current_speed;      /* 当前共模速度，m/s */
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
void PositionController_ResetDerivative(void);

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
                               float current_speed,
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

/* 调试计数：PID 基础循环实际执行次数（displayTask 读取） */
uint32_t PIDTask_GetRunCount(void);

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

