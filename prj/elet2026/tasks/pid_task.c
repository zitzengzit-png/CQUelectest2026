/**
  ******************************************************************************
  * @file    pid_task.c
  * @brief   PID 任务：变量定义与函数实现。
  ******************************************************************************
  * @note    类型、宏、参数结构体与函数声明见 pid_task.h。
  ******************************************************************************
  */

/* 头文件包含 ----------------------------------------------------------------*/
#include <string.h>
#include "FreeRTOS.h"
#include "main.h"
#include "dwt_time.h"
#include "pid_task.h"
#include "imu_task.h"

/* 私有变量 ------------------------------------------------------------------*/
/* 三个控制器的参数实例 */
static PositionControllerParams_t s_position_params =
{
  .kp_distance = PID_POS_KP_DISTANCE,
  .kd_distance = PID_POS_KD_DISTANCE,
  .kp_diff_position = PID_POS_KP_DIFF_POSITION,
  .kd_diff_position = PID_POS_KD_DIFF_POSITION,
  .max_speed = PID_POS_MAX_SPEED_PULSES_S,
  .max_diff_speed = PID_POS_MAX_DIFF_SPEED_PULSES_S,
  .encoder_start = 0,
  .encoder_current = 0,
  .encoder_diff_start = 0,
  .encoder_diff_current = 0
};

static SpeedControllerParams_t s_speed_params =
{
  .kp_speed = PID_SPEED_KP,
  .kd_speed = PID_SPEED_KD,
  .max_pitch = PID_SPEED_MAX_PITCH_DEG,
  .max_diff_speed = PID_SPEED_MAX_DIFF_SPEED_PULSES_S
};

static AttitudeControllerParams_t s_attitude_params =
{
  .kp_pitch = PID_ATT_KP_PITCH,
  .kd_pitch = PID_ATT_KD_PITCH,
  .kp_diff = PID_ATT_KP_DIFF,
  .ki_diff = PID_ATT_KI_DIFF,
  .max_pwm = PID_ATT_MAX_PWM,
  .max_diff_pwm = PID_ATT_MAX_DIFF_PWM,
  .max_diff_integral = PID_ATT_MAX_DIFF_INTEGRAL
};

/* 三个控制器的内部状态 */
static float s_position_last_distance_error = 0.0f;
static float s_position_last_diff_position_error = 0.0f;
static float s_speed_last_speed_error = 0.0f;
static float s_attitude_diff_integral = 0.0f;

/* 当前状态，仅本文件可见 */
static PID_State_t s_pid_state = PID_STATE_STOP;

/*
 * 多环调度计数器：0 ~ PID_OUTER_TICK_DIV-1。
 * PID 基础任务每执行一次前进一格，三环共用这一个计数器：
 *   - 每个 tick（250 Hz）运行内环；
 *   - tick % 5 == 0 运行中环（50 Hz）；
 *   - tick == 0 运行外环（10 Hz）。
 * 所有 tick 在 0 处同时命中，顺序固定为外环 -> 中环 -> 内环。
 */
static uint32_t s_pid_loop_counter = 0U;

/*
 * Controlqueue 轮询计数器：每个 250 Hz 基础 tick 前进一格，0 时读取队列；
 * 250 / 10 = 25 Hz（40 ms）。计数器在所有状态下都运行，不随模式切换复位。
 */
static uint32_t s_control_queue_counter = 0U;

/*
 * 实测 dt：DWT->CYCCNT 时间戳。
 * DWT 不可用时由 DWTTime_CalculateDt() 自动回退标称 dt。
 */
static uint32_t s_pid_last_inner_cycles = 0U;
static uint32_t s_pid_last_middle_cycles = 0U;
static uint32_t s_pid_last_outer_cycles = 0U;

/* 任务循环的 1 ms tick 时间戳，用于检测超过 DWT 32 位回绕周期的停顿 */
static uint32_t s_pid_last_loop_tick = 0U;

/* 慢环之间保持的输出，避免未到执行周期的环读到栈上临时变量 */
static float s_outer_target_speed = 0.0f;
static float s_outer_target_diff_speed = 0.0f;
static float s_middle_target_pitch = 0.0f;
static float s_middle_diff_speed_command = 0.0f;

/* 下一次唤醒的绝对 tick，用于 osDelayUntil 实现固定周期 */
static uint32_t s_pid_wake_tick = 0U;

/* 任务输入（目标由 Controlqueue 写入，反馈由 PIDTask_SetFeedback 写入） */
static PIDTask_Input_t s_pid_input;

/* 对外函数 ------------------------------------------------------------------*/
/**
  * @brief  PID 任务入口。
  * @param  argument 任务参数（未使用）。
  * @retval 无
  */
void StartPIDTask(void *argument)
{
  (void)argument;

  /*
   * 启动同步：defaultTask（44）先完成 IMU 校准并永久挂起，之后 PID（28）才运行。
   * 但 defaultTask 挂起时 IMU 首帧未必已到，若直接进入循环，可能在
   * kalmaneddataQueue 为空时用 pitch=0 处理 Controlqueue。这里先让出 CPU，
   * 等 IMU 真正发布首帧后再初始化控制。
   * 若 IMU 初始化失败，本任务会一直等待，输出保持关闭。
   */
  while (IMU_IsReady() == 0U)
  {
    (void)osDelay(10U);
  }

  PIDTask_Init();

  for (;;)
  {
    /* 非阻塞读取最新 IMU，更新 pitch / pitch_rate */
    PIDTask_UpdateIMU();

    /* 检测任务是否停顿超过 DWT 32 位回绕保护窗口 */
    PIDTask_CheckTimestampOverflow();

    /*
     * 状态机之前按 25 Hz 读取 Controlqueue；先更新状态/目标，
     * 再由下面的 switch 进入新状态。
     */
    PIDTask_UpdateControlQueue();

    switch (s_pid_state)
    {
      case PID_STATE_POSITION:
        PIDTask_StatePosition();
        break;

      case PID_STATE_SPEED:
        PIDTask_StateSpeed();
        break;

      case PID_STATE_STOP:
        PIDTask_StateStop();
        break;

      default:
        PIDTask_StateStop();
        break;
    }

    /* 任务节拍：按固定周期运行 */
    PIDTask_DelayUntil();
  }
}

/* ========================================================================== */
/*                              以下为函数实现                                */
/* ========================================================================== */


/*
 * 功能：PID 任务的一次性初始化（输入清零、三个控制器复位、调度计数器清零、
 *       DWT 实测 dt 时钟初始化）
 * 输入：无
 * 输出：无
 */
void PIDTask_Init(void)
{
  s_pid_state = PID_STATE_STOP;
  s_pid_wake_tick = osKernelGetTickCount();
  s_pid_last_loop_tick = s_pid_wake_tick;

  (void)memset(&s_pid_input, 0, sizeof(s_pid_input));

  s_position_params.encoder_start = 0;
  s_position_params.encoder_current = 0;
  s_position_params.encoder_diff_start = 0;
  s_position_params.encoder_diff_current = 0;

  PIDTask_ResetLoopCounter();
  s_control_queue_counter = 0U;

  PIDTask_InitHighResClock();
  PIDTask_ResetLoopTimers();

  s_outer_target_speed = 0.0f;
  s_outer_target_diff_speed = 0.0f;
  s_middle_target_pitch = 0.0f;
  s_middle_diff_speed_command = 0.0f;

  PositionController_Start();
  SpeedController_Reset();
  AttitudeController_Reset();
}

/*
 * 功能：从 kalmaneddataQueue 读取最新解算结果并更新 pitch / pitch_rate
 * 输入：无
 * 输出：无
 */
void PIDTask_UpdateIMU(void)
{
  KalmanData_t data;

  /*
   * 队列项是 KalmanData_t 本身，osMessageQueueGet 会把 16 字节完整复制到
   * 本地变量 data；IMU 任务之后修改自己的发布缓冲不会影响这里，因此不需要
   * 再对共享指针做临界区保护。
   */
  if (osMessageQueueGet(kalmaneddataQueueHandle, &data, NULL, 0U) == osOK)
  {
    s_pid_input.current_pitch = data.pitch;
    s_pid_input.current_pitch_rate = data.pitch_rate;
  }
}

/*
 * 功能：按 25 Hz 轮询 Controlqueue，并在状态机之前更新状态/目标
 * 输入：无
 * 输出：无
 */
void PIDTask_UpdateControlQueue(void)
{
  if (s_control_queue_counter == 0U)
  {
    PIDTask_ProcessControlQueue();
  }

  s_control_queue_counter++;
  if (s_control_queue_counter >= PID_CONTROL_QUEUE_TICK_DIV)
  {
    s_control_queue_counter = 0U;
  }
}

/*
 * 功能：启动共享的 DWT CYCCNT 周期计数器（幂等）
 * 输入：无
 * 输出：无
 */
void PIDTask_InitHighResClock(void)
{
  DWTTime_Init();
}

/*
 * 功能：返回当前 DWT 周期计数；DWT 不可用时返回 0
 * 输入：无
 * 输出：当前 CYCCNT
 */
uint32_t PIDTask_GetTimestamp(void)
{
  return DWTTime_GetCycles();
}

/*
 * 功能：用两个 DWT 时间戳计算实际 dt
 * 输入：now —— 当前 CYCCNT；
 *       last_cycles —— 上一次该环的 CYCCNT，函数内更新；
 *       nominal_dt —— DWT 不可用或周期为 0 时的标称 dt；
 *       max_dt —— 防止任务长时间停顿造成 dt 过大的上限
 * 输出：实际 dt（s）
 */
float PIDTask_CalculateDt(uint32_t now, uint32_t *last_cycles,
                          float nominal_dt, float max_dt)
{
  return DWTTime_CalculateDt(now, last_cycles, nominal_dt, max_dt);
}

/*
 * 功能：把三个环的时间戳回退到各自一个标称周期前
 *       这样模式切换后的第一次执行不会因为 dt≈0 导致微分项异常
 * 输入：无
 * 输出：无
 */
void PIDTask_ResetLoopTimers(void)
{
  uint32_t now;
  uint32_t cycles_per_ms;

  if ((DWTTime_IsReady() == 0U) || (SystemCoreClock == 0U))
  {
    s_pid_last_inner_cycles = 0U;
    s_pid_last_middle_cycles = 0U;
    s_pid_last_outer_cycles = 0U;
    return;
  }

  now = DWTTime_GetCycles();
  cycles_per_ms = SystemCoreClock / 1000U;

  s_pid_last_inner_cycles = now - (cycles_per_ms * PID_TASK_PERIOD_MS);
  s_pid_last_middle_cycles = now - (cycles_per_ms * PID_MIDDLE_PERIOD_MS);
  s_pid_last_outer_cycles = now - (cycles_per_ms * PID_OUTER_PERIOD_MS);
}

/*
 * 功能：用 1 ms FreeRTOS tick 检测任务循环停顿是否超过 DWT 32 位回绕窗口
 *       超过时重置三个环的时间戳，避免使用回绕后的错误差值
 * 输入：无
 * 输出：无
 */
void PIDTask_CheckTimestampOverflow(void)
{
  uint32_t now_tick = osKernelGetTickCount();

  if ((now_tick - s_pid_last_loop_tick) >
      pdMS_TO_TICKS(PID_DT_OVERFLOW_GUARD_MS))
  {
    PIDTask_ResetLoopTimers();
  }

  s_pid_last_loop_tick = now_tick;
}

/*
 * 功能：返回当前多环调度 tick，并推进基础任务计数器
 * 输入：无
 * 输出：当前 tick（0 ~ PID_OUTER_TICK_DIV-1）
 */
uint32_t PIDTask_NextLoopTick(void)
{
  uint32_t tick = s_pid_loop_counter;

  s_pid_loop_counter++;
  if (s_pid_loop_counter >= PID_OUTER_TICK_DIV)
  {
    s_pid_loop_counter = 0U;
  }

  return tick;
}

/*
 * 功能：把多环调度计数器清零（模式切换后从 tick 0 开始）
 * 输入：无
 * 输出：无
 */
void PIDTask_ResetLoopCounter(void)
{
  s_pid_loop_counter = 0U;
}

/*
 * 功能：位置模式调度：外环 10 Hz -> 中环 50 Hz -> 内环 250 Hz
 * 输入：无（从 s_pid_input 读取目标与反馈）
 * 输出：无（更新左右 PWM）
 */
void PIDTask_StatePosition(void)
{
  uint32_t tick = PIDTask_NextLoopTick();

  /* 外环 10 Hz：每 25 个基础 tick 执行一次 */
  if ((tick % PID_OUTER_TICK_DIV) == 0U)
  {
    PIDTask_PositionOuterLoop();
  }

  /* 中环 50 Hz：每 5 个基础 tick 执行一次；必须在外环之后 */
  if ((tick % PID_MIDDLE_TICK_DIV) == 0U)
  {
    PIDTask_PositionMiddleLoop();
  }

  /* 内环 250 Hz：每个基础 tick 都执行 */
  PIDTask_AttitudeInnerLoop();
}

/*
 * 功能：速度模式调度：中环 50 Hz -> 内环 250 Hz（跳过位置外环）
 * 输入：无（从 s_pid_input 读取目标与反馈）
 * 输出：无（更新左右 PWM）
 */
void PIDTask_StateSpeed(void)
{
  uint32_t tick = PIDTask_NextLoopTick();

  /* 中环 50 Hz：每 5 个基础 tick 执行一次 */
  if ((tick % PID_MIDDLE_TICK_DIV) == 0U)
  {
    PIDTask_SpeedMiddleLoop();
  }

  /* 内环 250 Hz：每个基础 tick 都执行 */
  PIDTask_AttitudeInnerLoop();
}

/*
 * 功能：位置外环（10 Hz）：共模/差模位置 PD -> 目标速度/目标差模速度
 * 输入：无（从 s_pid_input 读取目标与编码器反馈）
 * 输出：无（更新 s_outer_target_speed / s_outer_target_diff_speed）
 */
void PIDTask_PositionOuterLoop(void)
{
  uint32_t now = PIDTask_GetTimestamp();
  float dt = PIDTask_CalculateDt(now, &s_pid_last_outer_cycles,
                                 PID_OUTER_DT_S, PID_OUTER_DT_MAX_S);

  /*
   * 共模位移与差模旋转量全部来自编码器脉冲：
   *   current_position      = 共模位置
   *   current_diff_position = 差模位置
   * IMU 的 pitch/pitch_rate 只用于姿态内环，不参与旋转目标/反馈。
   */
  s_position_params.encoder_current = (int32_t)s_pid_input.current_position;
  s_position_params.encoder_diff_current = (int32_t)s_pid_input.current_diff_position;

  PositionController_Update(&s_position_params,
                            s_pid_input.target_distance,
                            s_pid_input.target_diff_position,
                            dt,
                            &s_outer_target_speed,
                            &s_outer_target_diff_speed);
}

/*
 * 功能：位置模式中环（50 Hz）：共模速度 PD -> 目标倾角；差模目标速度透传
 * 输入：无（使用外环保持的 s_outer_target_speed / s_outer_target_diff_speed）
 * 输出：无（更新 s_middle_target_pitch / s_middle_diff_speed_command）
 */
void PIDTask_PositionMiddleLoop(void)
{
  uint32_t now = PIDTask_GetTimestamp();
  float dt = PIDTask_CalculateDt(now, &s_pid_last_middle_cycles,
                                 PID_MIDDLE_DT_S, PID_MIDDLE_DT_MAX_S);

  SpeedController_Update(&s_speed_params,
                         s_outer_target_speed,
                         s_pid_input.current_speed,
                         s_outer_target_diff_speed,
                         dt,
                         &s_middle_target_pitch,
                         &s_middle_diff_speed_command);
}

/*
 * 功能：速度模式中环（50 Hz）：共模速度 PD -> 目标倾角；差模目标速度透传
 * 输入：无（从 s_pid_input 读取目标/当前共模速度、目标差模速度）
 * 输出：无（更新 s_middle_target_pitch / s_middle_diff_speed_command）
 */
void PIDTask_SpeedMiddleLoop(void)
{
  uint32_t now = PIDTask_GetTimestamp();
  float dt = PIDTask_CalculateDt(now, &s_pid_last_middle_cycles,
                                 PID_MIDDLE_DT_S, PID_MIDDLE_DT_MAX_S);

  SpeedController_Update(&s_speed_params,
                         s_pid_input.target_speed,
                         s_pid_input.current_speed,
                         s_pid_input.target_diff_speed,
                         dt,
                         &s_middle_target_pitch,
                         &s_middle_diff_speed_command);
}

/*
 * 功能：内环（250 Hz）：姿态 PD + 差速 PI -> 左右 PWM 输出
 * 输入：无（使用中环保持的目标倾角/差模速度，以及 s_pid_input 的反馈）
 * 输出：无（更新左右 PWM）
 */
void PIDTask_AttitudeInnerLoop(void)
{
  uint32_t now = PIDTask_GetTimestamp();
  float dt = PIDTask_CalculateDt(now, &s_pid_last_inner_cycles,
                                 PID_INNER_DT_S, PID_INNER_DT_MAX_S);
  float pwm_left;
  float pwm_right;

  AttitudeController_Update(&s_attitude_params,
                            s_middle_target_pitch,
                            s_pid_input.current_pitch,
                            s_pid_input.current_pitch_rate,
                            s_middle_diff_speed_command,
                            s_pid_input.current_diff_speed,
                            dt,
                            &pwm_left,
                            &pwm_right);

  PIDTask_WriteOutput(pwm_left, pwm_right);
}

/*
 * 功能：STOP 状态处理，输出关闭；Controlqueue 由任务主循环按 25 Hz 轮询
 * 输入：无
 * 输出：无
 */
void PIDTask_StateStop(void)
{
  PIDTask_WriteOutput(0.0f, 0.0f);
}

/*
 * 功能：检查 Controlqueue，读取新命令并设置下一状态与目标值；
 *       由 PIDTask_UpdateControlQueue() 按 25 Hz 调用
 * 输入：无
 * 输出：无（更新 s_pid_state 与 s_pid_input）
 */
void PIDTask_ProcessControlQueue(void)
{
  uint64_t message;
  uint8_t mode;
  int16_t value1;
  int16_t value2;

  if (ControlqueueHandle == NULL)
  {
    return;
  }

  if (osMessageQueueGetCount(ControlqueueHandle) == 0U)
  {
    return;
  }

  /* CubeMX 队列项为 6 字节；STM32 小端，uint64_t 的低 6 字节即 48 位消息 */
  message = 0U;
  if (osMessageQueueGet(ControlqueueHandle, &message, NULL, 0U) != osOK)
  {
    return;
  }

  mode = (uint8_t)((message >> 46) & 0x3U);
  value1 = (int16_t)((message >> 16) & 0xFFFFU);
  value2 = (int16_t)(message & 0xFFFFU);

  switch (mode)
  {
    case CONTROL_MODE_STOP:
    default:
      PositionController_Start();
      SpeedController_Reset();
      AttitudeController_Reset();

      s_outer_target_speed = 0.0f;
      s_outer_target_diff_speed = 0.0f;
      s_middle_target_pitch = 0.0f;
      s_middle_diff_speed_command = 0.0f;

      PIDTask_ResetLoopCounter();
      PIDTask_ResetLoopTimers();
      s_pid_state = PID_STATE_STOP;
      break;

    case CONTROL_MODE_POSITION:
      /* 位置模式：value1 = 共模位移脉冲，value2 = 差模旋转脉冲（不是 IMU 角度） */
      s_pid_input.target_distance = (float)value1;
      s_pid_input.target_diff_position = (float)value2;
      PIDTask_StartPositionMode();
      break;

    case CONTROL_MODE_SPEED:
      s_pid_input.target_speed = (float)value1;
      s_pid_input.target_diff_speed = (float)value2;

      /* 从其他模式切入速度模式时复位中环/内环、计时戳并让第一拍从 tick 0 开始 */
      if (s_pid_state != PID_STATE_SPEED)
      {
        SpeedController_Reset();
        AttitudeController_Reset();

        s_middle_target_pitch = 0.0f;
        s_middle_diff_speed_command = 0.0f;

        PIDTask_ResetLoopCounter();
        PIDTask_ResetLoopTimers();
      }

      s_pid_state = PID_STATE_SPEED;
      break;
  }
}

/*
 * 功能：进入位置控制模式（读取当前共模/差模编码器并锁存起点；
 *       从其他模式切入时复位中环/内环、调度计数器和 dt 时间戳）
 * 输入：无
 * 输出：无（切换到 PID_STATE_POSITION）
 */
void PIDTask_StartPositionMode(void)
{
  s_position_params.encoder_current = (int32_t)s_pid_input.current_position;
  s_position_params.encoder_diff_current = (int32_t)s_pid_input.current_diff_position;

  /*
   * 从其他模式切入位置模式时，复位中环/内环和历史保持量；
   * 已在位置模式时只重新锁存起点，不重置调度相位，避免频繁位置指令
   * 把外环实际执行频率抬高到基础任务频率。
   */
  if (s_pid_state != PID_STATE_POSITION)
  {
    SpeedController_Reset();
    AttitudeController_Reset();

    s_outer_target_speed = 0.0f;
    s_outer_target_diff_speed = 0.0f;
    s_middle_target_pitch = 0.0f;
    s_middle_diff_speed_command = 0.0f;

    PIDTask_ResetLoopCounter();
    PIDTask_ResetLoopTimers();
  }

  PositionController_Start();
  s_pid_state = PID_STATE_POSITION;
}

/*
 * 功能：把有符号 PWM 转为方向 + 比较值，调用 SETPWM 输出
 * 输入：pwm_left / pwm_right —— 左右电机 PWM（正 = 正转，负 = 反转）
 * 输出：无
 */
void PIDTask_WriteOutput(float pwm_left, float pwm_right)
{
  uint16_t compare_left;
  uint16_t compare_right;
  uint8_t forward_left;
  uint8_t forward_right;

  forward_left = (pwm_left >= 0.0f) ? 1U : 0U;
  forward_right = (pwm_right >= 0.0f) ? 1U : 0U;

  compare_left = (uint16_t)((pwm_left >= 0.0f) ? pwm_left : -pwm_left);
  compare_right = (uint16_t)((pwm_right >= 0.0f) ? pwm_right : -pwm_right);

  SETPWM(1U, forward_left, compare_left);   /* 左轮 */
  SETPWM(0U, forward_right, compare_right); /* 右轮 */
}


/*
 * 功能：把数值限制在 [-limit, +limit] 内
 * 输入：value —— 待限幅数值；limit —— 正限幅值
 * 输出：限幅后的数值
 */
float PID_Clamp(float value, float limit)
{
  if (value > limit)
  {
    return limit;
  }

  if (value < -limit)
  {
    return -limit;
  }

  return value;
}

/*
 * 功能：位置外环（共模位置 PD -> 目标速度；差模位置 PD -> 目标差模速度）
 * 输入：params —— 位置控制器参数；
 *       target_distance —— 共模目标位移（编码器脉冲）；
 *       target_diff_position —— 差模目标旋转量（编码器脉冲）；
 *       dt —— 控制周期（s）
 * 输出：target_speed —— 目标速度（脉冲/s）；
 *       target_diff_speed —— 目标差模速度（脉冲/s）
 *
 * 说明：差模通道直接输出目标差模速度，中环只做透传；
 *       当前共模位置 = encoder_current - encoder_start；
 *       当前差模位置 = encoder_diff_current - encoder_diff_start。
 *       两者都是编码器脉冲，不经过 cm/deg 换算；IMU 不参与这里。
 */
void PositionController_Update(const PositionControllerParams_t *params,
                                      float target_distance,
                                      float target_diff_position,
                                      float dt,
                                      float *target_speed,
                                      float *target_diff_speed)
{
  float current_distance = (float)(params->encoder_current - params->encoder_start);
  float current_diff_position =
      (float)(params->encoder_diff_current - params->encoder_diff_start);
  float e_distance = target_distance - current_distance;
  float e_diff_position = target_diff_position - current_diff_position;
  float d_distance = 0.0f;
  float d_diff_position = 0.0f;

  if (dt > 0.0f)
  {
    d_distance = (e_distance - s_position_last_distance_error) / dt;
    d_diff_position = (e_diff_position - s_position_last_diff_position_error) / dt;
  }

  *target_speed = params->kp_distance * e_distance
                  + params->kd_distance * d_distance;
  *target_diff_speed = params->kp_diff_position * e_diff_position
                       + params->kd_diff_position * d_diff_position;

  *target_speed = PID_Clamp(*target_speed, params->max_speed);
  *target_diff_speed = PID_Clamp(*target_diff_speed, params->max_diff_speed);

  s_position_last_distance_error = e_distance;
  s_position_last_diff_position_error = e_diff_position;
}

/*
 * 功能：进入位置模式时锁存共模/差模起始编码器值，并清零微分历史
 * 输入：无（使用 s_position_params.encoder_current / encoder_diff_current）
 * 输出：无
 */
void PositionController_Start(void)
{
  s_position_params.encoder_start = s_position_params.encoder_current;
  s_position_params.encoder_diff_start = s_position_params.encoder_diff_current;
  s_position_last_distance_error = 0.0f;
  s_position_last_diff_position_error = 0.0f;
}

/*
 * 功能：中环速度控制器（共模速度 PD -> 目标倾角；差模目标速度直接透传）
 * 输入：params —— 速度控制器参数；
 *       target_speed / current_speed —— 目标/当前共模速度（脉冲/s）；
 *       target_diff_speed —— 目标差模速度（脉冲/s）；
 *                            位置模式由外环差模位置 PD 给出，
 *                            速度模式由 Controlqueue 的 value2 直接给出；
 *       dt —— 控制周期（s）
 * 输出：target_pitch —— 目标倾角（deg）；
 *       diff_speed_command —— 透传到内环的目标差模速度（脉冲/s）
 *
 * 说明：差模通道不做 PID，只做安全限幅后透传。
 */
void SpeedController_Update(const SpeedControllerParams_t *params,
                                   float target_speed, float current_speed,
                                   float target_diff_speed,
                                   float dt,
                                   float *target_pitch, float *diff_speed_command)
{
  float e_speed = target_speed - current_speed;
  float d_speed = 0.0f;

  if (dt > 0.0f)
  {
    d_speed = (e_speed - s_speed_last_speed_error) / dt;
  }

  /* 共模：速度 PD -> 目标倾角 */
  *target_pitch = params->kp_speed * e_speed + params->kd_speed * d_speed;
  *target_pitch = PID_Clamp(*target_pitch, params->max_pitch);

  /* 差模：目标差模速度直接透传，不做 PID */
  *diff_speed_command = PID_Clamp(target_diff_speed, params->max_diff_speed);

  s_speed_last_speed_error = e_speed;
}

/*
 * 功能：速度/姿态控制器复位（清零微分历史）
 * 输入：无
 * 输出：无
 */
void SpeedController_Reset(void)
{
  s_speed_last_speed_error = 0.0f;
}


/*
 * 功能：姿态/差速内环计算（倾角 PD -> 共模 PWM；差速 PI -> 差模 PWM）
 * 输入：params —— 姿态控制器参数；
 *       target_pitch / current_pitch —— 目标/当前倾角（deg）；
 *       pitch_rate —— 当前俯仰角速度（deg/s）；
 *       target_diff_speed / current_diff_speed —— 目标/当前差模速度（脉冲/s）；
 *       dt —— 控制周期（s）
 * 输出：pwm_left / pwm_right —— 左右电机 PWM
 */
void AttitudeController_Update(const AttitudeControllerParams_t *params,
                                      float target_pitch, float current_pitch,
                                      float pitch_rate,
                                      float target_diff_speed, float current_diff_speed,
                                      float dt,
                                      float *pwm_left, float *pwm_right)
{
  float e_pitch = current_pitch - target_pitch;
  float e_diff = target_diff_speed - current_diff_speed;
  float pwm_common;
  float pwm_diff;

  pwm_common = params->kp_pitch * e_pitch + params->kd_pitch * pitch_rate;
  pwm_common = PID_Clamp(pwm_common, params->max_pwm);

  if (dt > 0.0f)
  {
    s_attitude_diff_integral += e_diff * dt;
    s_attitude_diff_integral = PID_Clamp(s_attitude_diff_integral,
                                         params->max_diff_integral);
  }

  pwm_diff = params->kp_diff * e_diff
             + params->ki_diff * s_attitude_diff_integral;
  pwm_diff = PID_Clamp(pwm_diff, params->max_diff_pwm);

  *pwm_left = PID_Clamp(pwm_common - pwm_diff, params->max_pwm);
  *pwm_right = PID_Clamp(pwm_common + pwm_diff, params->max_pwm);
}

/*
 * 功能：姿态/差速内环复位（清零积分）
 * 输入：无
 * 输出：无
 */
void AttitudeController_Reset(void)
{
  s_attitude_diff_integral = 0.0f;
}

/*
 * 功能：延时到下一个控制节拍（用绝对 tick 定时，避免周期累计误差）
 * 输入：无
 * 输出：无
 */
void PIDTask_DelayUntil(void)
{
  s_pid_wake_tick += pdMS_TO_TICKS(PID_TASK_PERIOD_MS);
  (void)osDelayUntil(s_pid_wake_tick);
}

/*
 * 功能：更新编码器反馈量
 * 输入：position_pulses —— 当前共模位置（编码器脉冲）；
 *       diff_position_pulses —— 当前差模位置（编码器脉冲）；
 *       speed_pulses_s —— 当前共模速度（脉冲/s）；
 *       diff_speed_pulses_s —— 当前差模速度（脉冲/s）
 * 输出：无
 */
void PIDTask_SetFeedback(float position_pulses, float diff_position_pulses,
                         float speed_pulses_s, float diff_speed_pulses_s)
{
  s_pid_input.current_position = position_pulses;
  s_pid_input.current_diff_position = diff_position_pulses;
  s_pid_input.current_speed = speed_pulses_s;
  s_pid_input.current_diff_speed = diff_speed_pulses_s;
}

/*
 * 功能：设置单侧电机 PWM 方向与比较值（空实现，后续填 HAL TIM）
 * 输入：left_wheel —— 1 = 左轮，0 = 右轮；
 *       forward —— 1 = 正转，0 = 反转；
 *       compare —— 写入比较寄存器的值
 * 输出：无
 */
void SETPWM(uint8_t left_wheel, uint8_t forward, uint16_t compare)
{
  (void)left_wheel;
  (void)forward;
  (void)compare;

  /* TODO:
   * 左轮：left_wheel = 1；右轮：left_wheel = 0。
   * 正转：正引脚 CCR = compare，负引脚 CCR = 0；
   * 反转：正引脚 CCR = 0，负引脚 CCR = compare。
   * 使用 __HAL_TIM_SET_COMPARE / HAL_TIM_PWM_Start。 */
}

/************************ (C) COPYRIGHT *****END OF FILE***********************/

