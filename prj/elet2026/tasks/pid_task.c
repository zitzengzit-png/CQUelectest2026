/**
  ******************************************************************************
  * @file    pid_task.c
  * @brief   PID 任务：变量定义与函数实现。
  ******************************************************************************
  * @note    类型、宏、参数结构体与函数声明见 pid_task.h。
  ******************************************************************************
  */

/* 头文件包含 ----------------------------------------------------------------*/
#include <math.h>
#include <string.h>
#include "FreeRTOS.h"
#include "main.h"
#include "tim.h"
#include "dwt_time.h"
#include "pid_task.h"
#include "imu_task.h"
#include "encoder.h"
#include "display_task.h"

/* 私有函数声明 --------------------------------------------------------------*/
static float PIDTask_Wrap180(float angle);
static void PIDTask_ResetYawReference(void);
static float PIDTask_GetYawRelative(void);
static void PIDTask_UpdateEncoderFeedback(void);
static void PIDTask_UpdateDisplayQueue(void);
static void PIDTask_UpdateLoopTime(uint32_t start_cycles);
static void PIDTask_ResetOuterTimer(void);
static int32_t PIDTask_RoundToInt32(float value);
static int16_t PIDTask_ToInt16(float value);

/* 私有变量 ------------------------------------------------------------------*/
/* 三个控制器的参数实例 */
static PositionControllerParams_t s_position_params =
{
  .kp_distance = PID_POS_KP_DISTANCE,
  .kd_distance = PID_POS_KD_DISTANCE,
  .max_speed = PID_POS_MAX_SPEED_PULSES_S,
  .encoder_start = 0,
  .encoder_current = 0
};

static SpeedControllerParams_t s_speed_params =
{
  .kp_speed = PID_SPEED_KP,
  .kd_speed = PID_SPEED_KD,
  .max_pitch = PID_SPEED_MAX_PITCH_DEG
};

static YawAngleControllerParams_t s_yaw_angle_params =
{
  .kp_angle = PID_YAW_ANGLE_KP,
  .kd_angle = PID_YAW_ANGLE_KD,
  .max_rate_dps = PID_YAW_MAX_RATE_DPS
};

static AttitudeControllerParams_t s_attitude_params =
{
  .kp_pitch = PID_ATT_KP_PITCH,
  .kd_pitch = PID_ATT_KD_PITCH,
  .kp_yaw_rate = PID_YAW_KP,
  .ki_yaw_rate = PID_YAW_KI,
  .max_pwm = PID_ATT_MAX_PWM,
  .max_diff_pwm = PID_ATT_MAX_DIFF_PWM,
  .max_yaw_integral = PID_YAW_MAX_INTEGRAL
};

/* 三个控制器的内部状态 */
static float s_position_last_distance_error = 0.0f;
static float s_speed_last_speed_error = 0.0f;
static float s_yaw_angle_last_error = 0.0f;
static float s_yaw_rate_integral = 0.0f;

/* yaw 相对角零点：进入位置/速度模式时锁存九轴 yaw */
static float s_yaw_zero_deg = 0.0f;

/* 当前状态，仅本文件可见 */
static PID_State_t s_pid_state = PID_STATE_STOP;

/*
 * POSITION 命令当拍强制外环标志：
 * 收到 POSITION 命令后，即使当前 tick 不是 10 Hz 外环时隙，也在本轮执行
 * 一次 PositionController_Update()。中环仍按 50 Hz 调度。
 */
static uint8_t s_force_position_outer = 0U;

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
 * 250 / 5 = 50 Hz（20 ms）。计数器在所有状态下都运行，不随模式切换复位。
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
static float s_middle_target_pitch = 0.0f;
static float s_middle_target_yaw_rate = 0.0f;

/* 任务输入（目标由 Controlqueue 写入，编码器反馈由 PID 内部读取） */
static PIDTask_Input_t s_pid_input;

/* OLED 调试快照 -------------------------------------------------------------*/
/* dataToLCDQueue 元素为 68 字节，直接按值发送完整 LCD_Data_t。 */
static LCD_Data_t s_lcd_snapshot;
static uint32_t s_lcd_sequence = 0U;
static uint32_t s_display_div_accum = 0U;
static uint32_t s_display_queue_drop_count = 0U;

/* 最近一次输出的左右 PWM，供显示任务观察 */
static int16_t s_pid_output_left = 0;
static int16_t s_pid_output_right = 0;

/* 最近一次 250 Hz PID 循环耗时 */
static uint16_t s_pid_loop_time_us = 0U;

/* PID 基础循环实际执行次数；displayTask 用它计算实际频率 */
static volatile uint32_t s_pid_run_count = 0U;

/* IMU 有效性/时间戳，用于显示 IMU 是否停更 */
static uint8_t s_imu_data_valid = 0U;
static uint32_t s_imu_last_timestamp_ms = 0U;

/* 对外函数 ------------------------------------------------------------------*/
/**
  * @brief  PID 任务入口。
  * @param  argument 任务参数（未使用）。
  * @retval 无
  */
void StartPIDTask(void *argument)
{
  (void)argument;

  PIDTask_Init();

  for (;;)
  {
    uint32_t loop_start_cycles;

    /*
     * PID 只由 USART1 RX DMA 完成中断释放的信号量驱动：
     * 每次 DMA 完成 -> IMUdatadoneBinarySem -> PID 立即抢占并执行一轮。
     * 不调用 osDelayUntil()，节拍完全跟随 IMU 的 250 Hz 数据。
     */
    if (osSemaphoreAcquire(IMUdatadoneBinarySemHandle, osWaitForever) != osOK)
    {
      continue;
    }

    s_pid_run_count++;
    loop_start_cycles = PIDTask_GetTimestamp();

    /* 解析 DMA 中断保存的原始字节，再取最新姿态/角速度 */
    (void)IMU_Parse();
    PIDTask_UpdateIMU();

    /* 检测任务停顿是否超过 DWT 32 位回绕保护窗口 */
    PIDTask_CheckTimestampOverflow();

    /*
     * 状态机之前按 50 Hz 读取 Controlqueue；先更新状态/目标，
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

      case PID_STATE_ATTITUDE:
        PIDTask_StateAttitude();
        break;

      case PID_STATE_STOP:
        PIDTask_StateStop();
        break;

      default:
        PIDTask_StateStop();
        break;
    }

    /* 记录本轮耗时，并按 20 Hz 向显示任务发送快照 */
    PIDTask_UpdateLoopTime(loop_start_cycles);
    PIDTask_UpdateDisplayQueue();
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
  s_force_position_outer = 0U;
  s_pid_last_loop_tick = osKernelGetTickCount();

  (void)memset(&s_pid_input, 0, sizeof(s_pid_input));

  s_position_params.encoder_start = 0;
  s_position_params.encoder_current = 0;

  YawAngleController_Reset();
  s_yaw_zero_deg = 0.0f;
  s_pid_input.target_pitch = 0.0f;
  s_pid_input.target_yaw_angle = 0.0f;
  s_pid_input.target_yaw_rate = 0.0f;

  PIDTask_ResetLoopCounter();
  s_control_queue_counter = 0U;

  PIDTask_InitHighResClock();
  PIDTask_ResetLoopTimers();

  s_outer_target_speed = 0.0f;
  s_middle_target_pitch = 0.0f;
  s_middle_target_yaw_rate = 0.0f;

  PositionController_Start();
  SpeedController_Reset();
  AttitudeController_Reset();

  /* OLED 调试快照与运行统计初始化 */
  (void)memset(&s_lcd_snapshot, 0, sizeof(s_lcd_snapshot));
  s_lcd_sequence = 0U;
  s_display_div_accum = 0U;
  s_display_queue_drop_count = 0U;
  s_pid_output_left = 0;
  s_pid_output_right = 0;
  s_pid_loop_time_us = 0U;
  s_pid_run_count = 0U;
  s_imu_data_valid = 0U;
  s_imu_last_timestamp_ms = 0U;
}

/*
 * 功能：在 PID 任务上下文读取编码器并更新共模位置/速度
 * 输入：无
 * 输出：无
 *
 * 说明：编码器只在 PID 中环 / 进入位置模式时读取，
 *       其他任务不得调用 Encoder_Update()。
 */
static void PIDTask_UpdateEncoderFeedback(void)
{
  float position;
  float speed;

  Encoder_Update(&position, &speed);
  s_pid_input.current_position = position;
  s_pid_input.current_speed = speed;
}

/*
 * 功能：从 IMU 驱动读取最新解析结果并更新 pitch / pitch_rate / yaw_rate / yaw_angle
 * 输入：无
 * 输出：无
 */
void PIDTask_UpdateIMU(void)
{
  IMU_Data_t data;

  IMU_GetData(&data);

  s_imu_data_valid = data.valid;

  if (data.valid != 0U)
  {
    s_imu_last_timestamp_ms = data.timestamp_ms;
    s_pid_input.current_pitch = data.pitch_deg;
    s_pid_input.current_pitch_rate = data.pitch_rate_dps;
    s_pid_input.current_yaw_rate = data.yaw_rate_dps;
    s_pid_input.current_yaw_angle = data.yaw_deg;
  }
}

/*
 * 功能：按 50 Hz 轮询 Controlqueue，并在状态机之前更新状态/目标
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
 * 功能：读取 PID 基础循环实际执行次数
 * 输入：无
 * 输出：运行计数（32 位，displayTask 用差值计算频率）
 */
uint32_t PIDTask_GetRunCount(void)
{
  return s_pid_run_count;
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
 * 功能：把位置外环时间戳回退一个标称周期
 * 输入：无
 * 输出：无
 *
 * 用于 POSITION 命令强制外环立即执行的场合：避免距离上一次 10 Hz 外环很近时
 * 用很小的 dt 计算位置 PD 的微分项。
 */
static void PIDTask_ResetOuterTimer(void)
{
  uint32_t now;
  uint32_t cycles_per_ms;

  if ((DWTTime_IsReady() == 0U) || (SystemCoreClock == 0U))
  {
    s_pid_last_outer_cycles = 0U;
    return;
  }

  now = DWTTime_GetCycles();
  cycles_per_ms = SystemCoreClock / 1000U;
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
  uint8_t force_outer = s_force_position_outer;

  s_force_position_outer = 0U;

  /*
   * 外环 10 Hz：通常每 25 个基础 tick 执行一次；
   * 如果本拍刚收到 POSITION 命令，则 force_outer 为 1，无条件立即执行一次。
   */
  if ((force_outer != 0U) || ((tick % PID_OUTER_TICK_DIV) == 0U))
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
 * 功能：姿态模式调度：只运行内环 250 Hz，方便直接给目标 Pitch/yaw rate 调参
 * 输入：无（目标来自 Controlqueue 模式 11）
 * 输出：无（更新左右 PWM）
 */
void PIDTask_StateAttitude(void)
{
  s_middle_target_pitch = PID_Clamp(s_pid_input.target_pitch,
                                    PID_ATT_TARGET_PITCH_MAX_DEG);
  s_middle_target_yaw_rate = PID_Clamp(s_pid_input.target_yaw_rate,
                                       PID_YAW_MAX_RATE_DPS);

  PIDTask_AttitudeInnerLoop();
}

/*
 * 功能：位置外环（10 Hz）：共模位置 PD -> 目标直行速度
 * 输入：无（从 s_pid_input 读取目标距离与编码器共模位置）
 * 输出：无（更新 s_outer_target_speed）
 */
void PIDTask_PositionOuterLoop(void)
{
  uint32_t now = PIDTask_GetTimestamp();
  float dt = PIDTask_CalculateDt(now, &s_pid_last_outer_cycles,
                                 PID_OUTER_DT_S, PID_OUTER_DT_MAX_S);

  s_position_params.encoder_current = (int32_t)s_pid_input.current_position;

  PositionController_Update(&s_position_params,
                            s_pid_input.target_distance,
                            dt,
                            &s_outer_target_speed);
}

/*
 * 功能：位置模式中环（50 Hz）：共模速度 PD -> 目标倾角；
 *       目标 yaw 角 PD -> 目标 yaw rate（高频角度环放在中环）
 * 输入：无（使用 s_outer_target_speed、目标 yaw 与 IMU yaw 相对角）
 * 输出：无（更新 s_middle_target_pitch / s_middle_target_yaw_rate）
 */
void PIDTask_PositionMiddleLoop(void)
{
  PIDTask_UpdateEncoderFeedback();

  uint32_t now = PIDTask_GetTimestamp();
  float dt = PIDTask_CalculateDt(now, &s_pid_last_middle_cycles,
                                 PID_MIDDLE_DT_S, PID_MIDDLE_DT_MAX_S);

  SpeedController_Update(&s_speed_params,
                         s_outer_target_speed,
                         s_pid_input.current_speed,
                         dt,
                         &s_middle_target_pitch);

  YawAngleController_Update(&s_yaw_angle_params,
                            s_pid_input.target_yaw_angle,
                            PIDTask_GetYawRelative(),
                            dt,
                            &s_middle_target_yaw_rate);
}

/*
 * 功能：速度模式中环（50 Hz）：共模速度 PD -> 目标倾角；value2 直接作为目标 yaw rate
 * 输入：无（目标 yaw rate 已在 ProcessControlQueue 中按 1°/s 换算并限幅）
 * 输出：无（更新 s_middle_target_pitch / s_middle_target_yaw_rate）
 *
 * 速度模式不使用 yaw 零点，也不做 yaw 角度环。
 */
void PIDTask_SpeedMiddleLoop(void)
{
  PIDTask_UpdateEncoderFeedback();

  uint32_t now = PIDTask_GetTimestamp();
  float dt = PIDTask_CalculateDt(now, &s_pid_last_middle_cycles,
                                 PID_MIDDLE_DT_S, PID_MIDDLE_DT_MAX_S);

  SpeedController_Update(&s_speed_params,
                         s_pid_input.target_speed,
                         s_pid_input.current_speed,
                         dt,
                         &s_middle_target_pitch);

  s_middle_target_yaw_rate = PID_Clamp(s_pid_input.target_yaw_rate,
                                       PID_YAW_MAX_RATE_DPS);
}

/*
 * 功能：内环（250 Hz）：姿态 PD + yaw rate PI -> 左右 PWM 输出
 * 输入：无（使用中环保持的目标倾角/目标 yaw rate，以及 s_pid_input 的反馈）
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
                            s_middle_target_yaw_rate,
                            s_pid_input.current_yaw_rate,
                            dt,
                            &pwm_left,
                            &pwm_right);

  PIDTask_WriteOutput(pwm_left, pwm_right);
}

/*
 * 功能：STOP 状态处理，输出关闭；Controlqueue 由任务主循环按 50 Hz 轮询
 * 输入：无
 * 输出：无
 */
void PIDTask_StateStop(void)
{
  PIDTask_WriteOutput(0.0f, 0.0f);
}

/*
 * 功能：检查 Controlqueue，读取新命令并设置下一状态与目标值；
 *       由 PIDTask_UpdateControlQueue() 按 50 Hz 调用
 * 输入：无
 * 输出：无（更新 s_pid_state 与 s_pid_input）
 */
void PIDTask_ProcessControlQueue(void)
{
  uint64_t message;
  uint8_t mode;
  int16_t value1;
  int16_t value2;
  float yaw_delta_deg;

  if (ControlqueueHandle == NULL)
  {
    return;
  }

  /* 在首帧有效 IMU 数据之前不要锁存 yaw 零点 */
  if (IMU_IsReady() == 0U)
  {
    return;
  }

  if (osMessageQueueGetCount(ControlqueueHandle) == 0U)
  {
    /*
     * 队列为空时直接返回，但保留 s_pid_state、目标值和 encoder_start。
     * 因此收到一次 POSITION 命令后会持续执行位置控制（外环 10 Hz、中环 50 Hz、
     * 内环 250 Hz），直到新的 STOP/SPEED/ATTITUDE/POSITION 命令到来。
     */
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

  /* value2: 相对当前角度的转向增量，单位 0.1°，逆时针为正 */
  yaw_delta_deg = (float)value2 * PID_CONTROL_YAW_UNIT_DEG;

  switch (mode)
  {
    case CONTROL_MODE_STOP:
    default:
      s_position_params.encoder_current = (int32_t)s_pid_input.current_position;
      PositionController_Start();
      SpeedController_Reset();
      AttitudeController_Reset();
      YawAngleController_Reset();

      s_outer_target_speed = 0.0f;
      s_middle_target_pitch = 0.0f;
      s_middle_target_yaw_rate = 0.0f;
      s_pid_input.target_pitch = 0.0f;
      s_pid_input.target_yaw_angle = 0.0f;
      s_pid_input.target_yaw_rate = 0.0f;
      s_yaw_zero_deg = 0.0f;
      s_force_position_outer = 0U;

      PIDTask_ResetLoopCounter();
      PIDTask_ResetLoopTimers();
      s_pid_state = PID_STATE_STOP;
      break;

    case CONTROL_MODE_POSITION:
      /* value1 = 共模位移脉冲；value2 = 相对位置模式零点的目标角（0.1°） */
      s_pid_input.target_distance = (float)value1;

      if (s_pid_state != PID_STATE_POSITION)
      {
        /* 第一次进入位置模式：以当前九轴 yaw 为零点 */
        PIDTask_ResetYawReference();
      }

      /*
       * 同一位置模式内：新命令直接覆盖目标角，零点保持第一次进入时的值。
       * 例：当前目标 30°，新命令 90° -> 目标就是相对首次零点的 90°，
       * 不是 30°+90°，也不是当前实测+90°。
       */
      s_pid_input.target_yaw_angle = yaw_delta_deg;

      PIDTask_StartPositionMode();
      break;

    case CONTROL_MODE_ATTITUDE:
      /* 姿态模式：value1 = 目标 Pitch（0.1°），value2 = 目标 yaw rate（1°/s） */
      s_pid_input.target_pitch =
          (float)value1 * PID_CONTROL_PITCH_UNIT_DEG;
      s_pid_input.target_yaw_rate =
          (float)value2 * PID_CONTROL_YAW_RATE_UNIT_DPS;

      if (s_pid_state != PID_STATE_ATTITUDE)
      {
        AttitudeController_Reset();
        PIDTask_ResetLoopTimers();
      }

      s_middle_target_pitch = PID_Clamp(s_pid_input.target_pitch,
                                        PID_ATT_TARGET_PITCH_MAX_DEG);
      s_middle_target_yaw_rate = PID_Clamp(s_pid_input.target_yaw_rate,
                                           PID_YAW_MAX_RATE_DPS);
      s_force_position_outer = 0U;
      s_pid_state = PID_STATE_ATTITUDE;
      break;

    case CONTROL_MODE_SPEED:
      s_pid_input.target_speed = (float)value1;

      /* 速度模式：value2 = 目标 yaw rate，单位 1°/s；不使用 yaw 零点 */
      s_pid_input.target_yaw_rate =
          (float)value2 * PID_CONTROL_YAW_RATE_UNIT_DPS;

      if (s_pid_state != PID_STATE_SPEED)
      {
        SpeedController_Reset();
        AttitudeController_Reset();

        s_middle_target_pitch = 0.0f;

        PIDTask_ResetLoopCounter();
        PIDTask_ResetLoopTimers();
      }

      s_middle_target_yaw_rate = PID_Clamp(s_pid_input.target_yaw_rate,
                                           PID_YAW_MAX_RATE_DPS);
      s_force_position_outer = 0U;
      s_pid_state = PID_STATE_SPEED;
      break;
  }
}

/*
 * 功能：进入/更新位置控制模式（锁存共模编码器起点；当拍强制位置外环；
 *       从其他模式切入时复位中环/内环、调度计数器和 dt 时间戳；
 *       yaw 零点由 ProcessControlQueue 处理）
 * 输入：无
 * 输出：无（切换到 PID_STATE_POSITION）
 */
void PIDTask_StartPositionMode(void)
{
  /* 进入/更新位置模式前刷新一次编码器，避免用旧共模位置锁存起点 */
  PIDTask_UpdateEncoderFeedback();

  s_position_params.encoder_current = (int32_t)s_pid_input.current_position;

  if (s_pid_state != PID_STATE_POSITION)
  {
    SpeedController_Reset();
    AttitudeController_Reset();

    s_outer_target_speed = 0.0f;
    s_middle_target_pitch = 0.0f;
    s_middle_target_yaw_rate = 0.0f;

    PIDTask_ResetLoopCounter();
    PIDTask_ResetLoopTimers();
  }

  PositionController_Start();
  PIDTask_ResetOuterTimer();
  s_force_position_outer = 1U;
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

  /* 保存限幅后的有符号 PWM 供 LCD 观察 */
  s_pid_output_left = PIDTask_ToInt16(PID_Clamp(pwm_left, PID_ATT_MAX_PWM));
  s_pid_output_right = PIDTask_ToInt16(PID_Clamp(pwm_right, PID_ATT_MAX_PWM));

  SETPWM(1U, forward_left, compare_left);   /* 左轮 */
  SETPWM(0U, forward_right, compare_right); /* 右轮 */
}


/*
 * 功能：浮点四舍五入到 int32
 * 输入：value —— 浮点值
 * 输出：四舍五入后的 int32
 */
static int32_t PIDTask_RoundToInt32(float value)
{
  return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

/*
 * 功能：浮点四舍五入并饱和到 int16
 * 输入：value —— 浮点值
 * 输出：饱和后的 int16
 */
static int16_t PIDTask_ToInt16(float value)
{
  int32_t result = PIDTask_RoundToInt32(value);

  if (result > 32767)
  {
    result = 32767;
  }
  else if (result < -32768)
  {
    result = -32768;
  }

  return (int16_t)result;
}

/*
 * 功能：记录一次 PID 250 Hz 循环的 DWT 耗时
 * 输入：start_cycles —— 循环开始时的 DWT CYCCNT
 * 输出：无
 */
static void PIDTask_UpdateLoopTime(uint32_t start_cycles)
{
  uint32_t elapsed_cycles;
  uint32_t elapsed_us;

  if (DWTTime_IsReady() == 0U)
  {
    return;
  }

  elapsed_cycles = DWTTime_GetCycles() - start_cycles;
  elapsed_us = elapsed_cycles / (SystemCoreClock / 1000000U);
  if (elapsed_us > 65535U)
  {
    elapsed_us = 65535U;
  }

  s_pid_loop_time_us = (uint16_t)elapsed_us;
}

/*
 * 功能：按 20 Hz 生成 OLED 调试快照并按值投递到 dataToLCDQueue
 * 输入：无
 * 输出：无（队列项为 68 字节完整 LCD_Data_t）
 */
static void PIDTask_UpdateDisplayQueue(void)
{
  LCD_Data_t *snapshot;
  uint32_t now_ms;
  uint32_t imu_age_ms;
  float target_speed;
  float target_pitch;
  float target_yaw;
  float current_yaw;
  uint8_t flags;

  /*
   * 250 Hz 基础循环累加 20，每满 250 发送一次：
   * 平均 20 Hz，且不会因为 250/20 不是整数而产生长期漂移。
   */
  s_display_div_accum += PID_DISPLAY_UPDATE_HZ;
  if (s_display_div_accum < PID_INNER_LOOP_HZ)
  {
    return;
  }
  s_display_div_accum -= PID_INNER_LOOP_HZ;

  if (dataToLCDQueueHandle == NULL)
  {
    return;
  }

  /* CubeMX 队列元素必须是 68 字节，否则 osMessageQueueGet 会写越界 */
  if (osMessageQueueGetMsgSize(dataToLCDQueueHandle) !=
      (uint32_t)sizeof(LCD_Data_t))
  {
    return;
  }

  snapshot = &s_lcd_snapshot;
  snapshot->sequence = ++s_lcd_sequence;

  switch (s_pid_state)
  {
    case PID_STATE_POSITION:
      target_speed = s_outer_target_speed;
      target_pitch = s_middle_target_pitch;
      target_yaw = s_pid_input.target_yaw_angle;
      current_yaw = PIDTask_GetYawRelative();
      break;

    case PID_STATE_SPEED:
      target_speed = s_pid_input.target_speed;
      target_pitch = s_middle_target_pitch;
      target_yaw = 0.0f;
      current_yaw = s_pid_input.current_yaw_angle;
      break;

    case PID_STATE_ATTITUDE:
      target_speed = 0.0f;
      target_pitch = s_middle_target_pitch;
      target_yaw = 0.0f;
      current_yaw = s_pid_input.current_yaw_angle;
      break;

    case PID_STATE_STOP:
    default:
      target_speed = 0.0f;
      target_pitch = 0.0f;
      target_yaw = 0.0f;
      current_yaw = s_pid_input.current_yaw_angle;
      break;
  }

  now_ms = HAL_GetTick();

  if ((s_imu_data_valid != 0U) && (now_ms >= s_imu_last_timestamp_ms))
  {
    imu_age_ms = now_ms - s_imu_last_timestamp_ms;
  }
  else
  {
    imu_age_ms = 0xFFFFFFFFU;
  }

  flags = 0U;
  if (IMU_IsReady() != 0U)
  {
    flags |= LCD_FLAG_IMU_READY;
  }
  if (s_imu_data_valid != 0U)
  {
    flags |= LCD_FLAG_IMU_VALID;
  }
  if ((imu_age_ms == 0xFFFFFFFFU) || (imu_age_ms > LCD_IMU_STALE_MS))
  {
    flags |= LCD_FLAG_IMU_STALE;
  }
  if (s_display_queue_drop_count != 0U)
  {
    flags |= LCD_FLAG_QUEUE_DROP;
  }

  snapshot->timestamp_ms = now_ms;
  snapshot->imu_age_ms = imu_age_ms;
  snapshot->imu_error_count = IMU_GetErrorCount();
  snapshot->queue_drop_count = s_display_queue_drop_count;

  snapshot->target_distance = PIDTask_RoundToInt32(s_pid_input.target_distance);
  snapshot->current_distance = PIDTask_RoundToInt32(
      s_pid_input.current_position - (float)s_position_params.encoder_start);
  snapshot->target_speed = PIDTask_RoundToInt32(target_speed);
  snapshot->current_speed = PIDTask_RoundToInt32(s_pid_input.current_speed);
  snapshot->encoder_left = Encoder_GetLeftTotal();
  snapshot->encoder_right = Encoder_GetRightTotal();

  snapshot->target_pitch_0p1deg = PIDTask_ToInt16(target_pitch * 10.0f);
  snapshot->current_pitch_0p1deg =
      PIDTask_ToInt16(s_pid_input.current_pitch * 10.0f);
  snapshot->target_yaw_0p1deg = PIDTask_ToInt16(target_yaw * 10.0f);
  snapshot->current_yaw_0p1deg = PIDTask_ToInt16(current_yaw * 10.0f);
  snapshot->target_yaw_rate_0p1dps =
      PIDTask_ToInt16(s_middle_target_yaw_rate * 10.0f);
  snapshot->current_yaw_rate_0p1dps =
      PIDTask_ToInt16(s_pid_input.current_yaw_rate * 10.0f);
  snapshot->current_pitch_rate_0p1dps =
      PIDTask_ToInt16(s_pid_input.current_pitch_rate * 10.0f);
  snapshot->output_left = s_pid_output_left;
  snapshot->output_right = s_pid_output_right;

  snapshot->loop_time_us = s_pid_loop_time_us;
  snapshot->mode = (uint8_t)s_pid_state;
  snapshot->flags = flags;
  snapshot->imu_ready = IMU_IsReady();
  snapshot->imu_valid = s_imu_data_valid;

  if (osMessageQueuePut(dataToLCDQueueHandle, &s_lcd_snapshot, 0U, 0U) != osOK)
  {
    s_display_queue_drop_count++;
  }
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
 * 功能：位置外环（共模位置 PD -> 目标直行速度）
 * 输入：params —— 位置控制器参数；
 *       target_distance —— 目标共模位移（编码器脉冲）；
 *       dt —— 控制周期（s）
 * 输出：target_speed —— 目标速度（脉冲/s）
 *
 * 说明：current_distance = encoder_current - encoder_start；
 *       只使用编码器共模位置，不再使用差模量。
 */
void PositionController_Update(const PositionControllerParams_t *params,
                               float target_distance,
                               float dt,
                               float *target_speed)
{
  float current_distance = (float)(params->encoder_current - params->encoder_start);
  float e_distance = target_distance - current_distance;
  float d_distance = 0.0f;

  if (dt > 0.0f)
  {
    d_distance = (e_distance - s_position_last_distance_error) / dt;
  }

  *target_speed = params->kp_distance * e_distance
                  + params->kd_distance * d_distance;
  *target_speed = PID_Clamp(*target_speed, params->max_speed);

  s_position_last_distance_error = e_distance;
}

/*
 * 功能：进入位置模式时锁存共模起始编码器值，并清零微分历史
 * 输入：无（使用 s_position_params.encoder_current）
 * 输出：无
 */
void PositionController_Start(void)
{
  s_position_params.encoder_start = s_position_params.encoder_current;
  s_position_last_distance_error = 0.0f;
}

/*
 * 功能：中环共模速度控制器（速度 PD -> 目标倾角）
 * 输入：params —— 速度控制器参数；
 *       target_speed / current_speed —— 目标/当前共模速度（脉冲/s）；
 *       dt —— 控制周期（s）
 * 输出：target_pitch —— 目标倾角（deg）
 *
 * 说明：只处理共模/直行速度；yaw 角度由 YawAngleController_Update() 单独处理。
 */
void SpeedController_Update(const SpeedControllerParams_t *params,
                            float target_speed, float current_speed,
                            float dt,
                            float *target_pitch)
{
  float e_speed = target_speed - current_speed;
  float d_speed = 0.0f;

  if (dt > 0.0f)
  {
    d_speed = (e_speed - s_speed_last_speed_error) / dt;
  }

  *target_pitch = params->kp_speed * e_speed + params->kd_speed * d_speed;
  *target_pitch = PID_Clamp(*target_pitch, params->max_pitch);

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
 * 功能：把角度归一化到 [-180, 180)
 * 输入：angle —— 任意角度，deg
 * 输出：归一化后的角度，deg
 */
static float PIDTask_Wrap180(float angle)
{
  float wrapped = fmodf(angle + 180.0f, 360.0f);

  if (wrapped < 0.0f)
  {
    wrapped += 360.0f;
  }

  return wrapped - 180.0f;
}

/*
 * 功能：锁存 yaw 相对角零点（进入位置/速度模式时调用）
 * 输入：无
 * 输出：无
 */
static void PIDTask_ResetYawReference(void)
{
  s_yaw_zero_deg = s_pid_input.current_yaw_angle;
  s_pid_input.target_yaw_angle = 0.0f;
  YawAngleController_Reset();
}

/*
 * 功能：返回相对零点的当前 yaw 角
 * 输入：无
 * 输出：yaw 相对角，deg，范围 [-180, 180)
 */
static float PIDTask_GetYawRelative(void)
{
  return PIDTask_Wrap180(s_pid_input.current_yaw_angle - s_yaw_zero_deg);
}

/*
 * 功能：yaw 角度环（角度 PD -> 目标 yaw rate，并限幅）
 * 输入：params —— yaw 角度控制器参数；
 *       target_yaw_angle / current_yaw_angle —— 目标/当前 yaw 角（deg）；
 *       dt —— 控制周期（s）
 * 输出：target_yaw_rate —— 目标 yaw rate（deg/s）
 */
void YawAngleController_Update(const YawAngleControllerParams_t *params,
                               float target_yaw_angle,
                               float current_yaw_angle,
                               float dt,
                               float *target_yaw_rate)
{
  float e_yaw = PIDTask_Wrap180(target_yaw_angle - current_yaw_angle);
  float d_yaw = 0.0f;

  if (dt > 0.0f)
  {
    d_yaw = (e_yaw - s_yaw_angle_last_error) / dt;
  }

  *target_yaw_rate = params->kp_angle * e_yaw
                     + params->kd_angle * d_yaw;
  *target_yaw_rate = PID_Clamp(*target_yaw_rate, params->max_rate_dps);

  s_yaw_angle_last_error = e_yaw;
}

/*
 * 功能：yaw 角度控制器复位（清零微分历史）
 * 输入：无
 * 输出：无
 */
void YawAngleController_Reset(void)
{
  s_yaw_angle_last_error = 0.0f;
}

/*
 * 功能：姿态/yaw rate 内环计算（倾角 PD -> 共模 PWM；yaw rate PI -> 差模 PWM）
 * 输入：params —— 姿态控制器参数；
 *       target_pitch / current_pitch —— 目标/当前倾角（deg）；
 *       pitch_rate —— 当前俯仰角速度（deg/s）；
 *       target_yaw_rate / current_yaw_rate —— 目标/当前 yaw 角速度（deg/s）；
 *       dt —— 控制周期（s）
 * 输出：pwm_left / pwm_right —— 左右电机 PWM
 */
void AttitudeController_Update(const AttitudeControllerParams_t *params,
                                      float target_pitch, float current_pitch,
                                      float pitch_rate,
                                      float target_yaw_rate, float current_yaw_rate,
                                      float dt,
                                      float *pwm_left, float *pwm_right)
{
  float e_pitch = current_pitch - target_pitch;
  float e_yaw_rate = target_yaw_rate - current_yaw_rate;
  float pwm_common;
  float pwm_diff;

  pwm_common = params->kp_pitch * e_pitch + params->kd_pitch * pitch_rate;
  pwm_common = PID_Clamp(pwm_common, params->max_pwm);

  if (dt > 0.0f)
  {
    s_yaw_rate_integral += e_yaw_rate * dt;
    s_yaw_rate_integral = PID_Clamp(s_yaw_rate_integral,
                                    params->max_yaw_integral);
  }

  pwm_diff = params->kp_yaw_rate * e_yaw_rate
             + params->ki_yaw_rate * s_yaw_rate_integral;
  pwm_diff = PID_Clamp(pwm_diff, params->max_diff_pwm);

  *pwm_left = PID_Clamp(pwm_common - pwm_diff, params->max_pwm);
  *pwm_right = PID_Clamp(pwm_common + pwm_diff, params->max_pwm);
}

/*
 * 功能：姿态/yaw rate 内环复位（清零积分）
 * 输入：无
 * 输出：无
 */
void AttitudeController_Reset(void)
{
  s_yaw_rate_integral = 0.0f;
}

/*
 * 功能：设置单侧电机 PWM 方向与比较值
 * 输入：left_wheel —— 1 = 左轮，0 = 右轮；
 *       forward —— 1 = 正转，0 = 反转；
 *       compare —— PID 输出比较值，0 ~ PID_ATT_MAX_PWM（1000）
 * 输出：无
 *
 * 电机驱动逻辑：
 *   AIN1=1, AIN2=0, PWM -> 正转
 *   AIN1=0, AIN2=1, PWM -> 反转
 *   AIN1=0, AIN2=0       -> 短路刹车（compare=0 时使用）
 *   AIN1=1, AIN2=1       -> 自由停车（当前未使用）
 *
 * 引脚：
 *   左轮：LF=PD11 -> AIN1，LB=PD10 -> AIN2，LPWM=PA0/TIM2_CH1
 *   右轮：RF=PA4  -> AIN1，RB=PA5  -> AIN2，RPWM=PA1/TIM2_CH2
 */
void SETPWM(uint8_t left_wheel, uint8_t forward, uint16_t compare)
{
  static uint8_t s_pwm_started = 0U;

  GPIO_TypeDef *ain1_port;
  GPIO_TypeDef *ain2_port;
  uint16_t ain1_pin;
  uint16_t ain2_pin;
  uint32_t channel;
  uint32_t arr;
  uint32_t compare_max;
  uint32_t ccr;

  if (left_wheel != 0U)
  {
    ain1_port = LF_GPIO_Port;
    ain1_pin = LF_Pin;
    ain2_port = LB_GPIO_Port;
    ain2_pin = LB_Pin;
    channel = TIM_CHANNEL_1;
  }
  else
  {
    ain1_port = RF_GPIO_Port;
    ain1_pin = RF_Pin;
    ain2_port = RB_GPIO_Port;
    ain2_pin = RB_Pin;
    channel = TIM_CHANNEL_2;
  }

  /*
   * TIM2 的 ARR 必须是合理的 PWM 周期。CubeMX 当前若仍是默认
   * Period=4294967295，则 PWM 频率约 0.02 Hz，禁止启动，保持刹车。
   * 推荐在 CubeMX 中设置 20 kHz：Prescaler=0、Period=4199。
   */
  arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
  if ((arr == 0U) || (arr > 100000U))
  {
    HAL_GPIO_WritePin(ain1_port, ain1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ain2_port, ain2_pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, channel, 0U);
    return;
  }

  if (s_pwm_started == 0U)
  {
    if ((HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK))
    {
      HAL_GPIO_WritePin(ain1_port, ain1_pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ain2_port, ain2_pin, GPIO_PIN_RESET);
      return;
    }
    s_pwm_started = 1U;
  }

  /* compare=0：AIN1=AIN2=0，短路刹车 */
  if (compare == 0U)
  {
    HAL_GPIO_WritePin(ain1_port, ain1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ain2_port, ain2_pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, channel, 0U);
    return;
  }

  if (forward != 0U)
  {
    HAL_GPIO_WritePin(ain1_port, ain1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ain2_port, ain2_pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(ain1_port, ain1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ain2_port, ain2_pin, GPIO_PIN_SET);
  }

  compare_max = (uint32_t)PID_ATT_MAX_PWM;
  if (compare > compare_max)
  {
    compare = (uint16_t)compare_max;
  }

  ccr = ((uint32_t)compare * arr) / compare_max;
  if (ccr > arr)
  {
    ccr = arr;
  }

  __HAL_TIM_SET_COMPARE(&htim2, channel, ccr);
}

/************************ (C) COPYRIGHT *****END OF FILE***********************/

