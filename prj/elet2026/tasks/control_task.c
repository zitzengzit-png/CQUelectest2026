/**
  ******************************************************************************
  * @file    control_task.c
  * @brief   主控任务：调参/原地停止/路径/巡线对齐状态机与 Controlqueue 命令发布。
  ******************************************************************************
  * @note    状态机 1：调参模式（CONTROL_STATE_TUNING），currentmode 0~5：
  *            0 速度控制   -> SPEED     value1 = 速度(mm/s)    value2 = 0
  *            1 位置控制   -> POSITION  value1 = 位移(mm)      value2 = 0
  *            2 倾角控制   -> ATTITUDE  value1 = Pitch(0.1°)   value2 = 0
  *            3 角度控制   -> POSITION  value1 = 0(不前进)     value2 = yaw(0.1°)
  *            4 角速度控制 -> SPEED     value1 = 0(不前进)     value2 = yaw rate(1°/s)
  *            5 停止       -> STOP      value1 = 0             value2 = 0
  *
  *            KEY0（PE4）下降沿 EXTI 回调里只把 currentmode 加 1 并在 0~5
  *            循环；controlTask 识别 currentmode 后进入对应模式并只发布一次。
  *
  *          状态机 2：原地停止模式（CONTROL_STATE_HOLD）：
  *            进入时只发布一次 POSITION 命令，value1=0（不前进），
  *            value2=0（不转向），保持当前位置和航向。
  *
  *          状态机 3：路径测试模式（CONTROL_STATE_RUN）：
  *            进入时发布一次 POSITION(1100 mm, 0)，向前行驶 110 cm；
  *            行驶期间每 20 ms 轮询 F1~F8 并保存原始电平和掩码；
  *            任意一路检测到黑线 -> 立即发布 POSITION(0, 0) 原地停车，
  *            并用 FreeRTOS 单次软件定时器让蜂鸣器响 1 s；
  *            响期间频率从 2500 Hz 开始，每个 controlTask 周期减 50 Hz
  *            （PSC=51，保持 50% 占空比），形成频率扫描。
  *
  *          状态机 4：巡线对齐模式（CONTROL_STATE_SM4）：
  *            直行 110 cm -> 首次压线巡线 -> 前轮丢线后退 5 cm ->
  *            前后加权中心对齐 -> 再直行 110 cm -> 再次巡线 ->
  *            第二次前轮丢线停车（POSITION 0/0）。
  *            每次前轮黑线出现/消失都触发 1 s 蜂鸣器频率扫描。
  *            巡线用前后加权中心的共模/差模偏差做转向 P 和速度反比例 P；
  *            对齐用前后中心差做 P，连续 10 帧在容差内认为对齐。
  *
  *          全局 currentstatemachine 选择状态机：
  *            0 IDLE / 1 调参 / 2 原地停止 / 3 路径测试 / 4 巡线对齐 / 5 STOP
  *            （0 IDLE、5 STOP 暂未实现）
  *          KEY_UP（PA0）下降沿 EXTI 按 1 -> 2 -> 3 -> 4 -> 1 循环切换状态机，
  *          同时把 currentmode 复位为 0（速度）。上电默认 2（原地停止）。
  *
  *          后续在这里实现：完整 A/B/C/D 路径、到点提示和计时。
  ******************************************************************************
  */

/* 头文件包含 ----------------------------------------------------------------*/
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "tim.h"
#include "cmsis_os.h"
#include "control_task.h"
#include "pid_task.h"
#include "line_sensor.h"

/* 调参模式各子模式的目标值（单位与 Controlqueue 编码一致） ----------------*/
/*
 * 调参时直接修改这里的值并重新编译；KEY0 每次进入对应模式时按这些值
 * 发布一次命令。默认值均可安全工作在架空/场地测试。
 */
#define CONTROL_TUNE_SPEED_MM_S          100   /* 速度模式：+100 mm/s */
#define CONTROL_TUNE_POSITION_MM         500   /* 位置模式：向前 500 mm */
#define CONTROL_TUNE_PITCH_0P1DEG        50    /* 倾角模式：+5.0° */
#define CONTROL_TUNE_YAW_ANGLE_0P1DEG    900   /* 角度模式：+90.0° */
#define CONTROL_TUNE_YAW_RATE_DPS        30    /* 角速度模式：+30°/s */

/* 状态机 3：路径测试参数 */
#define CONTROL_RUN_FORWARD_MM           1100  /* 位置模式向前 110 cm */
#define CONTROL_RUN_BUZZER_MS            1000U /* 蜂鸣器响 1 s */

/* 蜂鸣器频率扫描：开始响后每个 controlTask 周期（20 ms）频率减 50 Hz */
#define CONTROL_BUZZER_START_FREQ_HZ     2500U /* 起始频率 2500 Hz */
#define CONTROL_BUZZER_FREQ_STEP_HZ      50U   /* 每个 20 ms 周期减小 50 Hz */
#define CONTROL_BUZZER_MIN_FREQ_HZ       50U   /* 最低频率 50 Hz */

/*
 * TIM11 时钟 = APB2 定时器时钟 168 MHz。16 位 ARR 在 PSC=0 时最低只能到
 * 2563 Hz，因此这里把 PSC 设为 51：
 *   TIM11 计数时钟 = 168 MHz / (51 + 1) = 3.230769 MHz
 *   2500 Hz -> ARR = 1291
 *     50 Hz -> ARR = 64614（仍在 16 位范围内）
 * 这样 2500 -> 50 Hz 的整个扫描范围都能用 16 位 ARR 表示。
 */
#define CONTROL_BUZZER_TIM_PRESCALER     51U
#define CONTROL_BUZZER_TIMER_CLOCK_HZ    \
  (168000000UL / (CONTROL_BUZZER_TIM_PRESCALER + 1U))

/* ========================================================================== */
/* 状态机 4：直行 + 巡线 + 后退对齐                                          */
/* ========================================================================== */

#define CONTROL_SM4_FORWARD_MM            1100   /* 每段直行 110 cm */
#define CONTROL_SM4_REVERSE_MM            (-50)  /* 前轮丢线后后退 5 cm */
#define CONTROL_SM4_REVERSE_WAIT_MS       600U   /* 后退命令等待时间，按实车调 */

/*
 * 8 路光电加权中心：下标 0..7 对应 x1..x8，按传感器排中心为 0 计算位置。
 * 如果某一排传感器安装方向相反，把对应 POS_SIGN 改为 -1。
 */
#define CONTROL_LF_FRONT_POS_SIGN         (+1.0f)
#define CONTROL_LF_REAR_POS_SIGN          (+1.0f)
#define CONTROL_LF_STEER_SIGN             (-1.0f) /* 实车转向反了改 +1 */
#define CONTROL_LF_COMMON_TARGET_MM       0.0f
#define CONTROL_LF_DIFF_TARGET_MM         0.0f
#define CONTROL_LF_STEER_DIFF_WEIGHT      1.0f
#define CONTROL_LF_SPEED_DIFF_WEIGHT      1.0f

/*
 * 巡线 P 控制：
 *   转向角速度 ∝ 综合偏差；
 *   前进速度 = 基础速度 - K * 综合偏差（反比例），并限幅。
 * 参数是初始值，实车再整定。
 */
#define CONTROL_LF_BASE_SPEED_MM_S        400.0f
#define CONTROL_LF_MIN_SPEED_MM_S         80.0f
#define CONTROL_LF_MAX_SPEED_MM_S         400.0f
#define CONTROL_LF_SPEED_KP               8.0f   /* mm/s per mm */
#define CONTROL_LF_YAW_KP                 6.0f   /* deg/s per mm */
#define CONTROL_LF_MAX_RATE_DPS           90.0f

/*
 * 对齐 P 控制：前后加权中心差 - 目标差 -> 目标 yaw rate。
 * 目标差需要按前后传感器排的机械安装位置实测标定。
 */
#define CONTROL_ALIGN_TARGET_DIFF_MM      0.0f
#define CONTROL_ALIGN_KP                  8.0f   /* deg/s per mm */
#define CONTROL_ALIGN_MAX_RATE_DPS        60.0f
#define CONTROL_ALIGN_STEER_SIGN          (-1.0f)
#define CONTROL_ALIGN_TOLERANCE_MM        2.0f
#define CONTROL_ALIGN_SETTLE_FRAMES       10U
#define CONTROL_ALIGN_TIMEOUT_MS          5000U

/* KEY0 / KEY_UP 下降沿中断消抖：200 ms 内的重复边沿忽略 */
#define CONTROL_KEY_DEBOUNCE_MS          200U

/* 私有类型定义 --------------------------------------------------------------*/
typedef enum
{
  CONTROL_STATE_IDLE = 0,  /* 空闲：等待启动条件 */
  CONTROL_STATE_TUNING,    /* 状态机 1：调参，按 currentmode 进入六个模式之一 */
  CONTROL_STATE_HOLD,      /* 状态机 2：原地停止，POSITION 0/0 只发一次 */
  CONTROL_STATE_RUN,       /* 状态机 3：前进 110cm + F1~F8 黑线停车 + 蜂鸣器 1s */
  CONTROL_STATE_SM4,       /* 状态机 4：直行 + 巡线 + 后退 5cm + 对齐 */
  CONTROL_STATE_STOP       /* 停止：输出停车命令并等待复位 */
} Control_State_t;

/* 调参模式六个模式 ----------------------------------------------------------*/
typedef enum
{
  CONTROL_TUNE_MODE_SPEED = 0,   /* 速度控制：SPEED，前进不转弯 */
  CONTROL_TUNE_MODE_POSITION,    /* 位置控制：POSITION，前进不转弯 */
  CONTROL_TUNE_MODE_PITCH,       /* 倾角控制：ATTITUDE，只给目标 Pitch */
  CONTROL_TUNE_MODE_YAW_ANGLE,   /* 角度控制：POSITION，不前进只转角度 */
  CONTROL_TUNE_MODE_YAW_RATE,    /* 角速度控制：SPEED，不前进只给 yaw rate */
  CONTROL_TUNE_MODE_STOP,        /* 停止：STOP */
  CONTROL_TUNE_MODE_COUNT
} Control_TuneMode_t;

/* 状态机 3 的内部阶段 --------------------------------------------------------*/
typedef enum
{
  CONTROL_RUN_STATE_IDLE = 0,   /* 尚未下发 110cm 前进命令 */
  CONTROL_RUN_STATE_DRIVING,    /* 已下发前进命令，正在检测 F1~F8 */
  CONTROL_RUN_STATE_STOPPED     /* 已下发原地停车命令，蜂鸣器计时中/已结束 */
} Control_RunState_t;

/* 状态机 4 的内部阶段 --------------------------------------------------------*/
typedef enum
{
  CONTROL_SM4_STRAIGHT_1 = 0,   /* 第一次直行 110 cm */
  CONTROL_SM4_FOLLOW_1,         /* 第一次巡线 */
  CONTROL_SM4_REVERSE,          /* 前轮丢线：后退 5 cm */
  CONTROL_SM4_ALIGN,            /* 前后光电中心对齐 */
  CONTROL_SM4_STRAIGHT_2,       /* 第二次直行 110 cm */
  CONTROL_SM4_FOLLOW_2,         /* 第二次巡线 */
  CONTROL_SM4_STOPPING,         /* 第二次丢线：POSITION(0,0) 停车 */
  CONTROL_SM4_DONE              /* 已完成 */
} Control_Sm4State_t;

/* 私有变量 ------------------------------------------------------------------*/
/*
 * 当前状态机选择，由 KEY_UP EXTI 按 1 -> 2 -> 3 -> 4 -> 1 循环切换，
 * 也可在调试器里直接修改：
 *   0 IDLE / 1 调参 / 2 原地停止 / 3 RUN / 4 巡线对齐 / 5 STOP
 *   （0 IDLE、5 STOP 暂未实现）
 */
volatile uint8_t currentstatemachine = CONTROL_STATE_HOLD;

/*
 * 调参状态机（状态机 1）当前模式，由 KEY0 EXTI 回调修改：
 *   0 速度 / 1 位置 / 2 倾角 / 3 角度 / 4 角速度 / 5 停止
 * 定义为全局 volatile，方便在调试器里直接观察/修改。
 */
volatile uint8_t currentmode = CONTROL_TUNE_MODE_STOP;

static uint32_t s_control_wake_tick = 0U;
static volatile uint32_t s_control_run_count = 0U;

/* 上一次执行的状态机；用于检测 currentstatemachine 变化并重置一次性发布标志 */
static uint8_t s_active_state = 0xFFU;

/* 最近一次成功进入的调参模式；0xFF 表示尚未进入任何模式 */
static uint8_t s_applied_mode = 0xFFU;

/* 原地停止命令是否已经发布成功 */
static uint8_t s_hold_applied = 0U;

/* 状态机 3 内部阶段、最近一次 F1~F8 掩码、触发黑线掩码和电平快照 */
static Control_RunState_t s_run_state = CONTROL_RUN_STATE_IDLE;
static uint8_t s_run_last_front_mask = 0U;
static uint8_t s_run_black_mask = 0U;
static uint8_t s_run_trigger_levels[LINE_SENSOR_COUNT];

/* 蜂鸣器 PWM（TIM11_CH1/PF7）与 FreeRTOS 单次软件定时器 */
static osTimerId_t s_buzzer_timer = NULL;
static volatile uint8_t s_buzzer_on = 0U;
static volatile uint32_t s_buzzer_frequency_hz = CONTROL_BUZZER_START_FREQ_HZ;

/* 状态机 4 内部状态 */
static Control_Sm4State_t s_sm4_state = CONTROL_SM4_STRAIGHT_1;
static uint8_t s_sm4_command_sent = 0U;
static uint8_t s_sm4_front_prev_mask = 0U;
static uint8_t s_sm4_wait_clear = 0U;
static uint8_t s_sm4_align_good_frames = 0U;
static uint32_t s_sm4_phase_tick = 0U;
static float s_sm4_align_error_mm = 0.0f;

/* KEY0 / KEY_UP EXTI 消抖时间戳（ms） */
static volatile uint32_t s_key0_last_tick = 0U;
static volatile uint32_t s_keyup_last_tick = 0U;

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
static void ControlTask_StateTuning(void);
static void ControlTask_StateHold(void);
static void ControlTask_StateRun(void);
static void ControlTask_StateSm4(void);
static void ControlTask_StateStop(void);
static void ControlTask_DelayUntil(void);

/* 状态机 4：巡线/对齐辅助 */
static void ControlTask_Sm4Reset(void);
static void ControlTask_Sm4EnterPhase(Control_Sm4State_t phase);
static void ControlTask_Sm4LineFollow(void);
static void ControlTask_Sm4Align(void);
static uint8_t ControlTask_LineWeightedCenter(uint8_t mask,
                                              float position_sign,
                                              float *center_mm);
static int16_t ControlTask_ToInt16(float value);

/* Controlqueue 命令发布 */
static uint8_t ControlTask_PublishCommand(uint8_t queue_mode,
                                          int16_t value1,
                                          int16_t value2);
static uint8_t ControlTask_PublishModeCommand(uint8_t tune_mode);
static uint8_t ControlTask_PublishHoldCommand(void);

/* 蜂鸣器 PWM 与 FreeRTOS 软件定时器 */
static void ControlTask_BuzzerStart(void);
static void ControlTask_BuzzerStop(void);
static void ControlTask_BuzzerCancel(void);
static void ControlTask_BuzzerBeepOneSecond(void);
static void ControlTask_BuzzerSetFrequency(uint32_t frequency_hz);
static void ControlTask_BuzzerTick(void);
static void ControlTask_BuzzerTimerCallback(void *argument);

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
    uint8_t state;

    s_control_run_count++;
    ControlTask_Update();

    state = currentstatemachine;

    /* 切换状态机时先关闭蜂鸣器，并重置“只发布一次”标志 */
    if (state != s_active_state)
    {
      ControlTask_BuzzerCancel();

      s_active_state = state;
      s_applied_mode = 0xFFU;
      s_hold_applied = 0U;
      s_run_state = CONTROL_RUN_STATE_IDLE;
      s_run_last_front_mask = 0U;
      s_run_black_mask = 0U;

      if (state == CONTROL_STATE_SM4)
      {
        ControlTask_Sm4Reset();
      }
    }

    switch (state)
    {
      case CONTROL_STATE_TUNING:
        ControlTask_StateTuning();
        break;

      case CONTROL_STATE_HOLD:
        ControlTask_StateHold();
        break;

      case CONTROL_STATE_RUN:
        ControlTask_StateRun();
        break;

      case CONTROL_STATE_SM4:
        ControlTask_StateSm4();
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

  /* 调试结束：上电进入状态机 2——原地停止（POSITION 0/0，只发一次） */
  currentstatemachine = CONTROL_STATE_HOLD;
  s_active_state = 0xFFU;
  s_hold_applied = 0U;

  /* 状态机 3 复位 */
  s_run_state = CONTROL_RUN_STATE_IDLE;
  s_run_last_front_mask = 0U;
  s_run_black_mask = 0U;

  /* 状态机 1 保留：currentmode=5(STOP)，第一次 KEY0 下降沿进入速度模式 */
  currentmode = CONTROL_TUNE_MODE_STOP;
  s_applied_mode = 0xFFU;
  s_key0_last_tick = HAL_GetTick();
  s_keyup_last_tick = HAL_GetTick();

  s_control_wake_tick = osKernelGetTickCount();

  for (i = 0U; i < LINE_SENSOR_COUNT; i++)
  {
    s_front_sensor_levels[i] = 0U;
    s_back_sensor_levels[i] = 0U;
    s_run_trigger_levels[i] = 0U;
  }

  s_front_line_mask = 0U;
  s_back_line_mask = 0U;

  /* 蜂鸣器 PWM（TIM11_CH1/PF7）先关闭 */
  s_buzzer_on = 0U;
  s_buzzer_frequency_hz = CONTROL_BUZZER_START_FREQ_HZ;

  /*
   * 2500 Hz 在 PSC=0 时超出 16 位 ARR 能表示的范围，先把 TIM11 分频
   * 设为 51；这样 2500~50 Hz 都能用 ARR 表示。
   */
  __HAL_TIM_SET_PRESCALER(&htim11, CONTROL_BUZZER_TIM_PRESCALER);

  /*
   * 运行中会不断改 ARR/CCR，必须打开预装载：
   * 新值在下一个更新事件生效，避免计数器大于新 ARR 时越界。
   */
  htim11.Instance->CR1 |= TIM_CR1_ARPE;
  htim11.Instance->CCMR1 |= TIM_CCMR1_OC1PE;

  /* 先写入起始频率，再用 UG 强制把 PSC/ARR/CCR 装载到活动寄存器 */
  ControlTask_BuzzerSetFrequency(CONTROL_BUZZER_START_FREQ_HZ);
  htim11.Instance->EGR = TIM_EGR_UG;
  __HAL_TIM_CLEAR_FLAG(&htim11, TIM_FLAG_UPDATE);

  /* 创建 1 s 单次软件定时器；失败时状态机 3 不会启动蜂鸣器 */
  s_buzzer_timer = osTimerNew(ControlTask_BuzzerTimerCallback,
                              osTimerOnce, NULL, NULL);

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

  /* 蜂鸣器响期间，每个 20 ms 周期把频率减小 50 Hz */
  ControlTask_BuzzerTick();

  /* 巡线电平在此周期刷新；KEY0 由 EXTI 回调更新 currentmode */
}

/*
 * 功能：空闲状态处理
 * 输入：无
 * 输出：无
 *
 * TODO: 等待一键启动；满足条件后切换到 CONTROL_STATE_TUNING 或 RUN。
 */
static void ControlTask_StateIdle(void)
{
}

/*
 * 功能：调参模式状态机：识别 currentmode 变化，进入对应模式并发布一次命令
 * 输入：无
 * 输出：无
 *
 * KEY0 由 CubeMX 配置为下降沿 EXTI，回调里只更新 currentmode：
 *   0 速度 -> 1 位置 -> 2 倾角 -> 3 角度 -> 4 角速度 -> 5 停止 -> 0 ...
 * 本函数每 20 ms 读取一次 currentmode，与 s_applied_mode 比较；变化时
 * 向 Controlqueue 发布一次对应命令，发布成功才记录新模式。
 * 队列深度为 1，若上一条尚未被 PID 取走则下个周期自动重试。
 */
static void ControlTask_StateTuning(void)
{
  uint8_t mode = currentmode;

  if (mode >= CONTROL_TUNE_MODE_COUNT)
  {
    mode = CONTROL_TUNE_MODE_STOP;
  }

  if (mode == s_applied_mode)
  {
    return;
  }

  if (ControlTask_PublishModeCommand(mode) != 0U)
  {
    s_applied_mode = mode;
  }
}

/*
 * 功能：状态机 2——原地停止：进入时只发布一次 POSITION(0,0)
 * 输入：无
 * 输出：无
 *
 * POSITION + value1=0 + value2=0：
 *   - 位置外环目标位移 0（原地不前进）；
 *   - yaw 角度环目标 0（不转向）；
 *   - PID 收到 POSITION 会锁存编码器起点，因此保持进入瞬间的位置。
 * 只发布一次；队列忙时每个 20 ms 重试，成功后不再发送。
 */
static void ControlTask_StateHold(void)
{
  if (s_hold_applied != 0U)
  {
    return;
  }

  if (ControlTask_PublishHoldCommand() != 0U)
  {
    s_hold_applied = 1U;
  }
}

/*
 * 功能：按调参模式向 Controlqueue 发布一次命令
 * 输入：tune_mode —— CONTROL_TUNE_MODE_*（0~5）
 * 输出：1 = 发布成功；0 = 队列已满或 Controlqueue 句柄无效
 *
 * 模式与命令的对应关系：
 *   0 速度   -> SPEED     value1=+100 mm/s，value2=0（不转弯）
 *   1 位置   -> POSITION  value1=+500 mm， value2=0（不转弯）
 *   2 倾角   -> ATTITUDE  value1=+5.0°，   value2=0
 *   3 角度   -> POSITION  value1=0（不前进），value2=+90.0°
 *   4 角速度 -> SPEED     value1=0（不前进），value2=+30°/s
 *   5 停止   -> STOP      value1=0，        value2=0
 *
 * 消息格式与 pid_task.c 的解析一致：
 *   bit47..46 模式，bit31..16 value1，bit15..0 value2；
 * CubeMX 队列项为 6 字节，STM32 小端，取 uint64_t 的低 6 字节。
 */
static uint8_t ControlTask_PublishModeCommand(uint8_t tune_mode)
{
  uint8_t queue_mode;
  int16_t value1 = 0;
  int16_t value2 = 0;

  switch (tune_mode)
  {
    case CONTROL_TUNE_MODE_SPEED:
      queue_mode = (uint8_t)CONTROL_MODE_SPEED;
      value1 = (int16_t)CONTROL_TUNE_SPEED_MM_S;
      value2 = 0;
      break;

    case CONTROL_TUNE_MODE_POSITION:
      queue_mode = (uint8_t)CONTROL_MODE_POSITION;
      value1 = (int16_t)CONTROL_TUNE_POSITION_MM;
      value2 = 0;
      break;

    case CONTROL_TUNE_MODE_PITCH:
      queue_mode = (uint8_t)CONTROL_MODE_ATTITUDE;
      value1 = (int16_t)CONTROL_TUNE_PITCH_0P1DEG;
      value2 = 0;
      break;

    case CONTROL_TUNE_MODE_YAW_ANGLE:
      queue_mode = (uint8_t)CONTROL_MODE_POSITION;
      value1 = 0;
      value2 = (int16_t)CONTROL_TUNE_YAW_ANGLE_0P1DEG;
      break;

    case CONTROL_TUNE_MODE_YAW_RATE:
      queue_mode = (uint8_t)CONTROL_MODE_SPEED;
      value1 = 0;
      value2 = (int16_t)CONTROL_TUNE_YAW_RATE_DPS;
      break;

    case CONTROL_TUNE_MODE_STOP:
    default:
      queue_mode = (uint8_t)CONTROL_MODE_STOP;
      value1 = 0;
      value2 = 0;
      break;
  }

  return ControlTask_PublishCommand(queue_mode, value1, value2);
}

/*
 * 功能：把模式与两个 int16 参数打包并发布到 Controlqueue
 * 输入：queue_mode —— CONTROL_MODE_*；value1/value2 —— int16 参数
 * 输出：1 = 发布成功；0 = 队列已满或 Controlqueue 句柄无效
 *
 * 消息格式与 pid_task.c 的解析一致：
 *   bit47..46 模式，bit31..16 value1，bit15..0 value2；
 * CubeMX 队列项为 6 字节，STM32 小端，取 uint64_t 的低 6 字节。
 */
static uint8_t ControlTask_PublishCommand(uint8_t queue_mode,
                                          int16_t value1,
                                          int16_t value2)
{
  uint64_t message;

  message = ((uint64_t)(queue_mode & 0x3U) << 46)
            | ((uint64_t)(uint16_t)value1 << 16)
            | (uint64_t)(uint16_t)value2;

  if (ControlqueueHandle == NULL)
  {
    return 0U;
  }

  return (osMessageQueuePut(ControlqueueHandle, &message, 0U, 0U) == osOK)
         ? 1U : 0U;
}

/*
 * 功能：发布原地停止命令：POSITION，value1=0（不前进），value2=0（不转向）
 * 输入：无
 * 输出：1 = 发布成功；0 = 队列已满或 Controlqueue 句柄无效
 */
static uint8_t ControlTask_PublishHoldCommand(void)
{
  return ControlTask_PublishCommand((uint8_t)CONTROL_MODE_POSITION, 0, 0);
}

/*
 * 功能：把目标频率换算成 TIM11 ARR/CCR 并写入
 * 输入：frequency_hz —— 目标 PWM 频率（Hz）
 * 输出：无
 *
 * 公式：ARR = TIM11 计数时钟 / 频率 - 1；
 *       CCR = (ARR + 1) / 2，保持 50% 占空比。
 * ARR/CCR 已开启预装载，新值在下一个更新事件生效。
 * 调用者负责临界区；本函数只写寄存器。
 */
static void ControlTask_BuzzerSetFrequency(uint32_t frequency_hz)
{
  uint32_t autoreload;

  if (frequency_hz == 0U)
  {
    return;
  }

  autoreload = CONTROL_BUZZER_TIMER_CLOCK_HZ / frequency_hz;

  if (autoreload > 0U)
  {
    autoreload--;
  }

  if (autoreload > 65535U)
  {
    autoreload = 65535U;
  }

  __HAL_TIM_SET_AUTORELOAD(&htim11, autoreload);
  __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1,
                        (autoreload + 1U) / 2U);
}

/*
 * 功能：启动蜂鸣器 PWM（TIM11_CH1/PF7），起始 2500 Hz、50% 占空比
 * 输入：无
 * 输出：无
 *
 * 同一引脚 PF7/BEEP_LED 同时驱动蜂鸣器和 LED。
 */
static void ControlTask_BuzzerStart(void)
{
  taskENTER_CRITICAL();

  if (s_buzzer_on == 0U)
  {
    s_buzzer_frequency_hz = CONTROL_BUZZER_START_FREQ_HZ;

    ControlTask_BuzzerSetFrequency(s_buzzer_frequency_hz);

    /* 让新的 ARR/CCR 预装载值立即生效，再启动 PWM */
    htim11.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim11, TIM_FLAG_UPDATE);

    if (HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1) == HAL_OK)
    {
      s_buzzer_on = 1U;
    }
    else
    {
      __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0U);
    }
  }

  taskEXIT_CRITICAL();
}

/*
 * 功能：关闭蜂鸣器 PWM（同时关闭同引脚 LED）
 * 输入：无
 * 输出：无
 */
static void ControlTask_BuzzerStop(void)
{
  taskENTER_CRITICAL();

  if (s_buzzer_on != 0U)
  {
    s_buzzer_on = 0U;

    (void)HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0U);
  }

  taskEXIT_CRITICAL();
}

/*
 * 功能：停止蜂鸣器并取消 1 s 软件定时器
 * 输入：无
 * 输出：无
 */
static void ControlTask_BuzzerCancel(void)
{
  if (s_buzzer_timer != NULL)
  {
    (void)osTimerStop(s_buzzer_timer);
  }

  ControlTask_BuzzerStop();
}

/*
 * 功能：蜂鸣器响 1 s（单次 FreeRTOS 软件定时器）
 * 输入：无
 * 输出：无
 */
static void ControlTask_BuzzerBeepOneSecond(void)
{
  if (s_buzzer_timer == NULL)
  {
    return;
  }

  /* 重新计时：先停定时器，再启动 PWM 和 1 s 单次定时器 */
  (void)osTimerStop(s_buzzer_timer);
  ControlTask_BuzzerStart();

  if (osTimerStart(s_buzzer_timer,
                   pdMS_TO_TICKS(CONTROL_RUN_BUZZER_MS)) != osOK)
  {
    ControlTask_BuzzerStop();
  }
}

/*
 * 功能：蜂鸣器频率扫描：每个 controlTask 周期把频率减小 50 Hz
 * 输入：无
 * 输出：无
 *
 * 2500 -> 2450 -> ... -> 50 Hz；减到 50 Hz 后保持。
 * ARR/CCR 已开启预装载，新值在下一个更新事件生效。
 */
static void ControlTask_BuzzerTick(void)
{
  taskENTER_CRITICAL();

  if (s_buzzer_on != 0U)
  {
    if (s_buzzer_frequency_hz > CONTROL_BUZZER_MIN_FREQ_HZ)
    {
      if (s_buzzer_frequency_hz >=
          (CONTROL_BUZZER_MIN_FREQ_HZ + CONTROL_BUZZER_FREQ_STEP_HZ))
      {
        s_buzzer_frequency_hz -= CONTROL_BUZZER_FREQ_STEP_HZ;
      }
      else
      {
        s_buzzer_frequency_hz = CONTROL_BUZZER_MIN_FREQ_HZ;
      }
    }

    ControlTask_BuzzerSetFrequency(s_buzzer_frequency_hz);
  }

  taskEXIT_CRITICAL();
}

/*
 * 功能：1 s 软件定时器回调：关闭蜂鸣器 PWM
 * 输入：argument —— 未使用
 * 输出：无
 *
 * 在 FreeRTOS 定时器任务上下文执行，只调用 HAL 的 PWM 停止，不调用阻塞 API。
 */
static void ControlTask_BuzzerTimerCallback(void *argument)
{
  (void)argument;
  ControlTask_BuzzerStop();
}

/*
 * 功能：KEY0 / KEY_UP EXTI 回调
 * 输入：GPIO_Pin —— 触发中断的引脚
 * 输出：无
 *
 * KEY0：切换 currentmode（0~5 循环），用于状态机 1 的调参模式；
 * KEY_UP：按 1（调参）-> 2（原地停止）-> 3（路径测试）-> 1 循环切换
 *         状态机，并把 currentmode 复位为 0（速度）。
 *
 * CubeMX 两个键都配置为下降沿触发，NVIC 优先级 5；200 ms 内的重复边沿
 * 视为抖动忽略。回调只改变量，不调用 RTOS API；具体命令由 controlTask
 * 在任务上下文发布。
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t now;

  /* KEY0：调参模式切换（currentmode 0~5 循环） */
  if (GPIO_Pin == KEY0_Pin)
  {
    now = HAL_GetTick();
    if ((now - s_key0_last_tick) < CONTROL_KEY_DEBOUNCE_MS)
    {
      return;
    }
    s_key0_last_tick = now;

    currentmode++;
    if (currentmode >= CONTROL_TUNE_MODE_COUNT)
    {
      currentmode = CONTROL_TUNE_MODE_SPEED;
    }
    return;
  }

  /* KEY_UP：状态机切换（1 调参 -> 2 原地停止 -> 3 路径 -> 4 巡线对齐 -> 1） */
  if (GPIO_Pin == KEY_UP_Pin)
  {
    now = HAL_GetTick();
    if ((now - s_keyup_last_tick) < CONTROL_KEY_DEBOUNCE_MS)
    {
      return;
    }
    s_keyup_last_tick = now;

    /* 状态机切换顺序：调参 -> 原地停止 -> 路径测试 -> 巡线对齐 -> 调参 */
    switch (currentstatemachine)
    {
      case CONTROL_STATE_TUNING:
        currentstatemachine = CONTROL_STATE_HOLD;
        break;

      case CONTROL_STATE_HOLD:
        currentstatemachine = CONTROL_STATE_RUN;
        break;

      case CONTROL_STATE_RUN:
        currentstatemachine = CONTROL_STATE_SM4;
        break;

      case CONTROL_STATE_SM4:
      default:
        currentstatemachine = CONTROL_STATE_TUNING;
        break;
    }

    currentmode = CONTROL_TUNE_MODE_SPEED;
  }
}

/*
 * 功能：状态机 3——前进 110 cm，F1~F8 检测到黑线后原地停车并蜂鸣 1 s
 * 输入：无
 * 输出：无
 *
 * 阶段：
 *   IDLE    : 发布一次 POSITION(1100,0)，成功后进入 DRIVING；
 *   DRIVING : 每个 20 ms 任务周期读取 s_front_line_mask（由
 *             ControlTask_Update() 轮询 F1~F8 后生成），任一位为 1
 *             表示检测到黑线；立即发布 POSITION(0,0) 原地停车，
 *             成功后进入 STOPPED 并启动 1 s 蜂鸣器定时器；
 *   STOPPED : 保持停车，等待 KEY_UP 切换到其他状态机。
 *
 * 位置命令发布失败（队列深度 1）时保持当前阶段，下个周期自动重试。
 */
static void ControlTask_StateRun(void)
{
  uint8_t i;

  switch (s_run_state)
  {
    case CONTROL_RUN_STATE_IDLE:
      /* 进入状态机 3：向前 110 cm，不转向 */
      if (ControlTask_PublishCommand((uint8_t)CONTROL_MODE_POSITION,
                                     (int16_t)CONTROL_RUN_FORWARD_MM,
                                     0) != 0U)
      {
        s_run_state = CONTROL_RUN_STATE_DRIVING;
      }
      break;

    case CONTROL_RUN_STATE_DRIVING:
      /*
       * F1~F8 原始电平已由 ControlTask_Update() 每 20 ms 更新到
       * s_front_sensor_levels[]，并按 LINE_SENSOR_ACTIVE_LEVEL 生成
       * s_front_line_mask（bit0=F1 ... bit7=F8，1=检测到黑线）。
       */
      s_run_last_front_mask = s_front_line_mask;

      if (s_front_line_mask != 0U)
      {
        /* 先下发原地停车；成功后才启动蜂鸣器，避免边跑边响 */
        if (ControlTask_PublishHoldCommand() != 0U)
        {
          s_run_black_mask = s_front_line_mask;

          /* 保存触发瞬间的 F1~F8 原始电平，便于调试 */
          for (i = 0U; i < LINE_SENSOR_COUNT; i++)
          {
            s_run_trigger_levels[i] = s_front_sensor_levels[i];
          }

          s_run_state = CONTROL_RUN_STATE_STOPPED;
          ControlTask_BuzzerBeepOneSecond();
        }
      }
      break;

    case CONTROL_RUN_STATE_STOPPED:
    default:
      /* 已原地停车，蜂鸣器由 1 s 软件定时器自动关闭 */
      break;
  }
}

/*
 * 功能：把 8 路光电掩码转换成加权中心位置
 * 输入：mask —— bit0=x1 ... bit7=x8；
 *       position_sign —— 传感器排列方向修正（+1 或 -1）；
 *       center_mm —— 输出中心位置，mm
 * 输出：1 = 至少一路检测到线；0 = 没有检测到线
 *
 * 传感器按等间距 LINE_SENSOR_PITCH_MM 排列，中心在 3.5 号位置：
 *   x1=-3.5*pitch, x2=-2.5*pitch, ... x8=+3.5*pitch。
 */
static uint8_t ControlTask_LineWeightedCenter(uint8_t mask,
                                              float position_sign,
                                              float *center_mm)
{
  uint32_t i;
  uint32_t count = 0U;
  float sum = 0.0f;

  if ((mask == 0U) || (center_mm == NULL))
  {
    return 0U;
  }

  for (i = 0U; i < LINE_SENSOR_COUNT; i++)
  {
    if ((mask & (uint8_t)(1U << i)) != 0U)
    {
      sum += (((float)i - 3.5f) * LINE_SENSOR_PITCH_MM) * position_sign;
      count++;
    }
  }

  if (count == 0U)
  {
    return 0U;
  }

  *center_mm = sum / (float)count;
  return 1U;
}

/*
 * 功能：浮点四舍五入并饱和到 int16
 * 输入：value —— 浮点值
 * 输出：四舍五入后的 int16
 */
static int16_t ControlTask_ToInt16(float value)
{
  int32_t result;

  result = (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));

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
 * 功能：状态机 4 进入时复位
 * 输入：无
 * 输出：无
 */
static void ControlTask_Sm4Reset(void)
{
  s_sm4_state = CONTROL_SM4_STRAIGHT_1;
  s_sm4_command_sent = 0U;
  s_sm4_front_prev_mask = s_front_line_mask;
  s_sm4_wait_clear = 0U;
  s_sm4_align_good_frames = 0U;
  s_sm4_phase_tick = HAL_GetTick();
  s_sm4_align_error_mm = 0.0f;
}

/*
 * 功能：切换状态机 4 的内部阶段
 * 输入：phase —— 目标阶段
 * 输出：无
 */
static void ControlTask_Sm4EnterPhase(Control_Sm4State_t phase)
{
  s_sm4_state = phase;
  s_sm4_command_sent = 0U;
  s_sm4_phase_tick = HAL_GetTick();

  if (phase == CONTROL_SM4_ALIGN)
  {
    s_sm4_align_good_frames = 0U;
    s_sm4_align_error_mm = 0.0f;
  }
  else if (phase == CONTROL_SM4_STRAIGHT_2)
  {
    /*
     * 对齐结束时前轮可能仍压在线上：先等它离开当前线，
     * 之后出现的黑线才当作第二条线的起点。
     */
    s_sm4_wait_clear = (s_front_line_mask != 0U) ? 1U : 0U;
  }
}

/*
 * 功能：状态机 4 巡线：前后加权中心 -> 共模/差模偏差 -> 速度/yaw rate
 * 输入：无
 * 输出：无
 *
 * 共模偏差 common = (前中心 + 后中心)/2，表示整车横向偏离；
 * 差模偏差 diff   = 前中心 - 后中心，表示车身相对黑线的方向偏差。
 *
 * 转向角速度：yaw = Kp_yaw * (common + W_diff * diff)，正比例 P；
 * 前进速度  ：V = V_base - Kp_speed * (|common| + W_speed*|diff|)，
 *             误差越大速度越低（反比例），并限制最小值。
 *
 * 两个目标通过 SPEED 模式发布：value1=速度(mm/s)，value2=yaw rate(°/s)。
 */
static void ControlTask_Sm4LineFollow(void)
{
  float front_center = 0.0f;
  float rear_center = 0.0f;
  float common;
  float diff;
  float common_error;
  float diff_error;
  float steer_mm;
  float error_mag;
  float speed;
  float yaw_rate;
  uint8_t front_valid;
  uint8_t rear_valid;

  front_valid = ControlTask_LineWeightedCenter(s_front_line_mask,
                                               CONTROL_LF_FRONT_POS_SIGN,
                                               &front_center);
  rear_valid = ControlTask_LineWeightedCenter(s_back_line_mask,
                                              CONTROL_LF_REAR_POS_SIGN,
                                              &rear_center);

  if ((front_valid == 0U) && (rear_valid == 0U))
  {
    /* 前后都丢线：先停车，等待下一帧/状态处理 */
    (void)ControlTask_PublishCommand((uint8_t)CONTROL_MODE_SPEED, 0, 0);
    return;
  }

  if ((front_valid != 0U) && (rear_valid != 0U))
  {
    common = (front_center + rear_center) * 0.5f;
    diff = front_center - rear_center;
  }
  else if (front_valid != 0U)
  {
    common = front_center;
    diff = 0.0f;
  }
  else
  {
    common = rear_center;
    diff = 0.0f;
  }

  common_error = common - CONTROL_LF_COMMON_TARGET_MM;
  diff_error = diff - CONTROL_LF_DIFF_TARGET_MM;

  /* 转向：共模 + 差模综合偏差，正比例 P */
  steer_mm = common_error
             + (CONTROL_LF_STEER_DIFF_WEIGHT * diff_error);
  yaw_rate = CONTROL_LF_YAW_KP * steer_mm * CONTROL_LF_STEER_SIGN;
  yaw_rate = PID_Clamp(yaw_rate, CONTROL_LF_MAX_RATE_DPS);

  /* 速度：误差越大速度越低（反比例 P），限制在 [min, max] */
  error_mag = fabsf(common_error)
              + (CONTROL_LF_SPEED_DIFF_WEIGHT * fabsf(diff_error));
  speed = CONTROL_LF_BASE_SPEED_MM_S
          - (CONTROL_LF_SPEED_KP * error_mag);
  speed = PID_Clamp(speed, CONTROL_LF_MAX_SPEED_MM_S);
  if (speed < CONTROL_LF_MIN_SPEED_MM_S)
  {
    speed = CONTROL_LF_MIN_SPEED_MM_S;
  }

  (void)ControlTask_PublishCommand((uint8_t)CONTROL_MODE_SPEED,
                                   ControlTask_ToInt16(speed),
                                   ControlTask_ToInt16(yaw_rate));
}

/*
 * 功能：状态机 4 对齐：前后加权中心差 -> P -> SPEED(0, yaw rate)
 * 输入：无
 * 输出：无
 *
 * align_error = (前中心 - 后中心) - CONTROL_ALIGN_TARGET_DIFF_MM；
 * yaw_rate = Kp_align * align_error * 方向修正；
 * 速度模式发布 value1=0（不前进），value2=yaw rate（原地转向）。
 * 连续 CONTROL_ALIGN_SETTLE_FRAMES 帧 |align_error| <= 容差即认为对齐。
 */
static void ControlTask_Sm4Align(void)
{
  float front_center = 0.0f;
  float rear_center = 0.0f;
  float error;
  float yaw_rate;
  uint8_t front_valid;
  uint8_t rear_valid;

  /* 超时保护：一直找不到线/对不齐时停车，避免原地无限等待 */
  if ((HAL_GetTick() - s_sm4_phase_tick) >= CONTROL_ALIGN_TIMEOUT_MS)
  {
    ControlTask_Sm4EnterPhase(CONTROL_SM4_STOPPING);
    return;
  }

  front_valid = ControlTask_LineWeightedCenter(s_front_line_mask,
                                               CONTROL_LF_FRONT_POS_SIGN,
                                               &front_center);
  rear_valid = ControlTask_LineWeightedCenter(s_back_line_mask,
                                              CONTROL_LF_REAR_POS_SIGN,
                                              &rear_center);

  if ((front_valid == 0U) || (rear_valid == 0U))
  {
    /* 前后任一排没看到线：停止转向，等待下一帧 */
    (void)ControlTask_PublishCommand((uint8_t)CONTROL_MODE_SPEED, 0, 0);
    s_sm4_align_good_frames = 0U;
    return;
  }

  error = (front_center - rear_center) - CONTROL_ALIGN_TARGET_DIFF_MM;
  s_sm4_align_error_mm = error;

  if (fabsf(s_sm4_align_error_mm) <= CONTROL_ALIGN_TOLERANCE_MM)
  {
    if (s_sm4_align_good_frames < CONTROL_ALIGN_SETTLE_FRAMES)
    {
      s_sm4_align_good_frames++;
    }
  }
  else
  {
    s_sm4_align_good_frames = 0U;
  }

  if (s_sm4_align_good_frames >= CONTROL_ALIGN_SETTLE_FRAMES)
  {
    /* 连续 10 帧对齐成功：停止转向，开始第二段直行 */
    (void)ControlTask_PublishCommand((uint8_t)CONTROL_MODE_SPEED, 0, 0);
    ControlTask_Sm4EnterPhase(CONTROL_SM4_STRAIGHT_2);
    return;
  }

  yaw_rate = CONTROL_ALIGN_KP * error * CONTROL_ALIGN_STEER_SIGN;
  yaw_rate = PID_Clamp(yaw_rate, CONTROL_ALIGN_MAX_RATE_DPS);

  (void)ControlTask_PublishCommand((uint8_t)CONTROL_MODE_SPEED,
                                   0,
                                   ControlTask_ToInt16(yaw_rate));
}

/*
 * 功能：状态机 4：直行 110 -> 巡线 -> 丢线后退 5cm -> 对齐 -> 直行 110
 *       -> 巡线 -> 第二次丢线停车
 * 输入：无
 * 输出：无
 *
 * 阶段：
 *   STRAIGHT_1 : POSITION(1100,0)，等待前轮首次压线；
 *   FOLLOW_1   : 巡线；前轮丢线 -> REVERSE；
 *   REVERSE    : POSITION(-50,0)，等待固定时间让后退完成；
 *   ALIGN      : 前后加权中心差 P 控制，SPEED(0, yaw rate)；
 *                连续 10 帧在容差内 -> STRAIGHT_2；
 *   STRAIGHT_2 : POSITION(1100,0)；若前轮仍压线，先等它离开，
 *                再等下一条黑线出现 -> FOLLOW_2；
 *   FOLLOW_2   : 巡线；第二次前轮丢线 -> STOPPING；
 *   STOPPING   : POSITION(0,0) 原地停车 -> DONE。
 *
 * 每次前轮黑线出现/消失的边沿都触发一次 1 s 频率扫描蜂鸣器。
 */
static void ControlTask_StateSm4(void)
{
  uint8_t front_mask = s_front_line_mask;
  uint8_t appeared;
  uint8_t disappeared;

  appeared = ((front_mask != 0U) &&
              (s_sm4_front_prev_mask == 0U)) ? 1U : 0U;
  disappeared = ((front_mask == 0U) &&
                 (s_sm4_front_prev_mask != 0U)) ? 1U : 0U;

  /* 黑线出现或消失：触发 1 s 蜂鸣器频率扫描 */
  if ((appeared != 0U) || (disappeared != 0U))
  {
    ControlTask_BuzzerBeepOneSecond();
  }

  switch (s_sm4_state)
  {
    case CONTROL_SM4_STRAIGHT_1:
      if (s_sm4_command_sent == 0U)
      {
        if (ControlTask_PublishCommand((uint8_t)CONTROL_MODE_POSITION,
                                       (int16_t)CONTROL_SM4_FORWARD_MM,
                                       0) != 0U)
        {
          s_sm4_command_sent = 1U;
        }
      }
      else if (front_mask != 0U)
      {
        ControlTask_Sm4EnterPhase(CONTROL_SM4_FOLLOW_1);
      }
      break;

    case CONTROL_SM4_FOLLOW_1:
      if (disappeared != 0U)
      {
        ControlTask_Sm4EnterPhase(CONTROL_SM4_REVERSE);
      }
      else
      {
        ControlTask_Sm4LineFollow();
      }
      break;

    case CONTROL_SM4_REVERSE:
      if (s_sm4_command_sent == 0U)
      {
        if (ControlTask_PublishCommand((uint8_t)CONTROL_MODE_POSITION,
                                       (int16_t)CONTROL_SM4_REVERSE_MM,
                                       0) != 0U)
        {
          s_sm4_command_sent = 1U;
          s_sm4_phase_tick = HAL_GetTick();
        }
      }
      else if ((HAL_GetTick() - s_sm4_phase_tick) >=
               CONTROL_SM4_REVERSE_WAIT_MS)
      {
        ControlTask_Sm4EnterPhase(CONTROL_SM4_ALIGN);
      }
      break;

    case CONTROL_SM4_ALIGN:
      ControlTask_Sm4Align();
      break;

    case CONTROL_SM4_STRAIGHT_2:
      if (s_sm4_command_sent == 0U)
      {
        if (ControlTask_PublishCommand((uint8_t)CONTROL_MODE_POSITION,
                                       (int16_t)CONTROL_SM4_FORWARD_MM,
                                       0) != 0U)
        {
          s_sm4_command_sent = 1U;
        }
      }
      else if (s_sm4_wait_clear != 0U)
      {
        if (disappeared != 0U)
        {
          s_sm4_wait_clear = 0U;
        }
      }
      else if (front_mask != 0U)
      {
        ControlTask_Sm4EnterPhase(CONTROL_SM4_FOLLOW_2);
      }
      break;

    case CONTROL_SM4_FOLLOW_2:
      if (disappeared != 0U)
      {
        ControlTask_Sm4EnterPhase(CONTROL_SM4_STOPPING);
      }
      else
      {
        ControlTask_Sm4LineFollow();
      }
      break;

    case CONTROL_SM4_STOPPING:
      if (s_sm4_command_sent == 0U)
      {
        if (ControlTask_PublishHoldCommand() != 0U)
        {
          s_sm4_state = CONTROL_SM4_DONE;
        }
      }
      break;

    case CONTROL_SM4_DONE:
    default:
      break;
  }

  s_sm4_front_prev_mask = front_mask;
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

/*
 * 功能：读取状态机 3 最近一次 F1~F8 检测掩码
 * 输入：无
 * 输出：bit0 = F1 ... bit7 = F8；1 表示检测到黑线
 */
uint8_t ControlTask_GetRunLastFrontMask(void)
{
  return s_run_last_front_mask;
}

/*
 * 功能：读取状态机 3 触发原地停车时的 F1~F8 掩码
 * 输入：无
 * 输出：触发掩码；0 = 尚未触发
 */
uint8_t ControlTask_GetRunBlackMask(void)
{
  return s_run_black_mask;
}

/*
 * 功能：读取状态机 3 触发停车瞬间保存的 F1~F8 原始电平
 * 输入：无
 * 输出：指向 8 字节数组的只读指针，下标 0..7 = F1..F8
 */
const uint8_t *ControlTask_GetRunTriggerLevels(void)
{
  return s_run_trigger_levels;
}

/************************ (C) COPYRIGHT *****END OF FILE***********************/
