/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   TIM3/TIM4 正交编码器读取：共模位置/速度送 PID 任务。
  ******************************************************************************
  * @note    当前架构只使用编码器共模量（直行距离、直行速度），
  *          不使用左右差模量做 yaw。
  *
  *          右编码器：TIM3 CH1/CH2 = PA6/PA7
  *          左编码器：TIM4 CH1/CH2 = PD12/PD13
  ******************************************************************************
  */

#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * TIM3/TIM4 当前使用 TIM_ENCODERMODE_TI12，硬件四倍频：
 *   CNT、position、speed 的单位都是“4 倍频后的计数”，
 *   不是编码器每通道的线数。
 *
 * 上层把物理距离换算成脉冲时必须使用：
 *   有效CPR = 编码器每圈线数 × 4 × 减速比
 * 不要在本模块里再乘 4，否则会重复放大。
 */
#define ENCODER_QUADRATURE_FACTOR 4U

/*
 * 16 位编码器总计数 65536，初始值取 1/10：
 *   65536 / 10 = 6553.6 -> 6554
 * 这样向上还有约 1/10 计数范围到回绕点，反向有约 9/10；
 * 软件用 int16_t 差值累计，不依赖绝对 CNT。
 */
#define ENCODER_INIT_COUNT        6554U

/* 左右编码器方向修正：实车确认方向后改成 +1 或 -1 */
#define ENCODER_LEFT_SIGN         (+1)
#define ENCODER_RIGHT_SIGN        (+1)

/* 编码器更新标称周期：PID 中环 50 Hz，20 ms */
#define ENCODER_NOMINAL_DT_S      0.020f
#define ENCODER_DT_MAX_S          0.100f

/*
 * 功能：设置 TIM3/TIM4 初始 CNT 并启动编码器接口
 * 输入：无
 * 输出：1 = 两路都启动成功；0 = 启动失败
 */
uint8_t Encoder_Init(void);

/*
 * 功能：读取左右编码器增量，累计共模位置/速度
 * 输入：position_pulses —— 输出共模位置（脉冲）；
 *       speed_pulses_s —— 输出共模速度（脉冲/s）
 * 输出：无
 *
 * 只由 PID 中环在 PID 任务上下文调用，其他任务不得读取编码器。
 */
void Encoder_Update(float *position_pulses, float *speed_pulses_s);

/*
 * 功能：读取累计的左右编码器脉冲（调试用）
 * 输入：无
 * 输出：左/右累计脉冲
 */
int32_t Encoder_GetLeftTotal(void);
int32_t Encoder_GetRightTotal(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */

/************************ (C) COPYRIGHT *****END OF FILE***********************/
