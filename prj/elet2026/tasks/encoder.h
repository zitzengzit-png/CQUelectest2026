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
#include "robot_config.h"

/*
 * TIM3/TIM4 使用 TIM_ENCODERMODE_TI12，硬件四倍频。
 * CNT/position/speed 的单位都是 4 倍频后的计数；物理距离换算使用
 * 有效CPR = 编码器每圈线数 × 4 × 减速比。
 *
 * ENCODER_QUADRATURE_FACTOR / ENCODER_INIT_COUNT / 左右方向 / dt 参数
 * 全部在 robot_config.h 中定义。
 */

/*
 * 功能：设置 TIM3/TIM4 初始 CNT 并启动编码器接口
 * 输入：无
 * 输出：1 = 两路都启动成功；0 = 启动失败
 */
uint8_t Encoder_Init(void);

/*
 * 功能：读取左右编码器增量，累计共模位置/速度并转换为国际单位
 * 输入：position_m —— 输出共模位置（m）；
 *       speed_mps —— 输出共模速度（m/s）
 * 输出：无
 *
 * 只由 PID 中环在 PID 任务上下文调用，其他任务不得读取编码器。
 */
void Encoder_Update(float *position_m, float *speed_mps);

/*
 * 功能：只读共模位置（不消费计数、不产生速度样本）
 * 输入：position_m —— 输出共模位置（m）
 * 输出：无
 *
 * 用途：进入/更新位置模式时锁存起点。Encoder_Update() 会“消费”这段时间的
 *       计数并算出一个速度样本，如果一个 tick 内被调用两次，第二次的增量是 0、
 *       dt 退化为标称 20 ms，就会注入一个假 0 速度（ff_speed=6207 时会带来
 *       约 2.5 k 计数的共模冲击）。
 *       本函数只读计数器把未消费的增量算进位置，不改 s_last_*、s_left_total/
 *       s_right_total 与 dt 时间戳，因此不影响下一次 Encoder_Update() 的速度窗口。
 */
void Encoder_PeekCommonPosition(float *position_m);

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
