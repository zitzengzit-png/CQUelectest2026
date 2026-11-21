/**
  ******************************************************************************
  * @file    control_task.h
  * @brief   主控任务：调参/原地停止/路径/巡线对齐状态机与 Controlqueue 命令发布。
  ******************************************************************************
  * @note    CubeMX 生成的入口名为 StartcontrolTask（小写 c），不要改名。
  *
  *          状态机 1：调参模式。KEY0（PE4）下降沿 EXTI 在中断回调里切换
  *          currentmode（0~5），controlTask 识别后依次进入
  *          速度 / 位置 / 倾角 / 角度 / 角速度 / 停止 六个模式，
  *          每次进入对应模式只向 Controlqueue 发布一次命令。
  *
  *          状态机 2：原地停止模式。进入时只发布一次 POSITION 命令，
  *          value1=0（不前进），value2=0（不转向）。
  *
  *          状态机 3：路径测试模式。进入时发布一次 POSITION(1100,0)
  *          向前 110 cm；每 20 ms 轮询 F1~F8，任一路检测到黑线就发布
  *          POSITION(0,0) 原地停车，并用 FreeRTOS 单次软件定时器
  *          让蜂鸣器（TIM11_CH1/PF7）响 1 s；响期间每个 controlTask
  *          周期把频率从 2500 Hz 减 50 Hz，保持 50% 占空比。
  *
  *          状态机 4：巡线对齐模式。直行 110 cm -> 巡线 -> 前轮丢线后退
  *          5 cm -> 前后加权中心对齐 -> 再直行 110 cm -> 再次巡线 ->
  *          第二次前轮丢线 POSITION(0,0) 停车；每次前轮黑线出现/消失
  *          都触发 1 s 蜂鸣器频率扫描。
  *
  *          状态机 5：四次 zigzag + 巡线。进入后保持 1 s -> 对齐一次 ->
  *          右转 atan(1.25)≈51.34° -> 直行 130 cm；途中遇线巡线，到点未遇线
  *          则左转同角度并用 SPEED 模式找线；巡线丢线后退 5 cm -> 左转
  *          同角度 -> 直行 130 cm，重复同样的找线逻辑；第二次巡线丢线后
  *          计一个周期，连续执行 4 个周期，最后 POSITION(0,0) 停车。
  *
  *          状态机 4/5 巡线末段：每次开始巡线记录共模编码器起点，
  *          行进 30π cm 后最大前进速度线性降到 40π cm 的 50%。
  *
  *          KEY_UP（PA0）下降沿 EXTI 作为状态机切换按钮：按
  *          1 -> 2 -> 3 -> 4 -> 5 -> 1 循环，同时把 currentmode 复位为 0（速度）。
  *
  *          全局 currentstatemachine 选择状态机（0=IDLE，1=调参，
  *          2=原地停止，3=路径测试，4=巡线对齐，5=四次zigzag，6=STOP）；
  *          上电默认 2（调试结束，进入原地停止）。
  ******************************************************************************
  */

#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "cmsis_os.h"
#include "robot_config.h"

/*
 * 主控任务周期在 robot_config.h 的 CONTROL_TASK_PERIOD_MS 中配置。
 */

/*
 * 功能：主控任务入口（赛题流程、路径状态机、Controlqueue 命令）
 * 输入：argument —— 任务参数（未使用）
 * 输出：无
 */
void StartcontrolTask(void *argument);

/* 调试计数：controlTask 实际循环次数（displayTask 读取） */
uint32_t ControlTask_GetRunCount(void);

/*
 * 功能：获取最近一次 controlTask 采集的前/后 8 路传感器原始电平
 * 输入：无
 * 输出：指向 8 字节数组的只读指针，下标 0..7 对应 x1..x8
 *
 * 注意：数组由 controlTask 周期性更新；其他任务只读，不要修改。
 */
const uint8_t *ControlTask_GetFrontSensorLevels(void);
const uint8_t *ControlTask_GetBackSensorLevels(void);

/*
 * 功能：获取最近一次 controlTask 计算的前/后巡线位掩码
 * 输入：无
 * 输出：bit0 = x1 ... bit7 = x8；1 表示检测到线
 */
uint8_t ControlTask_GetFrontLineMask(void);
uint8_t ControlTask_GetBackLineMask(void);

/*
 * 功能：读取状态机 3 最近一次 F1~F8 检测掩码
 * 输入：无
 * 输出：bit0 = F1 ... bit7 = F8；1 表示检测到黑线
 */
uint8_t ControlTask_GetRunLastFrontMask(void);

/*
 * 功能：读取状态机 3 触发原地停车时的 F1~F8 掩码
 * 输入：无
 * 输出：触发掩码；0 = 尚未触发
 */
uint8_t ControlTask_GetRunBlackMask(void);

/*
 * 功能：读取状态机 3 触发停车瞬间保存的 F1~F8 原始电平
 * 输入：无
 * 输出：指向 8 字节数组的只读指针，下标 0..7 = F1..F8
 */
const uint8_t *ControlTask_GetRunTriggerLevels(void);

#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_TASK_H */

/************************ (C) COPYRIGHT *****END OF FILE***********************/
