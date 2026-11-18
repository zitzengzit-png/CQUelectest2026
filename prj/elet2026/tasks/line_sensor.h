/**
  ******************************************************************************
  * @file    line_sensor.h
  * @brief   八路红外巡线传感器 GPIO 读取：前 8 路 F1-F8，后 8 路 B1-B8。
  ******************************************************************************
  * @note    本驱动只读取 GPIO 电平，不配置 GPIO；引脚模式由 CubeMX 的
  *          MX_GPIO_Init() 配置为 GPIO_MODE_INPUT。
  *
  *          数组下标 0..7 对应 x1..x8：
  *            前：F1 = PE8,  F2 = PD15, F3 = PD14, F4 = PD9,
  *                F5 = PF4,  F6 = PF2,  F7 = PF0,  F8 = PB15
  *            后：B1 = PG8,  B2 = PG6,  B3 = PC8,  B4 = PC12,
  *                B5 = PC11, B6 = PG15, B7 = PD6,  B8 = PC6
  *
  *          LineSensor_ReadLevels() 返回引脚原始电平（0/1）。
  *          LineSensor_LevelsToMask() 按 LINE_SENSOR_ACTIVE_LEVEL 生成
  *          “1 = 检测到线”的位掩码，bit0 = x1，bit7 = x8。
  ******************************************************************************
  */

#ifndef __LINE_SENSOR_H
#define __LINE_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "robot_config.h"

/*
 * 功能：读取前后各 8 路传感器的原始 GPIO 电平
 * 输入：front_levels —— 前 8 路输出数组，元素为 0/1；
 *       back_levels  —— 后 8 路输出数组，元素为 0/1
 * 输出：无
 *
 * 注意：数组传入 NULL 时对应方向不读取。
 */
void LineSensor_ReadLevels(uint8_t front_levels[LINE_SENSOR_COUNT],
                           uint8_t back_levels[LINE_SENSOR_COUNT]);

/*
 * 功能：把原始电平数组转换为“检测到线”的位掩码
 * 输入：levels —— 8 路原始电平数组
 * 输出：bit0 = x1 ... bit7 = x8；1 表示该路电平等于
 *       LINE_SENSOR_ACTIVE_LEVEL
 */
uint8_t LineSensor_LevelsToMask(const uint8_t levels[LINE_SENSOR_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* __LINE_SENSOR_H */

/************************ (C) COPYRIGHT *****END OF FILE***********************/
