/**
  ******************************************************************************
  * @file    line_sensor.c
  * @brief   八路红外巡线传感器 GPIO 读取：前 F1-F8，后 B1-B8。
  ******************************************************************************
  */

/* 头文件包含 ----------------------------------------------------------------*/
#include <stddef.h>
#include "main.h"
#include "line_sensor.h"

/* 配置检查 ----------------------------------------------------------------*/
#if (LINE_SENSOR_COUNT != 8U)
#error "LINE_SENSOR_COUNT must be 8 for the fixed F1-F8 / B1-B8 pin table"
#endif

#if (LINE_SENSOR_ACTIVE_LEVEL != 0U) && (LINE_SENSOR_ACTIVE_LEVEL != 1U)
#error "LINE_SENSOR_ACTIVE_LEVEL must be 0 or 1"
#endif

/* 私有变量 ------------------------------------------------------------------*/

/* 前 8 路：数组下标 0..7 对应 x1..x8 */
static GPIO_TypeDef * const s_front_port[LINE_SENSOR_COUNT] =
{
  F1_GPIO_Port, F2_GPIO_Port, F3_GPIO_Port, F4_GPIO_Port,
  F5_GPIO_Port, F6_GPIO_Port, F7_GPIO_Port, F8_GPIO_Port
};

static const uint16_t s_front_pin[LINE_SENSOR_COUNT] =
{
  F1_Pin, F2_Pin, F3_Pin, F4_Pin,
  F5_Pin, F6_Pin, F7_Pin, F8_Pin
};

/* 后 8 路：数组下标 0..7 对应 x1..x8 */
static GPIO_TypeDef * const s_back_port[LINE_SENSOR_COUNT] =
{
  B1_GPIO_Port, B2_GPIO_Port, B3_GPIO_Port, B4_GPIO_Port,
  B5_GPIO_Port, B6_GPIO_Port, B7_GPIO_Port, B8_GPIO_Port
};

static const uint16_t s_back_pin[LINE_SENSOR_COUNT] =
{
  B1_Pin, B2_Pin, B3_Pin, B4_Pin,
  B5_Pin, B6_Pin, B7_Pin, B8_Pin
};

/* ========================================================================== */
/*                              以下为函数实现                                */
/* ========================================================================== */

/*
 * 功能：读取前后各 8 路传感器的原始 GPIO 电平
 * 输入：front_levels —— 前 8 路输出数组；back_levels —— 后 8 路输出数组
 * 输出：无
 */
void LineSensor_ReadLevels(uint8_t front_levels[LINE_SENSOR_COUNT],
                           uint8_t back_levels[LINE_SENSOR_COUNT])
{
  uint32_t i;

  if (front_levels != NULL)
  {
    for (i = 0U; i < LINE_SENSOR_COUNT; i++)
    {
      front_levels[i] = (HAL_GPIO_ReadPin(s_front_port[i],
                                          s_front_pin[i]) == GPIO_PIN_SET)
                        ? 1U : 0U;
    }
  }

  if (back_levels != NULL)
  {
    for (i = 0U; i < LINE_SENSOR_COUNT; i++)
    {
      back_levels[i] = (HAL_GPIO_ReadPin(s_back_port[i],
                                         s_back_pin[i]) == GPIO_PIN_SET)
                       ? 1U : 0U;
    }
  }
}

/*
 * 功能：把原始电平数组转换为“检测到线”的位掩码
 * 输入：levels —— 8 路原始电平数组
 * 输出：bit0 = x1 ... bit7 = x8；1 表示该路电平等于
 *       LINE_SENSOR_ACTIVE_LEVEL
 */
uint8_t LineSensor_LevelsToMask(const uint8_t levels[LINE_SENSOR_COUNT])
{
  uint8_t mask = 0U;
  uint32_t i;

  if (levels == NULL)
  {
    return 0U;
  }

  for (i = 0U; i < LINE_SENSOR_COUNT; i++)
  {
    if (levels[i] == LINE_SENSOR_ACTIVE_LEVEL)
    {
      mask |= (uint8_t)(1U << i);
    }
  }

  return mask;
}

/************************ (C) COPYRIGHT *****END OF FILE***********************/
