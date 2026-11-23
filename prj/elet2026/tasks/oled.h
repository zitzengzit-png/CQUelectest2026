/**
  ******************************************************************************
  * @file    oled.h
  * @brief   0.96 寸 128x64 SSD1306/SSD1315 OLED：I2C3 + DMA 写显存。
  ******************************************************************************
  * @note    4 针模块没有 RESET 引脚；地址默认 0x78，若 D/C# 接高则为 0x7A。
  *          显存 8 页 × 128 列，每字节 bit0 = 页顶行。
  ******************************************************************************
  */

#ifndef __OLED_H
#define __OLED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "robot_config.h"

/* OLED 尺寸与 I2C 地址在 robot_config.h 中配置 */

/*
 * 功能：初始化 OLED（探测地址、发送初始化命令、清屏并刷新一次）
 * 输入：无
 * 输出：1 = 成功；0 = 地址无应答或 I2C 失败
 */
uint8_t OLED_Init(void);

/*
 * 功能：清空 MCU 侧显存（不会立即刷新到屏幕）
 * 输入：无
 * 输出：无
 */
void OLED_Clear(void);

/*
 * 功能：在指定页写一个 ASCII 字符串（5x7 字体 + 1 列间隔）
 * 输入：x —— 起始列 0~127；
 *       page —— 页 0~7（每页 8 像素高）；
 *       text —— 以 '\0' 结尾的字符串
 * 输出：无
 */
void OLED_ShowString(uint8_t x, uint8_t page, const char *text);

/*
 * 功能：查询 OLED 显存是否可写、DMA 是否空闲
 * 输入：无
 * 输出：1 = 空闲可刷新；0 = 未初始化或上一次 DMA 未完成
 */
uint8_t OLED_IsReady(void);

/*
 * 功能：启动一次整屏 DMA 刷新（非阻塞）
 * 输入：无
 * 输出：1 = DMA 已启动；0 = 未就绪或启动失败
 *
 * 启动前会自动把 OLED 就绪标志置 0，完成/错误回调恢复为 1。
 */
uint8_t OLED_Refresh_DMA(void);

/*
 * 功能：等待上一次 DMA 刷新完成
 * 输入：timeout_ms —— 超时时间（ms）
 * 输出：1 = 成功；0 = 超时或 I2C 错误
 */
uint8_t OLED_WaitRefresh(uint32_t timeout_ms);

/*
 * 功能：读取 OLED I2C 错误计数（调试用）
 * 输入：无
 * 输出：错误计数
 */
uint32_t OLED_GetErrorCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_H */

/************************ (C) COPYRIGHT *****END OF FILE***********************/

