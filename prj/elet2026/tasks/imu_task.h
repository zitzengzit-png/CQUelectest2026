/**
  ******************************************************************************
  * @file    imu_task.h
  * @brief   ATK-MS901M 九轴 IMU 驱动：USART3 + DMA 接收、帧解析与数据发布。
  ******************************************************************************
  * @note    当前架构没有独立 IMU 任务：
  *          USART3 RX DMA 完成中断 -> 拷贝原始字节 + 释放 IMUdatadoneBinarySem
  *          -> PID 任务在信号量上醒来后调用 IMU_Parse() 解析
  *          -> IMU_GetData() 给 PID 控制使用。
  ******************************************************************************
  */

#ifndef __IMU_TASK_H
#define __IMU_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "cmsis_os.h"
#include "robot_config.h"

/*
 * IMU 波特率、上报内容、上报率、量程、轴映射等参数均在 robot_config.h。
 *
 * 当前每 250 Hz 周期回传 2 帧：
 *   0x01 姿态角      11 B
 *   0x03 陀螺/加速度 17 B
 * 合计 28 B。若增加 0x04 磁力计帧（13 B），必须同步修改
 * IMU_RETURNSET_VALUE 和 IMU_DMA_BYTES。
 */

/* 解析后的 IMU 数据（PID 任务使用） */
typedef struct
{
  float roll_deg;        /* 横滚角，deg */
  float pitch_deg;       /* 俯仰角，deg（已按 IMU_PITCH_SIGN 处理） */
  float yaw_deg;         /* 九轴绝对航向，deg；ENU 欧拉角，0° 不是正北（手册 +90°≈X+ 指北） */
  float pitch_rate_dps;  /* 俯仰角速度，deg/s（已按映射处理） */
  float yaw_rate_dps;    /* yaw 角速度，deg/s（已按映射处理） */
  float gyro_x_dps;
  float gyro_y_dps;
  float gyro_z_dps;
  float acc_x_g;
  float acc_y_g;
  float acc_z_g;
  int16_t mag_x;         /* 仅 RETURNSET 包含 0x04 时更新；当前不启用 */
  int16_t mag_y;         /* 仅 RETURNSET 包含 0x04 时更新；当前不启用 */
  int16_t mag_z;         /* 仅 RETURNSET 包含 0x04 时更新；当前不启用 */
  float temperature_c;   /* 仅 0x04 帧携带的磁力计温度，当前不启用 */
  uint32_t timestamp_ms;
  uint8_t valid;         /* 1 = 姿态和陀螺数据均已收到 */
} IMU_Data_t;

/* CubeMX 生成的信号量（实际名称为 IMUdatadoneBinarySem，对应需求中的 IMUdatadoneSEM） */
extern osSemaphoreId_t IMUdatadoneBinarySemHandle;

/*
 * 功能：初始化 USART3 接收 DMA，并向 ATK-MS901M 写入上报配置
 * 输入：无
 * 输出：1 = 已收到有效姿态/陀螺帧；0 = 超时或 DMA 启动失败
 */
uint8_t IMU_Init(void);

/*
 * 功能：解析 DMA 完成中断保存的一批原始字节（由 PID 任务调用）
 * 输入：无
 * 输出：1 = 本次至少解析出一帧；0 = 没有新数据或不完整
 */
uint8_t IMU_Parse(void);

/*
 * 功能：读取最新解析结果
 * 输入：data —— 输出结构体
 * 输出：无
 */
void IMU_GetData(IMU_Data_t *data);

/*
 * 功能：查询是否已收到姿态和陀螺数据
 * 输入：无
 * 输出：1 = 已就绪；0 = 未就绪
 */
uint8_t IMU_IsReady(void);

/*
 * 功能：读取 UART/DMA 错误计数（调试用）
 * 输入：无
 * 输出：错误计数
 */
uint32_t IMU_GetErrorCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_TASK_H */

/************************ (C) COPYRIGHT *****END OF FILE***********************/

