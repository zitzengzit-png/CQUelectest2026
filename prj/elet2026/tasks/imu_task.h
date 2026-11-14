/**
  ******************************************************************************
  * @file    imu_task.h
  * @brief   IMU 任务：MPU6050 DMA 采集、卡尔曼滤波与姿态数据发布。
  ******************************************************************************
  */

#ifndef __IMU_TASK_H
#define __IMU_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "cmsis_os.h"

/* 卡尔曼解算输出（kalmaneddataQueue 按值存放该结构体） */
typedef struct
{
  float pitch;        /* 俯仰角，deg */
  float pitch_rate;   /* 俯仰角速度，deg/s */
  float gyro_bias;    /* 陀螺零偏估计，deg/s */
  uint32_t timestamp; /* 采样时刻，ms */
} KalmanData_t;

/* CubeMX 生成的队列/信号量句柄（freertos.c） */
extern osMessageQueueId_t kalmaneddataQueueHandle;
extern osSemaphoreId_t IMUtaskBinarySemHandle;

/*
 * 功能：IMU 任务入口，等待 DMA 完成信号后执行解算
 * 输入：argument —— 任务参数（未使用）
 * 输出：无
 */
void StartIMUTask(void *argument);

/*
 * 功能：MPU6050 初始化（阻塞 I2C，在初始化任务中调用）
 *       初始化后做约 1 s 静止陀螺零偏校准，再启动滤波。
 * 输入：无
 * 输出：1 = I2C 配置与 WHO_AM_I 成功（零偏校准失败时仍为 1，滤波器会在线估计零偏）；
 *       0 = I2C 读写失败或 WHO_AM_I 校验失败
 */
uint8_t IMU_Init(void);

/*
 * 功能：查询 IMU 是否已成功发布首帧数据
 * 输入：无
 * 输出：1 = 首帧数据已发布；0 = 尚未就绪
 */
uint8_t IMU_IsReady(void);

/*
 * 功能：TIM7 周期回调中调用，启动一次 I2C DMA 读取
 * 输入：无
 * 输出：无
 */
void IMU_OnTimer7(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_TASK_H */

/************************ (C) COPYRIGHT *****END OF FILE***********************/
