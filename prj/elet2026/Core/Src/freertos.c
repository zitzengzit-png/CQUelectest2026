/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imu_task.h"
#include "encoder.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh4,
};
/* Definitions for PID */
osThreadId_t PIDHandle;
const osThreadAttr_t PID_attributes = {
  .name = "PID",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for controlTask */
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow5,
};
/* Definitions for Controlqueue */
osMessageQueueId_t ControlqueueHandle;
const osMessageQueueAttr_t Controlqueue_attributes = {
  .name = "Controlqueue"
};
/* Definitions for IMUdatadoneBinarySem */
osSemaphoreId_t IMUdatadoneBinarySemHandle;
const osSemaphoreAttr_t IMUdatadoneBinarySem_attributes = {
  .name = "IMUdatadoneBinarySem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void StartPIDTask(void *argument);
extern void StartcontrolTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of IMUdatadoneBinarySem */
  IMUdatadoneBinarySemHandle = osSemaphoreNew(1, 1, &IMUdatadoneBinarySem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* 确保初值为 0：PID 只能由 USART1 RX DMA 完成中断释放后运行 */
  (void)osSemaphoreAcquire(IMUdatadoneBinarySemHandle, 0U);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Controlqueue */
  ControlqueueHandle = osMessageQueueNew (1, 6, &Controlqueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of PID */
  PIDHandle = osThreadNew(StartPIDTask, NULL, &PID_attributes);

  /* creation of controlTask */
  controlTaskHandle = osThreadNew(StartcontrolTask, NULL, &controlTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  初始化任务：完成 IMU 初始化后永久挂起，不再参与调度
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;

  /* 先启动 TIM3/TIM4 正交编码器，初始 CNT=6554（16 位总量的 1/10） */
  (void)Encoder_Init();

  /* 初始化 ATK-MS901M：配置 921600 / 250 Hz / 九轴，并启动 USART1 RX DMA */

  /*
   * 初始化失败时每秒重试一次；重试期间 osDelay 让出 CPU，controlTask 仍可运行。
   * 成功后再永久挂起，把 CPU 全部留给 PID/controlTask。
   */
  while (IMU_Init() == 0U)
  {
    (void)osDelay(1000U);
  }

  /*
   * defaultTask 优先级 44，只负责一次性初始化。
   * 初始化完成后必须永久挂起，否则会持续占用 CPU，PID/controlTask 无法运行。
   * 用循环包住挂起：即使被意外 vTaskResume，也会立刻再次挂起。
   */
  for (;;)
  {
    vTaskSuspend(NULL);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
