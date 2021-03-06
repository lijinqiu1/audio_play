/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "audio.h"
#include "log.h"
#include "display.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* USER CODE BEGIN Variables */
EventGroupHandle_t xEventGroup;
TimerHandle_t xKeyDelayTimer;
SemaphoreHandle_t xSdioMutex;


extern QueueHandle_t xQueueLog;

#if defined(PLAY_WITH_LIST)
#if defined(IIS_DMA_A) || defined(IIS_DMA_B)
extern osThreadId audioplaywithlistTaskHandle;
#else
extern osThreadId audioplaywithlistTxTaskHandle;
extern osThreadId audioplaywithlistRxTaskHandle;
#endif

#elif defined(PLAY_WITH_RNG)
extern osThreadId audioplayTaskHandle;
#endif
extern osThreadId audiocontrollerHandle;
extern osThreadId logrecordHandle;
extern osThreadId displayprocessHandle;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

extern void MX_FATFS_Init(void);
extern void prvKeyDelayCallback(TimerHandle_t xTimer);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
//extern void FatfsTask(void const * argument);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */


  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  xSdioMutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_EVENTGROUP */
  xEventGroup = xEventGroupCreate();
  /* USER CODE END RTOS_EVENTGROUP */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  xKeyDelayTimer = xTimerCreate("OneShot",pdMS_TO_TICKS(10),pdFALSE,0,prvKeyDelayCallback);
  /* 周期性处理任务 */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /*录音、播放线程*/
  #if defined(PLAY_WITH_LIST)
  #if defined(IIS_DMA_A) || defined(IIS_DMA_B)
  osThreadDef(audioplaywithlistTask,AudioPlay_With_List_Task,osPriorityRealtime,0,8192);
  audioplaywithlistTaskHandle = osThreadCreate(osThread(audioplaywithlistTask),NULL);
  #else
  osThreadDef(audioplaywithlistTxTask,AudioPlay_With_List_Tx_Task,osPriorityRealtime,0,2048);
  audioplaywithlistTxTaskHandle = osThreadCreate(osThread(audioplaywithlistTxTask),NULL);
  osThreadDef(audioplaywithlistRxTask,AudioPlay_With_List_Rx_Task,osPriorityHigh,0,2048);
  audioplaywithlistRxTaskHandle = osThreadCreate(osThread(audioplaywithlistRxTask),NULL);
  #endif
  #elif defined(PLAY_WITH_RNG)
  osThreadDef(audioplayTask,AudioPlay_Task,osPriorityHigh,0,4096);
  audioplayTaskHandle = osThreadCreate(osThread(audioplayTask),NULL);
  #endif
  /*音频控制线程*/
  osThreadDef(audiocontrollerTask,AudioController_Task,osPriorityNormal,0,2048);
  audiocontrollerHandle = osThreadCreate(osThread(audiocontrollerTask),NULL);
  /*log线程*/
  osThreadDef(logrecordTask,Log_Record_Task,osPriorityLow,0,1024);
  logrecordHandle = osThreadCreate(osThread(logrecordTask),NULL);
  /*oled显示线程*/
  osThreadDef(displayprocessTask, Display_Process_Task, osPriorityBelowNormal, 0, 1024);
  displayprocessHandle = osThreadCreate(osThread(displayprocessTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  xQueueLog = xQueueCreate(QUEUE_LOG_ITEM_LENGTH,QUEUE_LOG_ITEM_SIZE);
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for FATFS */
//  MX_FATFS_Init();

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
