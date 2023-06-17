/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stream_buffer.h"
#include "usart.h"
#include "adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_TRESHOLD 2048
#define BUFFER_SIZE 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
StreamBufferHandle_t streamBuffer;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTaskMonitor */
osThreadId_t myTaskMonitorHandle;
const osThreadAttr_t myTaskMonitor_attributes = {
  .name = "myTaskMonitor",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myTaskSensor1 */
osThreadId_t myTaskSensor1Handle;
const osThreadAttr_t myTaskSensor1_attributes = {
  .name = "myTaskSensor1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTaskSensor2 */
osThreadId_t myTaskSensor2Handle;
const osThreadAttr_t myTaskSensor2_attributes = {
  .name = "myTaskSensor2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myBufferMutex */
osMutexId_t myBufferMutexHandle;
const osMutexAttr_t myBufferMutex_attributes = {
  .name = "myBufferMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTaskMonitor(void *argument);
void StartTaskSensor1(void *argument);
void StartTaskSensor2(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	streamBuffer = xStreamBufferGenericCreate(BUFFER_SIZE, 1, pdFALSE);
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of myBufferMutex */
  myBufferMutexHandle = osMutexNew(&myBufferMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTaskMonitor */
  myTaskMonitorHandle = osThreadNew(StartTaskMonitor, NULL, &myTaskMonitor_attributes);

  /* creation of myTaskSensor1 */
  myTaskSensor1Handle = osThreadNew(StartTaskSensor1, NULL, &myTaskSensor1_attributes);

  /* creation of myTaskSensor2 */
  myTaskSensor2Handle = osThreadNew(StartTaskSensor2, NULL, &myTaskSensor2_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  HAL_ADC_Start_IT(&hadc1);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTaskMonitor */
/**
* @brief Function implementing the myTaskMonitor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskMonitor */
void StartTaskMonitor(void *argument)
{
  /* USER CODE BEGIN StartTaskMonitor */
	//Buffer with the same
	uint8_t message[BUFFER_SIZE];
	uint16_t messageLen = 0;
  /* Infinite loop */
  for(;;)
  {
	  // check if something in the buffer
	  if ( !xStreamBufferIsEmpty(streamBuffer))
	  {

		  messageLen = xStreamBufferBytesAvailable(streamBuffer);
		  xStreamBufferReceive(streamBuffer, message, messageLen, pdMS_TO_TICKS(0));
		  HAL_UART_Transmit(&huart2, message, messageLen, HAL_MAX_DELAY);
	  }
    osDelay(1);
  }
  /* USER CODE END StartTaskMonitor */
}

/* USER CODE BEGIN Header_StartTaskSensor1 */
/**
* @brief Function implementing the myTaskSensor1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskSensor1 */
void StartTaskSensor1(void *argument)
{
  /* USER CODE BEGIN StartTaskSensor1 */
	uint32_t lux;
	uint8_t buf[32];
  /* Infinite loop */
  for(;;)
  {
	  osMutexWait(myBufferMutexHandle, osWaitForever);
	  lux = calculateLux();

	  sprintf(buf, "Sensor1: Lux = %d\n\r", lux);

	  xStreamBufferSend(streamBuffer, buf, strlen(buf), pdMS_TO_TICKS(10));

	  osMutexRelease(myBufferMutexHandle);

	  osDelay(1);
  }
  /* USER CODE END StartTaskSensor1 */
}

/* USER CODE BEGIN Header_StartTaskSensor2 */
/**
* @brief Function implementing the myTaskSensor2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskSensor2 */
void StartTaskSensor2(void *argument)
{
  /* USER CODE BEGIN StartTaskSensor2 */
	uint32_t lux;
	uint8_t buf[32];
  /* Infinite loop */
  for(;;)
  {
	  osMutexWait(myBufferMutexHandle, osWaitForever);
	  lux = calculateLux();

	  sprintf(buf, "Sensor2: Lux = %d\n\r", lux);

	  xStreamBufferSend(streamBuffer, buf, strlen(buf), pdMS_TO_TICKS(10));

	  osMutexRelease(myBufferMutexHandle);
    osDelay(1);
  }
  /* USER CODE END StartTaskSensor2 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (HAL_ADC_GetValue(hadc) > ADC_TRESHOLD)
	{
		osMutexWait(myBufferMutexHandle, osWaitForever);

		xStreamBufferSendFromISR(streamBuffer, "ADC value extend the treshold\n\r", strlen("ADC value extend the treshold\n\r"), NULL);

		osMutexRelease(myBufferMutexHandle);
	}

	HAL_ADC_Start_IT(hadc);
}
/* USER CODE END Application */

