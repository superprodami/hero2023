/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

#include "INS_task.h"
#include "Info_Transmit_task.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
osThreadId Info_Transmit_handle;
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
osThreadId testHandle;
osThreadId ShowRunningTaskHandle;
osThreadId ChassisTaskHandle;
osThreadId OutControlTaskHandle;
osThreadId GimbalTaskHandle;
osThreadId ShootTaskHandle;
osThreadId Info_TransmitHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void test_task(void const * argument);
void ShowRunningFun(void const * argument);
void ChassisFun(void const * argument);
void OutControlFun(void const * argument);
void GimbalFun(void const * argument);
void ShootFun(void const * argument);
void Info_Transmit_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];
  
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )  
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}                   
/* USER CODE END GET_TIMER_TASK_MEMORY */

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
  /* definition and creation of test */
  osThreadDef(test, test_task, osPriorityNormal, 0, 128);
  testHandle = osThreadCreate(osThread(test), NULL);

  /* definition and creation of ShowRunningTask */
  osThreadDef(ShowRunningTask, ShowRunningFun, osPriorityRealtime, 0, 128);
  ShowRunningTaskHandle = osThreadCreate(osThread(ShowRunningTask), NULL);

  /* definition and creation of ChassisTask */
  osThreadDef(ChassisTask, ChassisFun, osPriorityHigh, 0, 128);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  /* definition and creation of OutControlTask */
  osThreadDef(OutControlTask, OutControlFun, osPriorityAboveNormal, 0, 128);
  OutControlTaskHandle = osThreadCreate(osThread(OutControlTask), NULL);

  /* definition and creation of GimbalTask */
  osThreadDef(GimbalTask, GimbalFun, osPriorityHigh, 0, 128);
  GimbalTaskHandle = osThreadCreate(osThread(GimbalTask), NULL);

  /* definition and creation of ShootTask */
  osThreadDef(ShootTask, ShootFun, osPriorityHigh, 0, 128);
  ShootTaskHandle = osThreadCreate(osThread(ShootTask), NULL);

  /* definition and creation of Info_Transmit */
  osThreadDef(Info_Transmit, Info_Transmit_task, osPriorityHigh, 0, 1024);
  Info_TransmitHandle = osThreadCreate(osThread(Info_Transmit), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
//    osThreadDef(Info_Transmit, Info_Transmit_task, osPriorityHigh, 0, 1024);
//    Info_Transmit_handle = osThreadCreate(osThread(Info_Transmit), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_test_task */
/**
  * @brief  Function implementing the test thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_test_task */
__weak void test_task(void const * argument)
{
  /* USER CODE BEGIN test_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END test_task */
}

/* USER CODE BEGIN Header_ShowRunningFun */
/**
* @brief Function implementing the ShowRunningTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ShowRunningFun */
__weak void ShowRunningFun(void const * argument)
{
  /* USER CODE BEGIN ShowRunningFun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ShowRunningFun */
}

/* USER CODE BEGIN Header_ChassisFun */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisFun */
__weak void ChassisFun(void const * argument)
{
  /* USER CODE BEGIN ChassisFun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ChassisFun */
}

/* USER CODE BEGIN Header_OutControlFun */
/**
* @brief Function implementing the OutControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OutControlFun */
__weak void OutControlFun(void const * argument)
{
  /* USER CODE BEGIN OutControlFun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OutControlFun */
}

/* USER CODE BEGIN Header_GimbalFun */
/**
* @brief Function implementing the GimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GimbalFun */
__weak void GimbalFun(void const * argument)
{
  /* USER CODE BEGIN GimbalFun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END GimbalFun */
}

/* USER CODE BEGIN Header_ShootFun */
/**
* @brief Function implementing the ShootTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ShootFun */
__weak void ShootFun(void const * argument)
{
  /* USER CODE BEGIN ShootFun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ShootFun */
}

/* USER CODE BEGIN Header_Info_Transmit_task */
/**
* @brief Function implementing the Info_Transmit thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Info_Transmit_task */
__weak void Info_Transmit_task(void const * argument)
{
  /* USER CODE BEGIN Info_Transmit_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Info_Transmit_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */
