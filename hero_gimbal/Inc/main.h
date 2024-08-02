/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "imuuart.h"

#include "stm32f4xx_hal_can.h"
#include "stdbool.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "math.h"
#include "stdlib.h"
#include "string.h"
#include "arm_math.h"
#include "stdio.h"
#include "tim.h"

//components->algorithm
#include "kalman_filter.h"
#include "mydef.h"
#include "myfun.h"

//components->contoller
#include "pid.h"
#include "ffc.h"

//components->devices

//boards
#include "bspcan.h"
#include "dbus.h"
#include "cap.h"
#include "vision.h"
//application
//#include "motor.h"
#include "crc.h"

//task
//#include "protect_task.h"
//#include "shoot_task.h"
//#include "gimbal_task.h"
//#include "chassis_task.h"
//#include "INS_task.h"


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
//extern uint8_t state_judge;
//uint8_t state_judge;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
void SYSTEM_InitPeripheral(void);

//typedef enum
//{
//	NOMAL,		//普通模式
//	CLIMBING,	//爬坡模式
//}EnvironmentModeType;
//typedef enum
//{
//	KEYBOARD,
//	REMOTE,
//	UNUSUAL,
//}ControlModeType;
//typedef enum
//{
//    Starting = 0,
//    Running = 1,
//} SYSTEMVALUE;
//extern volatile ControlModeType ControlMode;
//extern volatile EnvironmentModeType EnvironmentMode;
//extern volatile SYSTEMVALUE SystemValue;	//程序状态
//extern uint32_t control_judge_flag;
//extern uint8_t ALL_DATA_BUFF[30];
//extern uint8_t Send_buf[5];

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Lazer_Pin GPIO_PIN_8
#define Lazer_GPIO_Port GPIOC
#define SIGNAL_Pin GPIO_PIN_1
#define SIGNAL_GPIO_Port GPIOF
#define SIGNAL_EXTI_IRQn EXTI1_IRQn
#define RSTN_IST8310_Pin GPIO_PIN_6
#define RSTN_IST8310_GPIO_Port GPIOG
#define DRDY_IST8310_Pin GPIO_PIN_3
#define DRDY_IST8310_GPIO_Port GPIOG
#define DRDY_IST8310_EXTI_IRQn EXTI3_IRQn
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
#define INT1_GYRO_Pin GPIO_PIN_5
#define INT1_GYRO_GPIO_Port GPIOC
#define INT1_GYRO_EXTI_IRQn EXTI9_5_IRQn
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
