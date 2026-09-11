/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* VM和补光灯仅输出控制电平，须接外部使能/功率驱动电路，不直接带负载。 */
#define VM_EN_Pin GPIO_PIN_0
#define VM_EN_GPIO_Port GPIOD
#define CAMERA_LIGHT_EN_Pin GPIO_PIN_1
#define CAMERA_LIGHT_EN_GPIO_Port GPIOD
/* 启动按键上拉输入，按下接地；这里只定义电平，不包含消抖和启动动作。 */
#define START_KEY1_Pin GPIO_PIN_2
#define START_KEY1_GPIO_Port GPIOD
#define START_KEY2_Pin GPIO_PIN_3
#define START_KEY2_GPIO_Port GPIOD
/* 已配置外设的用途标识：摄像头串口及夹爪舵机PWM，尚无应用协议。 */
#define CAMERA_TX_Pin GPIO_PIN_10
#define CAMERA_TX_GPIO_Port GPIOB
#define CAMERA_RX_Pin GPIO_PIN_11
#define CAMERA_RX_GPIO_Port GPIOB
#define GRIPPER_PWM_Pin GPIO_PIN_9
#define GRIPPER_PWM_GPIO_Port GPIOE

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
