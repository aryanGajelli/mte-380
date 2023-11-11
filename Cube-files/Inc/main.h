/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define MOTOR_L_IN1_Pin GPIO_PIN_0
#define MOTOR_L_IN1_GPIO_Port GPIOC
#define MOTOR_L_IN2_Pin GPIO_PIN_1
#define MOTOR_L_IN2_GPIO_Port GPIOC
#define IMU_MISO_Pin GPIO_PIN_2
#define IMU_MISO_GPIO_Port GPIOC
#define IMU_MOSI_Pin GPIO_PIN_3
#define IMU_MOSI_GPIO_Port GPIOC
#define MOTOR_R_EN_Pin GPIO_PIN_1
#define MOTOR_R_EN_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define DIST_IR_Pin GPIO_PIN_0
#define DIST_IR_GPIO_Port GPIOB
#define COLOR_S3_Pin GPIO_PIN_1
#define COLOR_S3_GPIO_Port GPIOB
#define IMU_SCK_Pin GPIO_PIN_10
#define IMU_SCK_GPIO_Port GPIOB
#define COLOR_3_CS_Pin GPIO_PIN_12
#define COLOR_3_CS_GPIO_Port GPIOB
#define COLOR_S0_Pin GPIO_PIN_13
#define COLOR_S0_GPIO_Port GPIOB
#define COLOR_S1_Pin GPIO_PIN_14
#define COLOR_S1_GPIO_Port GPIOB
#define COLOR_S2_Pin GPIO_PIN_15
#define COLOR_S2_GPIO_Port GPIOB
#define IMU_CS_Pin GPIO_PIN_10
#define IMU_CS_GPIO_Port GPIOA
#define COLOR_2_CS_Pin GPIO_PIN_11
#define COLOR_2_CS_GPIO_Port GPIOA
#define COLOR_1_CS_Pin GPIO_PIN_12
#define COLOR_1_CS_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define MOTOR_L_EN_Pin GPIO_PIN_15
#define MOTOR_L_EN_GPIO_Port GPIOA
#define MOTOR_R_IN4_Pin GPIO_PIN_10
#define MOTOR_R_IN4_GPIO_Port GPIOC
#define MOTOR_R_IN3_Pin GPIO_PIN_12
#define MOTOR_R_IN3_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define COLOR_IN_Pin GPIO_PIN_6
#define COLOR_IN_GPIO_Port GPIOB
#define SERVO_PWM_Pin GPIO_PIN_9
#define SERVO_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
