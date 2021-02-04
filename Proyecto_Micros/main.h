/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI_S_R_Pin GPIO_PIN_3
#define SPI_S_R_GPIO_Port GPIOE
#define SERVO_Pin GPIO_PIN_0
#define SERVO_GPIO_Port GPIOA
#define BUTON_1_Pin GPIO_PIN_1
#define BUTON_1_GPIO_Port GPIOA
#define BUTON_1_EXTI_IRQn EXTI1_IRQn
#define BOTON_2_Pin GPIO_PIN_2
#define BOTON_2_GPIO_Port GPIOA
#define BOTON_2_EXTI_IRQn EXTI2_IRQn
#define BOTON_3_Pin GPIO_PIN_3
#define BOTON_3_GPIO_Port GPIOA
#define BOTON_3_EXTI_IRQn EXTI3_IRQn
#define BOTON_4_Pin GPIO_PIN_4
#define BOTON_4_GPIO_Port GPIOA
#define BOTON_4_EXTI_IRQn EXTI4_IRQn
#define BOTON_5_Pin GPIO_PIN_5
#define BOTON_5_GPIO_Port GPIOC
#define BOTON_5_EXTI_IRQn EXTI9_5_IRQn
#define LDR_Pin GPIO_PIN_0
#define LDR_GPIO_Port GPIOB
#define TEMPERATURA_Pin GPIO_PIN_1
#define TEMPERATURA_GPIO_Port GPIOB
#define TRIGGER_Pin GPIO_PIN_8
#define TRIGGER_GPIO_Port GPIOE
#define ECHO_Pin GPIO_PIN_9
#define ECHO_GPIO_Port GPIOE
#define PARTY_MODE_Pin GPIO_PIN_12
#define PARTY_MODE_GPIO_Port GPIOD
#define AUTOMATIC_MODE_Pin GPIO_PIN_13
#define AUTOMATIC_MODE_GPIO_Port GPIOD
#define MANUAL_MODE_Pin GPIO_PIN_14
#define MANUAL_MODE_GPIO_Port GPIOD
#define ESP_WORKING_Pin GPIO_PIN_15
#define ESP_WORKING_GPIO_Port GPIOD
#define ALARM_1_Pin GPIO_PIN_8
#define ALARM_1_GPIO_Port GPIOC
#define ALARM_2_Pin GPIO_PIN_9
#define ALARM_2_GPIO_Port GPIOC
#define LUZ_INTERIOR_Pin GPIO_PIN_1
#define LUZ_INTERIOR_GPIO_Port GPIOD
#define LUZ_GARAJE_Pin GPIO_PIN_2
#define LUZ_GARAJE_GPIO_Port GPIOD
#define LUZ_ALARMA_1_Pin GPIO_PIN_3
#define LUZ_ALARMA_1_GPIO_Port GPIOD
#define LUZ_ALARMA_2_Pin GPIO_PIN_4
#define LUZ_ALARMA_2_GPIO_Port GPIOD
#define LUZ_ALARMA_3_Pin GPIO_PIN_5
#define LUZ_ALARMA_3_GPIO_Port GPIOD
#define LUZ_ALARMA_4_Pin GPIO_PIN_6
#define LUZ_ALARMA_4_GPIO_Port GPIOD
#define LUZ_FUERA_1_Pin GPIO_PIN_4
#define LUZ_FUERA_1_GPIO_Port GPIOB
#define LUZ_FUERA_2_Pin GPIO_PIN_5
#define LUZ_FUERA_2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
