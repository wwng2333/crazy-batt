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
#include "stm32g0xx_hal.h"
#include <stdio.h>
#include <string.h>
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
  typedef struct bq34_info_struct
  {
    uint8_t soc;
    uint16_t remaing_capacity;
    uint16_t full_charge_capacity;
    uint16_t voltage;
    int16_t average_current;
    int16_t temperature;
    uint8_t soh;
    uint16_t time_to_empty;
    uint16_t time_to_full;
    uint16_t cycle_count;
    int16_t recommended_charge_current;
    int16_t standby_current;
    uint16_t internal_temp;
    union flags
    {
      struct bits
      {
        uint8_t OTC : 1;
        uint8_t OTD : 1;
        uint8_t BATHI : 1;
        uint8_t BATLOW : 1;
        uint8_t CHG_INH : 1;
        uint8_t X_CHG : 1;
        uint8_t RSVD : 1;
        uint8_t FC : 1;
        uint8_t CHG : 1;
        uint8_t OCVTAKEN : 1;
        uint8_t ISD : 1;
        uint8_t SOC1 : 1;
        uint8_t SOCF : 1;
        uint8_t DSG : 1;
      } bits;
      uint16_t flag_bits;
    } flags;
  } bq34_info_struct;
	
	typedef struct hadc_result
  {
    uint16_t ch0;
    uint16_t ch2;
    uint16_t ch3;
    uint16_t ch6;
    uint16_t temp;
    uint16_t vref;
  } hadc_result;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
