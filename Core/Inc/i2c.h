/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    i2c.h
 * @brief   This file contains all the function prototypes for
 *          the i2c.c file
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
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

  /* USER CODE BEGIN Includes */

  /* USER CODE END Includes */

  /* USER CODE BEGIN Private defines */

#define BIG_LITTLE_SWAP16(x) ((((*(uint16_t *)&x) & 0xff00) >> 8) | \
                              (((*(uint16_t *)&x) & 0x00ff) << 8))
void SCL_Test(void);
  void bq34z100_IT_enable(void);
  int bq34_read(uint8_t cmd, uint8_t len);
  uint8_t bq34z100_get_soc(void);
  uint16_t bq34z100_get_remaining_capacity(void);
  uint16_t bq34z100_get_full_charge_capacity(void);
  int16_t bq34z100_get_temperature(void);
  int16_t bq34z100_get_average_current(void);
  uint16_t bq34z100_get_voltage(void);
  uint8_t bq34z100_get_soh(void);
  uint16_t bq34z100_get_time_to_empty(void);
  uint16_t bq34z100_get_time_to_full(void);
  uint16_t bq34z100_get_cycle_count(void);
  int16_t bq34z100_get_current(void);
  int16_t bq34z100_get_recommended_charge_current(void);
  uint16_t bq34z100_get_recommended_charge_volt(void);
	//void bq34z100_get_all_info(bq34_info_struct *info);
	//void bq34z100_get_packed_batt_info(smart_batt_info_struct *info);
	void I2C_Soft_Init(void);
  /* USER CODE END Private defines */

  /* USER CODE BEGIN Prototypes */

  /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */
