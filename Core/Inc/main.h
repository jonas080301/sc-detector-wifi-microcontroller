/**
  ******************************************************************************
  * @file    Inc/main.h
  * @author  Jonas
  * @brief   Header for main.c
  ******************************************************************************

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l475e_iot01.h"
#include "stm32l475e_iot01_accelero.h"
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_magneto.h"
#include "stm32l475e_iot01_psensor.h"
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_iwdg.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "es_wifi.h"
#include "wifi.h"

/* Exported constants --------------------------------------------------------*/

/**
 * @brief Supply voltage for analog components (in mV)
 */
#define VDDA_APPLI ((uint32_t)3300)

/**
 * @brief Internal temperature sensor parameters (typical values)
 */
#define INTERNAL_TEMPSENSOR_V30 ((int32_t)760)       ///< Voltage at 30°C (mV)
#define INTERNAL_TEMPSENSOR_AVGSLOPE ((int32_t)2500) ///< Slope (uV/°C)

/**
 * @brief Addresses of temperature calibration values in system memory
 */
#define TEMP30_CAL_ADDR                                                        \
  ((uint16_t *)((uint32_t)0x1FFF75A8)) ///< ADC raw at 30°C
#define TEMP110_CAL_ADDR                                                       \
  ((uint16_t *)((uint32_t)0x1FFF75CA)) ///< ADC raw at 110°C

/**
 * @brief Vdda during temperature calibration (in mV)
 */
#define VDDA_TEMP_CAL ((uint32_t)3000)

/* Exported macros -----------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief SPI3 interrupt handler (implemented in main.c)
 */
void SPI3_IRQHandler(void);

#endif /* __MAIN_H */