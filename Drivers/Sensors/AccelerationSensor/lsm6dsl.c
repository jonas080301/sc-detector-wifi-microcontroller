/*
 * LSM6DSL.c
 *
 *  Created on: May 26, 2021
 *      Author: MarkusKrug
 */

#include "lsm6dsl.h"
#include "main.h"
#include <limits.h>
#include <stdbool.h>
#include <string.h>
/* variables used within this file as global ones */

static LSM6DSL_RAW_VALUES lsm6dsl_raw_values;
static LSM6DSL_VALUES lsm6dsl_values;
static LSM6DSL_CTRL1 lsm6dsl_ctrl = {
    .ctrl1.reg_value =
        0x80, /* ODR = 1.6kHz for acc, all other values set to defaults */
    .ctrl2.reg_value =
        0x80, /* ODR = 1.6kHz for gyro, all other values set to defaults */
    .ctrl3.reg_value = 0x44,  /* BDU and autoincrement active, all other values
                                 set to defaults */
    .ctrl4.reg_value = 0x28,  /* DRDY_MASK = 1, all interrupts to pin int1, all
                                 other values set to defaults */
    .ctrl5.reg_value = 0x00,  /* all values set to defaults */
    .ctrl6.reg_value = 0x00,  /* all values set to defaults */
    .ctrl7.reg_value = 0x00,  /* all values set to defaults */
    .ctrl8.reg_value = 0x00,  /* all values set to defaults */
    .ctrl9.reg_value = 0x00,  /* all values set to defaults */
    .ctrl10.reg_value = 0x0}; /* all values set to defaults */

/* note interrupt behavior is configured but not uses because the corresponding
 * pin of the sensor is not connected to the µC on the discovery board that is
 * used */
static LSM6DSLINTCFG lsm6dsl_intcfg = {
    .int1ctrl.reg_value = 0x02,  /* interrupt when gyro has new data */
    .int2ctrl.reg_value = 0x00}; /* no interrupts on int2 pin */

extern I2C_HandleTypeDef hi2c2;

/*
 * Initialize the LSM6DSL
 */
HAL_StatusTypeDef LSM6DSL_Initialize(void) {
  HAL_StatusTypeDef HAL_status;
  uint8_t buffer;

  /* check if the sensor is reachable */
  HAL_status =
      HAL_I2C_Mem_Read_DMA(&hi2c2, (LSM6DSL_I2C_ADDR << 1), LSM6DSL_WHO_AM_I,
                           I2C_MEMADD_SIZE_8BIT, &buffer, sizeof(buffer));
  while (hi2c2.State != HAL_I2C_STATE_READY)
    ;
  if (buffer != LSM6DSL_WHO_AM_I_VALUE)
    return HAL_ERROR;

  /* set the sensor configuration */
  HAL_status = HAL_I2C_Mem_Write_DMA(
      &hi2c2, (LSM6DSL_I2C_ADDR << 1), LSM6DSL_CTRL1_XL, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lsm6dsl_ctrl, sizeof(lsm6dsl_ctrl));
  while (hi2c2.State != HAL_I2C_STATE_READY)
    ;

  /* read back the sensor configuration to do a basic check */
  memset(&lsm6dsl_ctrl, 0xFF, sizeof(lsm6dsl_ctrl));
  HAL_status = HAL_I2C_Mem_Read_DMA(
      &hi2c2, (LSM6DSL_I2C_ADDR << 1), LSM6DSL_CTRL1_XL, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lsm6dsl_ctrl, sizeof(lsm6dsl_ctrl));
  while (hi2c2.State != HAL_I2C_STATE_READY)
    ;

  /* set the interrupt behavior */
  HAL_status = HAL_I2C_Mem_Write_DMA(
      &hi2c2, (LSM6DSL_I2C_ADDR << 1), LSM6DSL_INT1_CTRL, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lsm6dsl_intcfg, sizeof(lsm6dsl_intcfg));
  while (hi2c2.State != HAL_I2C_STATE_READY)
    ;

  /* dummy read to make sure the DRDY signal is reset */
  HAL_status = HAL_I2C_Mem_Read_DMA(
      &hi2c2, (LSM6DSL_I2C_ADDR << 1), LSM6DSL_OUT_TEMP_L, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lsm6dsl_raw_values, sizeof(lsm6dsl_raw_values));
  while (hi2c2.State != HAL_I2C_STATE_READY)
    ;

  return HAL_status;
}

/**
 * @brief  EXTI line detection callback.
 * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
 * @retval None
 */
LSM6DSL_VALUES lsm6dsl_data_ready_alte_ISR(void) {
  HAL_StatusTypeDef HAL_status;
  /* start reading the acc-gyro sensor value */
  HAL_status = HAL_I2C_Mem_Read_DMA(
      &hi2c2, (LSM6DSL_I2C_ADDR << 1), LSM6DSL_OUT_TEMP_L, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lsm6dsl_raw_values, (sizeof(lsm6dsl_raw_values)));
  while (hi2c2.State != HAL_I2C_STATE_READY) {
  }

  if (HAL_status != HAL_OK) {
    lsm6dsl_values.ok = 0;
    return lsm6dsl_values;
  }

  // todo magic numbers raus
  lsm6dsl_values.temperature =
      lsm6dsl_raw_values.raw_temperature / 256.0 + 25.0;
  lsm6dsl_values.acc_x =
      lsm6dsl_raw_values.raw_acc_x * (4.0 / USHRT_MAX) * GRAVITY;
  lsm6dsl_values.acc_y =
      lsm6dsl_raw_values.raw_acc_y * (4.0 / USHRT_MAX) * GRAVITY;
  lsm6dsl_values.acc_z =
      lsm6dsl_raw_values.raw_acc_z * (4.0 / USHRT_MAX) * GRAVITY;
  // grad pro sekunde?? 500 ist magic - passt aber zum datenblatt (ein
  // messbereich)
  lsm6dsl_values.gyro_x = lsm6dsl_raw_values.raw_gyro_x * (500.0 / USHRT_MAX);
  lsm6dsl_values.gyro_y = lsm6dsl_raw_values.raw_gyro_y * (500.0 / USHRT_MAX);
  lsm6dsl_values.gyro_z = lsm6dsl_raw_values.raw_gyro_z * (500.0 / USHRT_MAX);

  if (HAL_status == HAL_OK)
    lsm6dsl_values.ok = 1;

  return lsm6dsl_values;
}