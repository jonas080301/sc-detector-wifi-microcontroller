/*
 * LIS3MDL.c
 *
 *  Created on: May 26, 2021
 *      Author: MarkusKrug
 */

#include "lis3mdl.h"
#include "main.h"

#include <stdbool.h>
#include <string.h>

/* variables used within this file as global ones */

static LIS3MDL_RAW_VALUES lis3mdl_raw_values;
static LIS3MDL_VALUES lis3mdl_values;
static LIS3MDL_CTRL lis3mdl_ctrl = {
    .ctrl1.reg_value =
        0xF0, /* temperature sensor enabled, ultra high performance mode for x/y
                 axis, ODR=10Hz, all other values set to defaults */
    .ctrl2.reg_value = 0x00, /* all values set to default, FS = +/- 4gauss */
    .ctrl3.reg_value =
        0x00, /* continuous conversion, all other values set to defaults */
    .ctrl4.reg_value = 0x0C,  /* ultra high performance mode for z axis, all
                                 other values set to defaults */
    .ctrl5.reg_value = 0x40}; /* BDU set on, all other values set to defaults */

/* note interrupt behavior is configured but not uses because the corresponding
 * pin of the sensor is not connected to the µC on the discovery board that is
 * used */
static LIS3MDLINTCFG lis3mdl_intcfg = {
    .intcfg.reg_value =
        0x27, /* interrupt when axis raise an interrupt, interupt enabled with
                 high active and latched interrupt pin */
    .intsrc.reg_value = 0x00, /* read only register */
    .intths.reg_value =
        0x6AE7}; /* set the threshold to just one bit below the maximum */

extern I2C_HandleTypeDef hi2c2;

/*
 * Initialize the LIS3MDL
 */
HAL_StatusTypeDef LIS3MDL_Initialize(void) {
  HAL_StatusTypeDef HAL_status;
  uint8_t buffer;

  /* check if the sensor is reachable */
  HAL_status =
      HAL_I2C_Mem_Read_DMA(&hi2c2, (LIS3MDL_I2C_ADDR << 1), LIS3MDL_WHO_AM_I,
                           I2C_MEMADD_SIZE_8BIT, &buffer, sizeof(buffer));
  while (hi2c2.State != HAL_I2C_STATE_READY)
    ;
  if (buffer != LIS3MDL_WHO_AM_I_VALUE)
    return HAL_ERROR;

  /* set the sensor configuration */
  HAL_status = HAL_I2C_Mem_Write_DMA(
      &hi2c2, (LIS3MDL_I2C_ADDR << 1), LIS3MDL_CTRL_REG1, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lis3mdl_ctrl, sizeof(lis3mdl_ctrl));
  while (hi2c2.State != HAL_I2C_STATE_READY)
    ;

  /* read back the sensor configuration to do a basic check */
  memset(&lis3mdl_ctrl, 0xFF, sizeof(lis3mdl_ctrl));
  HAL_status = HAL_I2C_Mem_Read_DMA(
      &hi2c2, (LIS3MDL_I2C_ADDR << 1), LIS3MDL_CTRL_REG1, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lis3mdl_ctrl, sizeof(lis3mdl_ctrl));
  while (hi2c2.State != HAL_I2C_STATE_READY)
    ;

  /* set the interrupt behavior */
  HAL_status = HAL_I2C_Mem_Write_DMA(
      &hi2c2, (LIS3MDL_I2C_ADDR << 1), LIS3MDL_INT_CFG, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lis3mdl_intcfg.intcfg, sizeof(lis3mdl_intcfg.intcfg));
  while (hi2c2.State != HAL_I2C_STATE_READY)
    ;
  HAL_status = HAL_I2C_Mem_Write_DMA(
      &hi2c2, (LIS3MDL_I2C_ADDR << 1), LIS3MDL_INT_THS_L, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lis3mdl_intcfg.intths, sizeof(lis3mdl_intcfg.intths));
  while (hi2c2.State != HAL_I2C_STATE_READY)
    ;

  /* dummy read to make sure the DRDY signal is reset */
  HAL_status = HAL_I2C_Mem_Read_DMA(
      &hi2c2, (LIS3MDL_I2C_ADDR << 1), LIS3MDL_OUT_X_L, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lis3mdl_raw_values.raw_x, sizeof(int16_t));
  while (hi2c2.State != HAL_I2C_STATE_READY)
    ;

  return HAL_status;
}

LIS3MDL_VALUES lis3mdl_data_ready_alte_ISR(void) {
  volatile HAL_StatusTypeDef HAL_status;
  /* start reading the magnetometer sensor value */
  HAL_status = HAL_I2C_Mem_Read_DMA(
      &hi2c2, (LIS3MDL_I2C_ADDR << 1), LIS3MDL_OUT_X_L, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lis3mdl_raw_values.raw_x, (3 * sizeof(int16_t)));
  while (hi2c2.State != HAL_I2C_STATE_READY) {
  }
  if (HAL_status != HAL_OK) {
    lis3mdl_values.ok = 0;
    return lis3mdl_values;
  }

  HAL_status = HAL_I2C_Mem_Read_DMA(
      &hi2c2, (LIS3MDL_I2C_ADDR << 1), LIS3MDL_TEMP_OUT_L, I2C_MEMADD_SIZE_8BIT,
      (uint8_t *)&lis3mdl_raw_values.raw_temperature, sizeof(int16_t));
  while (hi2c2.State != HAL_I2C_STATE_READY) {
  }
  if (HAL_status != HAL_OK) {
    lis3mdl_values.ok = 0;
    return lis3mdl_values;
  }

  lis3mdl_values.temperature = lis3mdl_raw_values.raw_temperature / 8.0 + 25.0;
  lis3mdl_values.x = lis3mdl_raw_values.raw_x / 6842.0;
  lis3mdl_values.y = lis3mdl_raw_values.raw_y / 6842.0;
  lis3mdl_values.z = lis3mdl_raw_values.raw_z / 6842.0;

  if (HAL_status == HAL_OK)
    lis3mdl_values.ok = 1;

  return lis3mdl_values;
}
