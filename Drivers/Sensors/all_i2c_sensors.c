// Copyright [2023] <mail@fabian-steger.de>

#include "all_i2c_sensors.h"

// achtung am I2C2 muss dma an sein
// isrs fuer dma muessen an sein
// isrs fuer dien i2c event interrupt muessen an sein

void all_i2c_sensors_poll(void) {
  // RangeSensor
  if (vl53l0x_please_poll_for_result() ||
      HAL_GPIO_ReadPin(VL53L0X_GPIO1_EXTI7_GPIO_Port,
                       VL53L0X_GPIO1_EXTI7_Pin)) {
    vl53l0_update_globals();
  }

  // // Temphumidity
  if (HAL_GPIO_ReadPin(HTS221_DRDY_EXTI15_GPIO_Port, HTS221_DRDY_EXTI15_Pin)) {
    hts221_update_globals();
  }

  // Pressure
  if (HAL_GPIO_ReadPin(LPS22HB_INT_DRDY_EXTI10_GPIO_Port,
                       LPS22HB_INT_DRDY_EXTI10_Pin)) {
    lsp22hb_update_globals();
  }

  // Magnetometer
  // When measurement data is available the sensor will pull this pin LOW????
  // warum dann HIGH abfragen? todo
  if (HAL_GPIO_ReadPin(LSM3MDL_DRDY_EXTI8_GPIO_Port, LSM3MDL_DRDY_EXTI8_Pin)) {
    lis3mdl_update_globals();
  }

  // Acceleration
  if (HAL_GPIO_ReadPin(LSM6DSL_INT1_EXTI11_GPIO_Port,
                       LSM6DSL_INT1_EXTI11_Pin)) {
    lsm6dsl_update_globals();
  }
}
