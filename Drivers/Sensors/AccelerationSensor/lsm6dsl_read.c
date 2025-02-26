// Copyright [2022] <Fabian Steger>
#include "lsm6dsl_read.h"
#include "lsm6dsl.h"
#include "main.h"

volatile float lsm6dsl_last_reading_acceleration_x__mpss;
volatile float lsm6dsl_last_reading_acceleration_y__mpss;
volatile float lsm6dsl_last_reading_acceleration_z__mpss;
volatile float lsm6dsl_last_reading_gyro_x__radps;
volatile float lsm6dsl_last_reading_gyro_y__radps;
volatile float lsm6dsl_last_reading_gyro_z__radps;
volatile float lsm6dsl_last_reading_temperature__degree_celsius;
volatile uint32_t lsm6dsl_last_reading_time__ticks;
volatile uint32_t lsm6dsl_last_reading_duration__ticks;

void lsm6dsl_init(void) {
  LSM6DSL_Initialize();
  lsm6dsl_last_reading_acceleration_x__mpss = 0;
  lsm6dsl_last_reading_acceleration_y__mpss = 0;
  lsm6dsl_last_reading_acceleration_z__mpss = 0;
  lsm6dsl_last_reading_gyro_x__radps = 0;
  lsm6dsl_last_reading_gyro_y__radps = 0;
  lsm6dsl_last_reading_gyro_z__radps = 0;
  lsm6dsl_last_reading_temperature__degree_celsius = 0;
  lsm6dsl_last_reading_time__ticks = 0;
  lsm6dsl_last_reading_duration__ticks = 0;
}
void lsm6dsl_update_globals(void) {
  uint32_t start = HAL_GetTick();
  LSM6DSL_VALUES values = lsm6dsl_data_ready_alte_ISR();
  // das bei allen sensoren
  if (values.ok) {
    lsm6dsl_last_reading_acceleration_x__mpss = values.acc_x;
    lsm6dsl_last_reading_acceleration_y__mpss = values.acc_y;
    lsm6dsl_last_reading_acceleration_z__mpss = values.acc_z;
    lsm6dsl_last_reading_gyro_x__radps =
        values.gyro_x / 360.0F * 2.0F * 3.1416F;
    lsm6dsl_last_reading_gyro_y__radps =
        values.gyro_y / 360.0F * 2.0F * 3.1416F;
    lsm6dsl_last_reading_gyro_z__radps =
        values.gyro_z / 360.0F * 2.0F * 3.1416F;
    lsm6dsl_last_reading_temperature__degree_celsius = values.temperature;
    lsm6dsl_last_reading_time__ticks = HAL_GetTick();
    lsm6dsl_last_reading_duration__ticks = HAL_GetTick() - start;
  }
}
