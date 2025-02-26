// Copyright [2022] <Fabian Steger>
#include "lis3mdl_read.h"

// Markus Krug wuenscht sich als interface wegen Matlab
// plain floats global, und ganz klar keine structs
volatile float lis3mdl_last_reading_mag_x__gauss;
volatile float lis3mdl_last_reading_mag_y__gauss;
volatile float lis3mdl_last_reading_mag_z__gauss;
volatile float lis3mdl_last_reading_temperature__degree_celsius;
volatile uint32_t lis3mdl_last_reading_time__ticks;
volatile uint32_t lis3mdl_last_reading_duration__ticks;

void lis3mdl_init(void) {
  LIS3MDL_Initialize();
  lis3mdl_last_reading_mag_x__gauss = 0;
  lis3mdl_last_reading_mag_y__gauss = 0;
  lis3mdl_last_reading_mag_z__gauss = 0;
  lis3mdl_last_reading_temperature__degree_celsius = 0;
  lis3mdl_last_reading_time__ticks = 0;
  lis3mdl_last_reading_duration__ticks = 0;
}
void lis3mdl_update_globals() {
  uint32_t start = HAL_GetTick();
  LIS3MDL_VALUES values = lis3mdl_data_ready_alte_ISR();
  // das bei allen sensoren
  if (values.ok) {
    lis3mdl_last_reading_mag_x__gauss = values.x;
    lis3mdl_last_reading_mag_y__gauss = values.y;
    lis3mdl_last_reading_mag_z__gauss = values.z;
    lis3mdl_last_reading_temperature__degree_celsius = values.temperature;
    lis3mdl_last_reading_time__ticks = HAL_GetTick();
    lis3mdl_last_reading_duration__ticks = HAL_GetTick() - start;
  }

  // todo was ist da los? // warum muss man das machen?
  LIS3MDL_Initialize();
}
