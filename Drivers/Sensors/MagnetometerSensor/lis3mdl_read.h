// Copyright [2022] <Fabian Steger>

#ifndef DRIVERS_SENSORS_MAGNETOMETERSENSOR_LIS3MDL_READ_H_
#define DRIVERS_SENSORS_MAGNETOMETERSENSOR_LIS3MDL_READ_H_

#include "lis3mdl.h" // der ISR

extern volatile float lis3mdl_last_reading_mag_x__gauss;
extern volatile float lis3mdl_last_reading_mag_y__gauss;
extern volatile float lis3mdl_last_reading_mag_z__gauss;
extern volatile float lis3mdl_last_reading_temperature__degree_celsius;
extern volatile uint32_t lis3mdl_last_reading_time__ticks;
extern volatile uint32_t lis3mdl_last_reading_duration__ticks;

void lis3mdl_init(void);
void lis3mdl_update_globals(void);
#endif // DRIVERS_SENSORS_MAGNETOMETERSENSOR_LIS3MDL_READ_H_
