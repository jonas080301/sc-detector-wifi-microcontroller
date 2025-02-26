// Copyright [2022] <Fabian Steger>

#ifndef DRIVERS_SENSORS_ACCELERATIONSENSOR_LSM6DSL_READ_H_
#define DRIVERS_SENSORS_ACCELERATIONSENSOR_LSM6DSL_READ_H_

#include "lsm6dsl.h" // der ISR

extern volatile float lsm6dsl_last_reading_acceleration_x__mpss;
extern volatile float lsm6dsl_last_reading_acceleration_y__mpss;
extern volatile float lsm6dsl_last_reading_acceleration_z__mpss;
extern volatile float lsm6dsl_last_reading_gyro_x__radps;
extern volatile float lsm6dsl_last_reading_gyro_y__radps;
extern volatile float lsm6dsl_last_reading_gyro_z__radps;
extern volatile float lsm6dsl_last_reading_temperature__degree_celsius;
extern volatile uint32_t lsm6dsl_last_reading_time__ticks;
extern volatile uint32_t lsm6dsl_last_reading_duration__ticks;

void lsm6dsl_init(void);
void lsm6dsl_update_globals(void);

#endif // DRIVERS_SENSORS_ACCELERATIONSENSOR_LSM6DSL_READ_H_
