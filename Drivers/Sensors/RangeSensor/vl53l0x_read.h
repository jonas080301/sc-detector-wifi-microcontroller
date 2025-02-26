// Copyright [2022] <Fabian Steger>

#ifndef DRIVERS_SENSORS_RANGESENSOR_VL53L0X_READ_H_
#define DRIVERS_SENSORS_RANGESENSOR_VL53L0X_READ_H_

#include "main.h"
#include <math.h>
#include <stdint.h>

void vl53l0x_init(void);
float vl53l0x_read_distance__meter(void);
// void vl53l0x_data_ready_ISR(void);
void vl53l0_update_globals(void);
uint8_t vl53l0x_please_poll_for_result(void);

// Markus Krug wuenscht sich als interface wegen Matlab
// plain floats global, und ganz klar keine structs
extern volatile float vl53l0x_last_reading_distance__meters;
extern volatile uint32_t vl53l0x_last_reading_time__ticks;
extern volatile uint32_t vl53l0x_last_reading_duration__ticks;
#endif