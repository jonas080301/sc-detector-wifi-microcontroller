// Copyright [2023] <mail@fabian-steger.de>

#ifndef DRIVERS_SENSORS_ALL_I2C_SENSORS_H_
#define DRIVERS_SENSORS_ALL_I2C_SENSORS_H_

#include "./AccelerationSensor/lsm6dsl_read.h"
#include "./MagnetometerSensor/lis3mdl_read.h"
#include "./PressureSensor/lps22hb_read.h"
#include "./RangeSensor/vl53l0x_read.h"
#include "./TempHumiditySensor/hts221_read.h"

void all_i2c_sensors_poll(void);

#endif // DRIVERS_SENSORS_ALL_I2C_SENSORS_H_
