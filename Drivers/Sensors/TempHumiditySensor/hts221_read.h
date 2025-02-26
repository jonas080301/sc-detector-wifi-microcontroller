// Copyright [2022] <Fabian Steger>

#ifndef DRIVERS_SENSORS_TEMPHUMIDITYSENSOR_HTS221_READ_H_
#define DRIVERS_SENSORS_TEMPHUMIDITYSENSOR_HTS221_READ_H_

#include "hts221.h"
#include "main.h"

void hts221_init(void);
void hts221_update_globals(void);

extern volatile float hts221_last_reading_rel_humidity__percent;
extern volatile float hts221_last_reading_temperature__degree_celsius;
extern volatile uint32_t hts221_last_reading_time__ticks;
extern volatile uint32_t hts221_last_reading_duration__ticks;

#endif
