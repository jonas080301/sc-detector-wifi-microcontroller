// Copyright [2022] <Fabian Steger>

#ifndef DRIVERS_SENSORS_PRESSURESENSOR_LPS22HB_READ_H_
#define DRIVERS_SENSORS_PRESSURESENSOR_LPS22HB_READ_H_

#include "lps22hb.h" // der ISR

extern volatile float lsp22hb_last_reading_pressure__pascal;
extern volatile float lsp22hb_last_reading_temperature__degree_celsius;
extern volatile uint32_t lsp22hb_last_reading_time__ticks;
extern volatile uint32_t lsp22hb_last_reading_duration__ticks;

void lps22hb_init(void);
void lsp22hb_update_globals(void);
#endif // DRIVERS_SENSORS_PRESSURESENSOR_LPS22HB_READ_H_
