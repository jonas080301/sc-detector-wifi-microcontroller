// Copyright [2022] <Fabian Steger>

#include "hts221_read.h"
#include "hts221.h"
#include "main.h"

volatile float hts221_last_reading_rel_humidity__percent = 0;
volatile float hts221_last_reading_temperature__degree_celsius = 0;
volatile uint32_t hts221_last_reading_time__ticks = 0;
volatile uint32_t hts221_last_reading_duration__ticks = 0;

void hts221_init(void) {
  // todo was passiert wenn chip nicht erreichbar?
  // volatile HAL_StatusTypeDef HAL_status;
  // HAL_status =
  HTS221_Initialize();
  __NOP();
}
void hts221_update_globals(void) {
  uint32_t start = HAL_GetTick();
  HTS221_VALUES values = hts221_data_ready_alter_ISR();
  hts221_last_reading_rel_humidity__percent = (float)values.humidity / 10.0F;
  hts221_last_reading_temperature__degree_celsius = values.temperature;
  hts221_last_reading_time__ticks = HAL_GetTick();
  hts221_last_reading_duration__ticks = HAL_GetTick() - start;
}
