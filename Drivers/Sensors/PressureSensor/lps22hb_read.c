// Copyright [2022] <Fabian Steger>
#include "lps22hb_read.h"
#include "lps22hb.h"
#include "main.h"

// Markus Krug wuenscht sich als interface wegen Matlab
// plain floats global, und ganz klar keine structs
volatile float lsp22hb_last_reading_pressure__pascal;
volatile float lsp22hb_last_reading_temperature__degree_celsius;
volatile uint32_t lsp22hb_last_reading_time__ticks;
volatile uint32_t lsp22hb_last_reading_duration__ticks;

void lps22hb_init(void) {
  LPS22HB_Initialize();
  lsp22hb_last_reading_pressure__pascal = 0;
  lsp22hb_last_reading_temperature__degree_celsius = 0;
  lsp22hb_last_reading_time__ticks = 0;
  lsp22hb_last_reading_duration__ticks = 0;
}
void lsp22hb_update_globals(void) {
  uint32_t start = HAL_GetTick();
  LPS22HB_VALUES values = lsp22hb_data_ready_alter_ISR();
  lsp22hb_last_reading_pressure__pascal =
      values.pressure * 100.0F; // von hpa auf SI Einheit Pascal
  lsp22hb_last_reading_temperature__degree_celsius = values.temperature;
  lsp22hb_last_reading_time__ticks = HAL_GetTick();
  lsp22hb_last_reading_duration__ticks = HAL_GetTick() - start;
}
