// Copyright [2022] <Fabian Steger>
// basiert auf Code Beispiel von Markus Krug

#include "vl53l0x_read.h"
#include "main.h" // VL53L0X_XSHUT_Pin
#include "vl53l0x/vl53l0x_api.h"
#include "vl53l0x/vl53l0x_api_core.h" //VL53L0X_measurement_poll_for_completion

extern I2C_HandleTypeDef hi2c2; // initialisiert in der main

// Markus Krug wuenscht sich als interface wegen Matlab
// plain floats global, und ganz klar keine structs
volatile float vl53l0x_last_reading_distance__meters;
volatile uint32_t vl53l0x_last_reading_time__ticks;
volatile uint32_t vl53l0x_last_reading_duration__ticks;

#define VL53L0X_I2C_ADDR 0x29 // Seite 29 um2153 <- i2c2 P47 um2153
#define VL53L0X_ID ((uint16_t)0xEEAA)

volatile VL53L0X_Dev_t
    Dev; // nicht static machen vl53l0x_tof.c greift drauf zu - mist!!!

// static void PollingExample(void) {
//   vl53l0x_init();
//   while (1) {
//     volatile float prox_value = 0;
//     prox_value = vl53l0x_read_distance__meter();
//   }
// }

/**
 * @brief  VL53L0X proximity sensor Initialization.
 */
void vl53l0x_init(void) {

  // Standardwerte
  vl53l0x_last_reading_distance__meters = INFINITY;
  vl53l0x_last_reading_time__ticks = 0;
  vl53l0x_last_reading_duration__ticks = 0;

  HAL_Delay(100);
  HAL_GPIO_WritePin(VL53L0X_XSHUT_GPIO_Port, VL53L0X_XSHUT_Pin,
                    GPIO_PIN_SET); // Xshutdown pin, Active LOW
  // XSHUT pin must always be driven to avoid leakage
  //  current. Pull-up is needed if the host
  //  state is not known.
  //  XSHUT is needed to use HW standby mode (no I2C comm).
  //  Note: XSHUT and GPIO1 pull up recommended values are 10k Ohms
  HAL_Delay(10); // tboot after Xshut is 1.2 ms max - DS p 14

  uint16_t vl53l0x_id = 0;
  VL53L0X_DeviceInfo_t VL53L0X_DeviceInfo;

  Dev.I2cHandle = &hi2c2;
  Dev.I2cDevAddr = ((uint16_t)(VL53L0X_I2C_ADDR)) << 1;

  memset(&VL53L0X_DeviceInfo, 0, sizeof(VL53L0X_DeviceInfo_t));

  if (VL53L0X_ERROR_NONE == VL53L0X_GetDeviceInfo(&Dev, &VL53L0X_DeviceInfo)) {
    if (VL53L0X_ERROR_NONE ==
        VL53L0X_RdWord(&Dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID,
                       &vl53l0x_id)) {
      if (vl53l0x_id == VL53L0X_ID) {
        if (VL53L0X_ERROR_NONE == VL53L0X_DataInit(&Dev)) {
          Dev.Present = 1;
          SetupSingleShot(Dev);
        } else {
          while (1) {
            // todo!!!!! das ist nix so!
          }
        }
      }
    } else {
      while (1) {
        // todo!!!!! das ist nix so!
      }
    }
  } else {
  }
}

// /**
//  * @brief GPIO1 of sensor triggered hw interrupt of uC, now reading the
//  values
//  * and storing to globals
//  *
//  */
// void vl53l0x_data_ready_ISR(void) {
//   extern UART_HandleTypeDef huart1;
//   char uart_tx_buffer[100];
//   snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
//            "detected rising edge: rangesensor\r\n");
//   HAL_UART_Transmit(&huart1, (uint8_t *)uart_tx_buffer,
//   strlen(uart_tx_buffer),
//                     HAL_MAX_DELAY);

//   vl53l0_update_globals();
// }

void vl53l0_update_globals(void) {
  uint32_t start = HAL_GetTick();

  // 32 bit controller atomarer zugriff
  vl53l0x_last_reading_distance__meters = vl53l0x_read_distance__meter();
  vl53l0x_last_reading_time__ticks = HAL_GetTick();
  vl53l0x_last_reading_duration__ticks = HAL_GetTick() - start;
}

static VL53L0X_Error FS_Status_1 = VL53L0X_ERROR_NONE;
static VL53L0X_Error FS_Status_2 = VL53L0X_ERROR_NONE;
static VL53L0X_DeviceModes FS_DeviceMode;
static uint32_t measurement_triggered__ticks = 0;

VL53L0X_Error FS_VL53L0X_PerformSingleMeasurement_Part1(VL53L0X_DEV Dev) {
  FS_Status_1 = VL53L0X_ERROR_NONE;

  /* Get Current DeviceMode */
  FS_Status_1 = VL53L0X_GetDeviceMode(Dev, &FS_DeviceMode);

  /* Start immediately to run a single ranging measurement in case of
   * single ranging or single histogram */
  if (FS_Status_1 == VL53L0X_ERROR_NONE &&
      FS_DeviceMode == VL53L0X_DEVICEMODE_SINGLE_RANGING)
    FS_Status_1 = VL53L0X_StartMeasurement(Dev);

  return FS_Status_1;
}

VL53L0X_Error FS_VL53L0X_PerformSingleMeasurement_Part2(VL53L0X_DEV Dev) {
  if (FS_Status_1 == VL53L0X_ERROR_NONE)
    FS_Status_1 = VL53L0X_measurement_poll_for_completion(Dev);

  /* Change PAL State in case of single ranging or single histogram */
  if (FS_Status_1 == VL53L0X_ERROR_NONE &&
      FS_DeviceMode == VL53L0X_DEVICEMODE_SINGLE_RANGING)
    PALDevDataSet(Dev, PalState, VL53L0X_STATE_IDLE);

  return FS_Status_1;
}

VL53L0X_Error
FS_VL53L0X_PerformSingleRangingMeasurement_Part1(VL53L0X_DEV Dev) {
  FS_Status_2 = VL53L0X_ERROR_NONE;

  /* This function will do a complete single ranging
   * Here we fix the mode! */
  FS_Status_2 = VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);

  if (FS_Status_2 == VL53L0X_ERROR_NONE) {
    FS_Status_2 = FS_VL53L0X_PerformSingleMeasurement_Part1(Dev);
  }
  return FS_Status_2;
}

VL53L0X_Error FS_VL53L0X_PerformSingleRangingMeasurement_Part2(
    VL53L0X_DEV Dev,
    VL53L0X_RangingMeasurementData_t *pRangingMeasurementData) {
  if (FS_Status_2 == VL53L0X_ERROR_NONE) {
    FS_Status_2 = FS_VL53L0X_PerformSingleMeasurement_Part2(Dev);
  }

  if (FS_Status_2 == VL53L0X_ERROR_NONE)
    FS_Status_2 =
        VL53L0X_GetRangingMeasurementData(Dev, pRangingMeasurementData);

  if (FS_Status_2 == VL53L0X_ERROR_NONE)
    FS_Status_2 = VL53L0X_ClearInterruptMask(Dev, 0);

  return FS_Status_2;
}

VL53L0X_Error FS_VL53L0X_PerformSingleRangingMeasurement(
    VL53L0X_DEV Dev,
    VL53L0X_RangingMeasurementData_t *pRangingMeasurementData) {
  static uint32_t next_part = 1;
  static VL53L0X_Error FS_Status_3 = VL53L0X_ERROR_NONE;
  VL53L0X_Error returnwert = VL53L0X_ERROR_NONE;
  if (next_part <= 1) {
    FS_Status_3 = VL53L0X_ERROR_NONE;
    FS_Status_3 = FS_VL53L0X_PerformSingleRangingMeasurement_Part1(Dev);
    next_part = 2;
  }
  if (next_part >= 2) {
    FS_Status_3 = FS_VL53L0X_PerformSingleRangingMeasurement_Part2(
        Dev, pRangingMeasurementData);
    next_part = 1;
    returnwert = FS_Status_3;
  }
  // jetzt waere der drdy pin high
  if (next_part <= 1) {
    FS_Status_3 = VL53L0X_ERROR_NONE;
    FS_Status_3 = FS_VL53L0X_PerformSingleRangingMeasurement_Part1(Dev);
    measurement_triggered__ticks = HAL_GetTick();
    next_part = 2;
  }
  // jetzt geht er drdy leider nicht high
  return returnwert;
}
uint8_t vl53l0x_please_poll_for_result(void) {
  if (HAL_GetTick() - measurement_triggered__ticks > 35)
    return 1;
  else
    return 0;
}
/**
 * @brief  Get distance from VL53L0X range sensor.
 * @retval Distance in mm
 */
float vl53l0x_read_distance__meter(void) {
  float returnwert;
  VL53L0X_RangingMeasurementData_t RangingMeasurementData;

  FS_VL53L0X_PerformSingleRangingMeasurement(&Dev, &RangingMeasurementData);
  // das dauert 30 ms! wegen VL53L0X_measurement_poll_for_completion() todo -
  // gleich triggern fuer den naechsten loop, dann ist die zeit weg
  if (RangingMeasurementData.RangeMilliMeter > 0) {
    returnwert = (float)(RangingMeasurementData.RangeMilliMeter) / 1000.0F;
  } else {
    returnwert = INFINITY;
  }
  return returnwert;
  // Megacps which means mege counts per seconds, it's a light energy unit. we
  // use photonic counts number to indicate the light strength as we based on
  // SPAD.

  //  RangingMeasurementData.SignalRateRtnMegaCps 1466880->45568  //
  //  SignalRateRtnMegaCps is signal energy indicator which shows how many
  //  photons bounce back from target to sensor. Normally short distance or high
  //  reflectance target will give high SignalRateRtnMegaCps vlaue. vice versa.

  //  RangingMeasurementData.AmbientRateRtnMegaCps  2560 -> 4608  //
  //  AmbientRateRtnMegaCps is noise energy indicator. there may some 940nm
  //  light in the surrounding, which means noise to TOF sensor, and this value
  //  indicate how much the noise it. if AMB is very high, then the maximum
  //  ranging distance will drop and accuracy will decrease.

  //  RangingMeasurementData.EffectiveSpadRtnCount  25092->50948  //
  //  EffectiveSpadRtnCount which means the SPAD number this frame are using,
  //  normally if target is far from sensor, sensor will enable all the
  //  available SPADs, if target distace is short, system may use less SPADS.
}
