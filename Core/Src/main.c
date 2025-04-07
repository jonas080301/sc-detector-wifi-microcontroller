/**
 ******************************************************************************
 * @file    Wifi/WiFi_HTTP_Server/src/main.c
 * @author  Jonas
 * @brief   This file provides main program functions
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2017 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private defines -----------------------------------------------------------*/
// Scenario states
typedef enum {
  NO_SCENARIO,
  POSSIBLE_SCENARIO,
  SCENARIO_DETECTED
} ScenarioState;

ScenarioState scenario_state = NO_SCENARIO;

// Deceleration capability in mm/s² (e.g., 1000 mm/s² = 1 m/s²)
const float max_decel = 1000.0f;

/* Update SSID and PASSWORD with own Access point settings */
#define SSID "WLAN-070011"
#define PASSWORD "6313476839050187"
#define PORT 80
#define MAX_SEGMENT_SIZE                                                       \
  512 // Definiere hier die maximale Segmentgröße (anpassen, falls nötig)

#define TERMINAL_USE

#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT 10000
#define SOCKET 0

#ifdef TERMINAL_USE
#define LOG(a) printf a
#else
#define LOG(a)
#endif
#define ALPHA 0.1f // Filterstärke (0.0 = nur alt, 1.0 = nur neu)
#define DIST_SAMPLE_INTERVAL 0.1f // 100ms zwischen Messungen (Sekunden)
#define CALIB_SAMPLES 100
#define DIST_AVG_SAMPLES 3
#define ACC_DT 0.1f // Zeitabstand in Sekunden (100ms)
// Optional: Beschleunigungs-Schwellenwert, um Rauschen zu unterdrücken
#define ACC_THRESHOLD 0.05f // g

TIM_HandleTypeDef htim2;

// Offset-Werte automatisch ermittelt
int16_t acc_offset[3] = {0};
float gyro_offset[3] = {0.0f};

/* Private typedef------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#if defined(TERMINAL_USE)
extern UART_HandleTypeDef hDiscoUart;
#endif /* TERMINAL_USE */

static uint8_t http[1024];
static uint8_t IP_Addr[4];
static int LedState = 0;

// Aktuelle Sensorwerte initialisieren
static int distance = 0; // distance 0 -> invalid measurement

// Gefilterte Werte initialisieren
static float acc_filtered[3] = {0};
static float gyro_filtered[3] = {0};
static uint16_t distance_buffer[DIST_AVG_SAMPLES] = {0};
static uint32_t distance_sum = 0;
static uint8_t distance_index = 0;
static uint16_t previous_distance = 0;
static uint16_t distance_avg = 0;

static float speed = 0.0f; // mm/s
float acc_velocity = 0.0f; // Geschwindigkeit über Integration [mm/s]

/* Private function prototypes -----------------------------------------------*/
#if defined(TERMINAL_USE)
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#endif /* TERMINAL_USE */

static void SystemClock_Config(void);
static WIFI_Status_t SendWebPage(uint8_t ledIsOn, uint8_t temperature);
static int wifi_server(void);
static int wifi_start(void);
static int wifi_connect(void);
static bool WebServerProcess(void);
void calibrateSensors(void);
void read_sensors(void);

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void) {

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Init Timer for Sensor*/
  MX_TIM2_Init();

  /* Configure LED2 */
  BSP_LED_Init(LED2);

  /*Initialize Temperature sensor */
  // HAL_ADCEx_Calibration_Start(&AdcHandle, ADC_SINGLE_ENDED);
  // HAL_ADC_Start(&AdcHandle) ;

  /* WIFI Web Server demonstration */
#if defined(TERMINAL_USE)
  /* Initialize all configured peripherals */
  hDiscoUart.Instance = DISCOVERY_COM1;
  hDiscoUart.Init.BaudRate = 115200;
  hDiscoUart.Init.WordLength = UART_WORDLENGTH_8B;
  hDiscoUart.Init.StopBits = UART_STOPBITS_1;
  hDiscoUart.Init.Parity = UART_PARITY_NONE;
  hDiscoUart.Init.Mode = UART_MODE_TX_RX;
  hDiscoUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hDiscoUart.Init.OverSampling = UART_OVERSAMPLING_16;
  hDiscoUart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hDiscoUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  SENSOR_IO_Init();
  BSP_COM_Init(COM1, &hDiscoUart);
  BSP_TSENSOR_Init();
  BSP_ACCELERO_Init();
  BSP_GYRO_Init();
  // Start the measure of tof
  startToF();

  calibrateSensors();

  /* Start the Sensor Timer */
  HAL_TIM_Base_Start_IT(&htim2);

  printf("****** WIFI Web Server demonstration****** \n\n");

#endif /* TERMINAL_USE */

  wifi_server();
}

// Scenario detection logic (call this regularly after read_sensors)
void check_scenario() {
  if (speed > 0.0f &&
      speed < -3000.00f) { // Maximum speed of microcontroller is 3m/s and
                           // should not be positiv, ignore other values
    speed = 0.0f;
  }
  // printf("prev: %.2f mm/s\n", previous_distance);
  // printf("Distancavg: %.2f mm/s\n", distance_avg);
  switch (scenario_state) {
  case NO_SCENARIO:
    // Check if object is approaching (distance decreasing and speed negative)
    if ((distance_avg < previous_distance) && (speed < 0)) {
      printf("→ Possible scenario detected. Switching state.\n");
      scenario_state = POSSIBLE_SCENARIO;
    }
    break;

  case POSSIBLE_SCENARIO: {
    // Check if a collision is avoidable using basic kinematics: v² / 2a < s
    float stopping_distance = (speed * speed) / (2.0f * max_decel); // mm
    printf(
        "→ Stopping distance: %.2f mm | Remaining: %d mm | Speed: %.2f mm/s\n",
        stopping_distance, distance_avg, speed);

    // Case: safe stop is still possible
    if (stopping_distance < distance_avg || distance_avg <= 1) {
      // Exit condition: no danger anymore
      if (distance_avg >= previous_distance || speed >= 0 ||
          distance_avg == 0) {
        printf("→ Scenario no longer likely. Returning to NO_SCENARIO.\n");
        scenario_state = NO_SCENARIO;
      } else {
        // Still possible scenario
        printf("→ Still possible scenario.\n");
      }
    } else {
      // Collision cannot be avoided
      printf("→ Collision unavoidable. SCENARIO DETECTED!\n");
      scenario_state = SCENARIO_DETECTED;
    }
    break;
  }

  case SCENARIO_DETECTED:
    // In detected state, do something – e.g., trigger alert or log
    printf("⚠️  Scenario detected: take action!\n");
    scenario_state = NO_SCENARIO;
    // Optional: remain here or return to NO_SCENARIO after some time
    break;
  }
}

/**
 * @brief  Send HTML page
 * @param  None
 * @retval None
 */

static int wifi_start(void) {
  uint8_t MAC_Addr[6];

  /*Initialize and use WIFI module */
  if (WIFI_Init() == WIFI_STATUS_OK) {
    LOG(("ES-WIFI Initialized.\n"));
    if (WIFI_GetMAC_Address(MAC_Addr, sizeof(MAC_Addr)) == WIFI_STATUS_OK) {
      LOG(("> eS-WiFi module MAC Address : %02X:%02X:%02X:%02X:%02X:%02X\n",
           MAC_Addr[0], MAC_Addr[1], MAC_Addr[2], MAC_Addr[3], MAC_Addr[4],
           MAC_Addr[5]));
    } else {
      LOG(("> ERROR : CANNOT get MAC address\n"));
      return -1;
    }
  } else {
    return -1;
  }
  return 0;
}

void update_acc_velocity() {
  float a_z = acc_filtered[2]; // Z-Achse, z. B. in g

  // Beschleunigung in m/s² umrechnen (1g ≈ 9.81 m/s²)
  float a_z_mps2 = a_z * 9.81f;

  // Optional: Rauschen ignorieren
  if (fabs(a_z) < ACC_THRESHOLD)
    a_z_mps2 = 0;

  // Geschwindigkeit integrieren (m/s → mm/s)
  acc_velocity += a_z_mps2 * ACC_DT * 1000.0f;
}

void calibrateSensors() {
  int32_t acc_sum[3] = {0};
  float gyro_sum[3] = {0};

  for (int i = 0; i < CALIB_SAMPLES; i++) {
    int16_t acc[3];
    float gyro[3];

    BSP_ACCELERO_AccGetXYZ(acc);
    BSP_GYRO_GetXYZ(gyro);

    for (int j = 0; j < 3; j++) {
      acc_sum[j] += acc[j];
      gyro_sum[j] += gyro[j];
    }

    HAL_Delay(5); // 5 ms warten
  }

  for (int i = 0; i < 3; i++) {
    acc_offset[i] = acc_sum[i] / CALIB_SAMPLES;
    gyro_offset[i] = gyro_sum[i] / CALIB_SAMPLES;
  }

  printf("Kalibrierung abgeschlossen.\n");
  printf("Acc Offset: X=%d Y=%d Z=%d\n", acc_offset[0], acc_offset[1],
         acc_offset[2]);
  printf("Gyro Offset: X=%.2f Y=%.2f Z=%.2f\n", gyro_offset[0], gyro_offset[1],
         gyro_offset[2]);
}

// Read sensordata (after interrupt)
void read_sensors() {
  startToF();
  HAL_Delay(5); // 5 ms warten
  getDistance(&distance);

  // Distance Moving Average
  // Distance Moving Average (excluding zero values)
  distance_sum -= distance_buffer[distance_index];
  distance_buffer[distance_index] = distance;

  // Only add to sum if the new value is not zero
  if (distance != 0) {
    distance_sum += distance;
  }

  distance_index = (distance_index + 1) % DIST_AVG_SAMPLES;

  // Recalculate average excluding zeros
  uint16_t valid_samples = 0;
  uint32_t sum = 0;

  for (int i = 0; i < DIST_AVG_SAMPLES; i++) {
    if (distance_buffer[i] != 0) {
      sum += distance_buffer[i];
      valid_samples++;
    }
  }

  if (valid_samples > 0) {
    distance_avg = sum / valid_samples;
  } else {
    distance_avg = 0; // fallback if all values are zero
  }

  // Geschwindigkeit berechnen (v = Δs / Δt)
  int16_t delta_distance = (int16_t)distance_avg - (int16_t)previous_distance;
  speed = (float)delta_distance / DIST_SAMPLE_INTERVAL; // mm/s

  printf("Distance: %d mm\n", distance_avg);
  printf("Speed: %.2f mm/s\n", speed);

  // check_scenario();

  // Letzte Distanz für nächste Messung merken
  previous_distance = distance_avg;
}

void read_sensorsold() {
  int16_t acc_raw[3];
  float gyro_raw[3];

  startToF();
  getDistance(&distance);

  // Distance Moving Average
  distance_sum -= distance_buffer[distance_index];
  distance_buffer[distance_index] = distance;
  distance_sum += distance;
  distance_index = (distance_index + 1) % DIST_AVG_SAMPLES;

  distance_avg = distance_sum / DIST_AVG_SAMPLES;

  // Geschwindigkeit berechnen (v = Δs / Δt)
  int16_t delta_distance = (int16_t)distance_avg - (int16_t)previous_distance;
  speed = (float)delta_distance / DIST_SAMPLE_INTERVAL; // mm/s

  // Letzte Distanz für nächste Messung merken
  previous_distance = distance_avg;

  BSP_ACCELERO_AccGetXYZ(acc_raw);
  BSP_GYRO_GetXYZ(gyro_raw);

  for (int i = 0; i < 3; i++) {
    float acc_corrected = (float)(acc_raw[i] - acc_offset[i]);
    float gyro_corrected = gyro_raw[i] - gyro_offset[i];

    acc_filtered[i] = (1 - ALPHA) * acc_filtered[i] + ALPHA * acc_corrected;
    gyro_filtered[i] = (1 - ALPHA) * gyro_filtered[i] + ALPHA * gyro_corrected;
  }

  // printf("Accelerometer -> X: %.2f, Y: %.2f, Z: %.2f\n", acc_filtered[0],
  //        acc_filtered[1], acc_filtered[2]);

  // printf("Gyroscope     -> X: %.2f, Y: %.2f, Z: %.2f\n", gyro_filtered[0],
  //        gyro_filtered[1], gyro_filtered[2]);

  printf("Distance: %u mm (avg)\n", distance_avg);
  update_acc_velocity();
  printf("Distance: %d mm\n", distance_avg);
  printf("Speed: %.2f mm/s\n", speed);
  printf("Speed (Accelerometer): %.2f mm/s\n", acc_velocity);
}

int wifi_connect(void) {

  wifi_start();

  LOG(("\nConnecting to %s , %s\n", SSID, PASSWORD));
  if (WIFI_Connect(SSID, PASSWORD, WIFI_ECN_WPA2_PSK) == WIFI_STATUS_OK) {
    if (WIFI_GetIP_Address(IP_Addr, sizeof(IP_Addr)) == WIFI_STATUS_OK) {
      LOG(("> es-wifi module connected: got IP Address : %d.%d.%d.%d\n",
           IP_Addr[0], IP_Addr[1], IP_Addr[2], IP_Addr[3]));
    } else {
      LOG((" ERROR : es-wifi module CANNOT get IP address\n"));
      return -1;
    }
  } else {
    LOG(("ERROR : es-wifi module NOT connected\n"));
    return -1;
  }
  return 0;
}

int wifi_server(void) {
  bool StopServer = false;

  LOG(("\nRunning HTML Server test\n"));
  if (wifi_connect() != 0)
    return -1;

  if (WIFI_STATUS_OK !=
      WIFI_StartServer(SOCKET, WIFI_TCP_PROTOCOL, 1, "", PORT)) {
    LOG(("ERROR: Cannot start server.\n"));
  }

  LOG(("Server is running and waiting for an HTTP  Client connection to "
       "%d.%d.%d.%d\n",
       IP_Addr[0], IP_Addr[1], IP_Addr[2], IP_Addr[3]));

  do {
    uint8_t RemoteIP[4];
    uint16_t RemotePort;

    while (WIFI_STATUS_OK != WIFI_WaitServerConnection(SOCKET, 1000, RemoteIP,
                                                       sizeof(RemoteIP),
                                                       &RemotePort)) {
      LOG(("Waiting connection to  %d.%d.%d.%d\n", IP_Addr[0], IP_Addr[1],
           IP_Addr[2], IP_Addr[3]));
    }

    LOG(("Client connected %d.%d.%d.%d:%d\n", RemoteIP[0], RemoteIP[1],
         RemoteIP[2], RemoteIP[3], RemotePort));

    StopServer = WebServerProcess();

    if (WIFI_CloseServerConnection(SOCKET) != WIFI_STATUS_OK) {
      LOG(("ERROR: failed to close current Server connection\n"));
      return -1;
    }
  } while (StopServer == false);

  if (WIFI_STATUS_OK != WIFI_StopServer(SOCKET)) {
    LOG(("ERROR: Cannot stop server.\n"));
  }

  LOG(("Server is stop\n"));
  return 0;
}

static bool WebServerProcess(void) {
  uint16_t respLen;
  static uint8_t resp[1024];
  bool stopserver = false;

  if (WIFI_STATUS_OK ==
      WIFI_ReceiveData(SOCKET, resp, 1000, &respLen, WIFI_READ_TIMEOUT)) {
    LOG(("get %d byte from server\n", respLen));

    if (respLen > 0) {
      if (strstr((char *)resp, "GET")) /* GET: put web page */
      {
        if (SendWebPage(LedState, distance_avg) != WIFI_STATUS_OK) {
          LOG(("> ERROR : Cannot send web page\n"));
        } else {
          LOG(("Send page after  GET command\n"));
        }
      } else if (strstr((char *)resp, "POST")) /* POST: received info */
      {
        LOG(("Post request\n"));

        if (strstr((char *)resp, "radio")) {
          if (strstr((char *)resp, "radio=0")) {
            LedState = 0;
            BSP_LED_Off(LED2);
          } else if (strstr((char *)resp, "radio=1")) {
            LedState = 1;
            BSP_LED_On(LED2);
          }
        }
        if (strstr((char *)resp, "stop_server")) {
          if (strstr((char *)resp, "stop_server=0")) {
            stopserver = false;
          } else if (strstr((char *)resp, "stop_server=1")) {
            stopserver = true;
          }
        }
        if (SendWebPage(LedState, distance_avg) != WIFI_STATUS_OK) {
          LOG(("> ERROR : Cannot send web page\n"));
        } else {
          LOG(("Send Page after POST command\n"));
        }
      }
    }
  } else {
    LOG(("Client close connection\n"));
  }
  return stopserver;
}

/**
 * @brief  Send HTML page
 * @param  None
 * @retval None
 */
static WIFI_Status_t SendWebPage(uint8_t ledIsOn, uint8_t distance) {
  uint8_t dist[10];
  int8_t current_speed[10];
  uint16_t SentDataLength;
  WIFI_Status_t ret;

  /* construct web page content */
  /* HTTP-Header und DOCTYPE definieren */
  strcpy(
      (char *)http,
      "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nPragma: no-cache\r\n\r\n");
  strcat((char *)http, "<html><head>");
  strcat((char *)http, "<title>SC Detector</title>");
  strcat((char *)http, "<meta http-equiv='refresh' content='1'>");
  /* TailwindCSS via Browser-Script einbinden */
  strcat((char *)http,
         "<script src='https://unpkg.com/@tailwindcss/browser@4'></script>");
  strcat((char *)http, "</head><body class='bg-gray-100 flex items-center "
                       "justify-center min-h-screen'>");

  /* Container */
  strcat((char *)http,
         "<div class='bg-white shadow-lg rounded-lg p-8 max-w-md w-full'>");
  strcat((char *)http, "<h2 class='text-3xl font-bold mb-6 "
                       "text-center'>Erkannte Szenarien</h2>");

  /* Formular */
  strcat((char *)http, "<form method='POST' class='space-y-6'>");

  /* Temperaturanzeige */
  strcat((char *)http, "<div class='flex items-center space-x-3'>");
  sprintf((char *)dist, "%d mm", distance);
  strcat((char *)http, "<p>Distanz: ");
  strcat((char *)http, (char *)dist);
  strcat((char *)http, "</p>");

  sprintf((char *)current_speed, "%.2f mm/s", speed);
  strcat((char *)http, "<p>Geschwindigkeit: ");
  strcat((char *)http, (char *)current_speed);
  strcat((char *)http, "</p>");
  strcat((char *)http, "</div>");

  /* Zwei Direkt-Buttons für POST mit Styling je nach Status */
  strcat((char *)http, "<div class='flex justify-center space-x-4'>");

  if (ledIsOn) {
    // Detector ist AN → dieser Button gefüllt
    strcat((char *)http, "<button name='radio' value='1' type='submit' "
                         "class='bg-green-500 hover:bg-green-600 text-white "
                         "font-bold py-2 px-4 rounded'>"
                         "Detector an</button>");
    strcat((char *)http, "<button name='radio' value='0' type='submit' "
                         "class='bg-white border border-gray-400 text-gray-700 "
                         "font-bold py-2 px-4 rounded'>"
                         "Detector aus</button>");
  } else {
    // Detector ist AUS → dieser Button gefüllt
    strcat((char *)http, "<button name='radio' value='1' type='submit' "
                         "class='bg-white border border-gray-400 text-gray-700 "
                         "font-bold py-2 px-4 rounded'>"
                         "Detector an</button>");
    strcat((char *)http, "<button name='radio' value='0' type='submit' "
                         "class='bg-red-500 hover:bg-red-600 text-white "
                         "font-bold py-2 px-4 rounded'>"
                         "Detector aus</button>");
  }

  strcat((char *)http, "</div>");

  strcat((char *)http, "</form>");
  strcat((char *)http, "</div>");
  strcat((char *)http, "</body></html>");

  ret = WIFI_SendData(0, (uint8_t *)http, strlen((char *)http), &SentDataLength,
                      WIFI_WRITE_TIMEOUT);

  if ((ret == WIFI_STATUS_OK) && (SentDataLength != strlen((char *)http))) {
    ret = WIFI_STATUS_ERROR;
  }

  return ret;
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (MSI)
 *            SYSCLK(Hz)                     = 80000000
 *            HCLK(Hz)                       = 80000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 1
 *            APB2 Prescaler                 = 1
 *            MSI Frequency(Hz)              = 4000000
 *            PLL_M                          = 1
 *            PLL_N                          = 40
 *            PLL_R                          = 2
 *            PLL_P                          = 7
 *            PLL_Q                          = 4
 *            Flash Latency(WS)              = 4
 * @param  None
 * @retval None
 */
static void SystemClock_Config(void) {
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    /* Initialization Error */
    while (1)
      ;
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                 RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    /* Initialization Error */
    while (1)
      ;
  }
}

#if defined(TERMINAL_USE)
/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE {
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission
   */
  HAL_UART_Transmit(&hDiscoUart, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
#endif /* TERMINAL_USE */

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
    number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
    line) */
  /* USER CODE END 6 */
}

#endif

/**
 * @brief  EXTI line detection callback.
 * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI
 * line.
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  switch (GPIO_Pin) {
  case (GPIO_PIN_1): {
    SPI_WIFI_ISR();
    break;
  }
  default: {
    break;
  }
  }
}

/**
 * @brief  Init Timer 2 for Sensor check
 * line.
 * @retval None
 */
void MX_TIM2_Init(void) {
  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999; // 8 MHz / (7999 + 1) = 1 kHz (1ms Tick)
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99; // 100 ms Interrupt: 1 kHz * 100 = 100 ms
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim2);

  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
 * @brief  SPI3 line detection callback.
 * @param  None
 * @retval None
 */
void SPI3_IRQHandler(void) { HAL_SPI_IRQHandler(&hspi); }

/**
 * @brief  TIM2 Interrupt handler
 * @param  None
 * @retval None
 */
void TIM2_IRQHandler(void) { HAL_TIM_IRQHandler(&htim2); }
/**
 * @brief  TIM2 Interrupt callback function
 * @param  None
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) {
    // Hier läuft alle 100ms der Sensorcode
    read_sensors();
  }
}
