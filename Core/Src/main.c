/**
 ******************************************************************************
 * @file    main.c
 * @author  Jonas
 * @brief   Combined Webserver and Scenario Detection for STM32L475E-IOT01A2
 ******************************************************************************
 * @attention
 * This software is provided AS-IS under the terms in the LICENSE file.
 ******************************************************************************
 */

#include "main.h"

/*--- Private Defines --------------------------------------------------------*/
#define SSID "WLAN-070011" /**< WiFi SSID */
#define PASSWORD "xxx"     /**< WiFi password */
#define PORT 80            /**< HTTP server port */
#define SOCKET 0           /**< TCP socket index */

#define WIFI_WRITE_TIMEOUT 10000 /**< Write timeout (ms) */
#define WIFI_READ_TIMEOUT 10000  /**< Read timeout (ms) */

#define ALPHA 0.1f                /**< Low-pass filter alpha */
#define DIST_SAMPLE_INTERVAL 0.5f /**< Distance sampling interval (s) */
#define CALIB_SAMPLES 100         /**< Number of calibration samples */
#define DIST_AVG_SAMPLES 3        /**< Moving average window size */
#define MAX_DECEL 100.0f          /**< Maximum deceleration (mm/s²) */
#define TERMINAL_USE              /**< Use in Terminal */

/*--- Type Definitions -------------------------------------------------------*/
typedef enum {
  NO_SCENARIO = 0,   /**< No collision scenario */
  POSSIBLE_SCENARIO, /**< Potential collision scenario */
  SCENARIO_DETECTED  /**< Collision detected */
} ScenarioState;

/*--- Private Globals --------------------------------------------------------*/
volatile ScenarioState scenario_state =
    NO_SCENARIO; /**< Current scenario FSM state */

static int16_t acc_offset[3] = {0};   /**< Accelerometer offsets */
static float gyro_offset[3] = {0.0f}; /**< Gyro offsets */

static int distance = 0; /**< Current Distance */
static uint16_t
    distance_buffer[DIST_AVG_SAMPLES]; /**< Circular buffer for distance avg */
static uint32_t distance_sum = 0;      /**< Sum of valid distances */
static uint8_t distance_index = 0;     /**< Index in distance buffer */

static uint16_t distance_avg = 0;      /**< Filtered distance */
static uint16_t previous_distance = 0; /**< Last distance reading */
static float speed = 0.0f;             /**< Computed speed (mm/s) */

static float acc_filtered[3] = {0};  /**< Low-pass filtered accel data */
static float gyro_filtered[3] = {0}; /**< Low-pass filtered gyro data */

static uint8_t http[4096];       /**< Buffer for HTML/JSON responses */
static uint8_t IP_Addr[4];       /**< Acquired IP address */
static int detectorStateUI = 0;  /**< UI button state (detector on/off) */
static char scenario_text[1024]; /**< HTML-formatted scenario message */

extern UART_HandleTypeDef hDiscoUart; /**< UART handle for debug prints */
TIM_HandleTypeDef htim2; /**< Timer2 handle for sensor interrupts */

/*--- Function Prototypes ----------------------------------------------------*/
static void SystemClock_Config(void);
static void MX_TIM2_Init(void);
static void calibrateSensors(void);
static void read_sensors(void);
static void check_scenario(void);
static int wifi_start(void);
static int wifi_connect(void);
static int wifi_server(void);
static bool WebServerProcess(void);
static WIFI_Status_t SendWebPage(void);
static WIFI_Status_t SendDataJson(void);
static void read_vehicle_position_sensors(void);

#if defined(TERMINAL_USE)
#define LOG(a) printf a
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
#endif

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Application entry point
 * @retval None
 */
int main(void) {

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Init Timer2 for sensor polling*/
  MX_TIM2_Init();

  /* Configure LED2 */
  BSP_LED_Init(LED2);

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

  SENSOR_IO_Init();                // Init sensor I/O (I2C/SPI)
  BSP_COM_Init(COM1, &hDiscoUart); // Init debug UART
  VL53L0X_PROXIMITY_Init();        // Init ToF sensor
  BSP_ACCELERO_Init();             // Init accelerometer
  BSP_GYRO_Init();                 // Init gyro

  calibrateSensors(); // Perform sensor calibration

  printf("****** WIFI Web Server demonstration****** \n\n");

  while (1) {
    // 1. Webservermode
    printf("Starting Webserver...\n");
    wifi_server(); // blocking until detectorStateUI=1

    // 2. Scenariodetection active
    printf("Scenariodetection started...\n");
    scenario_state = NO_SCENARIO;
    HAL_TIM_Base_Start_IT(&htim2);

    // 3. Wait for Scenario to be detected
    while (scenario_state != SCENARIO_DETECTED) {
      printf("In loop...");
      HAL_Delay(5); // Idle (no WLAN active)
    }

    // 4. Scenario detected
    printf("‼️ Szenario erkannt. Zurück zum Webserver...\n");

    // Delay until the webserver starts again.
    HAL_Delay(1000);
  }
}

/**
 * @brief Evaluates the current sensor data to determine the presence of a
 * critical driving scenario.
 *
 * This function implements a simple finite-state machine to detect potentially
 * dangerous driving scenarios based on distance and speed measurements. It
 * transitions between states depending on whether a collision is likely or
 * unavoidable. When a scenario is detected, sensor data is collected and
 * formatted into a status message that is sent to the web client.
 */
void check_scenario() {
  float stopping_distance = 0.0f;

  // Filter invalid speed values (e.g., too fast or positive values)
  if (speed > 0.0f || speed < -1000.0f) {
    speed = 0.0f;
  }

  switch (scenario_state) {
  case NO_SCENARIO:
    // Transition to POSSIBLE_SCENARIO if the object is approaching
    if ((distance_avg < previous_distance) && (speed < 0)) {
      scenario_state = POSSIBLE_SCENARIO;
    }
    break;

  case POSSIBLE_SCENARIO: {
    // Calculate stopping distance using basic physics: v² / (2a)
    stopping_distance = (speed * speed) / (2.0f * MAX_DECEL);

    printf("Stopping distance: %.2f mm | Remaining: %d mm | Speed: %.2f mm/s\n",
           stopping_distance, distance_avg, speed);

    // Continuously update filtered accelerometer and gyroscope values
    read_vehicle_position_sensors();

    // If stopping distance is within safety margin or no valid reading
    if (stopping_distance < distance_avg || distance_avg <= 1) {
      // If object is no longer approaching or speed reversed
      if (distance_avg >= previous_distance || speed >= 0 ||
          distance_avg == 0) {
        scenario_state = NO_SCENARIO;
      }
      // Otherwise, remain in POSSIBLE_SCENARIO
    } else {
      // Collision is considered unavoidable — scenario is confirmed
      snprintf(scenario_text, sizeof(scenario_text),
               "Collision cannot be avoided.<br>"
               "Stopping distance: %.2f mm<br>"
               "Remaining: %d mm<br>"
               "Speed: %.2f mm/s<br><br>"
               "Accelerometer -> X: %.2f, Y: %.2f, Z: %.2f<br>"
               "Gyroscope     -> X: %.2f, Y: %.2f, Z: %.2f<br><br>",
               stopping_distance, distance_avg, speed, acc_filtered[0],
               acc_filtered[1], acc_filtered[2], gyro_filtered[0],
               gyro_filtered[1], gyro_filtered[2]);

      // Switch to SCENARIO_DETECTED state
      scenario_state = SCENARIO_DETECTED;

      // Print the detailed scenario message
      printf("%s", scenario_text);

      // Stop the sensor update timer to prevent further measurements
      HAL_TIM_Base_Stop_IT(&htim2);
    }
    break;
  }

  case SCENARIO_DETECTED:
    // Reset the FSM (this case is never actually reached in the current
    // implementation)
    printf("!!!! Scenario detected: take action !!!!\n");
    scenario_state = NO_SCENARIO;
    break;
  }
}

/**
 * @brief Initializes the WiFi module and prints its MAC address.
 *
 * This function initializes the WiFi hardware and attempts to retrieve the
 * module's MAC address for logging purposes. If initialization or MAC retrieval
 * fails, it returns an error.
 *
 * @retval 0  WiFi module initialized successfully
 * @retval -1 Initialization failed or MAC address could not be retrieved
 */
static int wifi_start(void) {
  uint8_t MAC_Addr[6];

  // Attempt to initialize the WiFi module
  if (WIFI_Init() == WIFI_STATUS_OK) {
    LOG(("ES-WIFI Initialized.\n"));

    // Try to get the MAC address from the module
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

/**
 * @brief Calibrates accelerometer and gyroscope sensors.
 *
 * This function collects multiple samples from the accelerometer and gyroscope
 * to calculate and store offset values for each axis. These offsets are later
 * used to filter sensor readings and reduce measurement errors.
 */
void calibrateSensors() {
  int32_t acc_sum[3] = {0};
  float gyro_sum[3] = {0};

  // Collect multiple samples to calculate average offset
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    int16_t acc[3];
    float gyro[3];

    BSP_ACCELERO_AccGetXYZ(acc); // Read raw accelerometer data
    BSP_GYRO_GetXYZ(gyro);       // Read raw gyroscope data

    for (int j = 0; j < 3; j++) {
      acc_sum[j] += acc[j];   // Sum accelerometer data
      gyro_sum[j] += gyro[j]; // Sum gyroscope data
    }

    HAL_Delay(5); // Wait 5ms between samples to avoid overload
  }

  // Compute average offset for each axis
  for (int i = 0; i < 3; i++) {
    acc_offset[i] = acc_sum[i] / CALIB_SAMPLES;
    gyro_offset[i] = gyro_sum[i] / CALIB_SAMPLES;
  }

  // Output calibration result to terminal
  printf("Calibration complete.\n");
  printf("Acc Offset:  X=%d  Y=%d  Z=%d\n", acc_offset[0], acc_offset[1],
         acc_offset[2]);
  printf("Gyro Offset: X=%.2f  Y=%.2f  Z=%.2f\n", gyro_offset[0],
         gyro_offset[1], gyro_offset[2]);
}

/**
 * @brief Reads sensor data and updates distance and speed.
 *
 * This function is typically called periodically by a timer interrupt.
 * It reads the current distance from the proximity sensor, updates a
 * moving average buffer, calculates the speed based on distance change,
 * and triggers scenario detection logic if active.
 */
void read_sensors() {
  // Read raw distance from proximity sensor
  distance = VL53L0X_PROXIMITY_GetDistance();

  // Remove the oldest value from the sum
  distance_sum -= distance_buffer[distance_index];
  // Store the new distance in the buffer
  distance_buffer[distance_index] = distance;

  // Only add the new distance to the sum if it is non-zero
  if (distance != 0) {
    distance_sum += distance;
  }

  // Advance buffer index (circular)
  distance_index = (distance_index + 1) % DIST_AVG_SAMPLES;

  // Recalculate the average distance, excluding zeros
  uint16_t valid_samples = 0;
  uint32_t sum = 0;

  for (int i = 0; i < DIST_AVG_SAMPLES; i++) {
    if (distance_buffer[i] != 0) {
      sum += distance_buffer[i];
      valid_samples++;
    }
  }

  // Update average distance
  if (valid_samples > 0) {
    distance_avg = sum / valid_samples;
  } else {
    distance_avg = 0; // fallback if all values are zero
  }

  // Calculate speed: Δs / Δt
  int16_t delta_distance = (int16_t)distance_avg - (int16_t)previous_distance;
  speed = (float)delta_distance / DIST_SAMPLE_INTERVAL; // in mm/s

  // If the detector is active, evaluate the scenario
  if (detectorStateUI == 1) {
    check_scenario();
  }

  // Save current distance for next speed calculation
  previous_distance = distance_avg;
}

/**
 * @brief Reads and filters vehicle position sensor data (accelerometer &
 * gyroscope).
 *
 * This function reads raw data from the accelerometer and gyroscope sensors,
 * applies calibration offsets, and filters the values using an exponential
 * moving average to reduce noise.
 */
void read_vehicle_position_sensors() {
  int16_t acc_raw[3]; // Raw accelerometer values (X, Y, Z)
  float gyro_raw[3];  // Raw gyroscope values (X, Y, Z)

  // Read raw data from accelerometer and gyroscope
  BSP_ACCELERO_AccGetXYZ(acc_raw);
  BSP_GYRO_GetXYZ(gyro_raw);

  // Apply offset correction and low-pass filtering
  for (int i = 0; i < 3; i++) {
    // Remove calibration offset
    float acc_corrected = (float)(acc_raw[i] - acc_offset[i]);
    float gyro_corrected = gyro_raw[i] - gyro_offset[i];

    // Apply exponential moving average filter
    acc_filtered[i] = (1.0f - ALPHA) * acc_filtered[i] + ALPHA * acc_corrected;
    gyro_filtered[i] =
        (1.0f - ALPHA) * gyro_filtered[i] + ALPHA * gyro_corrected;
  }

  // Debug (optional):
  // printf("Accelerometer -> X: %.2f, Y: %.2f, Z: %.2f\n", acc_filtered[0],
  // acc_filtered[1], acc_filtered[2]); printf("Gyroscope     -> X: %.2f, Y:
  // %.2f, Z: %.2f\n", gyro_filtered[0], gyro_filtered[1], gyro_filtered[2]);
}

/**
 * @brief Connects the WiFi module to the specified SSID and retrieves an IP
 * address.
 *
 * This function initializes the WiFi module using the `wifi_start()` function,
 * connects to the configured network, and retrieves the IP address.
 *
 * @retval 0  if the connection and IP retrieval were successful.
 * @retval -1 if connection or IP acquisition failed.
 */
int wifi_connect(void) {
  wifi_start(); // Initialize WiFi module

  LOG(("\nConnecting to %s , %s\n", SSID, PASSWORD));

  // Attempt to connect to WiFi network
  if (WIFI_Connect(SSID, PASSWORD, WIFI_ECN_WPA2_PSK) == WIFI_STATUS_OK) {
    // Retrieve IP address after successful connection
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

/**
 * @brief Starts and manages a simple HTTP server on the WiFi module.
 *
 * This function connects to WiFi, starts the HTTP server on the specified port,
 * waits for incoming connections, and processes each request using
 * `WebServerProcess()`. The loop runs until the `stopserver` flag is set.
 *
 * @retval 0  if the server ran and stopped successfully.
 * @retval -1 if an error occurred during connection or server operation.
 */
int wifi_server(void) {
  bool StopServer = false;

  LOG(("\nRunning HTML Server\n"));

  // Connect to WiFi network
  if (wifi_connect() != 0)
    return -1;

  // Start HTTP server
  if (WIFI_STATUS_OK !=
      WIFI_StartServer(SOCKET, WIFI_TCP_PROTOCOL, 1, "", PORT)) {
    LOG(("ERROR: Cannot start server.\n"));
  }

  LOG(("Server is running and waiting for an HTTP Client connection to "
       "%d.%d.%d.%d\n",
       IP_Addr[0], IP_Addr[1], IP_Addr[2], IP_Addr[3]));

  // Main server loop
  do {
    uint8_t RemoteIP[4];
    uint16_t RemotePort;

    // Wait for client connection (non-blocking)
    while (WIFI_STATUS_OK != WIFI_WaitServerConnection(SOCKET, 1000, RemoteIP,
                                                       sizeof(RemoteIP),
                                                       &RemotePort)) {
      LOG(("Waiting connection to  %d.%d.%d.%d\n", IP_Addr[0], IP_Addr[1],
           IP_Addr[2], IP_Addr[3]));
    }

    LOG(("Client connected %d.%d.%d.%d:%d\n", RemoteIP[0], RemoteIP[1],
         RemoteIP[2], RemoteIP[3], RemotePort));

    // Process the incoming HTTP request
    StopServer = WebServerProcess();

    // Close client connection
    if (WIFI_CloseServerConnection(SOCKET) != WIFI_STATUS_OK) {
      LOG(("ERROR: failed to close current Server connection\n"));
      return -1;
    }
  } while (!StopServer);

  // Stop the server
  if (WIFI_STATUS_OK != WIFI_StopServer(SOCKET)) {
    LOG(("ERROR: Cannot stop server.\n"));
  }

  LOG(("Server is stop\n"));
  return 0;
}

/**
 * @brief Sends scenario data as a JSON HTTP response.
 *
 * This function generates a JSON string containing the scenario analysis
 * result, formats it into an HTTP response, and sends it via the WiFi socket.
 * If the send operation fails, it sends a fallback HTTP 500 error response.
 *
 * @retval WIFI_STATUS_OK     if the response was sent successfully.
 * @retval WIFI_STATUS_ERROR  if sending failed or byte count mismatch occurred.
 */
WIFI_Status_t SendDataJson(void) {
  char json[5000]; // Buffer for the HTTP+JSON response
  uint16_t SentDataLength;

  // Log the scenario text for debugging
  printf("%s", scenario_text);

  // Build the HTTP response with JSON content
  int len = snprintf(json, sizeof(json),
                     "HTTP/1.0 200 OK\r\nContent-Type: application/json\r\n\r\n"
                     "{\"d\":\"%s\"}",
                     scenario_text);

  printf("Sending JSON (%d bytes): %s\n", len, json);

  // Send the data over the socket
  WIFI_Status_t status = WIFI_SendData(0, (uint8_t *)json, len, &SentDataLength,
                                       WIFI_WRITE_TIMEOUT);

  // Add delay to avoid overwhelming the WiFi module
  HAL_Delay(50);

  // Verify if data was fully sent
  if (status != WIFI_STATUS_OK || SentDataLength != len) {
    printf("Send failed (len=%d, sent=%d)\n", len, SentDataLength);

    // Send fallback error response
    const char *fallback = "HTTP/1.0 500 Internal Server Error\r\n"
                           "Content-Type: text/plain\r\n\r\nERR";
    WIFI_SendData(0, (uint8_t *)fallback, strlen(fallback), &SentDataLength,
                  WIFI_WRITE_TIMEOUT);

    return WIFI_STATUS_ERROR;
  }

  return WIFI_STATUS_OK;
}

/**
 * @brief Processes incoming HTTP requests from the connected client.
 *
 * This function reads data from the WiFi socket, detects whether the request
 * is a `GET /data`, `GET /`, or `POST`, and responds accordingly by sending
 * scenario data or reloading the UI. It also handles state changes via POST.
 *
 * @retval true  if the server should stop (e.g., detector activated).
 * @retval false to continue accepting client connections.
 */
static bool WebServerProcess(void) {
  uint16_t respLen;
  static uint8_t resp[1024];
  bool stopserver = false;

  // Attempt to receive data from the connected client
  if (WIFI_STATUS_OK ==
      WIFI_ReceiveData(SOCKET, resp, 1000, &respLen, WIFI_READ_TIMEOUT)) {
    LOG(("get %d byte from server\n", respLen));

    if (respLen > 0) {
      // Handle GET requests
      if (strstr((char *)resp, "GET")) {
        if (strstr((char *)resp, "GET /data")) {
          LOG(("GET /data received\n"));

          // Send JSON scenario data
          if (SendDataJson() != WIFI_STATUS_OK) {
            LOG(("> ERROR: Cannot send /data response\n"));

            const char *error_response =
                "HTTP/1.0 500 Internal Server Error\r\n"
                "Content-Type: text/plain\r\n\r\nError";
            uint16_t len;
            WIFI_SendData(0, (uint8_t *)error_response, strlen(error_response),
                          &len, WIFI_WRITE_TIMEOUT);
          }
        } else {
          // Serve default HTML page
          if (SendWebPage() != WIFI_STATUS_OK) {
            LOG(("> ERROR : Cannot send web page\n"));
          } else {
            LOG(("Send page after GET command\n"));
          }
        }
      }
      // Handle POST requests (e.g., toggling detector state)
      else if (strstr((char *)resp, "POST")) {
        LOG(("Post request\n"));

        // Update detector state from form input
        if (strstr((char *)resp, "radio")) {
          if (strstr((char *)resp, "radio=0")) {
            detectorStateUI = 0;
            BSP_LED_Off(LED2);
          } else if (strstr((char *)resp, "radio=1")) {
            detectorStateUI = 1;
            BSP_LED_On(LED2);
            stopserver = true; // Trigger exit from server loop
          }
        }

        // Refresh the page after POST
        if (SendWebPage() != WIFI_STATUS_OK) {
          LOG(("> ERROR : Cannot send web page\n"));
        } else {
          LOG(("Send Page after POST command\n"));
        }
      }

      // Delay between responses to stabilize communication
      HAL_Delay(50);
    }
  } else {
    LOG(("Client close connection\n"));
  }

  return stopserver;
}

/**
 * @brief Sends the main HTML UI page to the client via WiFi.
 *
 * This function constructs an HTML response including embedded JavaScript for
 * periodically requesting scenario data via fetch. It includes a simple UI to
 * display the current detection state and toggle the detector via a form.
 *
 * @retval WIFI_STATUS_OK     if the full page was successfully sent.
 * @retval WIFI_STATUS_ERROR  if the page could not be sent completely.
 */
static WIFI_Status_t SendWebPage(void) {
  uint16_t SentDataLength;
  WIFI_Status_t ret;

  // Start building the HTTP + HTML response into the `http` buffer
  strcpy(
      (char *)http,
      "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nPragma: no-cache\r\n\r\n"
      "<html><head>"
      "<title>SC Detector</title>"
      "<script src='https://unpkg.com/@tailwindcss/browser@4'></script>"
      "<script>"
      "setInterval(() => {"
      "  const controller = new AbortController();"
      "  const timeout = setTimeout(() => controller.abort(), 500);"
      "  fetch('/data', { signal: controller.signal })"
      "    .then(res => res.json())"
      "    .then(data => {"
      "      document.getElementById('scenarioText').innerHTML = "
      "data.d.replace(/\\n/g, '<br>');"
      "    })"
      "    .catch(err => {"
      "      if (err.name === 'AbortError') {"
      "        console.warn('Request timed out');"
      "      }"
      "    });"
      "}, 2000);"
      "</script>"
      "</head><body class='bg-gray-100 flex items-center justify-center "
      "min-h-screen'>"
      "<div class='bg-white shadow-lg rounded-lg p-8 max-w-md w-full'>"
      "<h2 class='text-3xl font-bold mb-6 text-center'>Erkannte Szenarien</h2>"
      "<div class='flex items-center space-x-3'>"
      "<p>Scenario: <span id='scenarioText'>...</span></p>"
      "</div>"
      "<form method='POST' class='space-y-6'>"
      "<div class='flex justify-center space-x-4'>");

  // Add correct button based on current UI detector state
  if (detectorStateUI) {
    strcat((char *)http, "<button name='radio' value='1' type='submit' "
                         "class='bg-green-500 hover:bg-green-600 text-white "
                         "font-bold py-2 px-4 rounded'>"
                         "Detector aktiv</button>");
  } else {
    strcat((char *)http, "<button name='radio' value='1' type='submit' "
                         "class='bg-white border border-gray-400 text-gray-700 "
                         "font-bold py-2 px-4 rounded'>"
                         "Detector aktivieren</button>");
  }

  // Close the HTML structure
  strcat((char *)http, "</div></form></div></body></html>");

  // Send the full response over WiFi
  ret = WIFI_SendData(0, (uint8_t *)http, strlen((char *)http), &SentDataLength,
                      WIFI_WRITE_TIMEOUT);

  // Ensure full message was transmitted
  if ((ret == WIFI_STATUS_OK) && (SentDataLength != strlen((char *)http))) {
    ret = WIFI_STATUS_ERROR;
  }

  return ret;
}

/**
 * @brief  Configures the system clock to run at 80 MHz using PLL with MSI
 * source.
 *
 * This function sets up the oscillator, PLL multipliers/dividers and configures
 * the AHB and APB buses for optimal performance. It will block in an infinite
 * loop if any error occurs during configuration.
 *
 * @retval None
 */
static void SystemClock_Config(void) {
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  // Enable MSI oscillator and configure PLL
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
    // Stop execution on failure
    while (1)
      ;
  }

  // Configure bus clocks
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                 RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    // Stop execution on failure
    while (1)
      ;
  }
}

#if defined(TERMINAL_USE)
/**
 * @brief Redirects the printf output to UART for debugging.
 *
 * Allows standard output functions like printf to write via the UART interface.
 *
 * @param  ch Character to write
 * @retval Written character
 */
PUTCHAR_PROTOTYPE {
  HAL_UART_Transmit(&hDiscoUart, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
#endif /* TERMINAL_USE */

#ifdef USE_FULL_ASSERT
/**
 * @brief Reports the source file and line number where an assertion failed.
 *
 * @param file Pointer to the source file name
 * @param line Line number in the file where assert_param failed
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  // User can implement additional logging or error handling here
}
#endif

/**
 * @brief External interrupt callback handler.
 *
 * Called when an external interrupt is triggered.
 *
 * @param GPIO_Pin The pin number which triggered the interrupt
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  switch (GPIO_Pin) {
  case GPIO_PIN_1:
    SPI_WIFI_ISR();
    break;
  default:
    break;
  }
}

/**
 * @brief Initializes TIM2 as a periodic interrupt timer for sensor readings.
 *
 * Configures TIM2 to trigger every 100ms. Used to periodically update sensor
 * data.
 *
 * @retval None
 */
void MX_TIM2_Init(void) {
  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999; // 8 MHz / (7999 + 1) = 1 kHz
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99; // 100ms = 100 ticks at 1kHz
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  HAL_TIM_Base_Init(&htim2);

  // Configure TIM2 interrupt
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
 * @brief SPI3 interrupt handler.
 *
 * Forwards the interrupt to HAL SPI IRQ handler.
 */
void SPI3_IRQHandler(void) { HAL_SPI_IRQHandler(&hspi); }

/**
 * @brief TIM2 interrupt handler.
 *
 * Calls the HAL TIM IRQ handler for TIM2.
 */
void TIM2_IRQHandler(void) { HAL_TIM_IRQHandler(&htim2); }

/**
 * @brief Callback for TIM2 periodic interrupts.
 *
 * Triggered every 100ms to update sensor values.
 *
 * @param htim Timer handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) {
    read_sensors();
  }
}
