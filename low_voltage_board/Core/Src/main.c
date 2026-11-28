/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "fdcan.h"
#include "gpio.h"
#include "spi.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_common.h"
#include "mmc5983.h"
#include "ring_buffer.h"
#include "sch16t.h"
#include "uart_common.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include <stdbool.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_RING_SIZE 512u
#define CMD_TIMEOUT_MS 200u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// UART
static uint8_t tx_buf[PKT_MAX_RAW];
static uint32_t tx_dropped = 0;
static uint8_t tx_counter = 0;

static uint8_t rx_buf[RX_RING_SIZE];
static ring_buf_t rx_ring_buf;

static uint8_t rx_frame[PKT_MAX_ENCODED];
static size_t rx_frame_len = 0;
static bool rx_frame_overflow = false;

static uint32_t rx_ok, rx_err_cobs, rx_err_crc, rx_err_len;

// IMU
static imu_data_t imu_state;
static bool imu_ok = false;
static uint32_t imu_errors = 0;

// MAG
static mag_data_t mag_state;
static bool mag_ok = false;
static uint32_t mag_errors = 0;

// COMMAND STATES
static thruster_commands_t thruster_states;
static launch_actions_t launch_action_state;
static uint16_t launcher_angle = 0;
static boat_state_t boat_state;

// TIMEOUT
static uint32_t last_rx_ms = 0;
static bool cmd_valid = false;
static uint8_t last_counter = 0;

// BMS
static battery_info_t battery = {0};
static uint32_t last_bms_ms = 0;

// LAUNCHER
static uint8_t shot_id = 0;                        // inc per fire request
static launch_actions_t shot_action = REST;        // latest shot cmd
static launch_actions_t prev_launch_action = REST; // edge detection

// CAN
static uint32_t can_tx_dropped = 0;
static uint32_t can_bus_off_count = 0;
static const uint8_t estop_rst[2] = {ESTOP_RST_0, ESTOP_RST_1};

// CAN payload length -> CAN DLC
static const uint32_t can_dlc[9] = {
    FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
    FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
    FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void send_tx_packet(const packet_tx_t *tx_pkt, uint8_t counter);
void push_rx(const uint8_t *data, uint32_t len);

static void handle_rx_frame(const uint8_t *data, size_t len);
static void rx_feed(uint8_t b);
static void failsafe(void);

static void init_can(void);
static bool send_can_pkt(uint32_t id, const void *data, uint8_t len);
static void poll_can_rx(void);
static void get_can_status(void);
static void CAN_cb(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void send_tx_packet(const packet_tx_t *tx_pkt, uint8_t counter) {
  USBD_CDC_HandleTypeDef *hcdc =
      (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

  // previous transfer still using tx_buf
  if (hcdc == NULL || hcdc->TxState != 0) {
    tx_dropped++;
    return;
  }

  size_t n = serialize_packet(tx_pkt, sizeof(*tx_pkt), counter, tx_buf);
  if (n == 0)
    return;
  if (CDC_Transmit_FS(tx_buf, (uint16_t)n) != USBD_OK) {
    // just drop curr packet, send updated one next cycle
    tx_dropped++;
  }
}

void push_rx(const uint8_t *data, uint32_t len) {
  push_ring(&rx_ring_buf, data, len);
}

static void handle_rx_frame(const uint8_t *data, size_t len) {
  packet_rx_t pkt;
  uint8_t counter;

  switch (deserialize_packet(data, len, &pkt, sizeof(pkt), &counter)) {
  case PKT_OK:
    break;
  case PKT_ERR_COBS:
    rx_err_cobs++;
    return;
  case PKT_ERR_CRC:
    rx_err_crc++;
    return;
  case PKT_ERR_LEN:
    rx_err_len++;
    return;
  default:
    return;
  }

  // get a fire request on rising edge
  bool fire_req =
      (pkt.launch_action == FIRE_CANNON || pkt.launch_action == FIRE_PUMP);
  if (fire_req && pkt.launch_action != prev_launch_action &&
      pkt.state != SOFT_ESTOP) {
    // geneate new shot id
    shot_id++;
    shot_action = pkt.launch_action;
  }
  prev_launch_action = pkt.launch_action;

  boat_state = pkt.state;
  thruster_states = pkt.thruster_commands;
  launch_action_state = pkt.launch_action;
  launcher_angle = pkt.launcher_angle;
  last_counter = counter;
  last_rx_ms = HAL_GetTick();
  cmd_valid = true;
  rx_ok++;
}

static void rx_feed(uint8_t b) {
  // check if 0x00, if not append, else frame complete
  if (b == PKT_DELIMITER) {
    // frame complete, process:
    if (!rx_frame_overflow && rx_frame_len > 0)
      handle_rx_frame(rx_frame, rx_frame_len);
    rx_frame_len = 0;
    rx_frame_overflow = false;
    return;
  }
  if (rx_frame_len < sizeof(rx_frame))
    // append
    rx_frame[rx_frame_len++] = b;
  else
    // if too many bytes, discard until next delimiter
    rx_frame_overflow = true;
}

static void failsafe(void) {
  boat_state = SOFT_ESTOP;
  thruster_states = (thruster_commands_t){0};
  launch_action_state = REST;
}

static void init_can(void) {
  FDCAN_FilterTypeDef sFilterConfig = {0};

  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID2 = 0x7FF; // mask -> need all 11 bits to match

  // filter for BMS info
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterID1 = CAN_ID_BATTERY_INFO;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }

  // reject all other frames
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK) {
    Error_Handler();
  }

  // init comms
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }
}

static bool send_can_pkt(uint32_t id, const void *data, uint8_t len) {
  if (len > 8)
    return false;

  // tx fifo full -> drop pkt
  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0) {
    can_tx_dropped++;
    return false;
  }

  FDCAN_TxHeaderTypeDef hdr = {0};
  hdr.Identifier = id;
  hdr.IdType = FDCAN_STANDARD_ID;
  hdr.TxFrameType = FDCAN_DATA_FRAME;
  hdr.DataLength = can_dlc[len];
  hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  hdr.BitRateSwitch = FDCAN_BRS_OFF;
  hdr.FDFormat = FDCAN_CLASSIC_CAN;
  hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  hdr.MessageMarker = 0;

  // local payload copy
  uint8_t buf[8] = {0};
  if (data != NULL && len > 0)
    memcpy(buf, data, len);

  // add to tx can fifo
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &hdr, buf) != HAL_OK) {
    can_tx_dropped++;
    return false;
  }
  return true;
}

static void poll_can_rx(void) {
  FDCAN_RxHeaderTypeDef hdr;
  uint8_t payload[8];

  // iter through CAN RX FIFO
  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &hdr, payload) !=
        HAL_OK)
      break;
    // only messages should be BMS type
    if (hdr.Identifier == CAN_ID_BATTERY_INFO &&
        hdr.DataLength == can_dlc[sizeof(battery_info_t)]) {
      memcpy(&battery, payload, sizeof(battery));
      last_bms_ms = HAL_GetTick();
    }
  }
}

static void get_can_status(void) {
  FDCAN_ProtocolStatusTypeDef ps;
  // get can statis
  if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps) != HAL_OK)
    return;

  // check faulty, if so reset
  if (ps.BusOff && READ_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT)) {
    can_bus_off_count++;
    CLEAR_BIT(hfdcan1.Instance->CCCR, FDCAN_CCCR_INIT);
  }
}

static void CAN_cb(void) {
  // timers defined in can_common.h
  static uint32_t last_estop_ms = 0;
  static uint32_t last_thruster_ms = 0;
  static uint32_t last_state_ms = 0;
  static bool prev_estopped = false; // sends estop at startup curr
  static uint8_t prev_state = 0;     // 0 = invalid -> forces first send
  static uint32_t last_launcher_ms = 0;
  static uint8_t prev_shot_id = 0;

  uint32_t now = HAL_GetTick();
  bool estopped = (boat_state == SOFT_ESTOP);

  get_can_status();
  poll_can_rx();

  // estop/rst sent on change and periodically
  if (estopped != prev_estopped || now - last_estop_ms >= CAN_ESTOP_REPEAT_MS) {
    bool sent =
        estopped ? send_can_pkt(CAN_ID_ESTOP, NULL, 0)
                 : send_can_pkt(CAN_ID_ESTOP_RST, estop_rst, sizeof(estop_rst));
    if (sent) {
      prev_estopped = estopped;
      last_estop_ms = now;
    }
  }

  // thrusters cmds always streamed
  if (now - last_thruster_ms >= CAN_THRUSTER_PERIOD_MS) {
    last_thruster_ms = now;
    thruster_commands_t cmd =
        estopped ? (thruster_commands_t){0} : thruster_states;
    send_can_pkt(CAN_ID_THRUSTER_CMD, &cmd, sizeof(cmd));
  }

  // boat state sent on change & periodically
  if (boat_state != prev_state || now - last_state_ms >= CAN_STATE_PERIOD_MS) {
    boat_state_msg_t msg = {.state = boat_state};
    if (send_can_pkt(CAN_ID_BOAT_STATE, &msg, sizeof(msg))) {
      prev_state = boat_state;
      last_state_ms = now;
    }
  }
  // launcher cmd sent on new shot & periodically
  // repeats keep same shot_id, but cannon only fires on new id
  if (shot_id != prev_shot_id ||
      now - last_launcher_ms >= CAN_LAUNCHER_PERIOD_MS) {
    // generate & send shot command
    launcher_cmd_t cmd;
    cmd.angle_deg = launcher_angle % 360;
    cmd.power = 0;
    cmd.action = estopped ? REST : shot_action;
    cmd.shot_id = shot_id;
    if (send_can_pkt(CAN_ID_LAUNCHER_CMD, &cmd, sizeof(cmd))) {
      prev_shot_id = shot_id;
      last_launcher_ms = now;
    }
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */
  init_ring(&rx_ring_buf, rx_buf, sizeof(rx_buf));
  failsafe();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USB_Device_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  imu_ok = imu_init();
  mag_ok = mag_init();
  init_can();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t last_tx = 0;
    static uint32_t last_blink = 0;

    // status LED, 1 Hz
    if (HAL_GetTick() - last_blink >= 500) {
      last_blink = HAL_GetTick();
      HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
    }

    // 500 Hz
    if (HAL_GetTick() - last_tx >= 2) {
      last_tx = HAL_GetTick();
      packet_tx_t tx = {0};
      tx.stamp_ms = HAL_GetTick();

      // imu
      if (!imu_read(&imu_state))
        imu_errors++;
      tx.imu_data = imu_state;

      // magnetometer
      if (!mag_read(&mag_state)) {
        mag_errors++;
        mag_state.status = 0;
      }
      tx.mag_data = mag_state;

      // battery stats (from BMS over can)
      tx.voltage = battery.voltage_mV;
      tx.current = battery.current_mA;

      send_tx_packet(&tx, tx_counter++);
    }

    uint8_t b;
    while (pop_ring(&rx_ring_buf, &b)) {
      rx_feed(b);
    }

    if (!cmd_valid || (HAL_GetTick() - last_rx_ms) > CMD_TIMEOUT_MS) {
      failsafe();
    }

    // after failsafe so can always sends the latest (failsafed) state
    CAN_cb();
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  // LED high on error
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */