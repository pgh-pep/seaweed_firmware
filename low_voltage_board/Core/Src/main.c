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
#include "ring_buffer.h"
#include "sch16t.h"
#include "uart_common.h"
#include "usbd_cdc_if.h"
#include <stdbool.h>

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

// uart buffers/frames
static uint8_t tx_buf[PKT_MAX_RAW];
static uint32_t tx_dropped = 0;
static uint8_t tx_counter = 0;

static uint8_t rx_buf[RX_RING_SIZE];
static ring_buf_t rx_ring_buf;

static uint8_t rx_frame[PKT_MAX_ENCODED];
static size_t rx_frame_len = 0;
static bool rx_frame_overflow = false;

static uint32_t rx_ok, rx_err_cobs, rx_err_crc, rx_err_len;

// sensor states
static imu_data_t imu_state;
static bool imu_ok = false;
static uint32_t imu_errors = 0;

// static mag_data_t mag_state;
// static uint16_t voltage;
// static uint16_t current;
// command states
static thruster_commands_t thruster_states;
static launch_actions_t launch_action_state;
static uint8_t launcher_angle = 0;
static boat_state_t boat_state;
static uint16_t voltage = 15;
static uint16_t current = 21;
// timeouts
static uint32_t last_rx_ms = 0;
static bool cmd_valid = false;
static uint8_t last_counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void send_tx_packet(const packet_tx_t *tx_pkt, uint8_t counter);
void push_rx(const uint8_t *data, uint32_t len);

static void handle_rx_frame(const uint8_t *data, size_t len);
static void rx_feed(uint8_t b);
static void failsafe(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void send_tx_packet(const packet_tx_t *tx_pkt, uint8_t counter) {
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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t last_tx = 0;
    if (HAL_GetTick() - last_tx >= 5) {
      last_tx = HAL_GetTick();
      packet_tx_t tx = {0};
      tx.stamp_ms = HAL_GetTick();

      if (!imu_read(&imu_state))
        imu_errors++;
      tx.imu_data = imu_state;

      tx.mag_data.mag_x = thruster_states.br;
      tx.mag_data.status = cmd_valid;

      tx.voltage = voltage;
      tx.current = current;

      send_tx_packet(&tx, tx_counter++);
    }
    uint8_t b;
    while (pop_ring(&rx_ring_buf, &b)) {
      rx_feed(b);
    }

    if (!cmd_valid || (HAL_GetTick() - last_rx_ms) > CMD_TIMEOUT_MS) {
      failsafe();
    }
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
