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
#include "fdcan.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_common.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_NEUTRAL_US 1500
#define PWM_RANGE_US 400
#define FAILSAFE_TIMEOUT_MS 500
#define ARMING_HOLD_MS 5000 // TODO: test still
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile uint32_t lastCanRxTick = 0;
static volatile uint8_t is_estop_latched = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint16_t thrust_percent_to_us(int8_t pct) {
  // map -100 to +100 to 1100 to 1900 us
  if (pct > 100)
    pct = 100;
  if (pct < -100)
    pct = -100;
  return (uint16_t)(PWM_NEUTRAL_US + (pct * PWM_RANGE_US) / 100);
}

static void set_thrust_us(uint16_t fl_us, uint16_t fr_us, uint16_t bl_us,
                          uint16_t br_us) {
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, fl_us); // PWM 1
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, fr_us); // PWM 2
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, bl_us); // PWM 3
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, br_us); // PWM 4
}

static void set_thrust_percent(int8_t fl, int8_t fr, int8_t bl, int8_t br) {
  set_thrust_us(thrust_percent_to_us(fl), thrust_percent_to_us(fr),
                thrust_percent_to_us(bl), thrust_percent_to_us(br));
}

static void stop_all(void) {
  set_thrust_us(PWM_NEUTRAL_US, PWM_NEUTRAL_US, PWM_NEUTRAL_US, PWM_NEUTRAL_US);
}

static void arm_thrusters(void) { HAL_Delay(ARMING_HOLD_MS); }

static void init_can(void) {
  FDCAN_FilterTypeDef sFilterConfig = {0};

  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID2 = 0x7FF; // mask -> need all 11 bits to match

  // filter 0: thruster cmd (0x102)
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterID1 = CAN_ID_THRUSTER_CMD;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }

  // filter 1: estop (0x000)
  sFilterConfig.FilterIndex = 1;
  sFilterConfig.FilterID1 = CAN_ID_ESTOP;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }

  // filter 2: e-stop reset (0x001)
  sFilterConfig.FilterIndex = 2;
  sFilterConfig.FilterID1 = CAN_ID_ESTOP_RST;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }

  // reject non-matching frames
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    Error_Handler();
  }

  // init interrupt on new msg
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                     0) != HAL_OK) {
    Error_Handler();
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  if (!(RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE))
    return;

  FDCAN_RxHeaderTypeDef hdr;
  uint8_t payload[8];

  // go through entire can FIFO
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &hdr, payload) != HAL_OK)
      break;

    switch (hdr.Identifier) {
    case CAN_ID_ESTOP:
      is_estop_latched = 1;
      stop_all();
      break;

    case CAN_ID_ESTOP_RST:
      // val reset payload, prevent acc reset
      if (is_estop_latched && hdr.DataLength == FDCAN_DLC_BYTES_2 &&
          payload[0] == ESTOP_RST_0 && payload[1] == ESTOP_RST_1) {
        stop_all(); // stopped until next cmd
        is_estop_latched = 0;
        lastCanRxTick = HAL_GetTick(); // restart failsafe timer
      }
      break;

    case CAN_ID_THRUSTER_CMD:
      if (!is_estop_latched && hdr.DataLength == FDCAN_DLC_BYTES_4) {
        thruster_cmds_t cmd;
        memcpy(&cmd, payload, sizeof(cmd));
        set_thrust_percent(cmd.fl, cmd.fr, cmd.bl, cmd.br);
        lastCanRxTick = HAL_GetTick();
      }
      break;

    default:
      // should never happen
      break;
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

  /* MCU
   * Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the
   * Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  stop_all();
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  arm_thrusters();

  init_can();
  lastCanRxTick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    if (HAL_GetTick() - lastCanRxTick > FAILSAFE_TIMEOUT_MS) {
      // no can msgs -> canbus or LV issue -> stop thrusters
      stop_all();
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
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
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
  /* User can add his own implementation to report the HAL error return state
   */
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
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
     file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
