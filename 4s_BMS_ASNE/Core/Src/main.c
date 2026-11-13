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
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_ids.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_RES 12
#define ADC_RANGE 4095
#define R_DIV_K 160
#define ADC1_NUM_CHANNELS 5
#define CURR_SENSOR_ADC_CHANNEL 4

#define NUM_CELLS 4
#define ADC_V_REF 3300
#define CURR_SENSOR_SENSITIVITY 66 // mV/A
#define CURR_SENSOR_CALIBRATION_SAMPLES 100

// safe operation
#define MIN_CELL_mV 3000
#define MAX_CELL_mV 4200
#define MAX_CURR_mA 500
#define MAX_TEMP_mC 1000

#define CONSEC_OVERCURRENT_READING_THRESH 5
#define CONSEC_OVERVOLTAGE_READING_THRESH 5
#define CONSEC_UNDERVOLTAGE_READING_THRESH 5
#define CONSEC_OVERHEATING_READING_THRESH 5

#define CELL_UNDERVOLT_RECOVER_mV 3200
#define CELL_OVERVOLT_RECOVER_mV 4100

#define DATA_EXPIRATION_mS 250
#define ADC_RAW_MIN 20   // probs disconnected
#define ADC_RAW_MAX 4075 // maxxe

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void calc_cell_voltages(void);
static void calibrate_zero_current(void);
static void calc_current(void);
static void enable_charge_fet(void);
static void enable_discharge_fet(void);
static void disable_charge_fet(void);
static void disable_discharge_fet(void);
static bool is_overcurrent(void);
static bool is_overvoltage(int32_t volt_mV, uint8_t cell_no);
static bool any_overvoltage(void);
static bool is_cell_undervoltage(int32_t volt_mV, uint8_t cell_no);
static bool any_undervoltage(void);
static bool is_overheating(void);
static bool is_safe_charge(bool oc_fault, bool oh_fault);
static bool is_safe_discharge(bool oc_fault, bool oh_fault);
static bool can_enable_discharge(void);
static bool can_enable_charge(void);
static bool validate_cell_taps(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static const uint32_t R_TOP_K[4] = {62, 280, 510, 750};
volatile uint16_t adc1_raw[ADC1_NUM_CHANNELS];
volatile uint16_t tap_raw[4];
volatile uint32_t tap_mv[4];
volatile int32_t cell_mv[4];
volatile int32_t current_mA = 0;

// safe conditions
volatile int32_t temp_mC = 0;
volatile uint8_t adc1_ready = 0;
volatile int32_t calibrated_curr_zero_mv = 0;

volatile uint8_t is_charge_en = 0;
volatile uint8_t is_discharge_en = 0;

volatile uint8_t consec_overcurrent_readings = 0;
volatile uint8_t consec_overvoltage_readings[4] = {0};
volatile uint8_t consec_undervoltage_readings[4] = {0};
volatile uint8_t consec_overheating_readings = 0;

volatile uint32_t last_adc_tick = UINT32_MAX;
volatile bool is_temp_valid = 1;

static void calc_cell_voltages(void) {
  // tap voltage = raw * vref * (r_top + R_DIV_K) / (adc_range * R_DIV_K)
  // NOTE: cell tap 4 is BAT+
  for (int i = 0; i < NUM_CELLS; i++) {
    uint64_t raw = tap_raw[i];
    tap_mv[i] = (uint32_t)((raw * ADC_V_REF * (R_TOP_K[i] + R_DIV_K)) /
                           ((uint64_t)ADC_RANGE * R_DIV_K));
  }

  // cell voltage is diff of adj taps (except for tap 1)
  cell_mv[0] = (int32_t)tap_mv[0];
  for (int i = 1; i < NUM_CELLS; i++) {
    cell_mv[i] = (int32_t)tap_mv[i] - (int32_t)tap_mv[i - 1];
  }
}

static void calibrate_zero_current(void) {
  uint32_t sum = 0;
  for (int i = 0; i < CURR_SENSOR_CALIBRATION_SAMPLES; i++) {
    sum += adc1_raw[CURR_SENSOR_ADC_CHANNEL];
    HAL_Delay(1);
  }

  calibrated_curr_zero_mv =
      (int32_t)((sum * ADC_V_REF) /
                (CURR_SENSOR_CALIBRATION_SAMPLES * ADC_RANGE));
}

static void calc_current(void) {
  uint32_t raw = (uint32_t)adc1_raw[CURR_SENSOR_ADC_CHANNEL];
  int32_t pin_mv = (int32_t)((raw * ADC_V_REF) / ADC_RANGE);
  current_mA =
      ((pin_mv - calibrated_curr_zero_mv) * 1000) / CURR_SENSOR_SENSITIVITY;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    adc1_ready = 1;
    last_adc_tick = HAL_GetTick();
  }
}

static void enable_charge_fet(void) {
  is_charge_en = 1;
  HAL_GPIO_WritePin(CHRG_FET_GPIO_Port, CHRG_FET_Pin, GPIO_PIN_SET);
}

static void enable_discharge_fet(void) {
  is_discharge_en = 1;
  HAL_GPIO_WritePin(DISCHRG_FET_GPIO_Port, DISCHRG_FET_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
}

static void disable_charge_fet(void) {
  is_charge_en = 0;
  HAL_GPIO_WritePin(CHRG_FET_GPIO_Port, CHRG_FET_Pin, GPIO_PIN_RESET);
  for (uint8_t i = 0; i < NUM_CELLS; i++)
    consec_overvoltage_readings[i] = 0;
}

static void disable_discharge_fet(void) {
  is_discharge_en = 0;
  HAL_GPIO_WritePin(DISCHRG_FET_GPIO_Port, DISCHRG_FET_Pin, GPIO_PIN_RESET);
  for (uint8_t i = 0; i < NUM_CELLS; i++)
    consec_undervoltage_readings[i] = 0;
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
}

static bool is_safe_discharge(bool oc_fault, bool oh_fault) {
  return !any_undervoltage() && !oc_fault && !oh_fault;
}

static bool is_safe_charge(bool oc_fault, bool oh_fault) {
  return !any_overvoltage() && !oc_fault && !oh_fault;
}

static bool can_enable_discharge(void) {
  // avoid rapid enable/disable at volt thresh
  for (uint8_t i = 0; i < NUM_CELLS; i++) {
    if (cell_mv[i] < CELL_UNDERVOLT_RECOVER_mV)
      return false;
  }
  if (abs(current_mA) > MAX_CURR_mA)
    return false;
  if (temp_mC > MAX_TEMP_mC)
    return false;
  return true;
}

static bool can_enable_charge(void) {
  // avoid rapid enable/disable at volt thresh
  for (uint8_t i = 0; i < NUM_CELLS; i++) {
    if (cell_mv[i] > CELL_OVERVOLT_RECOVER_mV)
      return false;
  }
  if (abs(current_mA) > MAX_CURR_mA)
    return false;
  if (temp_mC > MAX_TEMP_mC)
    return false;
  return true;
}

static bool is_overcurrent(void) {
  // hardcutoff might be nice for safety
  int32_t i_mA = abs(current_mA);
  if (i_mA > MAX_CURR_mA * 10)
    return true;

  // sample lower cutoff
  if (i_mA > MAX_CURR_mA) {
    if (++consec_overcurrent_readings >= CONSEC_OVERCURRENT_READING_THRESH)
      return true;
  } else {
    consec_overcurrent_readings = 0;
  }
  return false;
}

static bool is_overvoltage(int32_t volt_mV, uint8_t cell_no) {
  // hardcutoff might be nice for safety
  // if (volt_mV > MAX_CELL_mV * 10) return true;

  // sample lower cutoff
  if (volt_mV > MAX_CELL_mV) {
    if (++consec_overvoltage_readings[cell_no] >=
        CONSEC_OVERVOLTAGE_READING_THRESH)
      return true;
  } else {
    consec_overvoltage_readings[cell_no] = 0;
  }
  return false;
}

static bool any_overvoltage(void) {
  bool fault = false;
  for (uint8_t i = 0; i < NUM_CELLS; i++) {
    if (is_overvoltage(cell_mv[i], i))
      fault = true;
  }
  return fault;
}

static bool is_cell_undervoltage(int32_t volt_mV, uint8_t cell_no) {
  // hardcutoff might be nice for safety
  // if (volt_mV < MIN_CELL_mV / 2) return true;

  // sample lower cutoff
  if (volt_mV < MIN_CELL_mV) {
    if (++consec_undervoltage_readings[cell_no] >=
        CONSEC_UNDERVOLTAGE_READING_THRESH)
      return true;
  } else {
    consec_undervoltage_readings[cell_no] = 0;
  }
  return false;
}

static bool any_undervoltage(void) {
  bool fault = false;
  for (uint8_t i = 0; i < NUM_CELLS; i++) {
    if (is_cell_undervoltage(cell_mv[i], i))
      fault = true;
  }
  return fault;
}

static bool is_overheating(void) {
  // hardcutoff might be nice for safety
  if (temp_mC > MAX_TEMP_mC * 10)
    return true;

  // sample lower cutoff
  if (temp_mC > MAX_TEMP_mC) {
    if (++consec_overheating_readings >= CONSEC_OVERHEATING_READING_THRESH)
      return true;
  } else {
    consec_overheating_readings = 0;
  }
  return false;
}

static bool validate_cell_taps(void) {
  for (int i = 0; i < NUM_CELLS; i++) {
    if (tap_raw[i] < ADC_RAW_MIN || tap_raw[i] > ADC_RAW_MAX)
      return false;
  }

  return true;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  disable_charge_fet();
  disable_discharge_fet();

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_raw, ADC1_NUM_CHANNELS) !=
      HAL_OK) {
    Error_Handler();
  }

  HAL_Delay(10);
  calibrate_zero_current();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // HAL_GPIO_TogglePin(FLT_STATUS1_GPIO_Port, FLT_STATUS1_Pin);
    // HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);

    // ensure data fresh
    bool has_recieved_data = (last_adc_tick != UINT32_MAX);
    bool is_adc_valid = (HAL_GetTick() - last_adc_tick) <= DATA_EXPIRATION_mS;
    bool is_new_sample = adc1_ready;

    if (is_new_sample) {
      adc1_ready = 0;
      for (int i = 0; i < NUM_CELLS; i++)
        tap_raw[i] = adc1_raw[i];
      calc_cell_voltages();
      calc_current();
    }

    bool data_valid = has_recieved_data && is_adc_valid && is_temp_valid &&
                      validate_cell_taps();

    if (!data_valid) {
      if (is_charge_en)
        disable_charge_fet();
      if (is_discharge_en)
        disable_discharge_fet();
    } else if (is_new_sample) {
      bool oc_fault = is_overcurrent();
      bool oh_fault = is_overheating();

      // protection
      if (is_charge_en && !is_safe_charge(oc_fault, oh_fault))
        disable_charge_fet();
      if (is_discharge_en && !is_safe_discharge(oc_fault, oh_fault))
        disable_discharge_fet();

      // enable
      if (!is_discharge_en && can_enable_discharge())
        enable_discharge_fet();
      if (!is_charge_en && can_enable_charge())
        enable_charge_fet();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1) {
    HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
