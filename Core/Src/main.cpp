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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include "FirstOrderFilter.hpp"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  ADC_HandleTypeDef *hadc;

  float empty_adc;      // 没有压力时的 ADC 值
  float weight_kg;      // 用于标定的砝码重量，单位：千克
  float weight_adc;     // 用于标定的砝码对应的 ADC 值
  float raw_adc;        // 原始 ADC 值
  float filtered_adc;   // 滤波后的 ADC 值
  float raw_force_kg;   // 根据原始 ADC 值计算的力，单位：千克
  float filtered_force_kg;// 根据滤波后 ADC 值计算的力，单位：千克
  float force_kg;   
} ForceSensor_t;

FirstOrderFilter filter_L;
FirstOrderFilter filter_R;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Select exactly one calibration mode here.
#define FORCE_SENSOR_CALIB_USE_KB //! 使用拟合后的kb参数进行标定
// #define FORCE_SENSOR_CALIB_USE_3POINT //! 使用三个标定点进行线性插值标定

// !Mode 1: force_kg = k * adc + b.
#define FORCE_SENSOR_L_LINEAR_K 0.0037902456f
#define FORCE_SENSOR_L_LINEAR_B -124.4756404968f
#define FORCE_SENSOR_R_LINEAR_K 0.0043918528f
#define FORCE_SENSOR_R_LINEAR_B -144.3149817305f

// !Mode 2: three calibration points. ADC values must be monotonic.
#define FORCE_SENSOR_L_CALIB_FORCE_KG_0 0.0f
#define FORCE_SENSOR_L_CALIB_ADC_0      32767.0f
#define FORCE_SENSOR_L_CALIB_FORCE_KG_1 50.0f
#define FORCE_SENSOR_L_CALIB_ADC_1      49151.0f
#define FORCE_SENSOR_L_CALIB_FORCE_KG_2 100.0f
#define FORCE_SENSOR_L_CALIB_ADC_2      65535.0f

#define FORCE_SENSOR_R_CALIB_FORCE_KG_0 0.0f
#define FORCE_SENSOR_R_CALIB_ADC_0      32767.0f
#define FORCE_SENSOR_R_CALIB_FORCE_KG_1 50.0f
#define FORCE_SENSOR_R_CALIB_ADC_1      49151.0f
#define FORCE_SENSOR_R_CALIB_FORCE_KG_2 100.0f
#define FORCE_SENSOR_R_CALIB_ADC_2      65535.0f

#if defined(FORCE_SENSOR_CALIB_USE_KB) && defined(FORCE_SENSOR_CALIB_USE_3POINT)
#error "Select only one force sensor calibration mode"
#endif

#if !defined(FORCE_SENSOR_CALIB_USE_KB) && !defined(FORCE_SENSOR_CALIB_USE_3POINT)
#error "Select one force sensor calibration mode"
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//TODO: 手动砝码标定
static ForceSensor_t force_sensor_L = {
  .hadc = &hadc2,
  .raw_adc = 0.0f,
  .filtered_adc = 0.0f,
  .raw_force_kg = 0.0f,
  .filtered_force_kg = 0.0f,
  .force_kg = 0.0f
};

static ForceSensor_t force_sensor_R = {
  .hadc = &hadc1,
  .raw_adc = 0.0f,
  .filtered_adc = 0.0f,
  .raw_force_kg = 0.0f,
  .filtered_force_kg = 0.0f,
  .force_kg = 0.0f
};

static volatile uint16_t adc1_dma_latest = 0;
static volatile uint16_t adc2_dma_latest = 0;
static volatile uint32_t adc1_sample_count = 0;
static volatile uint32_t adc2_sample_count = 0;

static uint8_t force_tx_packet[8];

static volatile uint8_t force_uart_tx_ready = 1;
static float force_filter_tau_s = 0.003f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ForceSensor_Init(void);
uint8_t ForceSensor_UpdateIfNewSamples(void);
float ForceSensor_AdcToKg(const ForceSensor_t *sensor, float adc);
float ForceSensor_Interpolate3Point(float adc,
                                    float force_kg_0, float adc_0,
                                    float force_kg_1, float adc_1,
                                    float force_kg_2, float adc_2);
void ForceSensor_SendPacket(void);
uint32_t ForceSensor_KgToDecigram(float force_kg);
void ForceSensor_PackU24(uint8_t *data, uint32_t value);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ForceSensor_Init(void)
{
  filter_R.Init(0.00005f, force_filter_tau_s);
  filter_L.Init(0.00005f, force_filter_tau_s);

  HAL_ADC_Start_DMA(force_sensor_R.hadc, (uint32_t *)&adc1_dma_latest, 1);
  HAL_ADC_Start_DMA(force_sensor_L.hadc, (uint32_t *)&adc2_dma_latest, 1);

  __HAL_TIM_SET_COUNTER(&htim6, 0);
  HAL_TIM_Base_Start(&htim6);
}

uint8_t ForceSensor_UpdateIfNewSamples(void)
{
  static uint32_t last_adc1_sample_count = 0;
  static uint32_t last_adc2_sample_count = 0;
  uint32_t now_adc1_sample_count = adc1_sample_count;
  uint32_t now_adc2_sample_count = adc2_sample_count;

  if ((now_adc1_sample_count == last_adc1_sample_count) ||
      (now_adc2_sample_count == last_adc2_sample_count))
  {
    return 0;
  }

  last_adc1_sample_count = now_adc1_sample_count;
  last_adc2_sample_count = now_adc2_sample_count;

  force_sensor_R.raw_adc = (float)adc1_dma_latest;
  force_sensor_L.raw_adc = (float)adc2_dma_latest;

  force_sensor_R.filtered_adc = filter_R.Update(force_sensor_R.raw_adc);
  force_sensor_L.filtered_adc = filter_L.Update(force_sensor_L.raw_adc);

  force_sensor_R.raw_force_kg = ForceSensor_AdcToKg(&force_sensor_R, force_sensor_R.raw_adc);
  force_sensor_L.raw_force_kg = ForceSensor_AdcToKg(&force_sensor_L, force_sensor_L.raw_adc);

  force_sensor_R.filtered_force_kg = ForceSensor_AdcToKg(&force_sensor_R, force_sensor_R.filtered_adc);
  force_sensor_L.filtered_force_kg = ForceSensor_AdcToKg(&force_sensor_L, force_sensor_L.filtered_adc);

  force_sensor_R.force_kg = force_sensor_R.filtered_force_kg;
  force_sensor_L.force_kg = force_sensor_L.filtered_force_kg;

  return 1;
}

float ForceSensor_AdcToKg(const ForceSensor_t *sensor, float adc)
{
  if (sensor == &force_sensor_L)
  {
#ifdef FORCE_SENSOR_CALIB_USE_KB
    return FORCE_SENSOR_L_LINEAR_K * adc + FORCE_SENSOR_L_LINEAR_B;
#elif defined(FORCE_SENSOR_CALIB_USE_3POINT)
    return ForceSensor_Interpolate3Point(adc,
                                         FORCE_SENSOR_L_CALIB_FORCE_KG_0,
                                         FORCE_SENSOR_L_CALIB_ADC_0,
                                         FORCE_SENSOR_L_CALIB_FORCE_KG_1,
                                         FORCE_SENSOR_L_CALIB_ADC_1,
                                         FORCE_SENSOR_L_CALIB_FORCE_KG_2,
                                         FORCE_SENSOR_L_CALIB_ADC_2);
#endif
  }

  if (sensor == &force_sensor_R)
  {
#ifdef FORCE_SENSOR_CALIB_USE_KB
    return FORCE_SENSOR_R_LINEAR_K * adc + FORCE_SENSOR_R_LINEAR_B;
#elif defined(FORCE_SENSOR_CALIB_USE_3POINT)
    return ForceSensor_Interpolate3Point(adc,
                                         FORCE_SENSOR_R_CALIB_FORCE_KG_0,
                                         FORCE_SENSOR_R_CALIB_ADC_0,
                                         FORCE_SENSOR_R_CALIB_FORCE_KG_1,
                                         FORCE_SENSOR_R_CALIB_ADC_1,
                                         FORCE_SENSOR_R_CALIB_FORCE_KG_2,
                                         FORCE_SENSOR_R_CALIB_ADC_2);
#endif
  }

  return 0.0f;
}

float ForceSensor_Interpolate3Point(float adc,
                                    float force_kg_0, float adc_0,
                                    float force_kg_1, float adc_1,
                                    float force_kg_2, float adc_2)
{
  float adc_a = adc_0;
  float adc_b = adc_1;
  float force_kg_a = force_kg_0;
  float force_kg_b = force_kg_1;
  float span_adc;

  if (((adc_0 <= adc_1) && (adc <= adc_1)) ||
      ((adc_0 > adc_1) && (adc >= adc_1)))
  {
    adc_a = adc_0;
    adc_b = adc_1;
    force_kg_a = force_kg_0;
    force_kg_b = force_kg_1;
  }
  else
  {
    adc_a = adc_1;
    adc_b = adc_2;
    force_kg_a = force_kg_1;
    force_kg_b = force_kg_2;
  }

  span_adc = adc_b - adc_a;
  if (fabsf(span_adc) < 1.0f)
  {
    return force_kg_a;
  }

  return force_kg_a + (adc - adc_a) * (force_kg_b - force_kg_a) / span_adc;
}

void ForceSensor_SendPacket(void)
{
  if (force_uart_tx_ready == 0)
  {
    return;
  }

  force_tx_packet[0] = 'L';
  ForceSensor_PackU24(&force_tx_packet[1], ForceSensor_KgToDecigram(force_sensor_L.force_kg));
  force_tx_packet[4] = 'R';
  ForceSensor_PackU24(&force_tx_packet[5], ForceSensor_KgToDecigram(force_sensor_R.force_kg));

  force_uart_tx_ready = 0;
  if (HAL_UART_Transmit_DMA(&huart2, force_tx_packet, 8) != HAL_OK)
  {
    force_uart_tx_ready = 1;
  }
}

uint32_t ForceSensor_KgToDecigram(float force_kg)
{
  if (force_kg < 0.0f)
  {
    force_kg = 0.0f;
  }
  else if (force_kg > 100.0f)
  {
    force_kg = 100.0f;
  }

  return (uint32_t)(force_kg * 10000.0f + 0.5f);
}

void ForceSensor_PackU24(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)(value & 0xFF);
  data[1] = (uint8_t)((value >> 8) & 0xFF);
  data[2] = (uint8_t)((value >> 16) & 0xFF);
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
  MX_ADC2_Init();
  MX_FDCAN1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  ForceSensor_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (ForceSensor_UpdateIfNewSamples() != 0)
    {
      ForceSensor_SendPacket();
    }
  }
  HAL_Delay(1);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 42;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    adc1_sample_count++;
  }
  else if (hadc->Instance == ADC2)
  {
    adc2_sample_count++;
  }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    force_uart_tx_ready = 1;
  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM20 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM20)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
