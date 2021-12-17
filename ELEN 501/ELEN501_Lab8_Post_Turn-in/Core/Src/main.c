/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Scheduler.h"
#include "arm_math.h"
#include "arm_const_structs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RED_LED_PIN     GPIOA, GPIO_PIN_9
#define GREEN_LED_PIN   GPIOA, GPIO_PIN_10
#define BLUE_LED_PIN    GPIOA, GPIO_PIN_8
#define WAVEFORM_SIZE   2048
#define BUCKET_SIZE     WAVEFORM_SIZE/4
#define BUCKET_MINIMUM  35000
#define BUCKET_MAXIMUM  200000
#define DECAY_COUNT     2
#define DECAY_STEP      25
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

uint16_t waveformInput[WAVEFORM_SIZE];
float32_t waveformCapture[WAVEFORM_SIZE/2];
float32_t waveformProcessed[BUCKET_SIZE];
uint8_t triggerWaveform = false;
uint8_t count = 0;
uint8_t subCount = 0;
uint8_t firstHalfCaptured = false;
uint8_t secondHalfCaptured = false;

uint32_t fftSize = BUCKET_SIZE;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;
uint32_t binIndex = 0;
uint32_t binRed;
uint32_t binGreen;
uint32_t binBlue;

uint16_t dutyRed;
uint16_t dutyGreen;
uint16_t dutyBlue;
uint16_t lastDutyRed;
uint16_t lastDutyGreen;
uint16_t lastDutyBlue;
uint8_t decayFlag;
uint16_t decayCount = DECAY_COUNT;

float32_t maxValue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void RGB_Process(float32_t *array);
void SetRedLedLevel(uint16_t r_level);
void SetGreenLedLevel(uint16_t g_level);
void SetBlueLedLevel(uint16_t b_level);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t ten_mS_Flag;
extern uint8_t twentyfive_mS_Flag;
extern uint8_t hundred_mS_Flag;
extern uint8_t one_S_Flag;

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
  MX_TIM6_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  // Start timer for  DMA
  HAL_TIM_Base_Start(&htim6);
  
  // Start timer for Blue LED
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
  
  // Start timer for Red LED
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
  
  // Start timer for Green LED
  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(ten_mS_Flag) {
      ten_mS_Flag = false;
      
    }
    
    if(twentyfive_mS_Flag) {
      twentyfive_mS_Flag = false;
      
      subCount++;
      
      // Wait 250ms before triggering DMA
      if(subCount == 10) {
        subCount = 0;
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *) waveformInput, WAVEFORM_SIZE);
      }
      
      if(binRed < BUCKET_MINIMUM) {dutyRed = 0;}
      else if(binRed > BUCKET_MAXIMUM) {dutyRed = 65535;}
      else {dutyRed = binRed / 3;}
      
      // Only update the pwm if the new value is higher than the previous
      if(dutyRed > lastDutyRed) {lastDutyRed = dutyRed;}
      
      
      // Set green LED value
      if(binGreen < BUCKET_MINIMUM) {dutyGreen = 0;}
      else if(binGreen > BUCKET_MAXIMUM) {dutyGreen = 65535;}
      else {dutyGreen = binGreen / 3;}
      
      // Only update the pwm if the new value is higher than the previous
      if(dutyGreen > lastDutyGreen) {lastDutyGreen = dutyGreen;}
      
      // Set Blue LED value
      if(binBlue < BUCKET_MINIMUM) {dutyBlue = 0;}
      else if(binBlue > BUCKET_MAXIMUM) {dutyBlue = 65535;}
      else {dutyBlue = binBlue / 3;}
      
      // Only update the pwm if the new value is higher than the previous
      if(dutyBlue > lastDutyBlue) {lastDutyBlue = dutyBlue;}
      
    }
    
    // Process the first half of the DMA capture
    if(firstHalfCaptured) {
      firstHalfCaptured = false;

      for(uint16_t i = 0; i < WAVEFORM_SIZE/2; i++) {waveformCapture[i] = waveformInput[i] - 1980;}
      
      arm_cfft_f32(&arm_cfft_sR_f32_len512, waveformCapture, ifftFlag, doBitReverse);
      arm_cmplx_mag_f32(waveformCapture, waveformProcessed, fftSize);
      
      RGB_Process(waveformProcessed);
    }
    
    // Process the first half of the DMA capture
    if(secondHalfCaptured) {
      secondHalfCaptured = false;

      for(uint16_t i = WAVEFORM_SIZE/2; i < WAVEFORM_SIZE; i++) {waveformCapture[i-(WAVEFORM_SIZE/2)] = waveformInput[i] - 1980;}
      
      arm_cfft_f32(&arm_cfft_sR_f32_len512, waveformCapture, ifftFlag, doBitReverse);
      arm_cmplx_mag_f32(waveformCapture, waveformProcessed, fftSize);
      
      RGB_Process(waveformProcessed);
    }
    
    if(hundred_mS_Flag) {
      hundred_mS_Flag = false;

    }
    
    if(one_S_Flag) {
      one_S_Flag = false;
      
    }
    
    // Decay the current PWM values
    if(lastDutyRed > DECAY_STEP) {lastDutyRed -= DECAY_STEP;}
    else {lastDutyRed = 0;}
    if(lastDutyBlue > DECAY_STEP) {lastDutyBlue -= DECAY_STEP;}
    else {lastDutyBlue = 0;}
    if(lastDutyGreen > DECAY_STEP) {lastDutyGreen -= DECAY_STEP;}
    else {lastDutyGreen = 0;}
    
    // Update the LED PWMs
    SetRedLedLevel(lastDutyRed);
    SetGreenLedLevel(lastDutyGreen);
    SetBlueLedLevel(lastDutyBlue);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 150;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void RGB_Process(float32_t *array) {
  uint16_t i = 0;
  binRed = 0;
  binGreen = 0;
  binBlue = 0;
  // Calculate red bucket
  for(i = 0; i < (BUCKET_SIZE/4); i++) {
    binRed += array[i];
  }
  
  // Calculate green bucket
  for(i = BUCKET_SIZE/4; i < (BUCKET_SIZE-(BUCKET_SIZE/4)); i++) {
    binGreen += array[i];
  }
  
  // Calculate blue bucket
  for(i = (BUCKET_SIZE-(BUCKET_SIZE/4)); i < BUCKET_SIZE; i++ ) {
    binBlue += array[i];
  }
}

void SetRedLedLevel(uint16_t r_level)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  //redValue = r_level;

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = r_level;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

void SetBlueLedLevel(uint16_t b_level)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  //blueValue = b_level;
 
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = b_level;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
 
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void SetGreenLedLevel(uint16_t g_level)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  //greenValue = g_level;
 
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = g_level;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
