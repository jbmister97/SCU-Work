/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "fonts.h"
#include "ssd1306.h"
#include "ux_manager.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUFFER_SIZE         100
#define ADC_INDEX_TEMP          2
#define ADC_INDEX_LOW           0
#define ADC_INDEX_MID           3
#define ADC_INDEX_HIGH          1
#define LOGIC_T1_BUFF_SIZE      50
#define LOGIC_T2_BUFF_SIZE      50
#define LOGIC_T3_BUFF_SIZE      50
#define LOGIC_T1_PIN            GPIOA, GPIO_PIN_12
#define LOGIC_T2_PIN            GPIOB, GPIO_PIN_0
#define LOGIC_T3_PIN            GPIOB, GPIO_PIN_7
#define LOGIC_HEATER_PIN        GPIOA, GPIO_PIN_5
#define LOGIC_GREEN_PIN         GPIOA, GPIO_PIN_4
#define LOGIC_RED_PIN           GPIOA, GPIO_PIN_6
#define HEATER_OFF_STATE              0
#define HEATER_ON_STATE               1
#define HEATER_RANGE            20      // Range of temperature chamber in F
#define ERROR_BUFF_SIZE         2
#define SAMPLE_INTERVAL         0.025   // Time in seconds between error captures
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
// Scheduler Variables
extern uint8_t ten_mS_Flag;
extern uint8_t twentyfive_mS_Flag;
extern uint8_t hundred_mS_Flag;
extern uint8_t one_S_Flag;
extern uint8_t five_hundred_mS_Flag;

// ADC Variables
const uint8_t gain = 2;
uint16_t adcValue[4];
uint16_t adcBuffer[ADC_BUFFER_SIZE];
uint16_t adcAvg;
float tempInC;
float tempF;
float voltageInmV;

// Controller
uint8_t inputs;
uint8_t t1;
uint8_t t2;
uint8_t t3;
uint8_t heaterState;
uint8_t greenOutput;
uint8_t redOutput;

uint16_t t1Buffer[LOGIC_T1_BUFF_SIZE];
uint16_t t2Buffer[LOGIC_T2_BUFF_SIZE];
uint16_t t3Buffer[LOGIC_T3_BUFF_SIZE];
uint16_t t1Avg;
uint16_t t2Avg;
uint16_t t3Avg;
uint32_t HeaterPWMDuty;
float tempThreshLowF;
float tempThreshMidF;
float tempThreshHighF;
float tempError;
float tempErrorLast;
float errorBuff[ERROR_BUFF_SIZE];
const float kp = 1.4;
const float kd = 0.3;
float proportional;
float integral;
float differentiator;
float sum;

// Display Varaibles
DWfloat temperature = {"%4.1f ", "----", 0, 0, true, 25.0};
DWfloat tempThreshLow = {"%4.1f ", "----", 0, 0, true, 25.0};
DWfloat tempThreshMid = {"%4.1f ", "----", 0, 0, true, 25.0};
DWfloat tempThreshHigh = {"%4.1f ", "----", 0, 0, true, 25.0};
DWstring units = {"%s", "----", 0, 0, true, "DegC"};
uint8_t unitChoices[2][5] = {"DegF", "DegC"};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void Update_TempC(void);
void Update_ADCBuff(uint16_t value);
float Calc_Temp_Thresh(uint16_t adcCounts);
float Calc_Temp_Thresh_Low(uint16_t adcCounts);
float Calc_Temp_Thresh_High(uint16_t adcCounts);
void Update_T1_Buff(uint16_t value);
void Update_T2_Buff(uint16_t value);
void Update_T3_Buff(uint16_t value);
void Update_T1_Avg(void);
void Update_T2_Avg(void);
void Update_T3_Avg(void);
void Heater_Calc_Duty(float temp);
void Heater_Set_Duty(uint32_t counts);
void Update_Error_Buff(float error);
void Update_Error_Controller(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  
    
  
  SSD1306_Init();  // initialise  
  
  adcBuffer[0] = 0;
  adcBuffer[1] = 0;
  adcBuffer[2] = 0;
  
  // Start PWM Timer
  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
  
  // Splash Screen
  SSD1306_Clear();
  SSD1306_GotoXY (0,0);
  SSD1306_Puts ("Welcome to", &Font_11x18, SSD1306_COLOR_WHITE);
  SSD1306_GotoXY (0,20);
  SSD1306_Puts ("The Thermal", &Font_11x18, SSD1306_COLOR_WHITE);
  SSD1306_GotoXY (0,40);
  SSD1306_Puts ("Chamber!", &Font_11x18, SSD1306_COLOR_WHITE);
  SSD1306_UpdateScreen(); 
  
  HAL_Delay (3000);

  SwitchScreens(HOME);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    // 10 ms scheduler
    if(ten_mS_Flag) {
      ten_mS_Flag = false;
      
      // Read ADC and update buffer
      HAL_ADC_Start_DMA(&hadc, (uint32_t *) adcValue, 4);
      Update_ADCBuff(adcValue[ADC_INDEX_TEMP]);
      
      // Update Threshold buffers
      Update_T1_Buff(adcValue[ADC_INDEX_LOW]);
      Update_T2_Buff(adcValue[ADC_INDEX_MID]);
      Update_T3_Buff(adcValue[ADC_INDEX_HIGH]);
      
      // Update output variables
      heaterState = (!t2)*(!t3)*(!((!heaterState)*(t1)));
      greenOutput = !t1;
      redOutput = t3;
    }
    
    // 25 ms scheduler
    if(twentyfive_mS_Flag) {
      twentyfive_mS_Flag = false;
      
      // Update error and sum
      tempErrorLast = tempError;
      tempError = tempThreshMidF - tempF;
      Update_Error_Controller();
      
      // Check thresholds
      t1 = (tempF > tempThreshLowF);
      t2 = (tempF > tempThreshMidF);
      t3 = (tempF > tempThreshHighF);
      inputs = (t3 << 2) | (t2 << 1) | (t1 << 0);
      
      // Update output pins
      // Heater output
      if(heaterState) {
        // Update PWM duty
        Heater_Calc_Duty(sum);
        // Set PWM duty cycle
        Heater_Set_Duty(HeaterPWMDuty);
      }
      else {
        Heater_Set_Duty(0);
      }
      
      // Red output
      if(redOutput) {
        HAL_GPIO_WritePin(LOGIC_RED_PIN, GPIO_PIN_SET);
      }
      else {
        HAL_GPIO_WritePin(LOGIC_RED_PIN, GPIO_PIN_RESET);
      }
      
      // Green output
      if(greenOutput) {
        HAL_GPIO_WritePin(LOGIC_GREEN_PIN, GPIO_PIN_SET);
      }
      else {
        HAL_GPIO_WritePin(LOGIC_GREEN_PIN, GPIO_PIN_RESET);
      }
    }
    
    // 100 ms scheduler
    
    if(hundred_mS_Flag) {
      hundred_mS_Flag = false;
      
      // Update Temp value in C
      Update_TempC();
    }
    
    // 500 ms scheduler
    if(five_hundred_mS_Flag){
      five_hundred_mS_Flag = false;
      
      // convert temp to F
      tempF = tempInC*(9.0/5.0) + 32.0;
      temperature.data = tempF;
      // Change units to F
      units.data[3] = 'F';
      
      // Update threshold values
      Update_T1_Avg();
      Update_T2_Avg();
      Update_T3_Avg();
      tempThreshLowF = Calc_Temp_Thresh_Low(t1Avg);
      tempThreshLow.data = tempThreshLowF;
      tempThreshMidF = Calc_Temp_Thresh(t2Avg);
      tempThreshMid.data = tempThreshMidF;
      tempThreshHighF = Calc_Temp_Thresh_High(t3Avg);
      tempThreshHigh.data = tempThreshHighF;
      
      // Update display;
      UpdateScreenValues();
    }
    
    
    // 1 second scheduler
    if(one_S_Flag) {
      one_S_Flag = false;
      
      
      
    }
    
    // Every loop
    
    
    
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_3CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = ENABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000708;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Update_ADCBuff(uint16_t value){

  // Shift values of the buffer
  for(uint8_t i = 1; i < ADC_BUFFER_SIZE; i++) {
    adcBuffer[i-1] = adcBuffer[i];
  }
    
  // Place new value at end of ADC buffer;
  adcBuffer[ADC_BUFFER_SIZE-1] = value;
}

void Update_TempC(void) {
  uint32_t temp;
  for(uint8_t i = 0; i < ADC_BUFFER_SIZE; i++) {
    temp += adcBuffer[i];
  }
  adcAvg = temp / ADC_BUFFER_SIZE;
  
  voltageInmV = (adcAvg/4096.0)*3300.0;
  
  // Based on 0C-100C range
  tempInC = (voltageInmV-1567.0)/(-8.2);
  
}

float Calc_Temp_Thresh(uint16_t adcCounts) {
  
  return ((adcCounts/4096.0)*60.0)+60.0;
}

float Calc_Temp_Thresh_Low(uint16_t adcCounts) {
  
  return tempThreshMidF - ((adcCounts/4096.0)*10.0);
}

float Calc_Temp_Thresh_High(uint16_t adcCounts) {
  
  return tempThreshMidF + ((adcCounts/4096.0)*10.0);
}

void Update_T1_Buff(uint16_t value){

  // Shift values of the buffer
  for(uint8_t i = 1; i < LOGIC_T1_BUFF_SIZE; i++) {
    t1Buffer[i-1] = t1Buffer[i];
  }
    
  // Place new value at end of buffer;
  t1Buffer[LOGIC_T1_BUFF_SIZE-1] = value;
}

void Update_T2_Buff(uint16_t value){

  // Shift values of the buffer
  for(uint8_t i = 1; i < LOGIC_T2_BUFF_SIZE; i++) {
    t2Buffer[i-1] = t2Buffer[i];
  }
    
  // Place new value at end of buffer;
  t2Buffer[LOGIC_T2_BUFF_SIZE-1] = value;
}

void Update_T3_Buff(uint16_t value){

  // Shift values of the buffer
  for(uint8_t i = 1; i < LOGIC_T3_BUFF_SIZE; i++) {
    t3Buffer[i-1] = t3Buffer[i];
  }
    
  // Place new value at end of buffer;
  t3Buffer[LOGIC_T3_BUFF_SIZE-1] = value;
}

void Update_T1_Avg(void) {
  uint32_t temp;
  for(uint8_t i = 0; i < LOGIC_T1_BUFF_SIZE; i++) {
    temp += t1Buffer[i];
  }
  t1Avg = temp / LOGIC_T1_BUFF_SIZE;
}

void Update_T2_Avg(void) {
  uint32_t temp;
  for(uint8_t i = 0; i < LOGIC_T2_BUFF_SIZE; i++) {
    temp += t2Buffer[i];
  }
  t2Avg = temp / LOGIC_T2_BUFF_SIZE;
}

void Update_T3_Avg(void) {
  uint32_t temp;
  for(uint8_t i = 0; i < LOGIC_T3_BUFF_SIZE; i++) {
    temp += t3Buffer[i];
  }
  t3Avg = temp / LOGIC_T3_BUFF_SIZE;
}

void Heater_Calc_Duty(float temp) {
  float error;
  error = temp / HEATER_RANGE;
  if(error > 1) {error = 1.0;}
  else if(error < 0) {error = 0;}
  HeaterPWMDuty = (uint32_t) error * 65535;
}

void Update_Error_Buff(float value) {
  // Shift values of the buffer
  for(uint8_t i = 1; i < ERROR_BUFF_SIZE; i++) {
    errorBuff[i-1] = errorBuff[i];
  }
  // Place new value at end of buffer;
  errorBuff[ERROR_BUFF_SIZE-1] = value;
}

void Update_Error_Controller(void) {
  
  proportional = kp * tempError;
  
  //integrator += ki * SAMPLE_INTERVAL * ((tempError+tempErrorLast)/2);
  
  differentiator = kd * ((tempError-tempErrorLast)/SAMPLE_INTERVAL);
  
  sum = proportional + differentiator;
  
}

void Heater_Set_Duty(uint32_t counts) {
  TIM_OC_InitTypeDef sConfigOC = {0};

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = counts;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
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
