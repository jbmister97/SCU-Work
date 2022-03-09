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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "servo.h"
#include "ux_manager.h"
#include "hc_sr04.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_BOARD_PIN           GPIOA, GPIO_PIN_5
#define SETPOINT_LEFT_PIN       GPIOB, GPIO_PIN_13
#define SETPOINT_MID_PIN        GPIOB, GPIO_PIN_14
#define SETPOINT_RIGHT_PIN      GPIOB, GPIO_PIN_15
#define SENSOR_TRIG_PIN         GPIOB, GPIO_PIN_1
    
#define DUTY_INTERVAL           50

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal1,
};
/* Definitions for errorCalc */
osThreadId_t errorCalcHandle;
const osThreadAttr_t errorCalc_attributes = {
  .name = "errorCalc",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal1,
};
/* Definitions for setPWM */
osThreadId_t setPWMHandle;
const osThreadAttr_t setPWM_attributes = {
  .name = "setPWM",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal2,
};
/* Definitions for readButtons */
osThreadId_t readButtonsHandle;
const osThreadAttr_t readButtons_attributes = {
  .name = "readButtons",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal1,
};
/* Definitions for displayUpdate */
osThreadId_t displayUpdateHandle;
const osThreadAttr_t displayUpdate_attributes = {
  .name = "displayUpdate",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal1,
};
/* Definitions for getDistance */
osThreadId_t getDistanceHandle;
const osThreadAttr_t getDistance_attributes = {
  .name = "getDistance",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal1,
};
/* USER CODE BEGIN PV */
uint8_t buttons;
uint16_t count;

//uint8_t servoState = SERVO_STATE_ZERO_POSITION;
uint8_t servoDir = MID;
uint8_t servoDirLast = RIGHT;
uint16_t servoDutyCurrent = SERVO_STATE_ZERO_POSITION;

// Display Varaibles
DWfloat speed = {"%4.1f ", "----", 0, 0, true, 25.0};
DWfloat distance = {"%4d ", "----", 0, 0, true, 25.0};

// Ultrasonic Sensor
uint8_t firstCaptured = false;
uint16_t pulseVal1, pulseVal2;
uint16_t pulseWidthValue;
float distanceInCM;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void *argument);
void Task_Error_Calc(void *argument);
void Task_Set_PWM(void *argument);
void Task_Read_Buttons(void *argument);
void Task_Display_Update(void *argument);
void Task_Get_Distance(void *argument);

/* USER CODE BEGIN PFP */
void Set_Servo_Position(uint16_t duty);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  //if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    if(!firstCaptured) {
      pulseVal1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      firstCaptured = true;
    }
    else {
      pulseVal2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      
      if(pulseVal1 < pulseVal2) {pulseWidthValue = pulseVal2 - pulseVal1;}
      else if (pulseVal1 > pulseVal2) {pulseWidthValue = (0xFFFF - pulseVal1) + pulseVal2;}
      
      // Calculate distance in centimeters
      distance.data = pulseWidthValue/58.0;
      
      __HAL_TIM_SET_COUNTER(htim, 0);
      firstCaptured = false;
    }
  //}
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // Initialize PWM timer
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  
  // Initialize timer for delay
  HAL_TIM_Base_Start(&htim8);
  
  // Initialize input capture interrupt for sensor
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
  
  // Initialize display
  SSD1306_Init();
  SwitchScreens(HOME);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of errorCalc */
  errorCalcHandle = osThreadNew(Task_Error_Calc, NULL, &errorCalc_attributes);

  /* creation of setPWM */
  setPWMHandle = osThreadNew(Task_Set_PWM, NULL, &setPWM_attributes);

  /* creation of readButtons */
  readButtonsHandle = osThreadNew(Task_Read_Buttons, NULL, &readButtons_attributes);

  /* creation of displayUpdate */
  displayUpdateHandle = osThreadNew(Task_Display_Update, NULL, &displayUpdate_attributes);

  /* creation of getDistance */
  getDistanceHandle = osThreadNew(Task_Get_Distance, NULL, &getDistance_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 32-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 16-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Board_LED_Out_GPIO_Port, Board_LED_Out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : Board_LED_Out_Pin */
  GPIO_InitStruct.Pin = Board_LED_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Board_LED_Out_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LEFT_SP_BTN_Pin PB14 RIGHT_SP_BTN_Pin */
  GPIO_InitStruct.Pin = LEFT_SP_BTN_Pin|GPIO_PIN_14|RIGHT_SP_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void delay_us(uint16_t us) {
  /*
  __HAL_TIM_SET_COUNTER(&htim14,0);  // set the counter value a 0
  while (__HAL_TIM_GET_COUNTER(&htim14) < us){;} // wait for the counter to reach the us input in the parameter
*/
  htim8.Instance->CNT = 0;
  while(htim8.Instance->CNT < us);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  portTickType lastWakeTime;
  const portTickType taskInterval = 1000; //(every n ticks in ms)
  lastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED_BOARD_PIN);
    vTaskDelayUntil(&lastWakeTime,taskInterval);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Task_Error_Calc */
/**
* @brief Function implementing the errorCalc thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Error_Calc */
void Task_Error_Calc(void *argument)
{
  /* USER CODE BEGIN Task_Error_Calc */
  portTickType lastWakeTime;
  const portTickType taskInterval = 1200; //(every n ticks in ms)
  lastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    vTaskDelayUntil(&lastWakeTime,taskInterval);
  }
  /* USER CODE END Task_Error_Calc */
}

/* USER CODE BEGIN Header_Task_Set_PWM */
/**
* @brief Function implementing the setPWM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Set_PWM */
void Task_Set_PWM(void *argument)
{
  /* USER CODE BEGIN Task_Set_PWM */
  portTickType lastWakeTime;
  const portTickType taskInterval = 50; //(every n ticks in ms)
  lastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    
    /*
    switch(servoDir) {
        case LEFT:
          if(servoDir != servoDirLast) {
            if(servoDutyCurrent != SERVO_NEG_90_POSITION_VALUE) {
              Set_Servo_Position(servoDutyCurrent);
              if(servoDutyCurrent < SERVO_NEG_90_POSITION_VALUE) {servoDutyCurrent += DUTY_INTERVAL;}
              if(servoDutyCurrent > SERVO_NEG_90_POSITION_VALUE) {servoDutyCurrent -= DUTY_INTERVAL;}
            }
            else {servoDirLast = servoDir;}
          }
          break;
        case MID:
          if(servoDir != servoDirLast) {
            if(servoDutyCurrent != SERVO_ZERO_POSITION_VALUE) {
              Set_Servo_Position(servoDutyCurrent);
              if(servoDutyCurrent < SERVO_ZERO_POSITION_VALUE) {servoDutyCurrent += DUTY_INTERVAL;}
              if(servoDutyCurrent > SERVO_ZERO_POSITION_VALUE) {servoDutyCurrent -= DUTY_INTERVAL;}
            }
            else {servoDirLast = servoDir;}
          }
          break;
        case RIGHT:
          if(servoDir != servoDirLast) {
            if(servoDutyCurrent != SERVO_POS_90_POSITION_VALUE) {
              Set_Servo_Position(servoDutyCurrent);
              if(servoDutyCurrent < SERVO_POS_90_POSITION_VALUE) {servoDutyCurrent += DUTY_INTERVAL;}
              if(servoDutyCurrent > SERVO_POS_90_POSITION_VALUE) {servoDutyCurrent -= DUTY_INTERVAL;}
            }
            else {servoDirLast = servoDir;}
          }
          break;
    }
  */
    /*
    switch(servoDir) {
        case LEFT:
          if(servoDir != servoDirLast) {
            Set_Servo_Position(SERVO_NEG_90_POSITION_VALUE);
            servoDirLast = servoDir;
          }
          break;
        case MID:
          if(servoDir != servoDirLast) {
            Set_Servo_Position(SERVO_ZERO_POSITION_VALUE);
            servoDirLast = servoDir;
          }
          break;
        case RIGHT:
          if(servoDir != servoDirLast) {
            Set_Servo_Position(SERVO_POS_90_POSITION_VALUE);
            servoDirLast = servoDir;
          }
          break;
    }
    */
    vTaskDelayUntil(&lastWakeTime,taskInterval);
  }
  /* USER CODE END Task_Set_PWM */
}

/* USER CODE BEGIN Header_Task_Read_Buttons */
/**
* @brief Function implementing the readButtons thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Read_Buttons */
void Task_Read_Buttons(void *argument)
{
  /* USER CODE BEGIN Task_Read_Buttons */
  portTickType lastWakeTime;
  const portTickType taskInterval = 15; //(every n ticks in ms)
  lastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    buttons = (HAL_GPIO_ReadPin(SETPOINT_LEFT_PIN) << 2) | (HAL_GPIO_ReadPin(SETPOINT_MID_PIN) << 1) | (HAL_GPIO_ReadPin(SETPOINT_RIGHT_PIN) << 0);
    switch(buttons){
      case 0x7:      // if no buttons pressed, do nothing
        break;
      case 0x6:      // if right button pressed
        servoDir = RIGHT;
        break;
      case 0x5:      // if mid button pressed
      case 0x4:
        servoDir = MID;
        break;
      case 0x3:      // if left button pressed
      case 0x2:
      case 0x1:
      case 0x0:
        servoDir = LEFT;
        break;
      
    }
    vTaskDelayUntil(&lastWakeTime,taskInterval);
  }
  /* USER CODE END Task_Read_Buttons */
}

/* USER CODE BEGIN Header_Task_Display_Update */
/**
* @brief Function implementing the displayUpdate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Display_Update */
void Task_Display_Update(void *argument)
{
  /* USER CODE BEGIN Task_Display_Update */
  portTickType lastWakeTime;
  const portTickType taskInterval = 33; //(every n ticks in ms)
  lastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    // Update display;
    UpdateScreenValues();
    vTaskDelayUntil(&lastWakeTime,taskInterval);
  }
  /* USER CODE END Task_Display_Update */
}

/* USER CODE BEGIN Header_Task_Get_Distance */
/**
* @brief Function implementing the getDistance thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Get_Distance */
void Task_Get_Distance(void *argument)
{
  /* USER CODE BEGIN Task_Get_Distance */
  portTickType lastWakeTime;
  const portTickType taskInterval = 75; //(every n ticks in ms)
  lastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    // Send read command to sensor
    HAL_GPIO_WritePin(SENSOR_TRIG_PIN, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(SENSOR_TRIG_PIN, GPIO_PIN_RESET);
    
    
    vTaskDelayUntil(&lastWakeTime,taskInterval);
  }
  /* USER CODE END Task_Get_Distance */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
