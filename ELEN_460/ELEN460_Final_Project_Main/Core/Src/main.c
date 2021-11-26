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
#include "KeyboardHoldRepeat.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
    
// Button GPIO defined in main.h
#define COIL_PIN                GPIOB, GPIO_PIN_6
#define LED_PIN                  GPIOA, GPIO_PIN_11
#define GOAL_1_PIN              GPIOB, GPIO_PIN_7
#define MOTOR_AIN1_PIN          GPIOB, GPIO_PIN_5
#define MOTOR_AIN2_PIN          GPIOB, GPIO_PIN_4
#define MOTOR_DIR_PIN           GPIOA, GPIO_PIN_10

#define MOTOR_STOP              0
#define MOTOR_FWD               1
#define MOTOR_REV               2
#define MOTOR_SPEED             65535

// I2C Device Addresses
//#define BOARD_ADDR              0x60
#define DISPLAY_1_ADDR          (0x70 << 1)
#define DISPLAY_2_ADDR          (0x71 << 1)
#define DISPLAY_3_ADDR          (0x72 << 1)
#define DISPLAY_4_ADDR          (0x73 << 1)

// Display system oscillator setup
#define DISPLAY_OSC_OFF                 0x20
#define DISPLAY_OSC_ON                  0x21
#define DISPLAY_OFF                     0x80
#define DISPLAY_ON                      0x81
#define DISPLAY_COM0_1                  0x00
#define DISPLAY_COM0_2                 0x01
#define DISPLAY_COM1_1                  0x02
#define DISPLAY_COM1_2                  0x03
#define DISPLAY_COM2_1                  0x04
#define DISPLAY_COM2_2                  0x05
#define DISPLAY_COM3_1                  0x06
#define DISPLAY_COM3_2                  0x07
#define DISPLAY_COM4_1                  0x08
#define DISPLAY_COM4_2                  0x09
#define DISPLAY_COM5_1                  0x0A
#define DISPLAY_COM5_2                  0x0B
#define DISPLAY_COM6_1                  0x0C
#define DISPLAY_COM6_2                  0x0D
#define DISPLAY_COM7_1                  0x0E
#define DISPLAY_COM7_2                  0x0F

// Display pins
#define A                        (1 << 0)
#define B                        (1 << 1)
#define C                        (1 << 2)
#define D                        (1 << 3)
#define E                        (1 << 4)
#define F                        (1 << 5)
#define G1                         (1 << 6)
#define G2                        (1 << 7)
#define H                        (1 << 0)
#define J                        (1 << 1)
#define K                        (1 << 2)
#define L                        (1 << 3)
#define M                        (1 << 4)
#define N                        (1 << 5)
#define DP                         (1 << 6)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
uint8_t keyCode = NO_KEY_PRESSED;
uint8_t buttonPressed = false;
uint8_t motorDir = 0;

// Display
uint8_t displayAddr = DISPLAY_1_ADDR;
uint8_t row0_7 = 0;
uint8_t row8_15 = 0;
uint8_t ram[4] = {DISPLAY_COM0_1,DISPLAY_COM0_2,DISPLAY_COM1_1,DISPLAY_COM1_2};
uint8_t buffer[3];
char arr[] = "hi";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM17_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void Motor_Set_State(uint8_t state);
void Motor_Set_AIN1(uint16_t duty);
void Motor_Set_AIN2(uint16_t duty);
void Display_Set_Char(uint8_t *first, uint8_t *second, char character);
void Display_Init(uint8_t addr);
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
  MX_TIM3_Init();
  MX_TIM17_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim17, TIM_CHANNEL_1);
  
  // Display test
  Display_Init(DISPLAY_1_ADDR);
  for(uint8_t i = 0; i < sizeof(arr) - 1; i++) {
    Display_Set_Char(&row0_7, &row8_15, arr[i]);
    // Send first half
    buffer[0] = ram[i*2];
    buffer[1] = row0_7;
    HAL_I2C_Master_Transmit(&hi2c2, DISPLAY_1_ADDR, buffer, 2, HAL_MAX_DELAY);
    
    // Send second half
    buffer[0] = ram[(i*2)+1];
    buffer[1] = row8_15;
    HAL_I2C_Master_Transmit(&hi2c2, DISPLAY_1_ADDR, buffer, 2, HAL_MAX_DELAY);
  }
  
  // Turn on the display
  buffer[0] = DISPLAY_ON;
  HAL_I2C_Master_Transmit(&hi2c2, DISPLAY_1_ADDR, buffer, 1, HAL_MAX_DELAY);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    // 10mS Tasks 
    if (ten_mS_Flag) {
      ten_mS_Flag = false;

    } 

    // 25mS Tasks 
    if (twentyfive_mS_Flag) {
      twentyfive_mS_Flag = false;
      
      /* // DC push-pull coil test
      keyCode = ScanKeyboard();
      DebounceKeyCode(keyCode);
      
      // If we've gotten a valid debounced keyCode, process it
      if (processKeyCode == true) {
        ProcessKeyCode(keyCode);
      }
      
      
      if(buttonPressed) {
        HAL_GPIO_WritePin(COIL_PIN, GPIO_PIN_SET);
      }
      else {
        HAL_GPIO_WritePin(COIL_PIN, GPIO_PIN_RESET);
      }
      */
      
      /*
      // Motor test
      keyCode = ScanKeyboard();
      DebounceKeyCode(keyCode);
      
      // If we've gotten a valid debounced keyCode, process it
      if (processKeyCode == true) {
        ProcessKeyCode(keyCode);
      }
      
      motorDir = HAL_GPIO_ReadPin(MOTOR_DIR_PIN);
      
      if(buttonPressed) {
        if(motorDir) {Motor_Set_State(MOTOR_FWD);}
        else {Motor_Set_State(MOTOR_REV);}
      }
      else {
        Motor_Set_State(MOTOR_STOP);
      }
      */
    }

    // 100mS Tasks 
    if (hundred_mS_Flag) {
      hundred_mS_Flag = false;
      
    }

    // 1 Sec Tasks 
    if (one_S_Flag) {
      one_S_Flag = false;

    }
    
    /* // Goal sensor test
    if(HAL_GPIO_ReadPin(GOAL_1_PIN)) {
      HAL_GPIO_WritePin(LED_PIN, GPIO_PIN_SET);
    }
    else {
      HAL_GPIO_WritePin(LED_PIN, GPIO_PIN_RESET);
    }
    */
    
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
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
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x40003EFF;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Motor_Set_State(uint8_t state) {
  
  switch(state){
  case MOTOR_STOP:
    // Set AIN1 and AIN2 to full duty to brake the motor
    Motor_Set_AIN1(MOTOR_SPEED);
    Motor_Set_AIN2(MOTOR_SPEED);
    break;
  case MOTOR_FWD:
    Motor_Set_AIN1(MOTOR_SPEED);
    Motor_Set_AIN2(0);
    break;
  case MOTOR_REV:
    Motor_Set_AIN1(0);
    Motor_Set_AIN2(MOTOR_SPEED);
    break;
  }
}

void Motor_Set_AIN1(uint16_t duty) {
  TIM_OC_InitTypeDef sConfigOC = {0};
 
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = duty;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
 
  HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);

}

void Motor_Set_AIN2(uint16_t duty) {
  TIM_OC_InitTypeDef sConfigOC = {0};
 
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = duty;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
 
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

}

void Display_Set_Char(uint8_t *first, uint8_t *second, char character){
  
  switch(character){
  case '0':
    *first = A | B | C | D | E | F;
    *second = 0;
    break;
  case '1':
    *first = B | C;
    *second = 0;
    break;
  case '2':
    *first = A | B | D | E | G1 | G2;
    *second = 0;
    break;
  case '3':
    *first = A | B | C | D | G1 | G2;
    *second = 0;
    break;
  case '4':
    *first = B | C | F | G1 | G2;
    *second = 0;
    break;
  case '5':
    *first = A | C | D | F | G1 | G2;
    *second = 0;
    break;
  case '6':
    *first = A | C | D | E | F | G1 | G2;
    *second = 0;
    break;
  case '7':
    *first = A | B | C;
    *second = 0;
    break;
  case '8':
    *first = A | B | C | D | E | F | G1 | G2;
    *second = 0;
    break;
  case '9':
    *first = A | B | C | D | F | G1 | G2;
    *second = 0;
    break;
  case 'a':
  case 'A':
    *first = A | B | C | E | F | G1 | G2;
    *second = 0;
    break;
  case 'b':
  case 'B':
    *first = C | D | E | F | G1 | G2;
    *second = 0;
    break;
  case 'c':
  case 'C':
    *first = A | D | E | F;
    *second = 0;
    break;
  case 'd':
  case 'D':
    
    break;
  case 'e':
  case 'E':
    *first = A | D | E | F | G1 | G2;
    *second = 0;
    break;
  case 'f':
  case 'F':
    *first = A | E | F | G1 | G2;
    *second = 0;
    break;
  case 'g':
  case 'G':
    
    break;
  case 'h':
  case 'H':
    *first = B | C | E | F | G1 | G2;
    *second = 0;
    break;
  case 'i':
  case 'I':
    *first = A | D;
    *second = J | M;
    break;
  case 'j':
  case 'J':
    *first = B | C | D | E;
    *second = 0;
    break;
  case 'k':
  case 'K':
    
    break;
  case 'l':
  case 'L':
    *first = D | E | F;
    *second = 0;
    break;
  case 'm':
  case 'M':
    *first = B | C | E | F;
    *second = H | K;
    break;
  case 'n':
  case 'N':
    *first = B | C | E | F;
    *second = H | N;
    break;
  case 'o':
  case 'O':
    *first = A | B | C | D | E | F;
    *second = 0;
    break;
  case 'p':
  case 'P':
    *first = A | B | E | F | G1 | G2;
    *second = 0;
    break;
  case 'q':
  case 'Q':
    
    break;
  case 'r':
  case 'R':
    *first = A | B | E | F | G1 | G2;
    *second = N;
    break;
  case 's':
  case 'S':
    *first = A | C | D | F | G1 | G2;
    *second = 0;
    break;
  case 't':
  case 'T':
    *first = A;
    *second = J | M;
    break;
  case 'u':
  case 'U':
    *first = B | C | D | E | F;
    *second = 0;
    break;
  case 'v':
  case 'V':
    
    break;
  case 'w':
  case 'W':
    
    break;
  case 'x':
  case 'X':
    
    break;
  case 'y':
  case 'Y':
    
    break;
  case 'z':
  case 'Z':
    
    break;
  }
  
}

void Display_Init(uint8_t addr){
  // Start internal clock
  uint8_t buf[2] = {DISPLAY_OSC_ON,0};
  HAL_I2C_Master_Transmit(&hi2c2, addr, buf, 1, HAL_MAX_DELAY);
  
  // Turn off all LEDs on display 1
  buf[0] = DISPLAY_COM0_1;
  buf[1] = 0;
  HAL_I2C_Master_Transmit(&hi2c2, addr, buf, 2, HAL_MAX_DELAY);
  buf[0] = DISPLAY_COM0_2;
  buf[1] = 0;
  HAL_I2C_Master_Transmit(&hi2c2, addr, buf, 2, HAL_MAX_DELAY);
  
  // Turn off all LEDs on display 2
  buf[0] = DISPLAY_COM1_1;
  buf[1] = 0;
  HAL_I2C_Master_Transmit(&hi2c2, addr, buf, 2, HAL_MAX_DELAY);
  buf[0] = DISPLAY_COM1_2;
  buf[1] = 0;
  HAL_I2C_Master_Transmit(&hi2c2, addr, buf, 2, HAL_MAX_DELAY);
  
  // Turn off all LEDs on display 3
  buf[0] = DISPLAY_COM2_1;
  buf[1] = 0;
  HAL_I2C_Master_Transmit(&hi2c2, addr, buf, 2, HAL_MAX_DELAY);
  buf[0] = DISPLAY_COM2_2;
  buf[1] = 0;
  HAL_I2C_Master_Transmit(&hi2c2, addr, buf, 2, HAL_MAX_DELAY);
  
  // Turn off all LEDs on display 4
  buf[0] = DISPLAY_COM3_1;
  buf[1] = 0;
  HAL_I2C_Master_Transmit(&hi2c2, addr, buf, 2, HAL_MAX_DELAY);
  buf[0] = DISPLAY_COM3_2;
  buf[1] = 0;
  HAL_I2C_Master_Transmit(&hi2c2, addr, buf, 2, HAL_MAX_DELAY);
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
