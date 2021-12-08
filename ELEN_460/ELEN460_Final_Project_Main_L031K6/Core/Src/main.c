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
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COIL_PIN                GPIOB, GPIO_PIN_1
#define COIN_SENSOR_PIN         GPIOB, GPIO_PIN_7
#define MOTOR_AIN1_PIN          GPIOB, GPIO_PIN_5
#define MOTOR_AIN2_PIN          GPIOB, GPIO_PIN_4
#define MOTOR_DIR_PIN           GPIOA, GPIO_PIN_10
#define ALL_BALLS_FIRED_PIN     GPIOA, GPIO_PIN_5
#define WIN_PIN                 GPIOA, GPIO_PIN_6
#define RESET_CTRL_PIN          GPIOA, GPIO_PIN_7

// Game Logic
#define IDLE                    2
#define RUNNING                 3
#define END                     4
#define NUM_OF_SHOTS            10
#define MESSAGE_TIME            5

// Motor
#define MOTOR_STOP              0
#define MOTOR_RIGHT             1
#define MOTOR_LEFT              2
#define MOTOR_SPEED             20000

// I2C Device Addresses
//#define BOARD_ADDR              0x60
#define DISPLAY_1_ADDR          (0x70 << 1)
#define DISPLAY_2_ADDR          (0x71 << 1)
#define DISPLAY_3_ADDR          (0x72 << 1)
#define DISPLAY_4_ADDR          (0x73 << 1)

// Display system setup
//#define DISPLAY_OSC_OFF                 0x20
#define DISPLAY_OSC_ON                  0x21
#define DISPLAY_OFF                     0x80
#define DISPLAY_ON                      0x81
#define DISPLAY_COM0_1                  0x00
#define DISPLAY_COM0_2                  0x01
#define DISPLAY_COM1_1                  0x02
#define DISPLAY_COM1_2                  0x03
#define DISPLAY_COM2_1                  0x04
#define DISPLAY_COM2_2                  0x05
#define DISPLAY_COM3_1                  0x06
#define DISPLAY_COM3_2                  0x07

// Display pins
#define A                        (1 << 0)
#define B                        (1 << 1)
#define C                        (1 << 2)
#define D                        (1 << 3)
#define E                        (1 << 4)
#define F                        (1 << 5)
#define G1                       (1 << 6)
#define G2                       (1 << 7)
#define H                        (1 << 0)
#define J                        (1 << 1)
#define K                        (1 << 2)
#define L                        (1 << 3)
#define M                        (1 << 4)
#define N                        (1 << 5)
#define DP                       (1 << 6)

#define NUMBER_OF_CHAR          16
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim21;
TIM_HandleTypeDef htim22;

/* USER CODE BEGIN PV */
uint8_t keyCode = NO_KEY_PRESSED;
uint8_t buttonPressed = false;

uint8_t state = RUNNING;
uint8_t shots = NUM_OF_SHOTS;
uint8_t timeoutFlag = false;

// Motor
uint8_t motorState = MOTOR_STOP;
uint8_t motorRightRequest = false;
uint8_t motorLeftRequest = false;
uint8_t limitRightSwitch = false;
uint8_t limitLeftSwitch = false; 

// Display
uint8_t displayAddr = DISPLAY_1_ADDR;
uint8_t row0_7 = 0;
uint8_t row8_15 = 0;
uint8_t ram[8] = {DISPLAY_COM0_1,DISPLAY_COM0_2,DISPLAY_COM1_1,DISPLAY_COM1_2,DISPLAY_COM2_1,DISPLAY_COM2_2,DISPLAY_COM3_1,DISPLAY_COM3_2};
uint8_t buffer[3];
char arr[16];
char displayString[NUMBER_OF_CHAR];
uint8_t fireRequest = false;
uint8_t fireBusy = false;
uint8_t count = 0;
uint8_t msgTimerCount = 0;
uint8_t msgTimerEn = false;
uint8_t msgTimerFinished = false;
char strShots[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM21_Init(void);
static void MX_TIM22_Init(void);
/* USER CODE BEGIN PFP */
void Motor_Set_State(uint8_t state);
void Motor_Set_AIN1(uint16_t duty);
void Motor_Set_AIN2(uint16_t duty);
void Display_Set_Char(uint8_t *first, uint8_t *second, char character);
void Display_Set(char *arr, uint8_t displayAddr);
void Display_Update_All(char *arr);
void Display_Init(uint8_t addr);
void Display_Shot_Convert(uint8_t shots, char *str);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t ten_mS_Flag;
extern uint8_t twentyfive_mS_Flag;
extern uint8_t hundred_mS_Flag;
extern uint8_t twohundred_fifty_mS_Flag;
extern uint8_t five_hundred_mS_Flag;
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
  MX_I2C1_Init();
  MX_TIM21_Init();
  MX_TIM22_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start_IT(&htim21, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start_IT(&htim22, TIM_CHANNEL_1);
  
    // Initialize displays
  Display_Init(DISPLAY_1_ADDR);
  Display_Init(DISPLAY_2_ADDR);
  Display_Init(DISPLAY_3_ADDR);
  Display_Init(DISPLAY_4_ADDR);
  
  
  strcpy(displayString, "Insert Coin");
  
  Display_Update_All(displayString);
  
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
      
    }

    // 100mS Tasks 
    if (hundred_mS_Flag) {
      hundred_mS_Flag = false;
    }
    
    // 250ms Tasks
    if(twohundred_fifty_mS_Flag) {
      twohundred_fifty_mS_Flag = false;
      
      if(fireBusy) {
        fireBusy = false;
        HAL_GPIO_WritePin(COIL_PIN, GPIO_PIN_RESET);
        shots--;
        
        // Update shot counter
        //itoa(shots,strShots,10);
        Display_Shot_Convert(shots,strShots);
        strcpy(displayString, "Shots left ");
        strcat(displayString,strShots);
        Display_Update_All(displayString);
      }
      
      if(fireRequest && !fireBusy && (shots != 0)) {
      //if(fireRequest) {
        fireRequest = false;
        fireBusy = true;
        HAL_GPIO_WritePin(COIL_PIN, GPIO_PIN_SET);
      }
    }
    
        // 500mS Tasks 
    if (five_hundred_mS_Flag) {
      five_hundred_mS_Flag = false;
      
    } 

    // 1 Sec Tasks 
    if (one_S_Flag) {
      one_S_Flag = false;
      if(msgTimerEn) {
        msgTimerCount++;
        if(msgTimerCount == MESSAGE_TIME) {
          msgTimerEn = false;
          msgTimerFinished = true;
        }
      }
      
    }
    
    switch(state) {
    case IDLE:
      if(HAL_GPIO_ReadPin(COIN_SENSOR_PIN)) {
        state = RUNNING;
        
        // Prepare secondary microcontroller
        HAL_GPIO_WritePin(RESET_CTRL_PIN, GPIO_PIN_RESET);
        
        Display_Shot_Convert(NUM_OF_SHOTS, strShots);
        strcpy(displayString, "Shots left ");
        strcat(displayString, strShots);
        Display_Update_All(displayString);
      }
      else {
        
      }
      break;
    case RUNNING:
      
      keyCode = ScanKeyboard();
      DebounceKeyCode(keyCode);
      
      // If we've gotten a valid debounced keyCode, process it
      if (processKeyCode == true) {
        ProcessKeyCode(keyCode);
      }
      
      if(motorRightRequest && !limitRightSwitch) {
        Motor_Set_State(MOTOR_RIGHT);
      }
      else if(motorLeftRequest && !limitLeftSwitch) {
        Motor_Set_State(MOTOR_LEFT);
      }
      else {Motor_Set_State(MOTOR_STOP);}
      
      if(shots == 0) {
        if(HAL_GPIO_ReadPin(ALL_BALLS_FIRED_PIN)){
          // Check if player won
          if(HAL_GPIO_ReadPin(WIN_PIN)) {
            // Player won
            strcpy(displayString, "Awesome you won");
            Display_Update_All(displayString);
          }
          else {
            // Player lost
            strcpy(displayString, "Nice try");
            Display_Update_All(displayString);
          }
          state = END;
        }
      }
      break;
    case END:
      shots = NUM_OF_SHOTS;
      motorLeftRequest = false;
      motorRightRequest = false;
      limitLeftSwitch = false;
      limitRightSwitch = false;
      fireRequest = false;
      fireBusy = false;
      
      // Reset secondary microcontroller
      HAL_GPIO_WritePin(RESET_CTRL_PIN, GPIO_PIN_SET);
      
      strcpy(displayString, "Insert Coin");
      Display_Update_All(displayString);
      state = IDLE;
      break;
      
    }
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
  * @brief TIM21 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 0;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 65535;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim21, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM21_Init 2 */

  /* USER CODE END TIM21_Init 2 */
  HAL_TIM_MspPostInit(&htim21);

}

/**
  * @brief TIM22 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM22_Init(void)
{

  /* USER CODE BEGIN TIM22_Init 0 */

  /* USER CODE END TIM22_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM22_Init 1 */

  /* USER CODE END TIM22_Init 1 */
  htim22.Instance = TIM22;
  htim22.Init.Prescaler = 0;
  htim22.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim22.Init.Period = 65535;
  htim22.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim22.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim22) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim22, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim22, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM22_Init 2 */

  /* USER CODE END TIM22_Init 2 */
  HAL_TIM_MspPostInit(&htim22);

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
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
    Motor_Set_AIN1(0);
    Motor_Set_AIN2(0);
    break;
  case MOTOR_RIGHT:
    Motor_Set_AIN1(MOTOR_SPEED);
    Motor_Set_AIN2(0);
    break;
  case MOTOR_LEFT:
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
  //sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
 //sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  //sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
 
  HAL_TIM_PWM_Stop(&htim21, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim21, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim21, TIM_CHANNEL_2);

}

void Motor_Set_AIN2(uint16_t duty) {
  TIM_OC_InitTypeDef sConfigOC = {0};
 
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = duty;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  //sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  //sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  //sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
 
  HAL_TIM_PWM_Stop(&htim22, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim22, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim22, TIM_CHANNEL_1);

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
    *first = A | B | C | D;
    *second = J | M;
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
    *first = E | F | G1;
    *second = K | N;
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
    
  default:
    *first = 0;
    *second = 0;
    break;
  }
  
}

void Display_Init(uint8_t addr){
  // Start internal clock
  uint8_t buf[2] = {DISPLAY_OSC_ON,0};
  HAL_I2C_Master_Transmit(&hi2c1, addr, buf, 1, HAL_MAX_DELAY);
  
  // Turn off all LEDs on display 1
  buf[0] = DISPLAY_COM0_1;
  buf[1] = 0;
  HAL_I2C_Master_Transmit(&hi2c1, addr, buf, 2, HAL_MAX_DELAY);
  buf[0] = DISPLAY_COM0_2;
  buf[1] = 0;
  HAL_I2C_Master_Transmit(&hi2c1, addr, buf, 2, HAL_MAX_DELAY);
  
  // Turn off all LEDs on display 2
  buf[0] = DISPLAY_COM1_1;
  buf[1] = 0;
  HAL_I2C_Master_Transmit(&hi2c1, addr, buf, 2, HAL_MAX_DELAY);
  buf[0] = DISPLAY_COM1_2;
  buf[1] = 0;
  HAL_I2C_Master_Transmit(&hi2c1, addr, buf, 2, HAL_MAX_DELAY);
  
  // Turn off all LEDs on display 3
  buf[0] = DISPLAY_COM2_1;
  buf[1] = 0;
  HAL_I2C_Master_Transmit(&hi2c1, addr, buf, 2, HAL_MAX_DELAY);
  buf[0] = DISPLAY_COM2_2;
  buf[1] = 0;
  HAL_I2C_Master_Transmit(&hi2c1, addr, buf, 2, HAL_MAX_DELAY);
  
  // Turn off all LEDs on display 4
  buf[0] = DISPLAY_COM3_1;
  buf[1] = 0;
  HAL_I2C_Master_Transmit(&hi2c1, addr, buf, 2, HAL_MAX_DELAY);
  buf[0] = DISPLAY_COM3_2;
  buf[1] = 0;
  HAL_I2C_Master_Transmit(&hi2c1, addr, buf, 2, HAL_MAX_DELAY);
}

void Display_Set(char *arr, uint8_t displayAddr){
  for(uint8_t i = 0; i < 4; i++){
    Display_Set_Char(&row0_7, &row8_15, *(arr + i));
    // Send first half
    buffer[0] = ram[i*2];
    buffer[1] = row0_7;
    HAL_I2C_Master_Transmit(&hi2c1, displayAddr, buffer, 2, HAL_MAX_DELAY);
    
    // Send second half
    buffer[0] = ram[(i*2)+1];
    buffer[1] = row8_15;
    HAL_I2C_Master_Transmit(&hi2c1, displayAddr, buffer, 2, HAL_MAX_DELAY);
  }
  
  // Turn on the display
  buffer[0] = DISPLAY_ON;
  HAL_I2C_Master_Transmit(&hi2c1, displayAddr, buffer, 1, HAL_MAX_DELAY);

}

void Display_Update_All(char *arr) {
  Display_Set(arr, DISPLAY_1_ADDR);
  Display_Set(&arr[4], DISPLAY_2_ADDR);
  Display_Set(&arr[8], DISPLAY_3_ADDR);
  Display_Set(&arr[12], DISPLAY_4_ADDR);
}

void Display_Shot_Convert(uint8_t shots, char *str){
  
  switch(shots){
  case 0:
    *str = '0';
    *(str+1) = '0';
    break;
  case 1:
    *str = '0';
    *(str+1) = '1';
    break;
  case 2:
    *str = '0';
    *(str+1) = '2';
    break;
  case 3:
    *str = '0';
    *(str+1) = '3';
    break;
  case 4:
    *str = '0';
    *(str+1) = '4';
    break;
  case 5:
    *str = '0';
    *(str+1) = '5';
    break;
  case 6:
    *str = '0';
    *(str+1) = '6';
    break;
  case 7:
    *str = '0';
    *(str+1) = '7';
    break;
  case 8:
    *str = '0';
    *(str+1) = '8';
    break;
  case 9:
    *str = '0';
    *(str+1) = '9';
    break;
  case 10:
    *str = '1';
    *(str+1) = '0';
    break;
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
