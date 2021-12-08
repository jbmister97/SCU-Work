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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GOAL_1_PIN              GPIOA, GPIO_PIN_11
#define GOAL_2_PIN              GPIOB, GPIO_PIN_4
#define GOAL_3_PIN              GPIOA, GPIO_PIN_12
#define GOAL_4_PIN              GPIOB, GPIO_PIN_7
#define GOAL_5_PIN              GPIOB, GPIO_PIN_1
#define GOAL_6_PIN              GPIOB, GPIO_PIN_5

#define LED_1_PIN               GPIOA, GPIO_PIN_3
#define LED_2_PIN               GPIOA, GPIO_PIN_0
#define LED_3_PIN               GPIOA, GPIO_PIN_2
#define LED_4_PIN               GPIOA, GPIO_PIN_1
#define LED_5_PIN               GPIOA, GPIO_PIN_7
#define LED_6_PIN               GPIOA, GPIO_PIN_4

#define RESET_PIN               GPIOA, GPIO_PIN_10
#define WIN_PIN                 GPIOB, GPIO_PIN_0
#define ALL_BALLS_PIN           GPIOA, GPIO_PIN_9

#define NUMBER_OF_BALLS         10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t goals = 0;
uint8_t leds = 0;
uint8_t reset = 0;
uint8_t ballCount = 0;
uint8_t lastValue = 0;
uint8_t count = 0;
uint8_t winFlag = false;
uint8_t allFiredFlag = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void LED_Process(uint8_t mask);
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    reset = HAL_GPIO_ReadPin(RESET_PIN);
    if(reset) {
      leds = 0;
      goals = 0;
      lastValue = 0;
      ballCount = 0;
      winFlag = false;
      allFiredFlag = false;
      HAL_GPIO_WritePin(WIN_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(ALL_BALLS_PIN, GPIO_PIN_RESET);
    }
    else {
      goals = (HAL_GPIO_ReadPin(GOAL_6_PIN) << 5) | (HAL_GPIO_ReadPin(GOAL_5_PIN) << 4) | (HAL_GPIO_ReadPin(GOAL_4_PIN) << 3) | (HAL_GPIO_ReadPin(GOAL_3_PIN) << 2) | (HAL_GPIO_ReadPin(GOAL_2_PIN) << 1) | (HAL_GPIO_ReadPin(GOAL_1_PIN) << 0);
    }
    
    leds |= goals;
    LED_Process(leds);

    // Check if player won
    if(leds == 0x3F) {
      winFlag = true;
      HAL_GPIO_WritePin(WIN_PIN, GPIO_PIN_SET);
    }
    
        // Increment ball counter when ball passes through goal
    if(goals) {
      if(goals != lastValue) {
        ballCount++;
        lastValue = goals;
      }
    }
    else {lastValue = 0;}
    if(ballCount >= NUMBER_OF_BALLS) {
      allFiredFlag = true;
      HAL_GPIO_WritePin(ALL_BALLS_PIN, GPIO_PIN_SET);
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA7 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB4 PB5 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void LED_Process(uint8_t mask) {
  
  // Goal LED 1
  if((mask >> 0) & 0x1) {HAL_GPIO_WritePin(LED_1_PIN, GPIO_PIN_SET);}
  else {HAL_GPIO_WritePin(LED_1_PIN, GPIO_PIN_RESET);}
  
  // Goal LED 2
  if((mask >> 1) & 0x1) {HAL_GPIO_WritePin(LED_2_PIN, GPIO_PIN_SET);}
  else {HAL_GPIO_WritePin(LED_2_PIN, GPIO_PIN_RESET);}
  
  // Goal LED 3
  if((mask >> 2) & 0x1) {HAL_GPIO_WritePin(LED_3_PIN, GPIO_PIN_SET);}
  else {HAL_GPIO_WritePin(LED_3_PIN, GPIO_PIN_RESET);}
  
  // Goal LED 4
  if((mask >> 3) & 0x1) {HAL_GPIO_WritePin(LED_4_PIN, GPIO_PIN_SET);}
  else {HAL_GPIO_WritePin(LED_4_PIN, GPIO_PIN_RESET);}
  
  // Goal LED 5
  if((mask >> 4) & 0x1) {HAL_GPIO_WritePin(LED_5_PIN, GPIO_PIN_SET);}
  else {HAL_GPIO_WritePin(LED_5_PIN, GPIO_PIN_RESET);}
  
  // Goal LED 6
  if((mask >> 5) & 0x1) {HAL_GPIO_WritePin(LED_6_PIN, GPIO_PIN_SET);}
  else {HAL_GPIO_WritePin(LED_6_PIN, GPIO_PIN_RESET);}
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
