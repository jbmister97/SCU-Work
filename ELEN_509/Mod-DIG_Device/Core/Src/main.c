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
#include "scheduler.h"
#include "serial_user.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */
// scheduler stuff
extern uint8_t ten_mS_Flag;
extern uint8_t twentyfive_mS_Flag;
extern uint8_t hundred_mS_Flag;
extern uint8_t one_S_Flag;

// Keyboard
uint8_t button = true;
uint8_t lastButton = true;

// Seven Segment Stuff
volatile uint8_t segmentNumber = 0;
volatile uint8_t digitToggle = 0;

uint8_t digit1Value = 0;
uint8_t digit2Value = 0;
uint8_t digit1Segments = 0;
uint8_t digit2Segments = 0;
const uint8_t Num_2_Seg[16] = {0xFC, 0x60, 0xDA, 0xF2, 0x66, 0xB6, 0xBE, 0xE0, 0xFE, 0xF6, 0xEE, 0x3E, 0x9C, 0x7A, 0x9E, 0x8E};
uint8_t dpOn = 0;

// variables
uint8_t counter = 0;

// UART Test
uint8_t charCounter = 0;
uint8_t char2Send[2] = {0, 0};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void ParseToDigits(uint8_t _value, uint8_t _dpStatus);
void UpdateDisplay(void);
void InitSevenSegDisplay(void);

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
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */

  LL_LPUART_EnableIT_RXNE(LPUART1);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //---------------------------------
    // 10mS Tasks 
    if (ten_mS_Flag) {
      ten_mS_Flag = false;

#if(!seg_test)
      UpdateDisplay();
#endif
    }  // end of 10mS Tasks
    //---------------------------------

    
    //---------------------------------
    // 25mS Tasks 
    if (twentyfive_mS_Flag) {
      twentyfive_mS_Flag = false;

    }  // end of 25mS Tasks
    //---------------------------------

    
    //---------------------------------
    // 100mS Tasks 
    if (hundred_mS_Flag) {
      hundred_mS_Flag = false;
      
      button = HAL_GPIO_ReadPin(SW_IN);
      if ((button == 0) && (lastButton == 1)) {
        // do some stuff...
        dpOn++;
        if (dpOn >= 4)
          dpOn = 0;
      }        
        
      lastButton = button;
        

    }  // end of 100mS Tasks
    //---------------------------------

    
    //---------------------------------
    // 1 Sec Tasks 
    if (one_S_Flag) {
      one_S_Flag = false;
      
//      HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_10);
      HAL_GPIO_TogglePin(LED_OUT);
      
#if (seg_test)
      digitToggle ^= 1;
      
      if (digitToggle == 1) {
        HAL_GPIO_WritePin(Digit_1, Digit_ON);
        HAL_GPIO_WritePin(Digit_2, Digit_OFF);
      }
      else {
        HAL_GPIO_WritePin(Digit_1, Digit_OFF);
        HAL_GPIO_WritePin(Digit_2, Digit_ON);
      }
      
      if (digitToggle == 1) {
        switch (segmentNumber) {
        case 0:
          HAL_GPIO_WritePin(Seg_A, Seg_ON);
          HAL_GPIO_WritePin(Seg_DP, Seg_OFF);
          break;
        case 1:
          HAL_GPIO_WritePin(Seg_B, Seg_ON);
          HAL_GPIO_WritePin(Seg_A, Seg_OFF);
          break;
        case 2:
          HAL_GPIO_WritePin(Seg_C, Seg_ON);
          HAL_GPIO_WritePin(Seg_B, Seg_OFF);
          break;
        case 3:
          HAL_GPIO_WritePin(Seg_D, Seg_ON);
          HAL_GPIO_WritePin(Seg_C, Seg_OFF);
          break;
        case 4:
          HAL_GPIO_WritePin(Seg_E, Seg_ON);
          HAL_GPIO_WritePin(Seg_D, Seg_OFF);
          break;
        case 5:
          HAL_GPIO_WritePin(Seg_F, Seg_ON);
          HAL_GPIO_WritePin(Seg_E, Seg_OFF);
          break;
        case 6:
          HAL_GPIO_WritePin(Seg_G, Seg_ON);
          HAL_GPIO_WritePin(Seg_F, Seg_OFF);
          break;
        case 7:
          HAL_GPIO_WritePin(Seg_DP, Seg_ON);
          HAL_GPIO_WritePin(Seg_G, Seg_OFF);
          break;
        }
        
        segmentNumber++;
        if (segmentNumber >= 8) segmentNumber = 0;
      }
#else 
      counter++;
            
      ParseToDigits(counter, dpOn);
#endif

//      char2Send[0] = charCounter++ + 0x40;
//      HAL_UART_Transmit(&hlpuart1, char2Send, 1, 1000);
//
//      if (charCounter > 0x3A) {
//        charCounter = 0;
//        char2Send[0] = '\n';
//        HAL_UART_Transmit(&hlpuart1, char2Send, 1, 1000);
//      }
            
    } // end of 1Sec Tasks
    //---------------------------------

    
    //---------------------------------
    // Every time through the loop
    
    // end Every time through the loop
    //---------------------------------
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 9600;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&hlpuart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, Segment_A_Pin|Segment_C_Pin|Segment_E_Pin|Segment_F_Pin
                          |Segment_G_Pin|Segment_DP_Pin|LED_Pin|Segment_B_Pin
                          |Segment_D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Digit_2_Pin|Digit_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Segment_A_Pin Segment_C_Pin Segment_E_Pin Segment_F_Pin
                           Segment_G_Pin Segment_DP_Pin LED_Pin Segment_B_Pin
                           Segment_D_Pin */
  GPIO_InitStruct.Pin = Segment_A_Pin|Segment_C_Pin|Segment_E_Pin|Segment_F_Pin
                          |Segment_G_Pin|Segment_DP_Pin|LED_Pin|Segment_B_Pin
                          |Segment_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Digit_2_Pin Digit_1_Pin */
  GPIO_InitStruct.Pin = Digit_2_Pin|Digit_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Switch_Pin */
  GPIO_InitStruct.Pin = Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Switch_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void InitSevenSegDisplay(void)
{
  // reset digit drives...
  HAL_GPIO_WritePin(Digit_1, Digit_OFF);
  HAL_GPIO_WritePin(Digit_2, Digit_OFF);

  // ...and segment drives
  HAL_GPIO_WritePin(Seg_A, Seg_OFF);
  HAL_GPIO_WritePin(Seg_B, Seg_OFF);
  HAL_GPIO_WritePin(Seg_C, Seg_OFF);
  HAL_GPIO_WritePin(Seg_D, Seg_OFF);
  HAL_GPIO_WritePin(Seg_E, Seg_OFF);
  HAL_GPIO_WritePin(Seg_F, Seg_OFF);
  HAL_GPIO_WritePin(Seg_G, Seg_OFF);
  HAL_GPIO_WritePin(Seg_DP, Seg_OFF);
}


void ParseToDigits(uint8_t _value, uint8_t _dpStatus)
{
  digit1Value = _value % 16;
  digit2Value = _value / 16;
  
  digit1Segments = Num_2_Seg[digit1Value];
  digit2Segments = Num_2_Seg[digit2Value];
  
  switch (_dpStatus) {
  case 0: // no DP
    break;
  case 1: // dp first digit
    digit1Segments |= 0x01;
    break;
  case 2: // dp second digit
    digit2Segments |= 0x01;
    break;
  case 3: // dp both digits
    digit1Segments |= 0x01;
    digit2Segments |= 0x01;
    break;
  default:
    break;
  }
}


void UpdateDisplay(void)
{
  uint8_t mask = 0x80;
  static uint8_t activeDigit = 0;
  
  activeDigit ^= 1;
  
  if (activeDigit == 1) {
    HAL_GPIO_WritePin(Digit_2, Digit_OFF);
                      
    HAL_GPIO_WritePin(Seg_A, (digit1Segments & mask) ? Seg_ON : Seg_OFF);
    mask >>= 1;
    HAL_GPIO_WritePin(Seg_B, (digit1Segments & mask) ? Seg_ON : Seg_OFF);
    mask >>= 1;
    HAL_GPIO_WritePin(Seg_C, (digit1Segments & mask) ? Seg_ON : Seg_OFF);
    mask >>= 1;
    HAL_GPIO_WritePin(Seg_D, (digit1Segments & mask) ? Seg_ON : Seg_OFF);
    mask >>= 1;
    HAL_GPIO_WritePin(Seg_E, (digit1Segments & mask) ? Seg_ON : Seg_OFF);
    mask >>= 1;
    HAL_GPIO_WritePin(Seg_F, (digit1Segments & mask) ? Seg_ON : Seg_OFF);
    mask >>= 1;
    HAL_GPIO_WritePin(Seg_G, (digit1Segments & mask) ? Seg_ON : Seg_OFF);
    mask >>= 1;
    HAL_GPIO_WritePin(Seg_DP, (digit1Segments & mask) ? Seg_ON : Seg_OFF);
    
    HAL_GPIO_WritePin(Digit_1, Digit_ON);
  }
  else {
    HAL_GPIO_WritePin(Digit_1, Digit_OFF);

    HAL_GPIO_WritePin(Seg_A, (digit2Segments & mask) ? Seg_ON : Seg_OFF);
    mask >>= 1;
    HAL_GPIO_WritePin(Seg_B, (digit2Segments & mask) ? Seg_ON : Seg_OFF);
    mask >>= 1;
    HAL_GPIO_WritePin(Seg_C, (digit2Segments & mask) ? Seg_ON : Seg_OFF);
    mask >>= 1;
    HAL_GPIO_WritePin(Seg_D, (digit2Segments & mask) ? Seg_ON : Seg_OFF);
    mask >>= 1;
    HAL_GPIO_WritePin(Seg_E, (digit2Segments & mask) ? Seg_ON : Seg_OFF);
    mask >>= 1;
    HAL_GPIO_WritePin(Seg_F, (digit2Segments & mask) ? Seg_ON : Seg_OFF);
    mask >>= 1;
    HAL_GPIO_WritePin(Seg_G, (digit2Segments & mask) ? Seg_ON : Seg_OFF);
    mask >>= 1;
    HAL_GPIO_WritePin(Seg_DP, (digit2Segments & mask) ? Seg_ON : Seg_OFF);
    
    HAL_GPIO_WritePin(Digit_2, Digit_ON);
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
