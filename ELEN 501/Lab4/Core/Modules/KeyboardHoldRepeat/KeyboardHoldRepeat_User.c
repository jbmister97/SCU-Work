/***********************************************************************************************************************
* File Name    : KeyboardHoldRepeat.c
* Version      : 
* Device(s)    : Keyboard Handler code
* Tool-Chain   : IAR Systems ARM
* Description  : This file implements support code for a keyCode that is a result of a keyboard or button scan
*              : 
* Creation Date: 10OCT2021
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "main.h"
#include "KeyboardHoldRepeat.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "Serial.h"



/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/


/***********************************************************************************************************************
Global/module variables
***********************************************************************************************************************/
extern uint8_t processKeyCode;
extern uint8_t keyCodeProcessed;
extern uint8_t rampRed;
extern uint8_t rampGreen;
extern uint8_t rampBlue;
extern uint8_t led_state;
extern uint8_t greenPause_flag;
extern uint8_t bluePause_flag;
extern uint8_t customValue_flag;

extern uint16_t ledValue;

extern UART_HandleTypeDef huart1;

volatile uint8_t kbdTest = 0;
char testChar[1] = {'!'};


/***********************************************************************************************************************
module function prototypes
***********************************************************************************************************************/



/***********************************************************************************************************************
code start
***********************************************************************************************************************/
void ProcessKeyCode(uint8_t _kcode)
{
  //kbdTest = 27;
  
  switch (_kcode) {
  case 0x06:  // On-board user button pressed
    led_state++;
    if(led_state > 3) {led_state = 0;}
    customValue_flag = false;
    switch(led_state) {
    case 0:
      SendString("All LEDs cycling", 16, StripZeros, AddCRLF);
      break;
    case 1:
      SendString("All LEDs stopped", 16, StripZeros, AddCRLF);
      break;
    case 2:
      SendString("50% duty cycle", 14, StripZeros, AddCRLF);
      break;
    case 3:
      SendString("All LEDs off", 12, StripZeros, AddCRLF);
      break;
    default:
      break;
    }
    break;
  case 0x05:  // External button on D8 pressed
    if(bluePause_flag) {
      bluePause_flag = false;
      SendString("Resuming blue LED cycling", 25, StripZeros, AddCRLF);
    }
    else {
      bluePause_flag = true;
      SendString("Pausing blue LED cycling", 24, StripZeros, AddCRLF);
    }
    customValue_flag = false;
    
    break;
  case 0x03:  // External button on D7 pressed
    if(greenPause_flag) {
      greenPause_flag = false;
      SendString("Resuming green LED cycling", 26, StripZeros, AddCRLF);
    }
    else {
      greenPause_flag = true;
      SendString("Pausing green LED cycling", 25, StripZeros, AddCRLF);
    }
    customValue_flag = false;
    
    break;
  case 0x02:  // 2-key chords:  1+3
    kbdTest = 4;
    break;
  default:
    break;
  }
  
  processKeyCode = false;
  keyCodeProcessed = true;
  
}



uint8_t ValidKeyCode(uint8_t _kcode)
{
  uint8_t validKeyCode = false;
  
  switch (_kcode) {
  case 0x06:  // single keys:   1
  case 0x05:  //                2
  case 0x03:  //                3
  case 0x02:  // 2-key chords:  1+3
    validKeyCode = true;
    break;
  default:
    break;
  }
  return validKeyCode;
}


uint8_t ScanKeyboard(void)
{
  uint8_t keyCode = NO_KEY_PRESSED;
  
  keyCode = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) << 2) | (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) << 1) | (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) << 0);
  
  return keyCode;
}
