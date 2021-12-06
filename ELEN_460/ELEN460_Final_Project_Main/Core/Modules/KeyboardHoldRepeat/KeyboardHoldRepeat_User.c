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

#define MOTOR_LIMIT1_PRESSED    0x0F
#define MOTOR_LIMIT2_PRESSED    0x1D
#define JOYSTICK_1_PRESSED      0x17
#define JOYSTICK_2_PRESSED      0x1B
#define BUTTON_PRESSED          0x1E

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/


/***********************************************************************************************************************
Global/module variables
***********************************************************************************************************************/
extern uint8_t processKeyCode;
extern uint8_t keyCodeProcessed;
extern uint8_t buttonPressed;
extern uint8_t fireBusy;
extern uint8_t fireRequest;
extern uint8_t count;
extern uint8_t motorRightRequest;
extern uint8_t motorLeftRequest;
extern uint8_t limitRightSwitch;
extern uint8_t limitLeftSwitch;

//extern UART_HandleTypeDef huart1;

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
  count++;
  //buttonPressed = true;
  
  
  switch (_kcode) {
  case MOTOR_LIMIT1_PRESSED:  // single keys:   1
    limitRightSwitch = true;
    break;
  case MOTOR_LIMIT2_PRESSED:  // single keys:   1
    limitLeftSwitch = true;
    break;
  case JOYSTICK_1_PRESSED:  //                2
    motorRightRequest = true;
    break;
  case JOYSTICK_2_PRESSED:  //                3
    motorLeftRequest = true;
    break;
  case BUTTON_PRESSED:  // 2-key chords:  1+3
    if(!fireBusy) {fireRequest = true;}
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
  
  if(_kcode == 0) {
    validKeyCode = true;
  }

  switch (_kcode) {
  case MOTOR_LIMIT1_PRESSED:  // single keys:   1
  case MOTOR_LIMIT2_PRESSED:  // single keys:   1
  case JOYSTICK_1_PRESSED:  //                2
  case JOYSTICK_2_PRESSED:  //                3
  case BUTTON_PRESSED:  // 2-key chords:  1+3
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
  
  keyCode = (HAL_GPIO_ReadPin(MOTOR_LIMIT1_PIN) << 4) | (HAL_GPIO_ReadPin(MOTOR_LIMIT2_PIN) << 3) | (HAL_GPIO_ReadPin(JOYSTICK_1_PIN) << 2) | (HAL_GPIO_ReadPin(JOYSTICK_2_PIN) <<  1) | (HAL_GPIO_ReadPin(BUTTON_PIN) << 0);
  
  return keyCode;
}
