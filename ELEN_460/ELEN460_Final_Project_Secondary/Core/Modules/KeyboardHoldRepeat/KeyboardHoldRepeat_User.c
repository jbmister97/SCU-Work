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

#define JOYSTICK_1_PRESSED      0xE
#define JOYSTICK_2_PRESSED      0xD
#define MOTOR_RIGHT_LIMIT_PRESSED     0xB
#define MOTOR_LEFT_LIMIT_PRESSED        0x7
#define JOYSTICK_1_LIMIT_PRESSED        0x9
#define JOYSTICK_1_LIMIT_OPP_PRESSED    0x5
#define JOYSTICK_2_LIMIT_PRESSED        0x6
#define JOYSTICK_2_LIMIT_OPP_PRESSED    0xA

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
//extern uint8_t limitRightSwitch;
//extern uint8_t limitLeftSwitch;

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
  case JOYSTICK_1_PRESSED:  //                2
    motorRightRequest = true;
    motorLeftRequest = false;
    //limitRightSwitch = false;
    //limitLeftSwitch = false;
    break;
  case JOYSTICK_2_PRESSED:  //                3
    motorLeftRequest = true;
    motorRightRequest = false;
    //limitRightSwitch = false;
    //limitLeftSwitch = false;
    break;
  case MOTOR_RIGHT_LIMIT_PRESSED:
  case JOYSTICK_1_LIMIT_PRESSED:
    //limitRightSwitch = true;
    motorRightRequest = false;
    motorLeftRequest = false;
    break;
  case MOTOR_LEFT_LIMIT_PRESSED:
  case JOYSTICK_2_LIMIT_PRESSED:
    //limitLeftSwitch = true;
    motorLeftRequest = false;
    motorRightRequest = false;
    break;
  case JOYSTICK_1_LIMIT_OPP_PRESSED:
    motorLeftRequest = true;
    motorRightRequest = false;
    break;
  case JOYSTICK_2_LIMIT_OPP_PRESSED:
    motorRightRequest = true;
    motorLeftRequest = false;
    break;
  default:
    motorRightRequest = false;
    motorLeftRequest = false;
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
  case JOYSTICK_1_PRESSED:  //                2
  case JOYSTICK_2_PRESSED:  //                3
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
  
  keyCode = (HAL_GPIO_ReadPin(MOTOR_LEFT_LIMIT_PIN) << 3) | (HAL_GPIO_ReadPin(MOTOR_RIGHT_LIMIT_PIN) << 2) | (HAL_GPIO_ReadPin(JOYSTICK_1_PIN) << 1) | (HAL_GPIO_ReadPin(JOYSTICK_2_PIN) <<  0);
  
  return keyCode;
}
