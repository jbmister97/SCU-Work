// serial_user.c

#include "serial.h"
#include "serial_user.h"
#include "ASCII_numbers.h"


uint8_t packetBuffer[16];
uint8_t inPacket = false;
uint8_t nextPacketChar = 0;
uint8_t processPacket = false;

extern uint8_t flashLED;
extern uint8_t buttonPushed;
extern uint16_t flashDelay;
extern uint16_t flashDelaySeed;
extern uint8_t flashAtSpeed;
extern uint16_t serialValue;



// function to process the input buffer
uint8_t ProcessReceiveBuffer(void)
{
//  SendString((char const *)&rxBuffer[nextSerialRx2Proc], 1, StripZeros, NoAddCRLF);
  if (rxBuffer[nextSerialRx2Proc] == '$') {
    inPacket = true;
    packetBuffer[0] = rxBuffer[nextSerialRx2Proc];
    nextPacketChar = 1;
  }
  else {
    if (inPacket == true) {
      if (((rxBuffer[nextSerialRx2Proc] >= '0') && (rxBuffer[nextSerialRx2Proc] <= '9')) || 
          ((rxBuffer[nextSerialRx2Proc] >= 'r') && (rxBuffer[nextSerialRx2Proc] <= 'v')) ||
          ((rxBuffer[nextSerialRx2Proc] >= 'R') && (rxBuffer[nextSerialRx2Proc] <= 'V')) ||
          (rxBuffer[nextSerialRx2Proc] >= '\n') || (rxBuffer[nextSerialRx2Proc] <= '\r')) {
        
            packetBuffer[nextPacketChar++] = rxBuffer[nextSerialRx2Proc];

            if (rxBuffer[nextSerialRx2Proc] == '\n') {
              processPacket = true;
              inPacket = false;
            }
          }
      else {
        inPacket = false;
      }
    }
  }
  
  
  if (++nextSerialRx2Proc >= RX_BUFFER_SIZE) {
    nextSerialRx2Proc = 0;
  }
  return 0;

}


uint8_t ProcessPacket(void)
{
  switch (packetBuffer[1]) {
  // list of commands
  // each command has intentional fallthru for upper/lower case
  case 'r':     // r = turn on LED
  case 'R':     
    HAL_GPIO_WritePin(BOARD_MOUNTED_LED, GPIO_PIN_SET);
    flashLED = false;
    flashAtSpeed = false;
    break;
  case 's':     // s = turn off LED
  case 'S':
    HAL_GPIO_WritePin(BOARD_MOUNTED_LED, GPIO_PIN_RESET);
    flashLED = false;
    flashAtSpeed = false;
  break;
  case 't':     // t = toggle LED
  case 'T':
    HAL_GPIO_TogglePin(BOARD_MOUNTED_LED);
    flashLED = false;
    flashAtSpeed = false;
    break;
  case 'u':     // u = flash LED at speed (the number entered is the on and off time in 10mS 
  case 'U':     // increments, so a value of 100 is 1 sec on and 1 sec off
    flashLED = false;
    flashAtSpeed = true;
    ConvertASCII2UINT16((char const *)&packetBuffer[2], 5, '\n', &flashDelaySeed);
    flashDelay = flashDelaySeed;
    break;
  case 'v':     // v = check switch press
  case 'V':
    if (buttonPushed == true) {
      SendString("Pushed", 6, StripZeros, AddCRLF);
      buttonPushed = false;      
    }
    else {
      SendString("Not Pushed", 10, StripZeros, AddCRLF);
    }
    break;
  case 'w':     // w = write a value to the variable "serialValue"
  case 'W':
    ConvertASCII2UINT16(&packetBuffer[2], 5, '\n', &serialValue);
    break;
  case 'x':     // x = flash LED at a 1/2Hz rate (1 sec on, 1 sec off)
  case 'X':
    flashLED = true;
    flashAtSpeed = false;
    break;
  }

  processPacket = false;

  return 0;
}


