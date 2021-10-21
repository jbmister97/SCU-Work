// serial_user.c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "serial.h"
#include "serial_user.h"
#include "ASCII_numbers.h"

#define ARR_SIZE        20
#define MAX_VALUE       65535

uint8_t packetBuffer[16];
uint8_t inPacket = false;
uint8_t nextPacketChar = 0;
uint8_t processPacket = false;

char rValue[ARR_SIZE];
char gValue[ARR_SIZE];
char bValue[ARR_SIZE];



extern uint8_t flashLED;
extern uint8_t buttonPushed;
extern uint16_t flashDelay;
extern uint16_t flashDelaySeed;
extern uint8_t flashAtSpeed;
extern uint16_t serialValue;
extern uint8_t led_state;
extern uint16_t ledValue;
extern uint8_t customValue_flag;
extern uint16_t redValue;
extern uint16_t greenValue;
extern uint16_t blueValue;

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
  case 'r':     // r = Set LED state to cycle normally
  case 'R':     
    led_state = 0;
    customValue_flag = false;
    break;
  case 's':     // s = Set LED state to stop cycling
  case 'S':
    led_state = 1;
    customValue_flag = false;
    break;
  case 't':     // t = Set LED state to 50% duty cycle
  case 'T':
    led_state = 2;
    customValue_flag = false;
    break;
  case 'u':     // u = Set LED state to off
  case 'U':
    led_state = 3;      // 
    customValue_flag = false;
    break;
  case 'v':     // v = Report LED values
  case 'V':
    char rOut[ARR_SIZE] = "R = ";
    char gOut[ARR_SIZE] = "G = ";
    char bOut[ARR_SIZE] = "B = ";
    // Convert LED values into a string
    sprintf(rValue,"%d",redValue);
    sprintf(gValue,"%d",greenValue);
    sprintf(bValue,"%d",blueValue);

    strcat(rOut,rValue);
    strcat(gOut,gValue);
    strcat(bOut,bValue);
    
    SendString(rOut, sizeof(rOut), StripZeros, AddCRLF);
    SendString(gOut, sizeof(gOut), StripZeros, AddCRLF);
    SendString(bOut, sizeof(bOut), StripZeros, AddCRLF);
    
    // Clear arrays after sending
    for(uint8_t i = 0; i < sizeof(rOut); i++) {rValue[i] = '\0';}
    for(uint8_t i = 0; i < sizeof(gOut); i++) {gValue[i] = '\0';}
    for(uint8_t i = 0; i < sizeof(bOut); i++) {bValue[i] = '\0';}
    break;
  case 'w':     // w = Manually set the LED values
  case 'W':
    if((packetBuffer[7] >= '0') && (packetBuffer[7] <= '9')){
      SendString("Value too high", 14, StripZeros, AddCRLF);
      break;
    }
        
    uint8_t tempArr[5];
    uint32_t tempValue;
    for(uint8_t i = 0; i < 5; i++) {tempArr[i] = packetBuffer[i+2];}
    tempValue = atoi((const char *)&tempArr);
    
    if(tempValue <= MAX_VALUE){
      led_state = 0;
      customValue_flag = true;
      ledValue = tempValue;
    }
    else{SendString("Value too high", 14, StripZeros, AddCRLF);}
    break;
  case 'x':     // x = flash LED at a 1/2Hz rate (1 sec on, 1 sec off)
  case 'X':
    //flashLED = true;
    //flashAtSpeed = false;
    break;
  }

  processPacket = false;

  return 0;
}


