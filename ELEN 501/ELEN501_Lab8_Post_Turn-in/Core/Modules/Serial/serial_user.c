// serial_user.c

#include <stdio.h>
#include "serial.h"
#include "serial_user.h"
#include "ASCII_numbers.h"
#include "ux_manager.h"

#define PACKET_BUFFER_SIZE      27


uint8_t packetBuffer[PACKET_BUFFER_SIZE];
uint8_t inPacket = false;
uint8_t nextPacketChar = 0;
uint8_t processPacket = false;
char hum_str[20];

char textOut[10];
float tempF = 0;
float lastTempC = 0;

extern uint8_t flashLED;
extern uint8_t buttonPushed;
extern uint16_t flashDelay;
extern uint16_t flashDelaySeed;
extern uint8_t flashAtSpeed;
extern uint16_t serialValue;
extern DWfloat temperature;
extern DWstring message;
extern DWstring units;

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
  case 'r':     // r = Change the temperature to given value
  case 'R':     
    temperature.data = atof(&packetBuffer[2]);
    break;
  case 's':     // s = Report humidity number
  case 'S':
    for(uint8_t i = 0; i < sizeof(textOut); i++){textOut[i] = '\0';}
    for(uint8_t i = 0; i < sizeof(hum_str); i++){hum_str[i] = '\0';}
    sprintf((char *)textOut, humidity.format, humidity.data);
    strcat(hum_str, "Humidity: ");
    strcat(hum_str,textOut);
    SendString(hum_str, sizeof(hum_str), StripZeros, AddCRLF);
  break;
  case 't':     // t = Receive and display message
  case 'T':
    for(uint8_t i = 0; i < sizeof(message.data); i++) {message.data[i] = '\0';}
    for(uint8_t i = 2; i < PACKET_BUFFER_SIZE; i++) {
      if(packetBuffer[i] == '\n') {break;}
      else {message.data[i-2] = packetBuffer[i];}
    }
    SwitchScreens(MESSAGE);
    break;
  case 'u':     // u = Toggle between celcius and fahrenheit
  case 'U':     
    if(units.data[3] == 'C') {
      units.data[3] = 'F';
      if(lastTempC != temperature.data) { // temp changed while in C
        tempF = (temperature.data*(9.0/5.0)) + 32; // update temp in F
        temperature.data = tempF;
      }
      else {temperature.data = tempF;}
    }
    else if (units.data[3] == 'F') {
      tempF = temperature.data;
      units.data[3] = 'C';
      temperature.data = (tempF-32.0)*(5.0/9.0);
    }
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


