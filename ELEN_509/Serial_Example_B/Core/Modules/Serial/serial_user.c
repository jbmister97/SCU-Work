// serial_user.c

#include "serial.h"
#include "serial_user.h"
#include "ASCII_numbers.h"
#include <stdio.h>

uint8_t packetBuffer[PACKET_BUFFER_SIZE];
uint8_t rxTempBuffer[PACKET_BUFFER_SIZE];
uint8_t inPacket = false;
uint8_t nextPacketChar = 0;
uint8_t processPacket = false;

uint8_t processBinaryPacket = false;
int16_t packetLength = -1;
uint8_t escapeDetected = false;

uint16_t sum;
char format[15] = "%x";
char textString[25] = {0};
uint8_t temp;
uint8_t checksum;
uint8_t checkDataLength;

extern uint8_t flashLED;
extern uint8_t buttonPushed;
extern uint16_t flashDelay;
extern uint16_t flashDelaySeed;
extern uint8_t flashAtSpeed;
extern uint16_t serialValue;



// function to process the input buffer with an ASCII-based protocol
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
    ConvertASCII2UINT16((char const *)&packetBuffer[2], 5, '\n', &serialValue);
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

// function to process the input buffer with a simple binary packet
uint8_t ProcessReceiveByte(void)
{
  
  if (escapeDetected == true) {
    if (rxBuffer[nextSerialRx2Proc] == ESCAPE_CHAR) {
      packetBuffer[nextPacketChar++] = rxBuffer[nextSerialRx2Proc];
    }
    else {
      nextPacketChar = 0;
      packetBuffer[nextPacketChar++] = ESCAPE_CHAR;
      packetBuffer[nextPacketChar++] = rxBuffer[nextSerialRx2Proc];
    }
    escapeDetected = false; // either way we turn off the switch
  }
  else {
    if (rxBuffer[nextSerialRx2Proc] == ESCAPE_CHAR) {
      escapeDetected = true;
    }
    else {
      packetBuffer[nextPacketChar++] = rxBuffer[nextSerialRx2Proc];
    }
  }
  
  // Print out the byte received
  sprintf(textString, format, packetBuffer[nextPacketChar-1]);
  SendString(textString, 25, StripZeros, AddCRLF);
  //printf("R: %h\n", packetBuffer[nextPacketChar-1]);
  
  if (++nextSerialRx2Proc >= RX_BUFFER_SIZE) {
    nextSerialRx2Proc = 0;
  }
  return 0;

}


// function to process the input buffer with a binary packet that includes packet length
uint8_t ProcessReceiveByteWithLength(void)
{
  //char textString[25] = {0};
  
  if (escapeDetected == true) { // Start of a packet detected
    if (rxBuffer[nextSerialRx2Proc] == ESCAPE_CHAR) { // If two escape characters in a row, escape character in data
      packetBuffer[nextPacketChar++] = rxBuffer[nextSerialRx2Proc];
      packetLength--;
    }
    else { 
      nextPacketChar = 0;
      packetBuffer[nextPacketChar++] = ESCAPE_CHAR;
      packetLength = rxBuffer[nextSerialRx2Proc] + 1; // Get packet length from second byte in packet
      packetBuffer[nextPacketChar++] = rxBuffer[nextSerialRx2Proc];
    }
    escapeDetected = false; // either way we turn off the switch
  }
  else {
    if (rxBuffer[nextSerialRx2Proc] == ESCAPE_CHAR) {
      escapeDetected = true;
    }
    else {
      packetBuffer[nextPacketChar++] = rxBuffer[nextSerialRx2Proc];
      if (packetLength > 0) 
        packetLength--;
    }
  }
  
  //temp = packetBuffer[nextPacketChar-1];
  // Print out the byte received
  sprintf(textString, format, packetBuffer[nextPacketChar-1]);
  SendString(textString, 25, StripZeros, AddCRLF);
  
  if (++nextSerialRx2Proc >= RX_BUFFER_SIZE) {
    nextSerialRx2Proc = 0;
  }
  
  // Reached end of packet
  if (packetLength == 0) {
    processBinaryPacket = true;
    packetLength = -1;
  }
  return 0;

}


// Validate and handle the binary packet that was transmitted
void ProcessBinaryPacket(void)
{
  //HAL_Delay(2000);
  // Verify checksum is valid
  if(Verify_Checksum(packetBuffer, packetBuffer[1])) {
    SendString("Checksum PASSED", 15, StripZeros, AddCRLF);
  }
  else {
    SendString("Checksum FAILED", 15, StripZeros, AddCRLF);
  }
  
  processBinaryPacket = false;
}

// Function to generate checksum value for a message
// Value
uint8_t Get_Checksum(const uint8_t * msg, uint16_t len){
  
  uint16_t sum = 0;
  
  // Add up all the bytes in the array except the starting escape character
  for(uint8_t i = 1; i < len; i++) {sum += msg[i];}
  
  // Return the last 8 bits of the sum
  return (uint8_t) sum;
}

uint8_t Verify_Checksum(const uint8_t * msg, uint16_t len) {
  
  sum = 0;
  
  checkDataLength = len + 2; // Subtract one since len is total # of bytes in packet
  
  // Get checksum value from the packet
  checksum = msg[checkDataLength]; 
  
  // Add up all the bytes in the array except the starting escape character and checksum
  for(uint8_t i = 1; i < checkDataLength; i++) {sum += msg[i];}
  
  temp = (uint8_t) sum;
  
  // Return true or false if the last 8 bits of the sum match the checksum
  return ((((uint8_t) sum) == checksum) ? 1 : 0);
}
