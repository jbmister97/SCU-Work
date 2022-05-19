// serial_user.c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "serial.h"
#include "serial_user.h"
#include "ASCII_numbers.h"

#define ARR_SIZE                20
#define MAX_VALUE               65535

#define MSG_ID_BUFF_SIZE        6
#define GPGGA_MESSAGE_ID        "$GPGGA"
#define STATE_GPGGA_START       0
#define STATE_GPGGA_UTC         1
#define STATE_GPGGA_LAT         2
#define STATE_GPGGA_NS_IND      3
#define STATE_GPGGA_LONG        4
#define STATE_GPGGA_EW_IND      5
#define STATE_GPGGA_POS_IND     6
#define STATE_GPGGA_SAT_USED    7
#define STATE_GPGGA_HDOP        8
#define STATE_GPGGA_MSL         9
#define STATE_GPGGA_UNITS_MSL   10
#define STATE_GPGGA_GEO         11
#define STATE_GPGGA_UNITS_GEO   12
#define STATE_GPGGA_AGE_DIFF    13
#define STATE_GPGGA_CHECKSUM    14

#define TEMP_BUFF_SIZE          15

uint8_t packetBuffer[100];
uint8_t inPacket = false;
uint8_t nextPacketChar = 0;
uint8_t processPacket = false;
uint8_t packetValid = false;

char msgIDBuff[MSG_ID_BUFF_SIZE];
uint8_t msgIDBuffIndex  = 0;

GPGGA_Record_t gpsMsg;
GPGGA_Record_t validMsg;
uint8_t stateGPGGA;
uint8_t isGPGGA = false;

uint8_t tempBuff[TEMP_BUFF_SIZE];
uint8_t tempBuffIndex = 0;
uint8_t checksum = 0;
uint8_t testChecksum = 0;
char latitudePosMsgFormat[30] = "Latitude: %f";
char latitudeNegMsgFormat[30] = "Latitude: -%f";
char longitudePosMsgFormat[30] = "Longitude: %f";
char longitudeNegMsgFormat[30] = "Longitude: -%f";

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
    
    msgIDBuffIndex = 0;
    checksum = 0;
    testChecksum = 0;
    isGPGGA = false;
    stateGPGGA = STATE_GPGGA_START;
    packetBuffer[0] = rxBuffer[nextSerialRx2Proc];
    msgIDBuff[msgIDBuffIndex++] = (char) rxBuffer[nextSerialRx2Proc];
    nextPacketChar = 1;
  }
  else {
    if (inPacket == true) {
      packetBuffer[nextPacketChar++] = rxBuffer[nextSerialRx2Proc];
      // Get message ID
      if(msgIDBuffIndex < MSG_ID_BUFF_SIZE) {
        msgIDBuff[msgIDBuffIndex++] = (char) rxBuffer[nextSerialRx2Proc];
        Get_Checksum(rxBuffer[nextSerialRx2Proc]);
        // If ID data goes outside of capital letter ASCII range, ID is not valid
        if(rxBuffer[nextSerialRx2Proc] < 'A' && rxBuffer[nextSerialRx2Proc] > 'Z'){
          inPacket = false;
          SendString("BAD RECORD",10,StripZeros,AddCRLF);
        }
      }
      // Finished getting ID so process packet based on message ID
      else{
        //if(msgIDBuff == "$GPGGA") {
        if(!strcmp(msgIDBuff, "$GPGGA")) {
          isGPGGA = true;
          ProcessGpsPacket(rxBuffer[nextSerialRx2Proc]);
        }
        else{
          // Wrong message ID
          inPacket = false;
          SendString("WRONG RECORD",12,StripZeros,AddCRLF);
        }
        
        // Check if end of GPS packet as been reached
        if(rxBuffer[nextSerialRx2Proc] == '\n') {
          inPacket = false;
          char msgDisplayBuff[23];
          
          // Check if packet is GPGGA
          //if(isGPGGA == true) {
            // Verify checksum matches
            if(checksum == gpsMsg.checkSum) {
              validMsg = gpsMsg;
              // Display latitude
              if(validMsg.NSIndicator == 'S') { // If indicator is south make latitude negative
                sprintf(msgDisplayBuff, latitudeNegMsgFormat, validMsg.latitude);
                SendString(msgDisplayBuff, 23, StripZeros, AddCRLF);
              }
              else { // Make latitude positive
                sprintf(msgDisplayBuff, latitudePosMsgFormat, validMsg.latitude);
                SendString(msgDisplayBuff, 23, StripZeros, AddCRLF);
              }
              
              // Display longitude
              if(validMsg.EWIndicator == 'W') { // If indicator is west make longitude negative
                sprintf(msgDisplayBuff, longitudeNegMsgFormat, validMsg.longitude);
                SendString(msgDisplayBuff, 23, StripZeros, AddCRLF);
              }
              else { // Make longitude positive
                sprintf(msgDisplayBuff, longitudePosMsgFormat, validMsg.longitude);
                SendString(msgDisplayBuff, 23, StripZeros, AddCRLF);
              }
            }
            else { // GPGGA packet is bad since checksum did not match
              SendString("BAD RECORD",10,StripZeros,AddCRLF);
            }
          //}
          //else { // Packet is not GPGGA
            //SendString("WRONG RECORD",12,StripZeros,AddCRLF);
            
          //}
          // Clear message ID buffer
          for(uint8_t i = 0; i < MSG_ID_BUFF_SIZE; i++) {msgIDBuff[i] = 0;}
        }
      }
      
      
    }
  }
  
  
  if (++nextSerialRx2Proc >= RX_BUFFER_SIZE) {
    nextSerialRx2Proc = 0;
  }
  return 0;

}

void ProcessGpsPacket(uint8_t byte) {
  
  // calculate the checksum
  if(stateGPGGA != STATE_GPGGA_CHECKSUM) {
    if((byte != '\r') && (byte != '\n')){
      Get_Checksum(byte);
    }
    
  }
  
  switch(stateGPGGA) {
  case STATE_GPGGA_START:
    if(byte == ',') {
      stateGPGGA = STATE_GPGGA_UTC;
      // Clear temp buffer
      for(uint8_t i = 0; i < TEMP_BUFF_SIZE; i++) {tempBuff[i] = 0;}
      tempBuffIndex = 0;
    }
    break;
  case STATE_GPGGA_UTC:
    // If end of item data reached
    if(byte == ',') {
      // Convert time from string to float and add to structure
      gpsMsg.utcTime = atof((char const*) tempBuff);
      
      // Clear temp buffer
      for(uint8_t i = 0; i < TEMP_BUFF_SIZE; i++) {tempBuff[i] = 0;}
      tempBuffIndex = 0;
      
      // Move onto next item in packet
      stateGPGGA = STATE_GPGGA_LAT;
    }
    // Still inside item data
    else{
      tempBuff[tempBuffIndex++] = byte;
      if(tempBuffIndex == TEMP_BUFF_SIZE) {tempBuffIndex = 0;}
    }
    break;
  case STATE_GPGGA_LAT:
    // If end of item data reached
    if(byte == ',') {
      // Convert latitude from string to double
      //gpsMsg.latitude = atof((char const*) tempBuff);
      gpsMsg.latitude = strtod((char const*) tempBuff,NULL);
      
      // Clear temp buffer
      for(uint8_t i = 0; i < TEMP_BUFF_SIZE; i++) {tempBuff[i] = 0;}
      tempBuffIndex = 0;
      
      // Move onto next item in packet
      stateGPGGA = STATE_GPGGA_NS_IND;
    }
    // Still inside item data
    else{
      tempBuff[tempBuffIndex++] = byte;
      if(tempBuffIndex == TEMP_BUFF_SIZE) {tempBuffIndex = 0;}
    }
    break;
  case STATE_GPGGA_NS_IND:
    // If end of item data reached
    if(byte == ',') {
      
      // Set indicator value
      gpsMsg.NSIndicator = (char) tempBuff[0];
    
      // Clear temp buffer
      for(uint8_t i = 0; i < TEMP_BUFF_SIZE; i++) {tempBuff[i] = 0;}
      tempBuffIndex = 0;
      
      // Move onto next item in packet
      stateGPGGA = STATE_GPGGA_LONG;
    }
    // Still inside item data
    else{
      tempBuff[0] = byte;
    }
    break;
  case STATE_GPGGA_LONG:
    // If end of item data reached
    if(byte == ',') {
      // Convert latitude from string to double
      //gpsMsg.longitude = atof((char const*) tempBuff);
      gpsMsg.longitude = strtod((char const*) tempBuff,NULL);
      
      // Clear temp buffer
      for(uint8_t i = 0; i < TEMP_BUFF_SIZE; i++) {tempBuff[i] = 0;}
      tempBuffIndex = 0;
      
      // Move onto next item in packet
      stateGPGGA = STATE_GPGGA_EW_IND;
    }
    // Still inside item data
    else{
      tempBuff[tempBuffIndex++] = byte;
      if(tempBuffIndex == TEMP_BUFF_SIZE) {tempBuffIndex = 0;}
    }
    break;
  case STATE_GPGGA_EW_IND:
    // If end of item data reached
    if(byte == ',') {
      
      // Set indicator value
      gpsMsg.EWIndicator = (char) tempBuff[0];
    
      // Clear temp buffer
      for(uint8_t i = 0; i < TEMP_BUFF_SIZE; i++) {tempBuff[i] = 0;}
      tempBuffIndex = 0;
      
      // Move onto next item in packet
      stateGPGGA = STATE_GPGGA_POS_IND;
    }
    // Still inside item data
    else{
      tempBuff[0] = byte;
    }
    break;
  case STATE_GPGGA_POS_IND:
    // If end of item data reached
    if(byte == ',') {
      tempBuffIndex = 0;
      
      // Move onto next item in packet
      stateGPGGA = STATE_GPGGA_SAT_USED;
    }
    // Still inside item data
    else{
      // Set indicator value
      gpsMsg.posFix = (uint8_t) atoi((char const*) byte);
    }
    break;
  case STATE_GPGGA_SAT_USED:
    // If end of item data reached
    if(byte == ',') {
      // Convert # of satellites from string to float
      gpsMsg.satUsed = (uint8_t) atoi((char const*) tempBuff);
      
      // Clear temp buffer
      for(uint8_t i = 0; i < TEMP_BUFF_SIZE; i++) {tempBuff[i] = 0;}
      tempBuffIndex = 0;
      
      // Move onto next item in packet
      stateGPGGA = STATE_GPGGA_HDOP;
    }
    // Still inside item data
    else{
      tempBuff[tempBuffIndex++] = byte;
      if(tempBuffIndex == TEMP_BUFF_SIZE) {tempBuffIndex = 0;}
    }
    break;
  case STATE_GPGGA_HDOP:
    // If end of item data reached
    if(byte == ',') {
      // Convert # of satellites from string to float
      gpsMsg.hdop = atof((char const*) tempBuff);
      
      // Clear temp buffer
      for(uint8_t i = 0; i < TEMP_BUFF_SIZE; i++) {tempBuff[i] = 0;}
      tempBuffIndex = 0;
      
      // Move onto next item in packet
      stateGPGGA = STATE_GPGGA_MSL;
    }
    // Still inside item data
    else{
      tempBuff[tempBuffIndex++] = byte;
      if(tempBuffIndex == TEMP_BUFF_SIZE) {tempBuffIndex = 0;}
    }
    break;
  case STATE_GPGGA_MSL:
    // If end of item data reached
    if(byte == ',') {
      // Convert # of satellites from string to float
      gpsMsg.mslAltitude = atof((char const*) tempBuff);
      
      // Clear temp buffer
      for(uint8_t i = 0; i < TEMP_BUFF_SIZE; i++) {tempBuff[i] = 0;}
      tempBuffIndex = 0;
      
      // Move onto next item in packet
      stateGPGGA = STATE_GPGGA_UNITS_MSL;
    }
    // Still inside item data
    else{
      tempBuff[tempBuffIndex++] = byte;
      if(tempBuffIndex == TEMP_BUFF_SIZE) {tempBuffIndex = 0;}
    }
    break;
  case STATE_GPGGA_UNITS_MSL:
    // If end of item data reached
    if(byte == ',') {
      // Set units value
      gpsMsg.unitsMSL = (char) tempBuff[0];
      
      // Clear temp buffer
      for(uint8_t i = 0; i < TEMP_BUFF_SIZE; i++) {tempBuff[i] = 0;}
      tempBuffIndex = 0;
      
      // Move onto next item in packet
      stateGPGGA = STATE_GPGGA_GEO;
    }
    // Still inside item data
    else{
      tempBuff[0] = byte;
    }
    break;
  case STATE_GPGGA_GEO:
    // If end of item data reached
    if(byte == ',') {
      // Convert # of satellites from string to float
      gpsMsg.geoidalSep = atof((char const*) tempBuff);
      
      // Clear temp buffer
      for(uint8_t i = 0; i < TEMP_BUFF_SIZE; i++) {tempBuff[i] = 0;}
      tempBuffIndex = 0;
      
      // Move onto next item in packet
      stateGPGGA = STATE_GPGGA_UNITS_GEO;
    }
    // Still inside item data
    else{
      tempBuff[tempBuffIndex++] = byte;
      if(tempBuffIndex == TEMP_BUFF_SIZE) {tempBuffIndex = 0;}
    }
    break;
  case STATE_GPGGA_UNITS_GEO:
    // If end of item data reached
    if(byte == ',') {
      // Set units value
      gpsMsg.unitsGeo = (char) tempBuff[0];
      
      // Clear temp buffer
      for(uint8_t i = 0; i < TEMP_BUFF_SIZE; i++) {tempBuff[i] = 0;}
      tempBuffIndex = 0;
      
      // Move onto next item in packet
      stateGPGGA = STATE_GPGGA_AGE_DIFF;
    }
    // Still inside item data
    else{
      tempBuff[0] = byte;
    }
    break;
  case STATE_GPGGA_AGE_DIFF:
    // If end of item data reached
    if(byte == ',') {
      // Convert # of satellites from string to float
      gpsMsg.ageDiff = atoi((char const*) tempBuff);
      
      // Clear temp buffer
      for(uint8_t i = 0; i < TEMP_BUFF_SIZE; i++) {tempBuff[i] = 0;}
      tempBuffIndex = 0;
      
      // Move onto next item in packet
      stateGPGGA = STATE_GPGGA_CHECKSUM;
    }
    // Still inside item data
    else{
      tempBuff[tempBuffIndex++] = byte;
      if(tempBuffIndex == TEMP_BUFF_SIZE) {tempBuffIndex = 0;}
    }
    break;
  case STATE_GPGGA_CHECKSUM:
    // If end of item data reached
    if(byte == '\r') {
      // Convert checksum from string to hex in int form
      //gpsMsg.checkSum = (uint8_t) atoi((char const*) tempBuff);
      gpsMsg.checkSum = (uint8_t) strtol((char const*) tempBuff, NULL, 16);
      
      // Clear temp buffer
      for(uint8_t i = 0; i < TEMP_BUFF_SIZE; i++) {tempBuff[i] = 0;}
      tempBuffIndex = 0;
      
      // Last item in packet reached so reset to first item
      stateGPGGA = STATE_GPGGA_START;
      packetValid = true;
    }
    // Still inside item data
    else{
      if(byte != '*') {
        tempBuff[tempBuffIndex++] = byte;
        if(tempBuffIndex == TEMP_BUFF_SIZE) {tempBuffIndex = 0;}
      }
    }
    break;    
  }
}

void Get_Checksum(uint8_t byte) {
  checksum ^= byte;
}

void Init_GPS_Message(GPGGA_Record_t *msg) {
  msg->utcTime = 0;
  msg->latitude = 0;
  msg->NSIndicator = '\0';
  msg->longitude = 0;
  msg->EWIndicator = '\0';
  msg->posFix = 0;
  msg->satUsed = 0;
  msg->hdop = 0;
  msg->mslAltitude = 0;
  msg->unitsMSL = '\0';
  msg->geoidalSep = 0;
  msg->unitsGeo = '\0';
  msg->ageDiff = 0;
  msg->checkSum = 0;
}

uint8_t ProcessPacket(void)
{
  switch (packetBuffer[1]) {
  // list of commands
  // each command has intentional fallthru for upper/lower case
  case 'r':     // r = Set LED state to cycle normally
  case 'R':     

    break;
  case 's':     // s = Set LED state to stop cycling
  case 'S':

    break;
  case 't':     // t = Set LED state to 50% duty cycle
  case 'T':
 
    break;
  case 'u':     // u = Set LED state to off
  case 'U':

    break;
  case 'v':     // v = Report LED values
  case 'V':

    break;
  case 'w':     // w = Manually set the LED values
  case 'W':

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


