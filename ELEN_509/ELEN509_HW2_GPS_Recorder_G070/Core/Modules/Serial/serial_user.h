// serial_user.h

#ifndef _SERIAL_USER_
#define _SERIAL_USER_

typedef struct
{
  double latitude;
  double longitude;
  float utcTime;
  float hdop;
  float mslAltitude;
  float geoidalSep;
  uint32_t ageDiff;
  char NSIndicator;
  char EWIndicator;
  char unitsMSL;
  char unitsGeo;
  uint8_t posFix;
  uint8_t satUsed;
  uint8_t checkSum;
  
} GPGGA_Record_t;

// prototypes

uint8_t ProcessReceiveBuffer(void);
uint8_t ProcessPacket(void);
void ProcessGpsPacket(uint8_t byte);
void Init_GPS_Message(GPGGA_Record_t *msg);
void Get_Checksum(uint8_t byte);
void Send_Bad_Record(void);
void Send_Wrong_Record(void);

#endif