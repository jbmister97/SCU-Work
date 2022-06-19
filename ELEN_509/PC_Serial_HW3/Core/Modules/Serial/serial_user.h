// serial_user.h

#ifndef _SERIAL_USER_
#define _SERIAL_USER_


#include "serial.h"

#define ESCAPE_CHAR 0xAA
#define PACKET_BUFFER_SIZE 16

// public variables
extern uint8_t processPacket;

extern uint8_t flashLED;
extern uint8_t buttonPushed;
extern uint16_t flashDelay;
extern uint16_t flashDelaySeed;
extern uint8_t flashAtSpeed;
extern uint16_t serialValue;


// prototypes

uint8_t ProcessReceiveBuffer(void);
uint8_t ProcessReceiveByte(void);
uint8_t ProcessReceiveByteWithLength(void);
uint8_t ProcessReceiveByteWithoutLength(void);

uint8_t ProcessPacket(void);
void ProcessBinaryPacket(void);

#endif