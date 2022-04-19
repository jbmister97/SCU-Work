// serial_user.h

#ifndef _SERIAL_USER_
#define _SERIAL_USER_


#include "serial.h"

#define ESCAPE_CHAR 0xAA

// prototypes

uint8_t ProcessReceiveBuffer(void);
uint8_t ProcessReceiveByte(void);
uint8_t ProcessReceiveByteWithLength(void);

uint8_t ProcessPacket(void);
void ProcessBinaryPacket(void);

#endif