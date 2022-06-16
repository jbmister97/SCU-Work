

#ifndef _PC_INTERFACE_
#define _PC_INTERFACE_

#include <stdint.h>
#include "serial_user.h"


#define PC_PACKET_BUFFER_LENGTH 50


typedef enum _pc_packet_buffer_status {
  PC_WAITING_FOR_PACKET_START,
  PC_IN_PACKET,
  PC_IN_CHECKSUM,
  PC_PACKET_RECEIVED
} pc_packet_buffer_status;

extern uint8_t pcPacketBuffer[];

extern uint8_t pcPacketBufferIndex;

extern pc_packet_buffer_status pcPacketStatus;

extern uint8_t pcProcessPacket;

// prototypes
void ProcessPcInputChar(comm_buffer_t * _buff_instance);
void ProcessPcPacket(void);


#endif