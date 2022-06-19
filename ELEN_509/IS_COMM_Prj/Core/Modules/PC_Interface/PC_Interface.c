#include "PC_Interface.h"
#include "main.h"



uint8_t pcPacketBuffer[PC_PACKET_BUFFER_LENGTH] = "";

uint8_t pcPacketBufferIndex = 0;

pc_packet_buffer_status pcPacketStatus = PC_WAITING_FOR_PACKET_START;

uint8_t pcProcessPacket = false;

void ProcessPcInputChar(comm_buffer_t * _buff_instance)
{
  switch (pcPacketStatus) {
  case PC_WAITING_FOR_PACKET_START:
    if (_buff_instance->buffer[_buff_instance->nextBufferOut] == '$') {
      pcPacketBufferIndex = 1;
      pcPacketBuffer[0] = _buff_instance->buffer[_buff_instance->nextBufferOut];
      
      pcPacketStatus = PC_IN_PACKET;
    }
    break;
  case PC_IN_PACKET:
    pcPacketBuffer[pcPacketBufferIndex] = _buff_instance->buffer[_buff_instance->nextBufferOut];
    
    if ((pcPacketBuffer[pcPacketBufferIndex] == '\r') || (pcPacketBuffer[pcPacketBufferIndex] == '\n')) {
      // place this next line in this case if there is no cjecksum, if there is one, 
      // enable the checksum check case and place it in there instead
      pcProcessPacket = true;
      
      pcPacketStatus = PC_WAITING_FOR_PACKET_START;
    }
    
    if (pcPacketBufferIndex >= PC_PACKET_BUFFER_LENGTH - 1)
      pcPacketStatus = PC_WAITING_FOR_PACKET_START;
    
    
    pcPacketBufferIndex++;
    break;
//  case PC_IN_CHECKSUM:
//    pcPacketBuffer[pcPacketBufferIndex] = _buff_instance->buffer[_buff_instance->nextBufferOut];
//    
//    if ((pcPacketBuffer[pcPacketBufferIndex] == '\r') || (pcPacketBuffer[pcPacketBufferIndex] == '\n')) {
//      pcProcessPacket = true;
//      pcPacketStatus = PC_WAITING_FOR_PACKET_START;
//    }
//  
//    if (pcPacketBufferIndex >= PC_PACKET_BUFFER_LENGTH - 1)
//      pcPacketStatus = PC_WAITING_FOR_PACKET_START;
//
//    pcPacketBufferIndex++;
//    break;
  }
  
  _buff_instance->nextBufferOut++;
  if (_buff_instance->nextBufferOut >= _buff_instance->bufferLength) _buff_instance->nextBufferOut = 0;
}



// just a stub for now...
void ProcessPcPacket(void)
{
  pcProcessPacket = false;
  
}

