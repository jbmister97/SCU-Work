
#include <stdint.h>
#include "serial_user.h"


const char NIBBLE2HEX[16] = "0123456789ABCDEF";


uint8_t SendModBusCommand(uint8_t _slaveAddr, uint8_t _func, uint16_t _stAddr, uint16_t _noReg)
{
  uint8_t modbusPacket[17];
  
  // assemble the packet
  modBudPacket[0] = ':';
  modbusPacket[1] = NIBBLE2HEX[_slaveAddr / 16]; // turn the number into a hex ascii char
  modbusPacket[2] = NIBBLE2HEX[_slaveAddr % 16];
  
  modbusPacket[3] = NIBBLE2HEX[_func / 16];
  modbusPacket[4] = NIBBLE2HEX[_func % 16];
  
  modbusPacket[5] = NIBBLE2HEX[(_stAddr >> 12) & 0x0F];
  modbusPacket[6] = NIBBLE2HEX[(_stAddr >> 8) & 0x0F];
  modbusPacket[7] = NIBBLE2HEX[(_stAddr >> 4) & 0x0F];
  modbusPacket[8] = NIBBLE2HEX[(_stAddr >> 0) & 0x0F];

  modbusPacket[9] = NIBBLE2HEX[(_noReg >> 12) & 0x0F];
  modbusPacket[10] = NIBBLE2HEX[(_noReg >> 8) & 0x0F];
  modbusPacket[11] = NIBBLE2HEX[(_noReg >> 4) & 0x0F];
  modbusPacket[12] = NIBBLE2HEX[(_noReg >> 0) & 0x0F];

  modbusPacket[14] = 0;
  modbusPacket[15] = 0;

  modbusPacket[16] = '\r';
  modbusPacket[17] = '\n';
  
  SendString(MB_outputBuffer, modbusPacket, 17, noStripZeros, noAddCRLF); 
  
}