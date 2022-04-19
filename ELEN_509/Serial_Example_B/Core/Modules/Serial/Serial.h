// Serial.h


#ifndef _SERIAL_H_
#define _SERIAL_H_


// Copy the following into the serial interrupt handler in IRQn 0
// (not as a comment, of course)
//  HANDLE_INPUT_BUFFER

//  HANDLE_OUTPUT_BUFFER

//  HANDLE_ERRORS


// Includes
#include "main.h"
#include "project.h"
#include <stdint.h>


// typedefs, structs, and enums
typedef enum _strip_zeros_ {
  StripZeros,
  NoStripZeros
} strip_zeros;

typedef enum _add_crlf_ {
  AddCRLF,
  NoAddCRLF
} add_CRLF;

// defines


// private modular variables


// public variables
extern uint8_t rxBuffer[];
extern uint8_t txBuffer[];
extern uint8_t nextSerialTxOut;
extern uint8_t nextSerialTxIn;
extern uint8_t nextSerialRxIn;
extern uint8_t nextSerialRx2Proc;

// public function prototypes
uint8_t SendString(const char * _msg, uint16_t _len, strip_zeros _supressZeros, add_CRLF _add_crlf);
uint16_t CheckBuffer(void);
uint8_t SendBinary(const uint8_t * _msg, uint16_t _len);
uint8_t SendBinaryWithLength(const uint8_t * _msg, uint16_t _len);


#endif
