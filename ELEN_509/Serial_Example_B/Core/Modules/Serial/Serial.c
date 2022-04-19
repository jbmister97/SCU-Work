// Serial.c

// includes
#include "serial_user.h"
#include <stm32g0xx_ll_usart.h>


// modular variables
uint8_t rxBuffer[RX_BUFFER_SIZE];
uint8_t txBuffer[TX_BUFFER_SIZE];
uint8_t nextSerialTxOut = 0;
uint8_t nextSerialTxIn = 0;
uint8_t nextSerialRxIn = 0;
uint8_t nextSerialRx2Proc = 0;

// external variables
extern UART_HANDLE;

// Function to submit char and binary strings into the Xmit buffer
// it has a switch to decide if trailing zeros should be put into the buffer or ignored
uint8_t SendString(const char * _msg, uint16_t _len, strip_zeros _supressZeros, add_CRLF _add_crlf)
{
  uint8_t uartIdle = false;
  uint8_t status = 0;
  uint16_t freeBufferSpace;
  uint8_t i;
  uint16_t totalLength = _len;
  
  if (_add_crlf == AddCRLF) totalLength += 2;
  
  freeBufferSpace = CheckBuffer();
  uartIdle = (nextSerialTxOut == nextSerialTxIn) ? true : false;
  
  if (totalLength < freeBufferSpace) {
    for (i = 0; i < _len; i++) {
      if ( (_msg[i] != 0) || (_supressZeros == NoStripZeros) ) {
        txBuffer[nextSerialTxIn] = _msg[i];
        if (++nextSerialTxIn >= TX_BUFFER_SIZE) nextSerialTxIn = 0;
      }
    }
    
    if (_add_crlf == AddCRLF) {
      txBuffer[nextSerialTxIn] = '\r';
      if (++nextSerialTxIn >= TX_BUFFER_SIZE) nextSerialTxIn = 0;

      txBuffer[nextSerialTxIn] = '\n';
      if (++nextSerialTxIn >= TX_BUFFER_SIZE) nextSerialTxIn = 0;
    }
    
    if (uartIdle) {
      LL_USART_EnableIT_TXE(USART_INSTANCE);
    }
  }
  else status = 1;
  
  
  return status;
}


uint16_t CheckBuffer(void)
{
  uint16_t bufferAvailBytes = 0;
  
  if (nextSerialTxOut > nextSerialTxIn) bufferAvailBytes = nextSerialTxOut - nextSerialTxIn;
  else bufferAvailBytes = TX_BUFFER_SIZE - (nextSerialTxIn - nextSerialTxOut);
  
  return bufferAvailBytes;
}

// Function to submit char and binary strings into the Xmit buffer
// it has a switch to decide if trailing zeros should be put into the buffer or ignored
uint8_t SendBinary(const uint8_t * _msg, uint16_t _len)
{
  uint8_t uartIdle = false;
  uint8_t status = 0;
  uint16_t freeBufferSpace;
  uint8_t i;
  uint16_t totalLength = _len;
  
  freeBufferSpace = CheckBuffer();
  uartIdle = (nextSerialTxOut == nextSerialTxIn) ? true : false;
  
  if (totalLength < freeBufferSpace) {
    txBuffer[nextSerialTxIn] = ESCAPE_CHAR;
    if (++nextSerialTxIn >= TX_BUFFER_SIZE) nextSerialTxIn = 0;
    
    //for (i = 1; i < _len; i++) {
    for (i = 1; i < totalLength; i++) {
      if (_msg[i] != ESCAPE_CHAR) {
        txBuffer[nextSerialTxIn] = _msg[i];
        if (++nextSerialTxIn >= TX_BUFFER_SIZE) nextSerialTxIn = 0;
      }
      else {
        txBuffer[nextSerialTxIn] = ESCAPE_CHAR;
        if (++nextSerialTxIn >= TX_BUFFER_SIZE) nextSerialTxIn = 0;

        txBuffer[nextSerialTxIn] = ESCAPE_CHAR;
        if (++nextSerialTxIn >= TX_BUFFER_SIZE) nextSerialTxIn = 0;
      }
      
    }
    
    if (uartIdle) {
      LL_USART_EnableIT_TXE(USART_INSTANCE);
    }
  }
  else status = 1;
  
    return status;
}
  uint8_t SendBinaryWithLength(const uint8_t * _msg, uint16_t _len)
{
  uint8_t uartIdle = false;
  uint8_t status = 0;
  uint16_t freeBufferSpace;
  uint8_t i;
  uint16_t totalLength = _len + 2;      //Add two for the length byte and first escape char
  
  freeBufferSpace = CheckBuffer();
  uartIdle = (nextSerialTxOut == nextSerialTxIn) ? true : false;
  
  if (totalLength < freeBufferSpace) {
    // Add the initial escape character to the tx buffer
    txBuffer[nextSerialTxIn] = ESCAPE_CHAR;
    if (++nextSerialTxIn >= TX_BUFFER_SIZE) nextSerialTxIn = 0;
    
    // Add a byte for the length to the tx buffer
    txBuffer[nextSerialTxIn] = (uint8_t) _len;
    if (++nextSerialTxIn >= TX_BUFFER_SIZE) nextSerialTxIn = 0;
    
    // Add the message to the tx buffer
    for (i = 2; i < totalLength; i++) {
      if (_msg[i] != ESCAPE_CHAR) {
        txBuffer[nextSerialTxIn] = _msg[i];
        if (++nextSerialTxIn >= TX_BUFFER_SIZE) nextSerialTxIn = 0;
      }
      else {
        txBuffer[nextSerialTxIn] = ESCAPE_CHAR;
        if (++nextSerialTxIn >= TX_BUFFER_SIZE) nextSerialTxIn = 0;

        txBuffer[nextSerialTxIn] = ESCAPE_CHAR;
        if (++nextSerialTxIn >= TX_BUFFER_SIZE) nextSerialTxIn = 0;
      }
      
    }
    
    if (uartIdle) {
      LL_USART_EnableIT_TXE(USART_INSTANCE);
    }
  }
  else status = 1;
  
  return status;
}

