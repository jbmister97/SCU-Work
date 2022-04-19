// serial_user.h

#ifndef _SERIAL_USER_
#define _SERIAL_USER_


#include "serial.h"

// defines
#define ESCAPE_CHAR             0xAA
#define TX_BUFFER_SIZE          50
#define RX_BUFFER_SIZE          50
#define PACKET_BUFFER_SIZE      16
#define UART_HANDLE             UART_HandleTypeDef huart1
#define USART_INSTANCE          USART1

// prototypes

uint8_t ProcessReceiveBuffer(void);
uint8_t ProcessReceiveByte(void);
uint8_t ProcessReceiveByteWithLength(void);

uint8_t ProcessPacket(void);
void ProcessBinaryPacket(void);
uint8_t Get_Checksum(const uint8_t * msg, uint16_t len);
void Verify_Checksum(void);

// MACROS for Serial Buffer handling
#define HANDLE_INPUT_BUFFER                                                     \
  if(LL_USART_IsActiveFlag_RXNE(USART1) && LL_USART_IsEnabledIT_RXNE(USART1))   \
  {                                                                             \
    rxBuffer[nextSerialRxIn] =  LL_USART_ReceiveData8(USART1);                  \
    if (++nextSerialRxIn >= RX_BUFFER_SIZE) nextSerialRxIn = 0;                 \
  }

#define HANDLE_OUTPUT_BUFFER                                                    \
  if(LL_USART_IsEnabledIT_TXE(USART1) && LL_USART_IsActiveFlag_TXE(USART1))     \
  {                                                                             \
    if (nextSerialTxOut != nextSerialTxIn) {                                    \
      LL_USART_TransmitData8(USART1, txBuffer[nextSerialTxOut]);                \
      if (++nextSerialTxOut >= TX_BUFFER_SIZE) nextSerialTxOut = 0;             \
    }                                                                           \
    else LL_USART_DisableIT_TXE(USART1);                                        \
  }                                                                             \
  if(LL_USART_IsEnabledIT_TC(USART1) && LL_USART_IsActiveFlag_TC(USART1))       \
  {                                                                             \
    LL_USART_ClearFlag_TC(USART1);                                              \
    /*UART_CharTransmitComplete_Callback(); */                                  \
  }

#define HANDLE_ERRORS                                                           \
  if(LL_USART_IsEnabledIT_ERROR(USART1) && LL_USART_IsActiveFlag_NE(USART1))    \
  {                                                                             \
    /* Call Error function */                                                   \
    /*UART_Error_Callback();*/                                                  \
  }

#endif