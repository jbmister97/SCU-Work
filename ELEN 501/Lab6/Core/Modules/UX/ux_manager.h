// ux_manager.h



#ifndef UX_MGR
#define UX_MGR

// includes
#include <stdint.h>
#include "ssd1306.h"


// typedefs
typedef enum  Screens_{
  HOME,
  DETAIL,
  SETTINGS,
  MESSAGE,
  SET_TEMP,
//  NUMBER_OF_SCREENS
} ui_screen;


typedef struct DWuint8_t_
{
  char format[10];
  char invalidMsg[4];
  uint16_t xPos;
  uint16_t yPos;
  uint8_t valid;
  uint8_t data;
} DWuint8_t;


typedef struct DWint8_t_
{
  char format[10];
  char invalidMsg[4];
  uint16_t xPos;
  uint16_t yPos;
  uint8_t valid;
  int8_t data;
} DWint8_t;


typedef struct DWuint16_t_
{
  char format[10];
  char invalidMsg[4];
  uint16_t xPos;
  uint16_t yPos;
  uint8_t valid;
  uint16_t data;
} DWuint16_t;


typedef struct DWint16_t_
{
  char format[10];
  char invalidMsg[4];
  uint16_t xPos;
  uint16_t yPos;
  uint8_t valid;
  int16_t data;
} DWint16_t;


typedef struct DWstring_
{
  char format[10];
  char invalidMsg[4];
  uint16_t xPos;
  uint16_t yPos;
  uint8_t valid;
  char data[26];
} DWstring;


typedef struct DWfloat_
{
  char format[10];
  char invalidMsg[4];
  uint16_t xPos;
  uint16_t yPos;
  uint8_t valid;
  float data;
} DWfloat;


// Global variables
// live screen data variables
extern ui_screen screenNumber;
//extern DWuint8_t counter;
//extern DWint8_t counter;
// display data
extern DWfloat counter;
extern DWfloat tempInF;
extern DWfloat humidity;
extern DWint16_t tempCJ_F;

// Global Constants


// exposed function prototypes
void SwitchScreens(ui_screen screen_no);
//void PrintTest(char * formater, uint16_t variable, uint8_t position);
//uint8_t ProcessKeyCode (uint16_t key_code);
uint8_t ProcessKeyCodeInContext (uint16_t key_code);
void UpdateScreenValues(void);
uint8_t GetKeycode(void);

#endif