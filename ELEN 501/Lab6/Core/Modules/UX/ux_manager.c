// ux_manager.c


#include "main.h"
#include "ux_manager.h"
#include <stdio.h>

// private defines


// Global Constants


// Modular Constants


// global variables
// Screens
ui_screen currentScreen;
ui_screen lastScreen;
char endC[4] = " C ";
char endF[4] = " F ";

uint8_t degOffset = 0;
extern uint8_t timeout_flag;

// Display-wrapped values
// format seq (numeric): {<format string>, <error message>, <Xpos>, <Ypos>, <valid?>, <init value>}
DWfloat counter = {"%5.2f", "----", 0, 0, true, 0};
DWfloat tempInF = {"%4.1f", "----", 0, 0, true, 72.2};
DWfloat humidity = {"%4.1f", "----", 0, 0, true, 40.1};
DWint16_t tempCJ_F = {"%5d", "!!!!", 0, 0, true, 0};
extern DWfloat temperature;
extern DWint8_t count;
extern DWstring units;
extern DWstring message;
extern uint8_t unitChoices[2][5];
// format seq (string): {<format string>,  <error message>, <Xpos>, <Ypos>, <valid?>, "<init value>"


// modular variables


// module prototypes



// ***************
// Start Of Code
// ***************
// Screen switching utility that manages pre-, post-, and screen switch conditions
void SwitchScreens(ui_screen screen_no)
{
  lastScreen = currentScreen;

  
#pragma diag_suppress= Pa149
  // what must be done before current screen is switched out
  switch (lastScreen) {
  }
  
  
  // what must be done before screen is switched in
  switch (screen_no) {
  }
#pragma diag_warning= Pa149
  
  // Switch the screens
  switch (screen_no) {
  case HOME:
    // clear the screen from the previos dispayed data
    SSD1306_Clear();
    // Put up the "persistant" info (like data labels)
    SSD1306_GotoXY (0,0);
    SSD1306_Puts ("Home Screen", &Font_11x18, SSD1306_COLOR_WHITE);
    // Set u X/Y coordinates for "live" data to be displayed on this screen
    
    temperature.xPos = 31;
    temperature.yPos = 30;
    
    if(units.data[3] == 'C') {
      for(uint8_t i = 0; i < 4; i++) {units.data[i] = unitChoices[1][i];}
      for(uint8_t i = 4; i < 26; i++) {units.data[i] = '\0';}
    }
    else {
      for(uint8_t i = 0; i < 4; i++) {units.data[i] = unitChoices[0][i];}
      for(uint8_t i = 4; i < 26; i++) {units.data[i] = '\0';}
    }
    // Send a screen update (note this does not update the live data)
    break;
  case DETAIL:
    SSD1306_Clear();
    SSD1306_GotoXY (0,0);
    SSD1306_Puts ("Details", &Font_11x18, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY (0, 20);
    SSD1306_Puts ("Temp: ", &Font_11x18, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY (0, 40);
    SSD1306_Puts ("Hum: ", &Font_11x18, SSD1306_COLOR_WHITE);
    temperature.xPos = 55;
    temperature.yPos = 20;
    humidity.xPos = 45;
    humidity.yPos = 40;
    break;
  case SETTINGS:
    SSD1306_Clear();
    SSD1306_GotoXY (0,0);
    SSD1306_Puts ("Settings", &Font_11x18, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY (0, 30);
    SSD1306_Puts ("Units: ", &Font_11x18, SSD1306_COLOR_WHITE);
    units.xPos = 65;
    units.yPos = 30;
    break;
  case MESSAGE:
    SSD1306_Clear();
    SSD1306_GotoXY (0,0);
    SSD1306_Puts ("Messages", &Font_11x18, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY (0, 40);
    SSD1306_Puts ("Count: ", &Font_11x18, SSD1306_COLOR_WHITE);
    count.xPos = 60;
    count.yPos = 40;
    message.xPos = 14;
    message.yPos = 20;
    break;
  }
  
  timeout_flag = 1;
  SSD1306_UpdateScreen(); //display
  currentScreen = screen_no;
  
#pragma diag_suppress= Pa149
  // what must be done after screen is switched in
  switch (currentScreen) {
  }
#pragma diag_warning= Pa149
  
}


//// Keyboard Processor
//
//uint8_t ProcessKeyCode (uint16_t key_code)
//{
//  switch (key_code) {
//  case 0:
//    break;
//  case 1:
//    break;
//  case 2:
//    break;
//  case 3:
//    break;
//  }
//  
//  return true;
//}


// context sensitive keyboard processor
uint8_t ProcessKeyCodeInContext (uint16_t key_code)
{
  switch (currentScreen) {
  case  HOME:
    switch (key_code) {
    case 0:
      SwitchScreens(HOME);
      break;
    case 1:
      SwitchScreens(DETAIL);
      break;
    case 2:
      break;
    case 3:
      break;
    }
    break;
  case  DETAIL:
    switch (key_code) {
    case 0:
      SwitchScreens(HOME);
      break;
    case 1:
      SwitchScreens(SETTINGS);
      break;
    case 2:
      break;
    case 3:
      break;
    }
    break;
  case  SETTINGS:
    switch (key_code) {
    case 0:
      SwitchScreens(HOME);
      break;
    case 1:
      SwitchScreens(HOME);
      break;
    case 2:
      break;
    case 3:
      break;
    }
    break;
  case  MESSAGE:
    switch (key_code) {
    case 0:
      SwitchScreens(HOME);
      break;
    case 1:
      break;
    case 2:
      break;
    case 3:
      break;
    }
    break;
  case  SET_HUM:
    switch (key_code) {
    case 0:
      SwitchScreens(HOME);
      break;
    case 1:
      break;
    case 2:
      break;
    case 3:
      break;
    }
    break;
  case  SET_TIME:   
    switch (key_code) {
    case 0:
      break;
    case 1:
      break;
    case 2:
      break;
    case 3:
      break;
    }
    break;
  }
  
  return true;
}




void UpdateScreenValues(void)
{
  char displayString[25];
  
  
  switch (currentScreen) {
  case HOME:
    if(temperature.data >= 0) {
      temperature.xPos = 31;
      SSD1306_GotoXY (0, temperature.yPos); // to clear negative sign location
      SSD1306_Puts(displayString, &Font_11x18, SSD1306_COLOR_WHITE);
      degOffset = 44;
    }
    else {
      degOffset = 55;
      temperature.xPos = 20;
    }
    SSD1306_GotoXY (temperature.xPos, temperature.yPos);
    if (temperature.valid) {
      sprintf(displayString, temperature.format, temperature.data);
      SSD1306_Puts(displayString, &Font_11x18, SSD1306_COLOR_WHITE);
    }
    else 
      SSD1306_Puts(temperature.invalidMsg, &Font_11x18, SSD1306_COLOR_WHITE);
    
    SSD1306_GotoXY ((temperature.xPos + degOffset), temperature.yPos);
    if (units.data[3] == 'C') {SSD1306_Puts(endC, &Font_11x18, SSD1306_COLOR_WHITE);}
    else {SSD1306_Puts(endF, &Font_11x18, SSD1306_COLOR_WHITE);}
    break;
  case DETAIL:
    SSD1306_GotoXY (temperature.xPos, temperature.yPos);
    if (temperature.valid) {
      sprintf(displayString, temperature.format, temperature.data);
      SSD1306_Puts(displayString, &Font_11x18, SSD1306_COLOR_WHITE);
    }
    else 
      SSD1306_Puts(temperature.invalidMsg, &Font_11x18, SSD1306_COLOR_WHITE);
    
    SSD1306_GotoXY ((temperature.xPos + degOffset), temperature.yPos);
    if (units.data[3] == 'C') {SSD1306_Puts(endC, &Font_11x18, SSD1306_COLOR_WHITE);}
    else {SSD1306_Puts(endF, &Font_11x18, SSD1306_COLOR_WHITE);}
    
    SSD1306_GotoXY (humidity.xPos, humidity.yPos);
    if (humidity.valid) {
      sprintf(displayString, humidity.format, humidity.data);
      SSD1306_Puts(displayString, &Font_11x18, SSD1306_COLOR_WHITE);
    }
    else
      SSD1306_Puts(humidity.invalidMsg, &Font_11x18, SSD1306_COLOR_WHITE);
    break;
  case SETTINGS:
    SSD1306_GotoXY (units.xPos, units.yPos);
    if (units.valid) {
      sprintf(displayString, units.format, units.data);
      SSD1306_Puts(displayString, &Font_11x18, SSD1306_COLOR_WHITE);
    }
    else 
      SSD1306_Puts(units.invalidMsg, &Font_11x18, SSD1306_COLOR_WHITE);
    break;
  case MESSAGE:
    SSD1306_GotoXY (message.xPos, message.yPos);
    if (message.valid) {
      sprintf(displayString, message.format, message.data);
      SSD1306_Puts(displayString, &Font_11x18, SSD1306_COLOR_WHITE);
    }
    else 
      SSD1306_Puts(count.invalidMsg, &Font_11x18, SSD1306_COLOR_WHITE);
    
    SSD1306_GotoXY (count.xPos, count.yPos);
    if (count.valid) {
      sprintf(displayString, count.format, count.data);
      SSD1306_Puts(displayString, &Font_11x18, SSD1306_COLOR_WHITE);
    }
    else 
      SSD1306_Puts(count.invalidMsg, &Font_11x18, SSD1306_COLOR_WHITE);
    break;
  case SET_HUM:
    break;
  case SET_TIME:
    break;
  }
  SSD1306_UpdateScreen(); //display
}



uint8_t GetKeycode(void)
{
  return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
}