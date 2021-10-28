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

// Display-wrapped values
// format seq (numeric): {<format string>, <error message>, <Xpos>, <Ypos>, <valid?>, <init value>}
DWfloat counter = {"%5.2f", "----", 0, 0, true, 0};
DWfloat tempInF = {"%4.1f", "----", 0, 0, true, 72.2};
DWfloat humidity = {"%4.1f", "----", 0, 0, true, 40.1};
DWint16_t tempCJ_F = {"%5d", "!!!!", 0, 0, true, 0};
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
  case MAIN:
    // clear the screen from the previos dispayed data
    SSD1306_Clear();
    // Put up the "persistant" info (like data labels)
    SSD1306_GotoXY (0,0);
    SSD1306_Puts ("Main Screen", &Font_11x18, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY (0, 30);
    SSD1306_Puts ("Temp: ", &Font_11x18, SSD1306_COLOR_WHITE);
    // Set u X/Y coordinates for "live" data to be displayed on this screen
    tempInF.xPos = 55;
    tempInF.yPos = 30;
    // Send a screen update (note this does not update the live data)
    break;
  case SHOW_TEMP:
    SSD1306_Clear();
    SSD1306_GotoXY (0,0);
    SSD1306_Puts ("Ambient Tmp", &Font_11x18, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY (0, 20);
    SSD1306_Puts ("DegF: ", &Font_11x18, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY (0, 40);
    SSD1306_Puts ("Count: ", &Font_11x18, SSD1306_COLOR_WHITE);
    tempInF.xPos = 55;
    tempInF.yPos = 20;
    counter.xPos = 60;
    counter.yPos = 40;
    break;
  }
  
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
  case  MAIN:
    switch (key_code) {
    case 0:
      SwitchScreens(SHOW_TEMP);
      break;
    case 1:
      break;
    case 2:
      break;
    case 3:
      break;
    }
    break;
  case  SHOW_TEMP:
    switch (key_code) {
    case 0:
      SwitchScreens(MAIN);
      break;
    case 1:
      break;
    case 2:
      break;
    case 3:
      break;
    }
    break;
  case  SHOW_HUM:
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
  case  SET_TEMP:
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
  case  SET_HUM:
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
  case MAIN:
    SSD1306_GotoXY (tempInF.xPos, tempInF.yPos);
    if (tempInF.valid) {
      sprintf(displayString, tempInF.format, tempInF.data);
      SSD1306_Puts(displayString, &Font_11x18, SSD1306_COLOR_WHITE);
    }
    else 
      SSD1306_Puts(tempInF.invalidMsg, &Font_11x18, SSD1306_COLOR_WHITE);
    break;
  case SHOW_TEMP:
    SSD1306_GotoXY (tempInF.xPos, tempInF.yPos);
    if (tempInF.valid) {
      sprintf(displayString, tempInF.format, tempInF.data);
      SSD1306_Puts(displayString, &Font_11x18, SSD1306_COLOR_WHITE);
    }
    else 
      SSD1306_Puts(tempInF.invalidMsg, &Font_11x18, SSD1306_COLOR_WHITE);
    
    SSD1306_GotoXY (counter.xPos, counter.yPos);
    if (counter.valid) {
      sprintf(displayString, counter.format, counter.data);
      SSD1306_Puts(displayString, &Font_11x18, SSD1306_COLOR_WHITE);
    }
    else
      SSD1306_Puts(counter.invalidMsg, &Font_11x18, SSD1306_COLOR_WHITE);
    break;
  case SHOW_HUM:
    break;
  case SET_TEMP:
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