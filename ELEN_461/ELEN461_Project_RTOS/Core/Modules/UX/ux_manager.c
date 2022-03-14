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
//char endC[4] = "C";
//char endF[4] = "F";
char testStr[25];

uint8_t degOffset = 0;
//extern uint8_t timeout_flag;

// Display-wrapped values
// format seq (numeric): {<format string>, <error message>, <Xpos>, <Ypos>, <valid?>, <init value>}
//DWfloat counter = {"%5.2f", "----", 0, 0, true, 0};
//DWfloat tempInF = {"%4.1f", "----", 0, 0, true, 72.2};
//DWfloat humidity = {"%4.1f", "----", 0, 0, true, 40.1};
//DWint16_t tempCJ_F = {"%5d", "!!!!", 0, 0, true, 0};
//extern DWfloat target;
//extern DWfloat distance;
extern DWuint16_t target;
extern DWuint16_t distance;
//extern uint8_t processKeyCode;
//extern uint8_t keyCodeProcessed;
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
  case HOME:            // Temperature screen
    // clear the screen from the previos dispayed data
    SSD1306_Clear();
    
    // Set title
    SSD1306_GotoXY (18,0);
    SSD1306_Puts ("Distance", &Font_11x18, SSD1306_COLOR_WHITE);
    
    // Set measured distance label
    SSD1306_GotoXY (0,20);
    SSD1306_Puts ("Current:", &Font_7x10, SSD1306_COLOR_WHITE);
    distance.xPos = 63;
    distance.yPos = 20;
    
    // Set Distance units
    SSD1306_GotoXY (105,20);
    SSD1306_Puts ("cm", &Font_7x10, SSD1306_COLOR_WHITE);
    
    // Set Target label
    SSD1306_GotoXY (0,35);
    SSD1306_Puts ("Target:", &Font_7x10, SSD1306_COLOR_WHITE);
    target.xPos = 63;
    target.yPos = 35;
    
    // Set Target units
    SSD1306_GotoXY (105,35);
    SSD1306_Puts ("cm", &Font_7x10, SSD1306_COLOR_WHITE);
    
    // Send a screen update (note this does not update the live data)
    break;
  case DETAIL:          // Target screen
    
    break;
  case SETTINGS:        // Settings screen
    break;
  case SET_TEMP:        // Set custom temperature screen
    
    break;
  }
  
  //timeout_flag = 1;
  SSD1306_UpdateScreen(); //display
  currentScreen = screen_no;
  
#pragma diag_suppress= Pa149
  // what must be done after screen is switched in
  switch (currentScreen) {
  }
#pragma diag_warning= Pa149
  
}

// context sensitive keyboard processor
uint8_t ProcessKeyCodeInContext (uint16_t key_code)
{
  switch (currentScreen) {
  case  HOME:           // Temperature screen
    switch (key_code) {
    case 0:
      //SwitchScreens(HOME);
      break;
    case 1:
      //SwitchScreens(DETAIL);
      break;
    case 2:
      break;
    case 3:
      break;
    case 4:
      break;
    }
    break;
  case  DETAIL:         // Target screen
    switch (key_code) {
    case 0:
      break;
    case 1:
      break;
    case 2:
      break;
    case 3:
      break;
    case 4:
      break;
    }
    break;
  case  SETTINGS:       // Settings screen
    switch (key_code) {
    case 0:
      break;
    case 1:
      break;
    case 2:
      break;
    case 3:
      break;
    case 4:
      break;
    }
    break;
  case  SET_TEMP:       // Custom temperature screen
    // Reset temps to custom base values
    
    switch (key_code) {
    case 0:
      break;
    case 1:
      break;
    case 2:
      break;
    case 3:
      break;
    case 4:
      break;
    break;
    }
  }
  
  // Reset screen timeout when button is pressed
  //timeout_flag = 1;
  
  //processKeyCode = false;
  //keyCodeProcessed = true;
  return true;
}

void UpdateScreenValues(void)
{
  char displayString[25];
  
  
  switch (currentScreen) {
  case HOME:            // Update target screen values
    // Update target value
    SSD1306_GotoXY (target.xPos, target.yPos);
    if (target.valid) {
      sprintf(displayString, target.format, target.data);
      strcpy(testStr,displayString);
      SSD1306_Puts(displayString, &Font_7x10, SSD1306_COLOR_WHITE);
    }
    else 
      SSD1306_Puts(target.invalidMsg, &Font_7x10, SSD1306_COLOR_WHITE);
    
    // Update distance value
    SSD1306_GotoXY (distance.xPos, distance.yPos);
    if (distance.valid) {
      sprintf(displayString, distance.format, distance.data);
      strcpy(testStr,displayString);
      SSD1306_Puts(displayString, &Font_7x10, SSD1306_COLOR_WHITE);
    }
    else 
      SSD1306_Puts(distance.invalidMsg, &Font_7x10, SSD1306_COLOR_WHITE);

  case DETAIL:          // Update target screen values

    break;
  case SETTINGS:        // Update setting screen values
    break;
  case SET_TEMP:        // Update set custom temperature screen values
    break;
  }
  SSD1306_UpdateScreen(); //display
}



uint8_t GetKeycode(void)
{
  return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
}