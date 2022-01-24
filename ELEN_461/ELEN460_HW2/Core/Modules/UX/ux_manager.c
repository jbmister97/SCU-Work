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
char endC[4] = "C";
char endF[4] = "F";
char testStr[25];

uint8_t degOffset = 0;
//extern uint8_t timeout_flag;

// Display-wrapped values
// format seq (numeric): {<format string>, <error message>, <Xpos>, <Ypos>, <valid?>, <init value>}
//DWfloat counter = {"%5.2f", "----", 0, 0, true, 0};
//DWfloat tempInF = {"%4.1f", "----", 0, 0, true, 72.2};
//DWfloat humidity = {"%4.1f", "----", 0, 0, true, 40.1};
//DWint16_t tempCJ_F = {"%5d", "!!!!", 0, 0, true, 0};
extern DWfloat temperature;
extern DWstring units;
//extern DWfloat target;
extern uint8_t unitChoices[2][5];
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
    // Set screen ltitle
    SSD1306_GotoXY (0,0);
    SSD1306_Puts ("Temperature", &Font_11x18, SSD1306_COLOR_WHITE);
    
    // Set temperature reading
    /*
    SSD1306_GotoXY (0, 20);
    SSD1306_Puts ("Temp:", &Font_11x18, SSD1306_COLOR_WHITE);
    */
    //temperature.xPos = 31;
    //temperature.yPos = 30;
    //temperature.xPos = 55;
    temperature.xPos = 33;
    temperature.yPos = 20;
    
    /*
    // Set target temperature
    SSD1306_GotoXY (0, 40);
    SSD1306_Puts ("Target:", &Font_7x10, SSD1306_COLOR_WHITE);
    target.xPos = 49;
    target.yPos = 40;
    */
    
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
  case HOME:            // Update temperature screen values
    // Update temperature value
    SSD1306_GotoXY (temperature.xPos, temperature.yPos);
    if (temperature.valid) {
      //for(uint8_t i = 0; i < 25; i++) {displayString[i] = '\0';}
      sprintf(displayString, temperature.format, temperature.data);
      strcpy(testStr,displayString);
      //if(temperature.data < 100.0) {displayString[4] = '\0';}
      SSD1306_Puts(displayString, &Font_11x18, SSD1306_COLOR_WHITE);
    }
    else 
      SSD1306_Puts(temperature.invalidMsg, &Font_11x18, SSD1306_COLOR_WHITE);
    
    SSD1306_GotoXY ((temperature.xPos + 55), temperature.yPos);
    if (units.data[3] == 'C') {SSD1306_Puts(endC, &Font_11x18, SSD1306_COLOR_WHITE);}
    else {SSD1306_Puts(endF, &Font_11x18, SSD1306_COLOR_WHITE);}
    
    /*
    // Update target temperature
    SSD1306_GotoXY (target.xPos, target.yPos);
    if (target.valid) {
      sprintf(displayString, target.format, target.data);
      SSD1306_Puts(displayString, &Font_7x10, SSD1306_COLOR_WHITE);
    }
    else 
      SSD1306_Puts(target.invalidMsg, &Font_7x10, SSD1306_COLOR_WHITE);
    
    SSD1306_GotoXY ((target.xPos + 35), target.yPos);
    if (units.data[3] == 'C') {SSD1306_Puts(endC, &Font_7x10, SSD1306_COLOR_WHITE);}
    else {SSD1306_Puts(endF, &Font_7x10, SSD1306_COLOR_WHITE);}
    break;
    */
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