
#include "main.h"

#define SERVO_STATE_ZERO_POSITION       0
#define SERVO_STATE_NEG_90_POSITION     1
#define SERVO_STATE_POS_90_POSITION     2
#define LEFT                            0
#define RIGHT                           1
#define MID                             2
#define SERVO_ZERO_POSITION_VALUE       760
#define SERVO_NEG_90_POSITION_VALUE     (SERVO_ZERO_POSITION_VALUE-25)
//#define SERVO_NEG_90_POSITION_VALUE     300
//#define SERVO_POS_90_POSITION_VALUE     1050
#define SERVO_POS_90_POSITION_VALUE     (SERVO_ZERO_POSITION_VALUE+25)

// This timer must be changed depending on the board configuration
extern TIM_HandleTypeDef htim3;
#define SERVO_TIMER             htim3
#define SERVO_TIMER_CH          TIM_CHANNEL_1

