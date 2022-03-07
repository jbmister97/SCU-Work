

#include "servo.h"

void Set_Servo_Position(uint16_t duty) {
  TIM_OC_InitTypeDef servoConfig = {0};
  
  servoConfig.OCMode = TIM_OCMODE_PWM1;
  servoConfig.Pulse = duty;
  servoConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  servoConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  servoConfig.OCFastMode = TIM_OCFAST_DISABLE;
  servoConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
  servoConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  
  HAL_TIM_PWM_Stop(&SERVO_TIMER, SERVO_TIMER_CH);
  HAL_TIM_PWM_ConfigChannel(&SERVO_TIMER, &servoConfig, SERVO_TIMER_CH);
  HAL_TIM_PWM_Start(&SERVO_TIMER, SERVO_TIMER_CH);
}