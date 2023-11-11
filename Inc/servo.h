#ifndef __SERVO_H__
#define __SERVO_H__

#include "stm32f4xx_hal.h"

#define CLAW_OPEN_ANGLE 30
#define CLAW_CLOSED_ANGLE 140

HAL_StatusTypeDef setServoAngle(float angle);
HAL_StatusTypeDef setServoDutyCycle(float dutyCycle);
float getServoAngle();
HAL_StatusTypeDef servoInit();
#endif  // __SERVO_H__