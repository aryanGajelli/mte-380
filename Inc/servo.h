#ifndef __SERVO_H__
#define __SERVO_H__

#include "stm32f4xx_hal.h"

#define CLAW_OPEN_ANGLE 20
#define CLAW_CLOSED_ANGLE 120

HAL_StatusTypeDef servoSetAngle(float angle);
HAL_StatusTypeDef servoSetDutyCycle(float dutyCycle);
float servoGetAngle();
HAL_StatusTypeDef servoInit();
#endif  // __SERVO_H__