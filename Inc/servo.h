#ifndef __SERVO_H__
#define __SERVO_H__

#include "stm32f4xx_hal.h"

HAL_StatusTypeDef setServoAngle(float angle);
HAL_StatusTypeDef setServoDutyCycle(float dutyCycle);
HAL_StatusTypeDef servoInit();
#endif  // __SERVO_H__