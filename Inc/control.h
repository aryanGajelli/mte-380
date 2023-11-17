#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "stm32f4xx_hal.h"
HAL_StatusTypeDef controlInit();
void controlTurnToHeading(double targetDeg);
void controlMoveForward(double targetDist);
#endif  // __CONTROL_H__