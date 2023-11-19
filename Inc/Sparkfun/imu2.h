#ifndef __IMU2_H__
#define __IMU2_H__

#include "stm32f4xx_hal.h"
#include "Sparkfun/ICM_20948_C.h"
#include "Sparkfun/ICM_20948_REGISTERS.h"

HAL_StatusTypeDef imuInit();

#endif  // __IMU2_H__