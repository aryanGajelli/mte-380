#ifndef __IMU2_H__
#define __IMU2_H__

#include "Sparkfun/ICM_20948_C.h"
#include "Sparkfun/ICM_20948_REGISTERS.h"
#include "stm32f4xx_hal.h"
extern ICM_20948_Device_t imu;
HAL_StatusTypeDef imuInit();
ICM_20948_Device_t *imuGetInstance();
#endif  // __IMU2_H__