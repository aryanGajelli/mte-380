#ifndef __IMU2_H__
#define __IMU2_H__

#include "Sparkfun/ICM_20948_C.h"
#include "Sparkfun/ICM_20948_REGISTERS.h"
#include "stm32f4xx_hal.h"
#include "imu.h"
extern ICM_20948_Device_t imu;
HAL_StatusTypeDef imuInit();
ICM_20948_Device_t *imuGetInstance();
void imuScaleAndAssign(IMUData_T *imuData, ICM_20948_AGMT_t *agmt);
ICM_20948_Status_e imuRead(IMUData_T *imuData);
#endif  // __IMU2_H__