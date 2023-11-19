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
ICM_20948_Status_e read(ICM_20948_AGMT_t *agmt);
bool imuIsDataReady();
ICM_20948_Status_e spi_read(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user);
ICM_20948_Status_e imuRead(IMUData_T *imuData);
ICM_20948_Status_e imuLoadScale();
ICM_20948_Status_e imuIntEnableRawDataReady(bool enable);
#endif  // __IMU2_H__