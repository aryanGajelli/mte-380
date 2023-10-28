#ifndef __IMU_H__
#define __IMU_H__

// Header for icm20948 IMU https://invensense.tdk.com/wp-content/uploads/2016/06/DS-000189-ICM-20948-v1.3.pdf
#include "main.h"
#include "stm32f4xx_hal.h"

#include "ICM20948_register.h"

HAL_StatusTypeDef ICM_SelectBank(UserBankSel_E bank);
HAL_StatusTypeDef ICM_GetBank(UserBankSel_E userBank);
HAL_StatusTypeDef ICM_DisableI2C();
HAL_StatusTypeDef ICM_SetClock(ClockSel_E clockSel);
HAL_StatusTypeDef ICM_AccelGyroOff(void);
HAL_StatusTypeDef ICM_AccelGyroOn(void);
HAL_StatusTypeDef ICM_WhoAmI(uint8_t* whoami);
HAL_StatusTypeDef ICMInit();
#endif  // __IMU_H__
