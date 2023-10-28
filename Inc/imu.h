#ifndef __IMU_H__
#define __IMU_H__

// Header for icm20948 IMU https://invensense.tdk.com/wp-content/uploads/2016/06/DS-000189-ICM-20948-v1.3.pdf
#include "main.h"
#include "stm32f4xx_hal.h"

#include "ICM20948_register.h"

HAL_StatusTypeDef ICM_SelectBank(UserBankSel_E bank);
uint8_t ICM_GetBank();
void ICM_DisableI2C();
HAL_StatusTypeDef ICMInit();
#endif  // __IMU_H__
