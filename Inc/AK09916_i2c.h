#ifndef __AK09916_I2C_H__
#define __AK09916_I2C_H__

#include "stm32f4xx_hal.h"

HAL_StatusTypeDef AK0_ReadRegisters(uint8_t start_reg, size_t size);
HAL_StatusTypeDef AK0_WriteOneByte(uint8_t reg, uint8_t data);

#endif // __AK09916_I2C_H__