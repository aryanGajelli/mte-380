#ifndef __ICM20948_SPI_H__
#define __ICM20948_SPI_H__

#include "stm32f4xx_hal.h"
HAL_StatusTypeDef ICM_ReadBytes(uint8_t reg, uint8_t* pData, size_t size);

HAL_StatusTypeDef ICM_WriteBytes(uint8_t reg, uint8_t* pData, size_t size);

HAL_StatusTypeDef ICM_ReadOneByte(uint8_t reg, uint8_t* pData);
HAL_StatusTypeDef ICM_WriteOneByte(uint8_t reg, uint8_t data);
#endif // __ICM20948_SPI_H__