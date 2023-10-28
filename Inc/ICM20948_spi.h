#ifndef __ICM20948_SPI_H__
#define __ICM20948_SPI_H__

HAL_StatusTypeDef ICM_ReadOneByte(uint8_t reg, uint8_t* pData);
HAL_StatusTypeDef ICM_WriteOneByte(uint8_t reg, uint8_t data);
#endif // __ICM20948_SPI_H__