#include "imu.h"

#include "bsp.h"
#include "debug.h"
#include "stm32f4xx_hal.h"

#define READ_EN(reg) ((reg) |= 0x80)
#define WRITE_EN(reg) ((reg) &= 0x7F)

#define IMU_CS_EN() HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET)
#define IMU_CS_DIS() HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET)
/*
 *
 * SPI abstraction
 *
 */
void ICM_ReadBytes(uint8_t reg, uint8_t *pData, size_t size) // ***
{
	READ_EN(reg);
	IMU_CS_EN();
	HAL_SPI_Transmit_DMA(&IMU_SPI_HANDLE, &reg, 1);
	HAL_SPI_Receive_DMA(&IMU_SPI_HANDLE, pData, size);
	IMU_CS_DIS();
}

void ICM_WriteBytes(uint8_t reg, uint8_t *pData, size_t size) // ***
{
	WRITE_EN(reg);
	IMU_CS_EN();
	HAL_SPI_Transmit_DMA(&IMU_SPI_HANDLE, &reg, 1);
	HAL_SPI_Transmit_DMA(&IMU_SPI_HANDLE, pData, size);
	IMU_CS_DIS();
}


HAL_StatusTypeDef ICM_ReadOneByte(uint8_t reg, uint8_t* pData) {
    READ_EN(reg);
    IMU_CS_EN();
    if (HAL_SPI_Transmit_DMA(&IMU_SPI_HANDLE, &reg, 1) != HAL_OK) {
        return HAL_ERROR;
    }
    if(HAL_SPI_Receive_DMA(&IMU_SPI_HANDLE, pData, 1) != HAL_OK) {
        return HAL_ERROR;
    }
    IMU_CS_DIS();
    return HAL_OK;
}

HAL_StatusTypeDef ICM_WriteOneByte(uint8_t reg, uint8_t data) {
    WRITE_EN(reg);
    uint8_t tData[2] = {reg, data};
    IMU_CS_EN();
    if (HAL_SPI_Transmit_DMA(&IMU_SPI_HANDLE, tData, 2) != HAL_OK) {
        return HAL_ERROR;
    }
    IMU_CS_DIS();
    return HAL_OK;
}