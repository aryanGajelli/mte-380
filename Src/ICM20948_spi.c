#include "ICM20948_spi.h"

#include "bsp.h"
#include "debug.h"
#include "imu.h"
#include "stm32f4xx_hal.h"

#define READ_EN(reg) SET_BIT((reg), 0x80)
#define WRITE_EN(reg) CLEAR_BIT((reg), 0x80)

// #define IMU_CS_EN() HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET)
// #define IMU_CS_DIS() HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET)

#define IMU_CS_EN() (IMU_CS_GPIO_Port->BSRR = IMU_CS_Pin << 16U)
#define IMU_CS_DIS() (IMU_CS_GPIO_Port->BSRR = IMU_CS_Pin)
/*
 *
 * SPI abstraction
 *
 */

HAL_StatusTypeDef ICM_ReadBytes(uint8_t reg, uint8_t *pData, size_t size)  // ***
{
    READ_EN(reg);
    if (HAL_SPI_TransmitReceive_DMA(&IMU_SPI_HANDLE, &reg, pData, size + 1) != HAL_OK) return HAL_ERROR;
    while (HAL_SPI_GetState(&IMU_SPI_HANDLE) != HAL_SPI_STATE_READY)
        ;
    IMU_CS_DIS();
    return HAL_OK;
}

HAL_StatusTypeDef ICM_WriteBytes(uint8_t reg, uint8_t *pData, size_t size)  // ***
{
    WRITE_EN(reg);
    if (HAL_SPI_Transmit_DMA(&IMU_SPI_HANDLE, &reg, 1) != HAL_OK) return HAL_ERROR;
    if (HAL_SPI_Transmit_DMA(&IMU_SPI_HANDLE, pData, size) != HAL_OK) return HAL_ERROR;
    while (HAL_SPI_GetState(&IMU_SPI_HANDLE) != HAL_SPI_STATE_READY)
        ;
    IMU_CS_DIS();
    return HAL_OK;
}

HAL_StatusTypeDef ICM_ReadOneByte(uint8_t reg, uint8_t *pData) {
    READ_EN(reg);
    static uint8_t rData[2];
    if (HAL_SPI_TransmitReceive_DMA(&IMU_SPI_HANDLE, &reg, rData, 2) != HAL_OK) return HAL_ERROR;
    IMU_CS_DIS();
    *pData = rData[1];
    return HAL_OK;
}

HAL_StatusTypeDef ICM_WriteOneByte(uint8_t reg, uint8_t data) {
    WRITE_EN(reg);
    static uint8_t tData[2];
    tData[0] = reg;
    tData[1] = data;
    if (HAL_SPI_Transmit_DMA(&IMU_SPI_HANDLE, tData, 2) != HAL_OK) {
        return HAL_ERROR;
    }
    IMU_CS_DIS();
    return HAL_OK;
}