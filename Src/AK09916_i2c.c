#include "AK09916_i2c.h"
#include "AK09916_register.h"
#include "stm32f4xx_hal.h"
#include "ICM20948_register.h"
#include "ICM20948_spi.h"
#include "imu.h"

#define READ_EN(reg) ((reg) | 0x80)
#define WRITE_EN(reg) ((reg) & (~0x80))
/*
 *
 * AUX I2C abstraction for magnetometer
 *
 */

/**
 * @brief Read bytes from magnetometer on internal I2C line. The data will be stored in a separate register on the ICM
 * @param start_reg: register address to start reading from
 * @param size: number of bytes to read
 */
HAL_StatusTypeDef AK0_ReadRegisters(uint8_t start_reg, size_t size) {
    if (expected_CurrUserBank != USER_BANK_3) {
        if (ICM_SelectBank(USER_BANK_3) != HAL_OK) return HAL_ERROR;
    }
    if (ICM_WriteOneByte(I2C_SLV0_ADDR_REG, READ_EN(AK09916_ADDR)) != HAL_OK) return HAL_ERROR;
    if (ICM_WriteOneByte(I2C_SLV0_REG_REG, start_reg) != HAL_OK) return HAL_ERROR;
    HAL_Delay(50);
    ICM_WriteOneByte(I2C_SLV0_CTRL_REG, READ_EN(size));
    HAL_Delay(50);
    return HAL_OK;
}

/**
 * @brief Write one byte from magnetometer on internal I2C line
 * @param reg: magnetometer register address to write from
 * @param data: value to write to magnetometer register
 */
HAL_StatusTypeDef AK0_WriteOneByte(uint8_t reg, uint8_t data) {
    if (expected_CurrUserBank != USER_BANK_3) {
        ICM_SelectBank(USER_BANK_3);
    }
    if (ICM_WriteOneByte(I2C_SLV0_ADDR_REG, WRITE_EN(AK09916_ADDR)) != HAL_OK) return HAL_ERROR;
    if (ICM_WriteOneByte(I2C_SLV0_REG_REG, reg) != HAL_OK) return HAL_ERROR;
    if (ICM_WriteOneByte(I2C_SLV0_DO_REG, data) != HAL_OK) return HAL_ERROR;
    HAL_Delay(50);
    ICM_WriteOneByte(I2C_SLV0_CTRL_REG, READ_EN(1));
    HAL_Delay(50);
    return HAL_OK;
}