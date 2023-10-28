#include "imu.h"

#include "ICM20948_spi.h"
#include "bsp.h"
#include "debug.h"
#include "stm32f4xx_hal.h"

UserBankSel_E expected_CurrUserBank = USER_BANK_ERROR;

HAL_StatusTypeDef ICMInit() {
    uint8_t data;
    // user bank 0 contains critical configuration registers
    if (ICM_SelectBank(USER_BANK_0) != HAL_OK) return HAL_ERROR;
    // check who am i register
    if (ICM_WhoAmI(&data) != HAL_OK) return HAL_ERROR;
    if (data != WHO_AM_I_RESET_VAL) {
        uprintf("ICM20948 WHO_AM_I returned: 0x%02x\n", data);
        return HAL_ERROR;
    }

    if (ICM_DisableI2C() != HAL_OK) return HAL_ERROR;
    if (ICM_SetClock(CLK_BEST_AVAIL) != HAL_OK) return HAL_ERROR;

    if (ICM_AccelGyroInit() != HAL_OK) return HAL_ERROR;
    
    return HAL_OK;
}

HAL_StatusTypeDef ICM_AccelGyroInit() {
    if (ICM_AccelGyroOff() != HAL_OK) return HAL_ERROR;
    if (ICM_AccelGyroOn() != HAL_OK) return HAL_ERROR;

    if (ICM_SelectBank(USER_BANK_2) != HAL_OK) return HAL_ERROR;
    if (ICM_SetGyroRateLPF(GYRO_RATE_250, GYRO_LPF_17HZ) != HAL_OK) return HAL_ERROR;

    if (ICM_SetGyroSampleRate(100) != HAL_OK) return HAL_ERROR;

    // Set accelerometer low pass filter to 136hz (0x11) and the rate to 8G (0x04) in register ACCEL_CONFIG (0x14)
    ICM_WriteOneByte(0x14, (0x04 | 0x11));

    // Set accelerometer sample rate to 225hz (0x00) in ACCEL_SMPLRT_DIV_1 register (0x10)
    ICM_WriteOneByte(0x10, 0x00);
    HAL_Delay(10);

    // Set accelerometer sample rate to 100 hz (0x0A) in ACCEL_SMPLRT_DIV_2 register (0x11)
    ICM_WriteOneByte(0x11, 0x0A);
    return HAL_OK;
}

HAL_StatusTypeDef ICM_SelectBank(UserBankSel_E userBank) {
    if (userBank > USER_BANK_3) {
        return HAL_ERROR;
    }
    HAL_StatusTypeDef ret = ICM_WriteOneByte(USER_BANK_SEL_REG, userBank << 4);
    if (ret != HAL_OK) {
        expected_CurrUserBank = USER_BANK_ERROR;
        return ret;
    }
    expected_CurrUserBank = userBank;
    return HAL_OK;
}

HAL_StatusTypeDef ICM_GetBank(UserBankSel_E userBank) {
    uint8_t data;
    HAL_StatusTypeDef ret = ICM_ReadOneByte(USER_BANK_SEL_REG, &data);
    if (ret != HAL_OK) {
        expected_CurrUserBank = userBank = USER_BANK_ERROR;
        return ret;
    }
    expected_CurrUserBank = userBank = (UserBankSel_E)(data >> 4);
    return HAL_OK;
}

HAL_StatusTypeDef ICM_DisableI2C() {
    if (expected_CurrUserBank != USER_BANK_0) {
        return HAL_ERROR;
    }
    static const uint8_t FIFO_EN = 0x40,
                         I2C_IF_DIS = 0x10,
                         DMP_RST = 0x08;
    return ICM_WriteOneByte(USER_CTRL_REG, FIFO_EN | I2C_IF_DIS | DMP_RST);
}

HAL_StatusTypeDef ICM_SetClock(ClockSel_E clockSel) {
    if (expected_CurrUserBank != USER_BANK_0) {
        return HAL_ERROR;
    }
    return ICM_WriteOneByte(PWR_MGMT_1_REG, (uint8_t)clockSel);
}

HAL_StatusTypeDef ICM_AccelGyroOff() {
    if (expected_CurrUserBank != USER_BANK_0) {
        return HAL_ERROR;
    }
    static const uint8_t ACCEL_OFF = 0x38, GYRO_OFF = 0x07;
    return ICM_WriteOneByte(PWR_MGMT_2_REG, ACCEL_OFF | GYRO_OFF);
}

HAL_StatusTypeDef ICM_AccelGyroOn() {
    if (expected_CurrUserBank != USER_BANK_0) {
        return HAL_ERROR;
    }
    static const uint8_t ACCEL_GYRO_ON = 0x00;
    return ICM_WriteOneByte(PWR_MGMT_2_REG, ACCEL_GYRO_ON);
}

HAL_StatusTypeDef ICM_WhoAmI(uint8_t *whoami) {
    if (expected_CurrUserBank != USER_BANK_0) {
        return HAL_ERROR;
    }
    return ICM_ReadOneByte(WHO_AM_I_REG, whoami);
}

HAL_StatusTypeDef ICM_SetGyroRateLPF(GyroRate_E gyroRate, GyroLPF_E gyroLPF) {
    if (expected_CurrUserBank != USER_BANK_2) {
        return HAL_ERROR;
    }
    return ICM_WriteOneByte(GYRO_CONFIG_1_REG, (gyroRate << 1) | gyroLPF);
}

// sample rate in Hz
HAL_StatusTypeDef ICM_SetGyroSampleRate(float gyroSampleRate) {
    if (expected_CurrUserBank != USER_BANK_2) {
        return HAL_ERROR;
    }

    if (gyroSampleRate < 4.3 || gyroSampleRate > 1100) {
        return HAL_ERROR;
    }
    // formula for gyro sample rate: sample rate = 1.1kHz/(1+GYRO_SMPLRT_DIV[7:0])
    // reverse compute the value for GYRO_SMPLRT_DIV[7:0]
    return ICM_WriteOneByte(GYRO_SMPLRT_DIV_REG, (uint8_t)(1100 / gyroSampleRate - 1));
}