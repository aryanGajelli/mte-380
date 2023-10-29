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
    if (ICM_MagnetometerInit() != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef ICM_AccelGyroInit() {
    if (ICM_SelectBank(USER_BANK_0) != HAL_OK) return HAL_ERROR;
    if (ICM_AccelGyroOff() != HAL_OK) return HAL_ERROR;
    if (ICM_AccelGyroOn() != HAL_OK) return HAL_ERROR;

    if (ICM_SelectBank(USER_BANK_2) != HAL_OK) return HAL_ERROR;

    if (ICM_SetGyroDPSAndLPF(GYRO_DPS_250, GYRO_LPF_154HZ) != HAL_OK) return HAL_ERROR;
    if (ICM_SetGyroSampleRate(100) != HAL_OK) return HAL_ERROR;

    if (ICM_SetAccelScaleAndLPF(ACCEL_SCALE_2G, ACCEL_LPF_1248HZ) != HAL_OK) return HAL_ERROR;
    if (ICM_SetAccelSampleRate(225) != HAL_OK) return HAL_ERROR;
    HAL_Delay(50);
    return HAL_OK;
}

HAL_StatusTypeDef ICM_MagnetometerInit() {
    if (ICM_SelectBank(USER_BANK_0) != HAL_OK) return HAL_ERROR;
    
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
    static const uint8_t DMP_EN = 0x80,
                         FIFO_EN = 0x40,
                         I2C_IF_DIS = 0x10,
                         DMP_RST = 0x08;
    return ICM_WriteOneByte(USER_CTRL_REG, DMP_EN | FIFO_EN | I2C_IF_DIS | DMP_RST);
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

/**
 * @brief Set the Gyro Degrees Per Second and Low Pass Filter Frequency for ICM20948
 * @param gyroDPS: Gyro Degrees Per Second
 * @param gyroLPF: Gyro Low Pass Filter Frequency
 */
HAL_StatusTypeDef ICM_SetGyroDPSAndLPF(GyroDPS_E gyroDPS, GyroLPF_E gyroLPF) {
    if (expected_CurrUserBank != USER_BANK_2) {
        return HAL_ERROR;
    }
    return ICM_WriteOneByte(GYRO_CONFIG_1_REG, (gyroDPS << 1) | gyroLPF);
}

/**
 * @brief Set the Gyro Sample Rate ICM20948
 * @param gyroSampleRate: sample rate in Hz
 */
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

/**
 * @brief Set the Accel Scale and Low Pass Filter ICM20948
 * @param accelScale: Accelerometer scale in g
 * @param accelLPF: Accelerometer Low Pass Filter Frequency
 */
HAL_StatusTypeDef ICM_SetAccelScaleAndLPF(AccelScale_E accelScale, AccelLPF_E accelLPF) {
    if (expected_CurrUserBank != USER_BANK_2) {
        return HAL_ERROR;
    }
    return ICM_WriteOneByte(ACCEL_CONFIG_REG, (accelScale << 1) | accelLPF);
}

/**
 * @brief Set the Accel Sample Rate ICM20948
 * @param accelSampleRate: sample rate in Hz
 */
HAL_StatusTypeDef ICM_SetAccelSampleRate(float accelSampleRate) {
    if (expected_CurrUserBank != USER_BANK_2) {
        return HAL_ERROR;
    }

    if (accelSampleRate < 1125. / (4095 + 1) || accelSampleRate > 1125) {
        return HAL_ERROR;
    }
    // formula for accel sample rate: sample rate = 1.125kHz/(1+ACCEL_SMPLRT_DIV[11:0])
    // reverse compute the value for ACCEL_SMPLRT_DIV[11:0]
    uint16_t accelSampleRateDiv = (uint16_t)(1125 / accelSampleRate - 1) & 0xfff;
    if (ICM_WriteOneByte(ACCEL_SMPLRT_DIV_1_REG, (uint8_t)(accelSampleRateDiv >> 8)) != HAL_OK) return HAL_ERROR;
    return ICM_WriteOneByte(ACCEL_SMPLRT_DIV_2_REG, (uint8_t)(accelSampleRateDiv & 0xff));
}

/**
 * @brief Read the Accelerometer and Gyroscope data from ICM20948
 * @param accel_data: pointer to array of 3 floats to store the accelerometer data
 * @param gyro_data: pointer to array of 3 floats to store the gyroscope data
 */
HAL_StatusTypeDef ICM_ReadAccelGyro(vector3_t *accel, vector3_t *gyro) {
    const size_t NUM_BYTES = ACCEL_GYRO_END_REG - ACCEL_GYRO_START_REG + 1;
    uint8_t gotBytes[NUM_BYTES + 1];
    uint8_t *raw_data = gotBytes + 1;
    int16_t signed_data[NUM_BYTES / 2];
    float accel_data[3];
    float gyro_data[3];
    if (expected_CurrUserBank != USER_BANK_0) {
        if (ICM_SelectBank(USER_BANK_0) != HAL_OK) return HAL_ERROR;
    }

    if (ICM_ReadBytes(ACCEL_GYRO_START_REG, gotBytes, NUM_BYTES) != HAL_OK) return HAL_ERROR;

    signed_data[0] = (raw_data[0] << 8) | raw_data[1];
    signed_data[1] = (raw_data[2] << 8) | raw_data[3];
    signed_data[2] = (raw_data[4] << 8) | raw_data[5];

    signed_data[3] = (raw_data[6] << 8) | raw_data[7];
    signed_data[4] = (raw_data[8] << 8) | raw_data[9];
    signed_data[5] = (raw_data[10] << 8) | raw_data[11];

    accel->x = signed_data[0] * 9.81 / -16384.0;
    accel->y = signed_data[1] * 9.81 / -16384.0;
    accel->z = signed_data[2] * 9.81 / -16384.0;

    gyro->x = signed_data[3] / 131.;
    gyro->y = signed_data[4] / 131.;
    gyro->z = signed_data[5] / 131.;
 
    // print raw data
    // for (int i = 0; i < NUM_BYTES; i++) {
    //     uprintf("%02x ", raw_data[i]);
    // }
    // print out the data
    // uprintf("accel: %0.3f %0.3f %0.3f\t", accel_data[0], accel_data[1], accel_data[2]);
    // uprintf("gyro: %0.3f %0.3f %0.3f\n", gyro_data[0], gyro_data[1], gyro_data[2]);
    
    // print out the data
    // uprintf("accel: %0.3f %0.3f %0.3f\t", accel->x, accel->y, accel->z);
    // uprintf("gyro: %0.3f %0.3f %0.3f\n", gyro->x, gyro->y, gyro->z);
    return HAL_OK;
}