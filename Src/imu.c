#include "imu.h"

#include "AK09916_i2c.h"
#include "ICM20948_spi.h"
#include "bsp.h"
#include "debug.h"
#include "stm32f4xx_hal.h"

UserBankSel_E expected_CurrUserBank = USER_BANK_ERROR;
AppliedSensitivity_T appliedSensitivity;
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
    // enable auto reading
    AK0_ReadRegisters(MAG_DATA_ONSET_REG, 8);

    return HAL_OK;
}

HAL_StatusTypeDef ICM_AccelGyroInit() {
    if (ICM_SelectBank(USER_BANK_0) != HAL_OK) return HAL_ERROR;
    if (ICM_AccelGyroOff() != HAL_OK) return HAL_ERROR;
    if (ICM_AccelGyroOn() != HAL_OK) return HAL_ERROR;

    if (ICM_SelectBank(USER_BANK_2) != HAL_OK) return HAL_ERROR;

    appliedSensitivity.gyroDPS = GYRO_DPS_1000;
    static const double GYRO_SENS_250DPS = 131.0, GYRO_SENS_500DPS = 65.5,
                        GYRO_SENS_1000DPS = 32.8, GYRO_SENS_2000DPS = 16.4;
    switch (appliedSensitivity.gyroDPS) {
        case GYRO_DPS_250:
            appliedSensitivity.gyroSensitivity = GYRO_SENS_250DPS;
            break;
        case GYRO_DPS_500:
            appliedSensitivity.gyroSensitivity = GYRO_SENS_500DPS;
            break;
        case GYRO_DPS_1000:
            appliedSensitivity.gyroSensitivity = GYRO_SENS_1000DPS;
            break;
        case GYRO_DPS_2000:
            appliedSensitivity.gyroSensitivity = GYRO_SENS_2000DPS;
            break;
        default:
            return HAL_ERROR;
    }

    if (ICM_SetGyroDPSAndLPF(appliedSensitivity.gyroDPS, GYRO_LPF_17HZ) != HAL_OK) return HAL_ERROR;
    if (ICM_SetGyroSampleRate(1100) != HAL_OK) return HAL_ERROR;
    if (ICM_SetGyroOffset((vector3_t){0, 0, 0}) != HAL_OK) return HAL_ERROR;

    appliedSensitivity.accelScale = ACCEL_SCALE_2G;
    static const double ACCEL_SENS_2G = 16384.0, ACCEL_SENS_4G = 8192.0,
                        ACCEL_SENS_8G = 4096.0, ACCEL_SENS_16G = 2048.0;
    switch (appliedSensitivity.accelScale) {
        case ACCEL_SCALE_2G:
            appliedSensitivity.accelSensitivity = ACCEL_SENS_2G;
            break;
        case ACCEL_SCALE_4G:
            appliedSensitivity.accelSensitivity = ACCEL_SENS_4G;
            break;
        case ACCEL_SCALE_8G:
            appliedSensitivity.accelSensitivity = ACCEL_SENS_8G;
            break;
        case ACCEL_SCALE_16G:
            appliedSensitivity.accelSensitivity = ACCEL_SENS_16G;
            break;
        default:
            return HAL_ERROR;
    }
    if (ICM_SetAccelScaleAndLPF(appliedSensitivity.accelScale, ACCEL_LPF_500HZ) != HAL_OK) return HAL_ERROR;
    if (ICM_SetAccelSampleRate(1100) != HAL_OK) return HAL_ERROR;
    HAL_Delay(50);
    ICM_CalibrateGyro();
    return HAL_OK;
}

HAL_StatusTypeDef ICM_MagnetometerInit() {
    uint8_t data;
    // Configure AUX_I2C Magnetometer
    if (ICM_SelectBank(USER_BANK_0) != HAL_OK) return HAL_ERROR;
    // INT Pin
    const uint8_t INT1_LATCH__EN = 0x20,
                  INT_ANYRD_2CLEAR = 0x10;
    ICM_WriteOneByte(INT_PIN_CONFIG_REG, INT1_LATCH__EN | INT_ANYRD_2CLEAR);

    // I2C_MST_EN
    const uint8_t I2C_MST_EN = 0x20;
    ICM_ReadOneByte(USER_CTRL_REG, &data);
    data |= I2C_MST_EN;
    ICM_WriteOneByte(USER_CTRL_REG, data);

    // enable duty cycled mode for magnetometer
    const uint8_t I2C_MST_CYCLE = 0x40;
    ICM_WriteOneByte(LP_CONFIG_REG, I2C_MST_CYCLE);

    if (ICM_SelectBank(USER_BANK_3) != HAL_OK) return HAL_ERROR;
    // I2C Master mode and Speed 400 kHz
    const uint8_t I2C_MST_CLK_400kHz = 0x0D;
    ICM_WriteOneByte(I2C_MST_CTRL_REG, I2C_MST_CLK_400kHz);

    // set magnetometer data rate to 1.1kHz/ (2^3) = 136 Hz, page 68
    data = 4;
    ICM_WriteOneByte(I2C_MST_ODR_CONFIG_REG, data);

    // Reset AK8963
    const uint8_t MAG_RESET = 0x01;
    AK0_WriteOneByte(MAG_CNTL3_REG, MAG_RESET);
    HAL_Delay(100);
    AK0_WriteOneByte(MAG_CNTL2_REG, MAG_MODE_CONT_4);

    appliedSensitivity.magSensitivity = 0.15;
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
 * @brief Set Gyroscope offset
 * @param gyroOffset: vector3 containing the gyroscope offset
 */
HAL_StatusTypeDef ICM_SetGyroOffset(vector3_t offset) {
    // invert scale to be raw value
    offset.x *= appliedSensitivity.gyroSensitivity;
    offset.y *= appliedSensitivity.gyroSensitivity;
    offset.z *= appliedSensitivity.gyroSensitivity;

    int16_t signedData[3] = {(int16_t)offset.x, (int16_t)offset.y, (int16_t)offset.z};
    uint8_t raw[6] = {(uint8_t)(signedData[0] >> 8), (uint8_t)(signedData[0] & 0xff),
                      (uint8_t)(signedData[1] >> 8), (uint8_t)(signedData[1] & 0xff),
                      (uint8_t)(signedData[2] >> 8), (uint8_t)(signedData[2] & 0xff)};
    ICM_SelectBank(USER_BANK_2);
    return ICM_WriteBytes(XG_OFFS_USRH_REG, (uint8_t *)&raw, 6);
}

/**
 * @brief Convert raw accel data to m/s^2
 * @param raw: raw accel data
 * @return accel in m/s^2
 */
void ICM_ConvertRawAccel(vector3_t *raw, vector3_t *accel) {
    static const double GRAVITY = 9.81;

    accel->x = raw->x * -GRAVITY / appliedSensitivity.accelSensitivity;
    accel->y = raw->y * -GRAVITY / appliedSensitivity.accelSensitivity;
    accel->z = raw->z * -GRAVITY / appliedSensitivity.accelSensitivity;
}

/**
 * @brief Convert raw gyro data to deg/s
 * @param raw: raw gyro data
 * @return gyro in deg/s
 */
void ICM_ConvertRawGyro(vector3_t *raw, vector3_t *gyro) {
    gyro->x = raw->x / appliedSensitivity.gyroSensitivity;
    gyro->y = raw->y / appliedSensitivity.gyroSensitivity;
    gyro->z = raw->z / appliedSensitivity.gyroSensitivity;
}

/**
 * @brief Convert raw mag data to uT
 */
void ICM_ConvertRawMag(vector3_t *raw, vector3_t *mag) {
    static const vector3_t minMag = {-55.200, -20.700, -1.200},
                           maxMag = {21.000, 61.350, 84.000};
    mag->x = raw->x * appliedSensitivity.magSensitivity - (minMag.x + maxMag.x) / 2;
    mag->y = raw->y * appliedSensitivity.magSensitivity - (minMag.y + maxMag.y) / 2;
    mag->z = raw->z * appliedSensitivity.magSensitivity - (minMag.z + maxMag.z) / 2;
}

/**
 * @brief Read the Accelerometer and Gyroscope data from ICM20948
 * @param accel_data: pointer to array of 3 floats to store the accelerometer data
 * @param gyro_data: pointer to array of 3 floats to 
 * store the gyroscope data
 */
HAL_StatusTypeDef ICM_ReadAccelGyro(vector3_t *accel, vector3_t *gyro) {
    static const size_t NUM_BYTES = ACCEL_GYRO_END_REG - ACCEL_GYRO_START_REG + 1;
    uint8_t gotBytes[NUM_BYTES + 1];
    uint8_t *raw_data = gotBytes + 1;

    if (expected_CurrUserBank != USER_BANK_0) {
        if (ICM_SelectBank(USER_BANK_0) != HAL_OK) return HAL_ERROR;
    }

    if (ICM_ReadBytes(ACCEL_GYRO_START_REG, gotBytes, NUM_BYTES) != HAL_OK) return HAL_ERROR;

    vector3_t rawAccel, rawGyro;
    rawAccel.x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    rawAccel.y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    rawAccel.z = (int16_t)((raw_data[4] << 8) | raw_data[5]);

    rawGyro.x = (int16_t)((raw_data[6] << 8) | raw_data[7]);
    rawGyro.y = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    rawGyro.z = (int16_t)((raw_data[10] << 8) | raw_data[11]);

    ICM_ConvertRawAccel(&rawAccel, accel);
    ICM_ConvertRawGyro(&rawGyro, gyro);
    return HAL_OK;
}

/**
 * @brief Read the Magnetometer data from ICM20948
 * @param mag: pointer to vector3_t to store the magnetometer data
 */
HAL_StatusTypeDef ICM_ReadMag(vector3_t *mag) {
    static const size_t NUM_BYTES = ACCEL_GYRO_MAG_END_REG - MAG_START_REG + 1;
    uint8_t gotBytes[NUM_BYTES + 1];
    uint8_t *raw_data = gotBytes + 1;

    if (expected_CurrUserBank != USER_BANK_0) {
        if (ICM_SelectBank(USER_BANK_0) != HAL_OK) return HAL_ERROR;
    }

    if (ICM_ReadBytes(MAG_START_REG, gotBytes, NUM_BYTES) != HAL_OK) return HAL_ERROR;

    vector3_t rawMag;
    rawMag.x = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    rawMag.y = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    rawMag.z = (int16_t)((raw_data[5] << 8) | raw_data[4]);

    ICM_ConvertRawMag(&rawMag, mag);
    return HAL_OK;
}

/**
 * @brief Read the Accelerometer, Gyroscope, and Magnetometer data from ICM20948
 * @param data: pointer to IMUData_T struct to store the data
 */
HAL_StatusTypeDef ICM_Read(IMUData_T *data) {
    static const size_t NUM_BYTES = ACCEL_GYRO_MAG_END_REG - ACCEL_GYRO_START_REG + 1;
    uint8_t gotBytes[NUM_BYTES + 1];  // account for transmit byte garbage values in miso
    uint8_t *raw_data = gotBytes + 1;
    int16_t signed_data[9];
    if (expected_CurrUserBank != USER_BANK_0) {
        if (ICM_SelectBank(USER_BANK_0) != HAL_OK) return HAL_ERROR;
    }
    ICM_ReadBytes(ACCEL_GYRO_START_REG, gotBytes, NUM_BYTES);
    data->timestamp = HAL_GetTick();

    vector3_t rawAccel, rawGyro, rawMag;
    rawAccel.x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    rawAccel.y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    rawAccel.z = (int16_t)((raw_data[4] << 8) | raw_data[5]);

    rawGyro.x = (int16_t)((raw_data[6] << 8) | raw_data[7]);
    rawGyro.y = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    rawGyro.z = (int16_t)((raw_data[10] << 8) | raw_data[11]);

    rawMag.x = (int16_t)((raw_data[15] << 8) | raw_data[14]);
    rawMag.y = (int16_t)((raw_data[17] << 8) | raw_data[16]);
    rawMag.z = (int16_t)((raw_data[19] << 8) | raw_data[18]);

    ICM_ConvertRawAccel(&rawAccel, &data->accel);
    ICM_ConvertRawGyro(&rawGyro, &data->gyro);
    ICM_ConvertRawMag(&rawMag, &data->mag);
    return HAL_OK;
}

/**
 * @brief Calibrate the gyroscope
 */
HAL_StatusTypeDef ICM_CalibrateGyro() {
    vector3_t gyroSum = {0, 0, 0};
    vector3_t gyro;
    for (int i = 0; i < 100; i++) {
        if (ICM_ReadAccelGyro(NULL, &gyro) != HAL_OK)
            return HAL_ERROR;
        gyroSum.x += gyro.x;
        gyroSum.y += gyro.y;
        gyroSum.z += gyro.z;
        HAL_Delay(2);
    }
    // for some reason gotta divide by 4*count
    gyroSum.x /= -400;
    gyroSum.y /= -400;
    gyroSum.z /= -400;

    return ICM_SetGyroOffset(gyroSum);
}
