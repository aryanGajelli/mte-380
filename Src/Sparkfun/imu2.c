#include "imu2.h"

#include <stdlib.h>

#include "ICM20948_spi.h"
#include "Sparkfun/ICM_20948_C.h"
#include "Sparkfun/ICM_20948_REGISTERS.h"
#include "debug.h"
#include "stm32f4xx_hal.h"

ICM_20948_Device_t imu;
ICM_20948_Serif_t imuSerif;

ICM_20948_Status_e write(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user);
ICM_20948_Status_e read(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user);

HAL_StatusTypeDef imuInit() {
    ICM_20948_init_struct(&imu);

    // init and link serial interface
    imuSerif = (ICM_20948_Serif_t){.read = read, .write = write, .user = NULL};
    if (ICM_20948_link_serif(&imu, &imuSerif) != ICM_20948_Stat_Ok) return HAL_ERROR;

    uint8_t data;
    // ICM_20948_Status_e ret = ICM_20948_execute_r(&imu, AGB0_REG_WHO_AM_I, &data, 1);  // reset AK09916
    ICM_20948_Status_e ret = ICM_20948_get_who_am_i(&imu, &data);
    uprintf("IMU WHO_AM_I: %d, ret: %d\n", data, ret);
    return HAL_OK;
}

ICM_20948_Device_t *imuGet() {
    return &imu;
}

ICM_20948_Status_e write(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user) {
    if (len == 1)
        if (ICM_WriteOneByte(regaddr, *pdata) != HAL_OK) return ICM_20948_Stat_Err;
    if (ICM_WriteBytes(regaddr, pdata, len) != HAL_OK) return ICM_20948_Stat_Err;
    return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e read(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user) {
    // use extra buf cuz SPI_TransmitReceive reads on transmit byte
    static uint8_t *oneExtraBuf = NULL;
    if (oneExtraBuf == NULL)
        oneExtraBuf = (uint8_t *)malloc(len + 1);
    else if (len + 1 > sizeof(oneExtraBuf)) {
        oneExtraBuf = (uint8_t *)realloc(oneExtraBuf, len + 1);
    }
    if (ICM_ReadBytes(regaddr, oneExtraBuf, len) != HAL_OK) return ICM_20948_Stat_Err;
    memcpy(pdata, oneExtraBuf + 1, len);
    return ICM_20948_Stat_Ok;
}