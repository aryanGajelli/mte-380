#include "imu2.h"

#include <stdlib.h>
#include <string.h>

#include "ICM20948_register.h"
#include "ICM20948_spi.h"
#include "Sparkfun/ICM_20948_C.h"
#include "Sparkfun/ICM_20948_REGISTERS.h"
#include "debug.h"
#include "stm32f4xx_hal.h"

ICM_20948_Device_t imu;
ICM_20948_Serif_t _serif;

ICM_20948_Status_e spi_write(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user);
ICM_20948_Status_e spi_read(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user);
ICM_20948_Status_e imuAccelGyroInit();

HAL_StatusTypeDef imuInit() {
    ICM_20948_init_struct(&imu);
    _serif = (ICM_20948_Serif_t){.read = spi_read, .write = spi_write, .user = NULL};
    imu._serif = &_serif;

    if (ICM_20948_check_id(&imu) != ICM_20948_Stat_Ok) {
        uprintf("ICM_20948_check_id for whoami failed\n");
        return HAL_ERROR;
    }

    ICM_20948_sw_reset(&imu);
    HAL_Delay(50);

    ICM_20948_sleep(&imu, false);
    ICM_20948_low_power(&imu, false);
    ICM_20948_set_clock_source(&imu, ICM_20948_Clock_Auto);
    imuAccelGyroInit();
    uint8_t data;
    ICM_20948_set_bank(&imu, 0);
    ICM_20948_execute_r(&imu, AGB0_REG_PWR_MGMT_1, &data, 1);  // reset AK09916
    uprintf("swreset: %02x\n", data);

    return HAL_OK;
}

ICM_20948_Status_e imuAccelGyroInit() {
    ICM_20948_InternalSensorID_bm sensors = ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr;
    ICM_20948_set_sample_mode(&imu, sensors, ICM_20948_Sample_Mode_Continuous);
    ICM_20948_set_full_scale(&imu, sensors, (ICM_20948_fss_t){.a = gpm2, .g = dps250});
    ICM_20948_set_dlpf_cfg(&imu, sensors, (ICM_20948_dlpcfg_t){.a = acc_d473bw_n499bw, .g = gyr_d361bw4_n376bw5});
    ICM_20948_enable_dlpf(&imu, sensors, false);
    return ICM_20948_Stat_Ok;
}

ICM_20948_Device_t *imuGet() {
    return &imu;
}

ICM_20948_Status_e spi_write(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user) {
    if (len == 1)
        if (ICM_WriteOneByte(regaddr, *pdata) != HAL_OK) return ICM_20948_Stat_Err;
    if (ICM_WriteBytes(regaddr, pdata, len) != HAL_OK) return ICM_20948_Stat_Err;
    return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e spi_read(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user) {
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