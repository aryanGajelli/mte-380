#include "imu2.h"

#include <stdlib.h>
#include <string.h>

#include "ICM20948_register.h"
#include "ICM20948_spi.h"
#include "Sparkfun/AK09916_ENUMERATIONS.h"
#include "Sparkfun/AK09916_REGISTERS.h"
#include "Sparkfun/ICM_20948_C.h"
#include "Sparkfun/ICM_20948_REGISTERS.h"
#include "debug.h"
#include "dmp.h"
#include "imu.h"
#include "mag.h"
#include "stm32f4xx_hal.h"

ICM_20948_Device_t imu;
ICM_20948_Serif_t _serif;
ICM_20948_fss_t fss;

ICM_20948_Status_e spi_write(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user);
ICM_20948_Status_e spi_read(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user);
ICM_20948_Status_e imuAccelGyroInit();

HAL_StatusTypeDef imuInit() {
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
    ICM_20948_init_struct(&imu);
    _serif = (ICM_20948_Serif_t){.read = spi_read, .write = spi_write, .user = NULL};
    imu._serif = &_serif;

    if (ICM_20948_check_id(&imu) != ICM_20948_Stat_Ok) {
        uprintf("ICM_20948_check_id for whoami failed\n");
        return HAL_ERROR;
    }

    retval = ICM_20948_sw_reset(&imu);
    if (retval != ICM_20948_Stat_Ok) {
        uprintf("ICM_20948_sw_reset failed with %0x\n", retval);
        return HAL_ERROR;
    }
    HAL_Delay(50);

    retval = ICM_20948_sleep(&imu, false);
    if (retval != ICM_20948_Stat_Ok) {
        uprintf("ICM_20948_sleep failed with %0x\n", retval);
        return HAL_ERROR;
    }
    retval = ICM_20948_low_power(&imu, false);
    if (retval != ICM_20948_Stat_Ok) {
        uprintf("ICM_20948_low_power failed with %0x\n", retval);
        return HAL_ERROR;
    }
    retval = ICM_20948_set_clock_source(&imu, ICM_20948_Clock_Auto);
    if (retval != ICM_20948_Stat_Ok) {
        uprintf("ICM_20948_set_clock_source failed with %0x\n", retval);
        return HAL_ERROR;
    }
    retval = imuAccelGyroInit();
    if (retval != ICM_20948_Stat_Ok) {
        uprintf("imuAccelGyroInit failed with %0x\n", retval);
        return HAL_ERROR;
    }
    HAL_Delay(100);
    retval = magInit();
    if (retval != ICM_20948_Stat_Ok) {
        uprintf("magInit failed with %0x\n", retval);
        return HAL_ERROR;
    }
    HAL_Delay(100);

    retval = dmpInit();
    if (retval != ICM_20948_Stat_Ok) {
        uprintf("dmpInit failed with %0x\n", retval);
        vTaskDelay(1000);
        return HAL_ERROR;
    }

    retval = dmpPostInit();
    if (retval != ICM_20948_Stat_Ok) {
        uprintf("dmpPostInit failed with %0x\n", retval);
        vTaskDelay(1000);
        return HAL_ERROR;
    }

    imuLoadScale();
    return HAL_OK;
}

ICM_20948_Status_e imuAccelGyroInit() {
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
    ICM_20948_InternalSensorID_bm sensors = ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr;

    retval = ICM_20948_set_sample_mode(&imu, sensors, ICM_20948_Sample_Mode_Continuous);
    if (retval != ICM_20948_Stat_Ok) return retval;

    // retval = ICM_20948_set_full_scale(&imu, sensors, (ICM_20948_fss_t){.a = gpm2, .g = dps250});
    // if (retval != ICM_20948_Stat_Ok) return retval;

    retval = ICM_20948_set_dlpf_cfg(&imu, sensors, (ICM_20948_dlpcfg_t){.a = acc_d11bw5_n17bw, .g = gyr_d196bw6_n229bw8});
    if (retval != ICM_20948_Stat_Ok) return retval;

    retval = ICM_20948_enable_dlpf(&imu, sensors, false);
    if (retval != ICM_20948_Stat_Ok) return retval;

    return retval;
}

bool imuIsDataReady() {
    ICM_20948_Status_e retval = ICM_20948_data_ready(&imu);
    if (retval != ICM_20948_Stat_Ok) return false;
    return true;
}

ICM_20948_Status_e imuRead(IMUData_T *imuData) {
    ICM_20948_AGMT_t agmt;
    ICM_20948_Status_e retval = ICM_20948_get_agmt(&imu, &agmt);
    if (retval != ICM_20948_Stat_Ok) return retval;
    imuScaleAndAssign(imuData, &agmt);
    return ICM_20948_Stat_Ok;
}

void imuScaleAndAssign(IMUData_T *imuData, ICM_20948_AGMT_t *agmt) {
    static double scale;
    switch (fss.a) {
        case gpm2:
            scale = 16384.;
            break;
        case gpm4:
            scale = 8192.;
            break;
        case gpm8:
            scale = 4096.;
            break;
        case gpm16:
            scale = 2048.;
            break;
        default:
            break;
    }

    imuData->accel.x = agmt->acc.axes.x / scale;
    imuData->accel.y = agmt->acc.axes.y / scale;
    imuData->accel.z = agmt->acc.axes.z / scale;

    switch (fss.g) {
        case dps250:
            scale = 131.;
            break;
        case dps500:
            scale = 65.5;
            break;
        case dps1000:
            scale = 32.8;
            break;
        case dps2000:
            scale = 16.4;
            break;
        default:
            break;
    }

    imuData->gyro.x = agmt->gyr.axes.x / scale;
    imuData->gyro.y = agmt->gyr.axes.y / scale;
    imuData->gyro.z = agmt->gyr.axes.z / scale;

    scale = 0.15;
    imuData->mag.x = agmt->mag.axes.x * scale;
    imuData->mag.y = agmt->mag.axes.y * scale;
    imuData->mag.z = agmt->mag.axes.z * scale;
}

ICM_20948_Status_e imuLoadScale() {
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;

    retval |= ICM_20948_set_bank(&imu, 2);
    ICM_20948_ACCEL_CONFIG_t acfg;
    retval |= ICM_20948_execute_r(&imu, (uint8_t)AGB2_REG_ACCEL_CONFIG, (uint8_t *)&acfg, 1 * sizeof(acfg));
    fss.a = acfg.ACCEL_FS_SEL;  // Worth noting that without explicitly setting the FS range of the accelerometer it was showing the register value for +/- 2g but the reported values were actually scaled to the +/- 16g range
                                // Wait a minute... now it seems like this problem actually comes from the digital low-pass filter. When enabled the value is 1/8 what it should be...
    retval |= ICM_20948_set_bank(&imu, 2);
    ICM_20948_GYRO_CONFIG_1_t gcfg1;
    retval |= ICM_20948_execute_r(&imu, (uint8_t)AGB2_REG_GYRO_CONFIG_1, (uint8_t *)&gcfg1, 1 * sizeof(gcfg1));
    fss.g = gcfg1.GYRO_FS_SEL;
    ICM_20948_ACCEL_CONFIG_2_t acfg2;
    retval |= ICM_20948_execute_r(&imu, (uint8_t)AGB2_REG_ACCEL_CONFIG_2, (uint8_t *)&acfg2, 1 * sizeof(acfg2));

    return retval;
}

ICM_20948_Status_e imuIntEnableRawDataReady(bool enable) {
    ICM_20948_INT_enable_t en;                                          // storage
    ICM_20948_Status_e status = ICM_20948_int_enable(&imu, NULL, &en);  // read phase
    if (status != ICM_20948_Stat_Ok) {
        return status;
    }
    en.RAW_DATA_0_RDY_EN = enable;                  // change the setting
    status = ICM_20948_int_enable(&imu, &en, &en);  // write phase w/ readback
    if (status != ICM_20948_Stat_Ok) {
        return status;
    }
    if (en.RAW_DATA_0_RDY_EN != enable) {
        status = ICM_20948_Stat_Err;
        return status;
    }
    return status;
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