#include "mag.h"

#include "ICM20948_register.h"
#include "ICM20948_spi.h"
#include "Sparkfun/AK09916_ENUMERATIONS.h"
#include "Sparkfun/AK09916_REGISTERS.h"
#include "Sparkfun/ICM_20948_C.h"
#include "Sparkfun/ICM_20948_REGISTERS.h"
#include "imu2.h"
#include "debug.h"
#include "stm32f4xx_hal.h"

ICM_20948_Status_e magInit() {
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
    retval = ICM_20948_i2c_master_passthrough(&imu, false);  // Do not connect the SDA/SCL pins to AUX_DA/AUX_CL
    if (retval != ICM_20948_Stat_Ok) return retval;
    retval = ICM_20948_i2c_master_enable(&imu, true);
    if (retval != ICM_20948_Stat_Ok) return retval;

    // reset mag
    static const uint8_t SRST = 0x01;
    static const uint8_t MAX_MAGNETOMETER_STARTS = 10;
    // SRST: Soft reset
    // “0”: Normal
    // “1”: Reset
    // When “1” is set, all registers are initialized. After reset, SRST bit turns to “0” automatically.
    retval = magWrite(AK09916_REG_CNTL3, &SRST);
    if (retval != ICM_20948_Stat_Ok) return retval;

    // After a ICM reset the Mag sensor may stop responding over the I2C master
    // Reset the Master I2C until it responds
    uint8_t tries = 0;
    while (tries < MAX_MAGNETOMETER_STARTS) {
        tries++;

        // See if we can read the WhoIAm register correctly
        retval = magWhoIAm();
        if (retval == ICM_20948_Stat_Ok)
            break;  // WIA matched!

        ICM_20948_i2c_master_reset(&imu);  // Otherwise, reset the master I2C and try again

        HAL_Delay(10);
    }

    if (tries == MAX_MAGNETOMETER_STARTS) {
        uprintf("Failed to initialize magnetometer after max retries\n");
        return ICM_20948_Stat_WrongID;
    }

    // retval = magWrite(AK09916_REG_CNTL2, (uint8_t *)&((AK09916_CNTL2_Reg_t){.MODE = AK09916_mode_cont_100hz, .reserved_0 = 0}));
    // if (retval != ICM_20948_Stat_Ok) return retval;

    // retval = ICM_20948_i2c_controller_configure_peripheral(&imu, 0, MAG_AK09916_I2C_ADDR, AK09916_REG_ST1, 9, true, true, false, false, false, AK09916_mode_cont_100hz);
    return retval;
}

ICM_20948_Status_e magWhoIAm() {
    ICM_20948_Status_e retval = ICM_20948_Stat_Ok;
    uint8_t whoiam1, whoiam2;
    whoiam1 = magRead(AK09916_REG_WIA1);
    whoiam2 = magRead(AK09916_REG_WIA2);
    if ((whoiam1 != (MAG_AK09916_WHO_AM_I >> 8)) && (whoiam2 != (MAG_AK09916_WHO_AM_I & 0xFF)))
        return ICM_20948_Stat_WrongID;

    return ICM_20948_Stat_Ok;
}

uint8_t magRead(AK09916_Reg_Addr_e reg) {
    uint8_t data;
    ICM_20948_i2c_master_single_r(&imu, MAG_AK09916_I2C_ADDR, reg, &data);
    return data;
}

ICM_20948_Status_e magWrite(AK09916_Reg_Addr_e reg, uint8_t *pData) {
    return ICM_20948_i2c_master_single_w(&imu, MAG_AK09916_I2C_ADDR, reg, pData);
}