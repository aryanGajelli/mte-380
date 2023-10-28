#include "imu.h"

#include "ICM20948_spi.h"
#include "bsp.h"
#include "debug.h"
#include "stm32f4xx_hal.h"

HAL_StatusTypeDef ICMInit() {
    uint8_t data;
    // user bank 0 contains critical configuration registers
    ICM_SelectBank(USER_BANK_0);
    // check who am i register
    ICM_ReadOneByte(WHO_AM_I_REG, &data);
    if (data != WHO_AM_I_RESET_VAL) {
        uprintf("ICM20948 WHO_AM_I returned: 0x%02x\n", data);
        return HAL_ERROR;
    }

    ICM_DisableI2C();

    ICM_ReadOneByte(PWR_MGMT_1_REG, &data);
    uprintf("PWR_MGMT_1_REG: 0x%02x\n", data);

    ICM_SetClock(CLK_BEST_AVAIL);
    ICM_ReadOneByte(PWR_MGMT_1_REG, &data);
    uprintf("PWR_MGMT_1_REG: 0x%02x\n", data);
    return HAL_OK;
}

HAL_StatusTypeDef ICM_SelectBank(UserBankSel_E bank) {
    if (bank > USER_BANK_3) {
        return HAL_ERROR;
    }
    return ICM_WriteOneByte(USER_BANK_SEL_REG, bank << 4);
}

UserBankSel_E ICM_GetBank() {
    uint8_t data;
    ICM_ReadOneByte(USER_BANK_SEL_REG, &data);
    return (UserBankSel_E)(data >> 4);
}

void ICM_DisableI2C() {
    // disable I2C interface, enable
#define FIFO_EN 0x40
#define I2C_IF_DIS 0x10
#define DMP_RST 0x08
    ICM_WriteOneByte(USER_CTRL_REG, FIFO_EN | I2C_IF_DIS | DMP_RST);
}

void ICM_SetClock(ClockSel_E clockSel){
    ICM_WriteOneByte(PWR_MGMT_1_REG, (uint8_t)clockSel);
}
