#ifndef __ICM20948_REGISTER_H__
#define __ICM20948_REGISTER_H__

// registers
#define WHO_AM_I_REG 0x00
#define WHO_AM_I_RESET_VAL 0xEA
#define USER_CTRL_REG 0x03
#define USER_CTRL_RESET_VAL 0x00
#define PWR_MGMT_1_REG 0x06
#define USER_BANK_SEL_REG 0x7F
#define ACCEL_XOUT_H_REG 0x2D
#define ACCEL_XOUT_L_REG 0x2E

// user register banks
typedef enum UserBankSel_E {
    USER_BANK_0 = 0,
    USER_BANK_1 = 1,
    USER_BANK_2 = 2,
    USER_BANK_3 = 3
} UserBankSel_E;

// clock selctions
typedef enum ClockSel_E {
    CLK_BEST_AVAIL = 1,
    CLK_INTERNAL_20MHZ = 0,
    CLK_STOP = 7
} ClockSel_E;

#endif  // __ICM20948_REGISTER_H__