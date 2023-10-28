#ifndef __ICM20948_REGISTER_H__
#define __ICM20948_REGISTER_H__

// user bank 0
#define WHO_AM_I_REG 0x00
#define WHO_AM_I_RESET_VAL 0xEA
#define USER_CTRL_REG 0x03
#define PWR_MGMT_1_REG 0x06
#define PWR_MGMT_2_REG 0x07

#define USER_BANK_SEL_REG 0x7F

// user bank 2
#define GYRO_SMPLRT_DIV_REG 0x00
#define GYRO_CONFIG_1_REG 0x01
#define ACCEL_SMPLRT_DIV_1_REG 0x10
#define ACCEL_SMPLRT_DIV_2_REG 0x11
#define ACCEL_CONFIG_REG 0x14
#define ACCEL_GYRO_START_REG 0x2D
#define ACCEL_GYRO_END_REG 0x38

// user register banks
typedef enum UserBankSel_E {
    USER_BANK_0 = 0,
    USER_BANK_1 = 1,
    USER_BANK_2 = 2,
    USER_BANK_3 = 3,
    USER_BANK_ERROR = 0xff
} UserBankSel_E;

// clock selctions
typedef enum ClockSel_E {
    CLK_BEST_AVAIL = 1,
    CLK_INTERNAL_20MHZ = 0,
    CLK_STOP = 7
} ClockSel_E;

typedef enum GyroDPS_E {
    GYRO_DPS_250 = 0,
    GYRO_DPS_500 = 1,
    GYRO_DPS_1000 = 2,
    GYRO_DPS_2000 = 3
} GyroDPS_E;

typedef enum GyroLPF_E {
    GYRO_LPF_12316HZ = 0,
    GYRO_LPF_300HZ = 1,
    GYRO_LPF_188HZ = (1 << 3) | 1,
    GYRO_LPF_154HZ = (2 << 3) | 1,
    GYRO_LPF_73HZ = (3 << 3) | 1,
    GYRO_LPF_36HZ = (4 << 3) | 1,
    GYRO_LPF_17HZ = (5 << 3) | 1,
    GYRO_LPF_9HZ = (6 << 3) | 1,
    GYRO_LPF_377HZ = (7 << 3) | 1
} GyroLPF_E;

typedef enum AccelScale_E {
    ACCEL_SCALE_2G = 0,
    ACCEL_SCALE_4G = 1,
    ACCEL_SCALE_8G = 2,
    ACCEL_SCALE_16G = 3
} AccelScale_E;

typedef enum AccelLPF_E {
    ACCEL_LPF_1248HZ = 0,
    ACCEL_LPF_265HZ = 1,
    ACCEL_LPF_136HZ = (2 << 3) | 1,
    ACCEL_LPF_69HZ = (3 << 3) | 1,
    ACCEL_LPF_34HZ = (4 << 3) | 1,
    ACCEL_LPF_17HZ = (5 << 3) | 1,
    ACCEL_LPF_8HZ = (6 << 3) | 1,
    ACCEL_LPF_500HZ = (7 << 3) | 1
} AccelLPF_E;

#endif  // __ICM20948_REGISTER_H__