#ifndef __AK09916_H__
#define __AK09916_H__

#include "stm32f4xx_hal.h"
// magnetometer registers
#define AK09916_ADDR 0x0C
#define MAG_CNTL2_REG 0x31
#define MAG_CNTL3_REG 0x32
#define MAG_DATA_ONSET_REG 0x11
typedef enum MagMeasureMode_E {
    MAG_MODE_POWERDOWN = 0,
    MAG_MODE_SINGLE = 1,
    MAG_MODE_CONT_1 = 2,
    MAG_MODE_CONT_2 = 4,
    MAG_MODE_CONT_3 = 6,
    MAG_MODE_CONT_4 = 8,
    MAG_MODE_SELFTEST = 0x10,
} MagMeasureMode_E;

#endif // __AK09916_H__