#ifndef __MAG_H__
#define __MAG_H__

#include "Sparkfun/AK09916_ENUMERATIONS.h"
#include "Sparkfun/AK09916_REGISTERS.h"
#include "Sparkfun/ICM_20948_C.h"
#include "Sparkfun/ICM_20948_REGISTERS.h"
#include "stm32f4xx_hal.h"

ICM_20948_Status_e magInit();

ICM_20948_Status_e magWhoIAm();
uint8_t magRead(AK09916_Reg_Addr_e reg);
ICM_20948_Status_e magWrite(AK09916_Reg_Addr_e reg, uint8_t* pData);
#endif  // __MAG_H__