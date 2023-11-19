#ifndef __DMP_H__
#define __DMP_H__

#include "ICM20948_register.h"
#include "ICM20948_spi.h"
#include "Sparkfun/AK09916_ENUMERATIONS.h"
#include "Sparkfun/AK09916_REGISTERS.h"
#include "Sparkfun/ICM_20948_C.h"
#include "Sparkfun/ICM_20948_REGISTERS.h"
#include "imu2.h"
#include "stm32f4xx_hal.h"

ICM_20948_Status_e dmpInit();
ICM_20948_Status_e dmpPostInit();
ICM_20948_Status_e dmpSetStartAddress(uint16_t address);
ICM_20948_Status_e readDMPmems(uint16_t reg, uint32_t length, const uint8_t* data);
ICM_20948_Status_e writeDMPmems(uint16_t reg, uint32_t length, const uint8_t* data);
ICM_20948_Status_e dmpLoadFirmware();
ICM_20948_Status_e dmpEnableSensor(enum inv_icm20948_sensor sensor, bool enable);
ICM_20948_Status_e dmpSetODRrate(enum DMP_ODR_Registers odr_reg, int interval);
ICM_20948_Status_e dmpReadDataFromFIFO(icm_20948_DMP_data_t *data);
#endif  // __DMP_H__