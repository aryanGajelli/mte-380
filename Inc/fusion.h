#ifndef __FUSION_H__
#define __FUSION_H__

#include "stm32f4xx_hal.h"
#include "motion_fx.h"
#include "imu.h"

HAL_StatusTypeDef fusionInit(void);
void printKnobs(MFX_knobs_t* iKnobs);
void fusionGetOutputs(MFX_output_t* data_out, IMUData_T imuData, IMUData_T prevImuData);
#endif  // __FUSION_H__