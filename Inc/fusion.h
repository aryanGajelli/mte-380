#ifndef __FUSION_H__
#define __FUSION_H__

#include "stm32f4xx_hal.h"
#include "motion_fx.h"

HAL_StatusTypeDef fusionInit(void);
void printKnobs(MFX_knobs_t* iKnobs);
#endif  // __FUSION_H__