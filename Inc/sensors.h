#ifndef __SENSORS_H__
#define __SENSORS_H__

#include "stm32f4xx_hal.h"

typedef struct sensors_t {
    uint32_t ir1;
    uint32_t ir2;
    uint32_t ir3;
    uint32_t ir4;
    uint32_t ir5;
    uint32_t dist;
} sensors_t;

HAL_StatusTypeDef sensorsInit(void);
sensors_t* getSensors_Handle(void);
#endif // __SENSORS_H__