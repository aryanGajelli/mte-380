#ifndef __SENSORS_H__
#define __SENSORS_H__

#include "stm32f4xx_hal.h"

typedef struct sensors_t {
    double ir1;
    double ir2;
    double ir3;
    double ir4;
    double ir5;
    double dist;
} sensors_t;

#define NUM_ADC_CHANNELS 6
extern uint32_t adcBuf[NUM_ADC_CHANNELS];
HAL_StatusTypeDef sensorsInit(void);
sensors_t* getSensors_Handle(void);
double ADC_to_Volt(uint32_t adc_val);
#endif // __SENSORS_H__