#include "bsp.h"
#include "sensors.h"

sensors_t sensors;
HAL_StatusTypeDef sensorsInit(void) {
    return HAL_ADC_Start_DMA(&ADC_HANDLE, (uint32_t*)&sensors, 6);
}

sensors_t* getSensors_Handle(void) {
    return &sensors;
}