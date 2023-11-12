#ifndef __COLOR_SENSOR_H__
#define __COLOR_SENSOR_H__

#include "stm32f4xx_hal.h"

typedef enum {
    RED = 0b00,
    GREEN = 0b11,
    BLUE = 0b10,
    CLEAR = 0b01
} Color_E;

typedef enum {
    FREQ_SCALE_OFF = 0b00,
    FREQ_SCALE_2 = 0b01,
    FREQ_SCALE_20 = 0b10,
    FREQ_SCALE_100 = 0b11
} FreqScale_E;

typedef enum {
    COLOR_SENSOR_1,
    COLOR_SENSOR_2,
    COLOR_SENSOR_3,
    COLOR_SENSOR_ERROR
} ColorSensor_E;

typedef enum {
    SURFACE_WOOD,
    SURFACE_TAPE,
    SURFACE_BLACK,
    SERFACE_ERROR
} SurfaceType_E;

HAL_StatusTypeDef colorSensorInit();
uint32_t colorGetFreq(ColorSensor_E sensor);
void colorSetFreqScaling(FreqScale_E freqScale);
HAL_StatusTypeDef colorSelectSensor(ColorSensor_E cs);
double colorGetNormalizedOut(ColorSensor_E sensor);
SurfaceType_E colorGetLineDeviation(double *out);
void colorSet(Color_E color);
#endif  // __COLOR_SENSOR_H__