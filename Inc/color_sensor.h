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

typedef struct ColorFreq_T {
    Color_E color;
    uint32_t freq;
} ColorFreq_T;

typedef enum {
    COLOR_1,
    COLOR_2,
    COLOR_3
} ColorSensor_E;

extern volatile uint16_t gu16_TIM2_OVC;
extern volatile ColorFreq_T colorFreqs[4];
void colorSensorInit();
int32_t getFreq();
HAL_StatusTypeDef selectColorSensor(ColorSensor_E cs);
float getLineError();
char* colorToStr(Color_E color);
void setColor(Color_E color);
#endif  // __COLOR_SENSOR_H__