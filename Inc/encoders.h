#ifndef __ENCODERS_H__
#define __ENCODERS_H__

#include "stm32f4xx_hal.h"

typedef enum {
    ENCODER_RIGHT,
    ENCODER_LEFT,
} Encoder_E;

typedef struct Encoder_T {
    float ticks_per_s;
    int32_t ticks;
    int32_t prevTicks;
    int32_t overflow;

    double dist;
    Encoder_E encoderSide;
    uint32_t timeStamp;
} Encoder_T;

void encoderUpdate(Encoder_T *encoder);
void encoderHandleOverflow(Encoder_E encoderSide, TIM_HandleTypeDef *htim);

HAL_StatusTypeDef encodersInit(void);
Encoder_T* encoder_getInstance(Encoder_E encoder);
#endif  // __ENCODERS_H__