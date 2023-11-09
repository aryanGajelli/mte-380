#ifndef __ENCODERS_H__
#define __ENCODERS_H__

#include "stm32f4xx_hal.h"

typedef enum {
    ENCODER_RIGHT,
    ENCODER_LEFT,
} Encoder_E;

typedef struct Encoder_T {
    int32_t velocity;
    int64_t position;
    uint32_t lastCount;
    Encoder_E encoderSide;
} Encoder_T;

void update_encoder(Encoder_E encoderSide);

HAL_StatusTypeDef encodersInit(void);
Encoder_T* encoder_getInstance(Encoder_E encoder);
#endif  // __ENCODERS_H__