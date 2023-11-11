#ifndef __ENCODERS_H__
#define __ENCODERS_H__

#include "stm32f4xx_hal.h"

typedef enum {
    ENCODER_RIGHT,
    ENCODER_LEFT,
} Encoder_E;

typedef struct Encoder_T {
    float velocity;
    int32_t position;
    uint32_t lastPosition;
    int32_t overflow;
    Encoder_E encoderSide;
} Encoder_T;

char isEncodersInitialized();
void update_encoder(Encoder_E encoderSide);
#define IC_SIZE 5
extern int32_t encoderLeftVelocity;
extern uint16_t IC1[IC_SIZE];
extern uint16_t IC2[IC_SIZE];

HAL_StatusTypeDef encodersInit(void);
Encoder_T* encoder_getInstance(Encoder_E encoder);
#endif  // __ENCODERS_H__