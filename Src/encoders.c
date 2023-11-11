#include "encoders.h"

#include "bsp.h"
#include "debug.h"
#include "main.h"
#include "stm32f4xx_hal.h"

volatile Encoder_T encoderLeft;
volatile Encoder_T encoderRight;

void encoderUpdate(Encoder_T *encoder) {
    encoder->prevPosition = encoder->position;
    uint32_t timeStamp = HAL_GetTick();
    int8_t dir;
    TIM_HandleTypeDef *htim;
    if (encoder == &encoderLeft) {
        htim = &ENCODER_TIMER_LEFT_HANDLE;
        dir = 1;
    } else if (encoder == &encoderRight) {
        htim = &ENCODER_TIMER_RIGHT_HANDLE;
        dir = -1;
    } else {
        Error_Handler();
    }

    uint32_t counter = __HAL_TIM_GET_COUNTER(htim);
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    encoder->position = dir * (counter + encoder->overflow * arr);
    encoder->velocity = 1000*(encoder->position - encoder->prevPosition) / (float)(timeStamp - encoder->timeStamp);
     
    encoder->timeStamp = timeStamp;
}

void encoderHandleOverflow(Encoder_E encoderSide, TIM_HandleTypeDef *htim) {
    Encoder_T *encoder = encoder_getInstance(encoderSide);

    uint32_t counter = __HAL_TIM_GET_COUNTER(htim);
    uint32_t cc2 = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_2);
    encoder->overflow += (cc2 > counter) - (cc2 < counter);
}

HAL_StatusTypeDef encodersInit(void) {
    // reset encoder structs
    encoderLeft = (Encoder_T){0, 0, 0, 0, ENCODER_LEFT, HAL_GetTick()};
    encoderRight = (Encoder_T){0, 0, 0, 0, ENCODER_RIGHT, HAL_GetTick()};
    // enable the encoder interrupts to handle overflow
    __HAL_TIM_ENABLE_IT(&ENCODER_TIMER_LEFT_HANDLE, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&ENCODER_TIMER_RIGHT_HANDLE, TIM_IT_UPDATE);

    if (HAL_TIM_Encoder_Start(&ENCODER_TIMER_LEFT_HANDLE, TIM_CHANNEL_ALL) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_TIM_Encoder_Start(&ENCODER_TIMER_RIGHT_HANDLE, TIM_CHANNEL_ALL) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

Encoder_T *encoder_getInstance(Encoder_E encoder) {
    if (encoder == ENCODER_LEFT) {
        return &encoderLeft;
    } else if (encoder == ENCODER_RIGHT) {
        return &encoderRight;
    } else {
        return NULL;
    }
}