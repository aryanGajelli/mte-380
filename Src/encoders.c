#include "encoders.h"

#include "bsp.h"
#include "stm32f4xx_hal.h"
#include "debug.h"
#include "main.h"

volatile Encoder_T encoderLeft;
volatile Encoder_T encoderRight;
int32_t encoderLeftVelocity;

void update_encoder(Encoder_E encoderSide) {
    TIM_HandleTypeDef *htim;
    Encoder_T *encoder = encoder_getInstance(encoderSide);
    
    if (encoderSide == ENCODER_LEFT) {
        htim = &ENCODER_TIMER_LEFT_HANDLE;
    } else if (encoderSide == ENCODER_RIGHT){
        htim = &ENCODER_TIMER_RIGHT_HANDLE;
    } else {
        Error_Handler();
    }

    uint32_t temp_counter = __HAL_TIM_GET_COUNTER(htim);
    uint32_t temp_arr = __HAL_TIM_GET_AUTORELOAD(htim);
    static uint8_t first_time = 0;

    if (!first_time) {
        encoder->velocity = 0;
        first_time = 1;
    } else {
        if (temp_counter == encoder->lastCount) {
            encoder->velocity = 0;
        } else if (temp_counter > encoder->lastCount) {
            if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
                encoder->velocity = -encoder->lastCount - (temp_arr - temp_counter);
            else
                encoder->velocity = temp_counter - encoder->lastCount;

        } else {
            if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
                encoder->velocity = temp_counter - encoder->lastCount;
            else
                encoder->velocity = temp_counter + (temp_arr - encoder->lastCount);
        }
    }
    encoder->position += encoder->velocity;
    encoder->lastCount = temp_counter;
}

HAL_StatusTypeDef encodersInit(void) {
    __HAL_TIM_ENABLE_IT(&ENCODER_TIMER_LEFT_HANDLE, TIM_IT_CC1);
    __HAL_TIM_ENABLE_IT(&ENCODER_TIMER_LEFT_HANDLE, TIM_IT_UPDATE);
    // __HAL_TIM_ENABLE_IT(&ENCODER_TIMER_LEFT_HANDLE, TIM_IT_CC2);
    // __HAL_TIM_ENABLE_IT(&ENCODER_TIMER_RIGHT_HANDLE, TIM_IT_COM);
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

int32_t encoder_getCount(Encoder_E encoder) {
    if (encoder == ENCODER_LEFT) {
        return ENCODER_TIMER_LEFT_INSTANCE->CNT;
    } else if (encoder == ENCODER_RIGHT) {
        return ENCODER_TIMER_RIGHT_INSTANCE->CNT;
    } else {
        return -1;
    }
}