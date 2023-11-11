#include "color_sensor.h"

#include "bsp.h"
#include "debug.h"
#include "stm32f4xx_hal.h"

uint32_t F_CLK;

const uint32_t redLow = 2200;
const uint32_t redHigh = 80500;

const uint32_t greenLow = 1000;
const uint32_t greenHigh = 47500;

const uint32_t clearLow = 700;
const uint32_t clearHigh = 47500;

const uint32_t blueLow = 5600;
const uint32_t blueHigh = 170000;

#define COLOR_1_EN() HAL_GPIO_WritePin(COLOR_1_CS_GPIO_Port, COLOR_1_CS_Pin, GPIO_PIN_RESET)
#define COLOR_1_DIS() HAL_GPIO_WritePin(COLOR_1_CS_GPIO_Port, COLOR_1_CS_Pin, GPIO_PIN_SET)

#define COLOR_2_EN() HAL_GPIO_WritePin(COLOR_2_CS_GPIO_Port, COLOR_2_CS_Pin, GPIO_PIN_RESET)
#define COLOR_2_DIS() HAL_GPIO_WritePin(COLOR_2_CS_GPIO_Port, COLOR_2_CS_Pin, GPIO_PIN_SET)

#define COLOR_3_EN() HAL_GPIO_WritePin(COLOR_3_CS_GPIO_Port, COLOR_3_CS_Pin, GPIO_PIN_RESET)
#define COLOR_3_DIS() HAL_GPIO_WritePin(COLOR_3_CS_GPIO_Port, COLOR_3_CS_Pin, GPIO_PIN_SET)

volatile uint8_t isFirstCaptured = 0;
volatile uint32_t colorSensorPeriod = 0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == COLOR_TIMER_INSTANCE) {
        static volatile uint32_t colorSensor_T1 = 0;
        static volatile uint32_t colorSensor_T2 = 0;
        uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&COLOR_TIMER_HANDLE);
        if (isFirstCaptured == 0) {
            colorSensor_T1 = COLOR_TIMER_INSTANCE->CCR1;
            isFirstCaptured = 1;

        } else {
            colorSensor_T2 = COLOR_TIMER_INSTANCE->CCR1;

            if (colorSensor_T2 > colorSensor_T1) {
                colorSensorPeriod = colorSensor_T2 - colorSensor_T1;
            } else {
                colorSensorPeriod = (arr - colorSensor_T1) + colorSensor_T2;
            }

            __HAL_TIM_SET_COUNTER(&COLOR_TIMER_HANDLE, 0);
            isFirstCaptured = 0;
            // Stop capturing after recording 1 pulse to prevent saturation of interrupts on the cpu
            HAL_TIM_Base_Stop_IT(&COLOR_TIMER_HANDLE);
        }
    }
}

/**
 * @brief Takes one sample of the period of the signal and inverts it based on F_CLK frequency
 */
uint32_t colorGetFreq(ColorSensor_E sensor) {
    colorSelectSensor(sensor);
    HAL_TIM_Base_Start_IT(&COLOR_TIMER_HANDLE);
    while (HAL_TIM_Base_GetState(&COLOR_TIMER_HANDLE) != HAL_TIM_STATE_READY)
        ;
    return F_CLK / colorSensorPeriod;
}

void colorSetFreqScaling(FreqScale_E freqScale) {
    HAL_GPIO_WritePin(COLOR_S0_GPIO_Port, COLOR_S0_GPIO_Port, (freqScale & 0b10) >> 1);
    HAL_GPIO_WritePin(COLOR_S1_GPIO_Port, COLOR_S1_GPIO_Port, freqScale & 0b1);
}

void colorSet(Color_E color) {
    HAL_GPIO_WritePin(COLOR_S2_GPIO_Port, COLOR_S2_Pin, (color & 0b10) >> 1);
    HAL_GPIO_WritePin(COLOR_S3_GPIO_Port, COLOR_S3_Pin, color & 0b1);
}

HAL_StatusTypeDef colorSensorInit() {
    COLOR_1_DIS();
    COLOR_2_DIS();
    COLOR_3_DIS();

    F_CLK = HAL_RCC_GetSysClockFreq();

    if (HAL_TIM_IC_Start_IT(&COLOR_TIMER_HANDLE, TIM_CHANNEL_1) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_TIM_Base_Start_IT(&COLOR_TIMER_HANDLE) != HAL_OK) {
        return HAL_ERROR;
    }

    colorSetFreqScaling(FREQ_SCALE_20);
    colorSet(GREEN);
    return HAL_OK;
}

/**
 * @brief Garauntees that only 1 sensor will be selected at once.
 */
HAL_StatusTypeDef colorSelectSensor(ColorSensor_E sensor) {
    static ColorSensor_E prevSensor = COLOR_ERROR;
    if (prevSensor == sensor) {
        return HAL_OK;
    }
    prevSensor = sensor;
    COLOR_1_DIS();
    COLOR_2_DIS();
    COLOR_3_DIS();

    switch (sensor) {
        case COLOR_1:
            COLOR_1_EN();
            return HAL_OK;
        case COLOR_2:
            COLOR_2_EN();
            return HAL_OK;
        case COLOR_3:
            COLOR_3_EN();
            return HAL_OK;
        default:
            return HAL_ERROR;
    }
}

float getLineError() {
    // dot product of the color vector and weight vector, middle being the highest weight
    static const int32_t NO_LINE_FREQ = 30000;
    static const int32_t LINE_FREQ = 110000;
    static const float WEIGHTS[3] = {0.5, 1, 0.5};
    float error = 0;
    error += (LINE_FREQ - colorGetFreq(COLOR_1) + NO_LINE_FREQ) * WEIGHTS[0];
    error += (LINE_FREQ - colorGetFreq(COLOR_2) + NO_LINE_FREQ) * WEIGHTS[1];
    error += (LINE_FREQ - colorGetFreq(COLOR_3) + NO_LINE_FREQ) * WEIGHTS[2];
    return error;
}