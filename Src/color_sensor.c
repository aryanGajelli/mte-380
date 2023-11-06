#include "color_sensor.h"

#include "bsp.h"
#include "debug.h"
#include "stm32f4xx_hal.h"

uint32_t F_CLK;

volatile uint16_t gu16_TIM2_OVC = 0;

const uint32_t redLow = 2200;
const uint32_t redHigh = 80500;

const uint32_t greenLow = 1000;
const uint32_t greenHigh = 47500;

const uint32_t clearLow = 700;
const uint32_t clearHigh = 47500;

const uint32_t blueLow = 5600;
const uint32_t blueHigh = 170000;

uint32_t gu32_Freq = 0;

#define COLOR_1_EN() HAL_GPIO_WritePin(COLOR_1_CS_GPIO_Port, COLOR_1_CS_Pin, GPIO_PIN_RESET)
#define COLOR_1_DIS() HAL_GPIO_WritePin(COLOR_1_CS_GPIO_Port, COLOR_1_CS_Pin, GPIO_PIN_SET)

#define COLOR_2_EN() HAL_GPIO_WritePin(COLOR_2_CS_GPIO_Port, COLOR_2_CS_Pin, GPIO_PIN_RESET)
#define COLOR_2_DIS() HAL_GPIO_WritePin(COLOR_2_CS_GPIO_Port, COLOR_2_CS_Pin, GPIO_PIN_SET)

#define COLOR_3_EN() HAL_GPIO_WritePin(COLOR_3_CS_GPIO_Port, COLOR_3_CS_Pin, GPIO_PIN_RESET)
#define COLOR_3_DIS() HAL_GPIO_WritePin(COLOR_3_CS_GPIO_Port, COLOR_3_CS_Pin, GPIO_PIN_SET)

volatile uint8_t isFirstCaptured = 0;
volatile uint32_t gu32_Ticks = 0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == COLOR_TIMER_INSTANCE) {
        static volatile uint32_t gu32_T1 = 0;
        static volatile uint32_t gu32_T2 = 0;
        if (isFirstCaptured == 0) {
            gu32_T1 = COLOR_TIMER_INSTANCE->CCR1;
            isFirstCaptured = 1;

        } else {
            gu32_T2 = COLOR_TIMER_INSTANCE->CCR1;

            if (gu32_T2 > gu32_T1) {
                gu32_Ticks = gu32_T2 - gu32_T1;
            } else {
                gu32_Ticks = (0xFFFF - gu32_T1) + gu32_T2;
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
int32_t getFreq() {
    HAL_TIM_Base_Start_IT(&COLOR_TIMER_HANDLE);
    while (HAL_TIM_Base_GetState(&COLOR_TIMER_HANDLE) != HAL_TIM_STATE_READY)
        ;
    return F_CLK / gu32_Ticks;
}

void setColorSensorFreqScaling(FreqScale_E freqScale) {
    HAL_GPIO_WritePin(COLOR_S0_GPIO_Port, COLOR_S0_GPIO_Port, (freqScale & 0b10) >> 1);
    HAL_GPIO_WritePin(COLOR_S1_GPIO_Port, COLOR_S1_GPIO_Port, freqScale & 0b1);
}

char* colorToStr(Color_E color) {
    switch (color) {
        case RED:
            return "RED";
            break;
        case GREEN:
            return "GRN";
            break;
        case BLUE:
            return "BLU";
            break;
        case CLEAR:
            return "CLR";
            break;
    }
    return "";
}

void setColor(Color_E color) {
    HAL_GPIO_WritePin(COLOR_S2_GPIO_Port, COLOR_S2_Pin, (color & 0b10) >> 1);
    HAL_GPIO_WritePin(COLOR_S3_GPIO_Port, COLOR_S3_Pin, color & 0b1);
}

void colorSensorInit() {
    COLOR_1_DIS();
    COLOR_2_DIS();
    COLOR_3_DIS();

    F_CLK = HAL_RCC_GetSysClockFreq();
    HAL_TIM_Base_Start_IT(&COLOR_TIMER_HANDLE);
    HAL_TIM_IC_Start_IT(&COLOR_TIMER_HANDLE, TIM_CHANNEL_1);
    setColorSensorFreqScaling(FREQ_SCALE_20);
    setColor(BLUE);
}

/**
 * @brief Garauntees that only 1 sensor will be selected at once.
 */
HAL_StatusTypeDef selectColorSensor(ColorSensor_E cs) {
    COLOR_1_DIS();
    COLOR_2_DIS();
    COLOR_3_DIS();

    switch (cs) {
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
    selectColorSensor(COLOR_1);
    error += (LINE_FREQ - getFreq(COLOR_1) + NO_LINE_FREQ) * WEIGHTS[0];
    selectColorSensor(COLOR_2);
    error += (LINE_FREQ - getFreq(COLOR_2) + NO_LINE_FREQ) * WEIGHTS[1];
    selectColorSensor(COLOR_3);
    error += (LINE_FREQ - getFreq(COLOR_3) + NO_LINE_FREQ) * WEIGHTS[2];
    return error;
}