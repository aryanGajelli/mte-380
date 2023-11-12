#include "color_sensor.h"

#include "bsp.h"
#include "debug.h"
#include "mathUtils.h"
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
 * @brief Takes multiple samples of the period of the signal and inverts it based on F_CLK frequency
 */
uint32_t colorGetFreq(ColorSensor_E sensor) {
    colorSelectSensor(sensor);
// Need to take 2 samples as first one is always junky when switching between sensors
#define NTH_SAMPLE_VALID 2
    for (uint8_t i = 0; i < NTH_SAMPLE_VALID; i++) {
        HAL_TIM_Base_Start_IT(&COLOR_TIMER_HANDLE);
        while (HAL_TIM_Base_GetState(&COLOR_TIMER_HANDLE) != HAL_TIM_STATE_READY)
            ;
    }
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

/**
 * @brief Garauntees that only 1 sensor will be selected at once.
 * @param sensor The sensor to select.
 */
HAL_StatusTypeDef colorSelectSensor(ColorSensor_E sensor) {
    static ColorSensor_E prevSensor = COLOR_SENSOR_ERROR;
    if (prevSensor == sensor) {
        return HAL_OK;
    }
    prevSensor = sensor;

    COLOR_1_DIS();
    COLOR_2_DIS();
    COLOR_3_DIS();

    switch (sensor) {
        case COLOR_SENSOR_1:
            COLOR_1_EN();
            return HAL_OK;
        case COLOR_SENSOR_2:
            COLOR_2_EN();
            return HAL_OK;
        case COLOR_SENSOR_3:
            COLOR_3_EN();
            return HAL_OK;
        default:
            return HAL_ERROR;
    }
}

/**
 * @brief Normalizes the values of all 3 sensors to be between 0 and 1.
 *        With 1 representing wood and 0 representing red tape.
 * @param sensor The sensor to get the normalized value of.
 */
double colorGetNormalizedOut(ColorSensor_E sensor) {
    static const uint32_t c1_wood = 49500, c2_wood = 66500, c3_wood = 44500;
    static const uint32_t c1_tape = 30300, c2_tape = 35200, c3_tape = 26900;
    uint32_t freq = colorGetFreq(sensor);
    switch (sensor) {
        case COLOR_SENSOR_1:
            return map(freq, c1_tape, c1_wood, 0, 1);
        case COLOR_SENSOR_2:
            return map(freq, c2_tape, c2_wood, 0, 1);
        case COLOR_SENSOR_3:
            return map(freq, c3_tape, c3_wood, 0, 1);
        default:
            return -255;
    }
}

/**
 * @brief Returns a weighted average of the 3 sensors.
 *        If the average of the 3 sensors is above a wood threshold, then the average is added to the weighted average.
 *        This is to prevent the robot from thinking it is on wood when it is on red tape.
 *        See https://theultimatelinefollower.blogspot.com/2015/12/interpolation.html
 */
SurfaceType_E colorGetLineDeviation(double* out) {
    #define WOOD_THRESHOLD 0.95
    // measured sensor displacements from center of robot
    static const double sensorLocs_mm[3] = {15, 0, 17.95};

    double c1 = colorGetNormalizedOut(COLOR_SENSOR_1);
    double c2 = colorGetNormalizedOut(COLOR_SENSOR_2);
    double c3 = colorGetNormalizedOut(COLOR_SENSOR_3);

    double num = sensorLocs_mm[0] * (c3 - c1);
    double denom = c1 + c2 + c3;

    *out = num / denom + sensorLocs_mm[1];

    // if 2 sensors are above the wood threshold, then we are on wood
    if (c1 > WOOD_THRESHOLD && c2 > WOOD_THRESHOLD)
        return SURFACE_WOOD;
    if (c1 > WOOD_THRESHOLD && c3 > WOOD_THRESHOLD)
        return SURFACE_WOOD;
    if (c2 > WOOD_THRESHOLD && c3 > WOOD_THRESHOLD)
        return SURFACE_WOOD;

    return SURFACE_TAPE;
}