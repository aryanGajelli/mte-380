#include "color_sensor.h"

#include "bsp.h"
#include "debug.h"
#include "mathUtils.h"
#include "stm32f4xx_hal.h"

uint32_t F_CLK;
ColorSensor_T colorSensors;

#define COLOR_1_EN() HAL_GPIO_WritePin(COLOR_1_CS_GPIO_Port, COLOR_1_CS_Pin, GPIO_PIN_RESET)
#define COLOR_1_DIS() HAL_GPIO_WritePin(COLOR_1_CS_GPIO_Port, COLOR_1_CS_Pin, GPIO_PIN_SET)

#define COLOR_2_EN() HAL_GPIO_WritePin(COLOR_2_CS_GPIO_Port, COLOR_2_CS_Pin, GPIO_PIN_RESET)
#define COLOR_2_DIS() HAL_GPIO_WritePin(COLOR_2_CS_GPIO_Port, COLOR_2_CS_Pin, GPIO_PIN_SET)

#define COLOR_3_EN() HAL_GPIO_WritePin(COLOR_3_CS_GPIO_Port, COLOR_3_CS_Pin, GPIO_PIN_RESET)
#define COLOR_3_DIS() HAL_GPIO_WritePin(COLOR_3_CS_GPIO_Port, COLOR_3_CS_Pin, GPIO_PIN_SET)

HAL_StatusTypeDef colorSensorInit() {
    COLOR_1_DIS();
    COLOR_2_DIS();
    COLOR_3_DIS();

    colorSensors = (ColorSensor_T){.lineDeviation = 0,
                                   .normalizedOut = {0, 0, 0},
                                   .freq = {0, 0, 0},
                                   .surface = SURFACE_WOOD};
    F_CLK = HAL_RCC_GetSysClockFreq();
    if (HAL_TIM_Encoder_Start(&COLOR_TIMER_HANDLE, TIM_CHANNEL_ALL) != HAL_OK) {
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
    static const uint32_t DELAY_TIME_MS = 3;
    static TickType_t lastWakeTime;
    lastWakeTime = xTaskGetTickCount();
    taskENTER_CRITICAL();
    uint32_t start = HAL_GetTick();
    uint32_t c1 = __HAL_TIM_GET_COUNTER(&COLOR_TIMER_HANDLE);
    taskEXIT_CRITICAL();
    vTaskDelayUntil(&lastWakeTime, DELAY_TIME_MS);
    taskENTER_CRITICAL();
    uint32_t c2 = __HAL_TIM_GET_COUNTER(&COLOR_TIMER_HANDLE);
    uint32_t end = HAL_GetTick();
    taskEXIT_CRITICAL();
    if (c1 < c2) {
        c1 += __HAL_TIM_GET_AUTORELOAD(&COLOR_TIMER_HANDLE);
    }
    return (uint32_t)(500. / (end - start) * (c1 - c2));
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
#define NUM_SAMPLES 10
uint32_t c1[NUM_SAMPLES];
uint32_t c2[NUM_SAMPLES];
uint32_t c3[NUM_SAMPLES];

uint32_t counter[3] = {0, 0, 0};

uint32_t sum[3] = {0, 0, 0};
void colorUpdate() {
    // color readings already has delays of 5ms each
    // moving average of samples
    uint32_t val = colorGetFreq(COLOR_SENSOR_1);
    sum[0] = sum[0] + val - c1[counter[0]];
    colorSensors.freq[0] = sum[0] / NUM_SAMPLES;
    c1[counter[0]] = val;
    counter[0] = (counter[0] + 1) % NUM_SAMPLES;

    // val = colorGetFreq(COLOR_SENSOR_2);
    // sum[1] = sum[1] + val - c2[counter[1]];
    // colorSensors.freq[1] = sum[1] / NUM_SAMPLES;
    // c2[counter[1]] = val;
    // counter[1] = (counter[1] + 1) % NUM_SAMPLES;

    val = colorGetFreq(COLOR_SENSOR_3);
    sum[2] = sum[2] + val - c3[counter[2]];
    colorSensors.freq[2] = sum[2] / NUM_SAMPLES;
    c3[counter[2]] = val;
    counter[2] = (counter[2] + 1) % NUM_SAMPLES;

    colorSensors.normalizedOut.x = colorGetNormalizedOut(COLOR_SENSOR_1);
    colorSensors.normalizedOut.y = colorGetNormalizedOut(COLOR_SENSOR_2);
    colorSensors.normalizedOut.z = colorGetNormalizedOut(COLOR_SENSOR_3);
    colorSensors.surface = colorGetLineDeviation(&colorSensors.lineDeviation);
}

/**
 * @brief Normalizes the values of all 3 sensors to be between 0 and 1.
 *        With 1 representing wood and 0 representing red tape.
 * @param sensor The sensor to get the normalized value of.
 */
double colorGetNormalizedOut(ColorSensor_E sensor) {
    // Home Values
    // static const uint32_t c1_wood = 41500, c2_wood = 55500, c3_wood = 39500;
    // static const uint32_t c1_tape = 27300, c2_tape = 28000, c3_tape = 23500;
    // Field Values
    static const uint32_t c1_wood = 45800, c2_wood = 60000, c3_wood = 41500;
    static const uint32_t c1_tape = 21500, c2_tape = 21000, c3_tape = 20000;
    switch (sensor) {
        case COLOR_SENSOR_1:
            return map(colorSensors.freq[0], c1_tape, c1_wood, 0, 1);
        case COLOR_SENSOR_2:
            return map(colorSensors.freq[1], c2_tape, c2_wood, 0, 1);
        case COLOR_SENSOR_3:
            return map(colorSensors.freq[2], c3_tape, c3_wood, 0, 1);
        default:
            return -255;
    }
}

/**
 * @brief Returns the surface type based on the color sensor readings.
 * @param c1 The normalized value of color sensor 1.
 * @param c2 The normalized value of color sensor 2.
 * @param c3 The normalized value of color sensor 3.
 */
SurfaceType_E colorDetectSurface(double c1, double c2, double c3) {
    static const double WOOD_THRESHOLD = 0.8;
    static const double BLACK_THRESHOLD = 0.06;
    // if 2 sensors are above the wood threshold, then we are on wood
    if (c1 > WOOD_THRESHOLD && c2 > WOOD_THRESHOLD)
        return SURFACE_WOOD;
    if (c1 > WOOD_THRESHOLD && c3 > WOOD_THRESHOLD)
        return SURFACE_WOOD;
    if (c2 > WOOD_THRESHOLD && c3 > WOOD_THRESHOLD)
        return SURFACE_WOOD;
    if (c1 > 0.75 * WOOD_THRESHOLD && c2 > 0.75 * WOOD_THRESHOLD && c3 > 0.75 * WOOD_THRESHOLD)
        return SURFACE_WOOD;

    // if 2 sensors are below the black threshold, then we are on black tape
    if (c1 < BLACK_THRESHOLD && c2 < BLACK_THRESHOLD)
        return SURFACE_BLACK;
    if (c1 < BLACK_THRESHOLD && c3 < BLACK_THRESHOLD)
        return SURFACE_BLACK;
    if (c2 < BLACK_THRESHOLD && c3 < BLACK_THRESHOLD)
        return SURFACE_BLACK;
    return SURFACE_TAPE;
}

/**
 * @brief Returns a weighted average of the 3 sensors.
 *        If the average of the 3 sensors is above a wood threshold, then the average is added to the weighted average.
 *        This is to prevent the robot from thinking it is on wood when it is on red tape.
 *        See https://theultimatelinefollower.blogspot.com/2015/12/interpolation.html
 */
SurfaceType_E colorGetLineDeviation(double* out) {
    // measured sensor displacements from center of robot
    static const double sensorLocs_mm[3] = {15, 0, -17.95};

    double c1 = colorSensors.normalizedOut.x;
    double c2 = colorSensors.normalizedOut.y;
    double c3 = colorSensors.normalizedOut.z;

    double num = sensorLocs_mm[0] * (1-c3) + sensorLocs_mm[1] * (1-c2) + sensorLocs_mm[2] * (1-c1);
    double denom = c1 + c2 + c3;

    *out = num;

    return colorDetectSurface(c1, c2, c3);
}