#include <math.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "bsp.h"
#include "color_sensor.h"
#include "debug.h"
#include "demo.h"
#include "encoders.h"
#include "imu.h"
#include "main.h"
#include "mathUtils.h"
#include "motors.h"
#include "sensors.h"
#include "servo.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "task.h"

HAL_StatusTypeDef mainTaskInit();

void mainTask(void *pvParameters) {
    uprintf("mainTask\n");

    if (mainTaskInit() != HAL_OK) {
        Error_Handler();
    }

    double line_deviation = 0;
    // uint32_t max = 0;
    // uint32_t min = 0xFFFFFFFF;
    while (1) {
        // uint32_t freq = colorGetFreq(COLOR_SENSOR_3);
        // if (freq > max) {
        //     max = freq;
        // }
        // if (freq < min) {
        //     min = freq;
        // }
        // uprintf("%lu %lu %lu\n",freq, min, max);

        // uprintf("%lu\t",colorGetFreq(COLOR_SENSOR_2));
        // uprintf("%lu\n",colorGetFreq(COLOR_SENSOR_3));

        uprintf("%.3f\t", colorGetNormalizedOut(COLOR_SENSOR_1));
        uprintf("%.3f\t", colorGetNormalizedOut(COLOR_SENSOR_2));
        uprintf("%.3f\t", colorGetNormalizedOut(COLOR_SENSOR_3));
        uprintf("%d %.3f\n", colorGetLineDeviation(&line_deviation), line_deviation);
    
        vTaskDelay(10);
    }
}

HAL_StatusTypeDef mainTaskInit() {
    if (ICMInit() != HAL_OK) {
        return HAL_ERROR;
    }

    if (colorSensorInit() != HAL_OK) {
        return HAL_ERROR;
    }

    if (motorsInit() != HAL_OK) {
        return HAL_ERROR;
    }

    if (servoInit() != HAL_OK) {
        return HAL_ERROR;
    }

    if (encodersInit() != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}