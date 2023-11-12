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

    while (1) {

        uprintf("%.3f\t", colorGetNormalizedOut(COLOR_SENSOR_1));
        uprintf("%.3f\t", colorGetNormalizedOut(COLOR_SENSOR_2));
        uprintf("%.3f\t", colorGetNormalizedOut(COLOR_SENSOR_3));
        uprintf("%.3f\n", colorGetWeightedValue());
    
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