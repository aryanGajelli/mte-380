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
        uprintf("mainTaskInit failed\n");
        Error_Handler();
    }

    while (1) {
        vTaskDelay(10);
    }
}

HAL_StatusTypeDef mainTaskInit() {
    if (ICMInit() != HAL_OK) {
        uprintf("ICMInit failed\n");
        return HAL_ERROR;
    }

    if (colorSensorInit() != HAL_OK) {
        uprintf("colorSensorInit failed\n");
        return HAL_ERROR;
    }

    if (motorsInit() != HAL_OK) {
        uprintf("motorsInit failed\n");
        return HAL_ERROR;
    }

    if (servoInit() != HAL_OK) {
        uprintf("servoInit failed\n");
        return HAL_ERROR;
    }    

    if (encodersInit() != HAL_OK) {
        uprintf("encodersInit failed\n");
        return HAL_ERROR;
    }    

    return HAL_OK;
}