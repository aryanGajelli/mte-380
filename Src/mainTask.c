#include <stdio.h>

#include "FreeRTOS.h"
#include "arm_math.h"
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

    IMUData_T imuData;
    ICM_CalibrateGyro();
    while (1) {
        ICM_Read(&imuData);

        uprintf("%.3f\t %.3f  %.3f %.3f\n", imuData.mag.x, imuData.mag.y, imuData.mag.z, 180/PI * atan2(imuData.mag.x, imuData.mag.y));

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