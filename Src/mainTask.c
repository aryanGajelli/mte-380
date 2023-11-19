#include <math.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "ICM20948_register.h"
#include "ICM20948_spi.h"
#include "arm_math.h"
#include "bsp.h"
#include "color_sensor.h"
#include "control.h"
#include "debug.h"
#include "demo.h"
#include "encoders.h"
#include "fusion.h"
#include "imu.h"
#include "imu2.h"
#include "main.h"
#include "mathUtils.h"
#include "motion_fx.h"
#include "motors.h"
#include "odometry.h"
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

    TickType_t xLastWakeTime = xTaskGetTickCount();
    IMUData_T imuData;
    // ICM_ReadAccelGyro(&imuData);
    double k = 0.1;
    ICM_20948_AGMT_t agmt;
    while (1) {
        ICM_20948_get_agmt(&imu, &agmt);
        // ICM_ReadAccelGyro(&imuData);
        // print rotation
        uprintf("x: %d, y: %d, z: %d\n", agmt.mag.axes.x, agmt.mag.axes.y, agmt.mag.axes.z);
        vTaskDelayUntil(&xLastWakeTime, 10);
    }
}

HAL_StatusTypeDef mainTaskInit() {
    if (imuInit() != HAL_OK) {
        Error_Handler();
    }
    // if (ICMInit() != HAL_OK) {
    //     Error_Handler();
    // }

    if (colorSensorInit() != HAL_OK) {
        Error_Handler();
    }

    if (motorsInit() != HAL_OK) {
        Error_Handler();
    }

    if (servoInit() != HAL_OK) {
        Error_Handler();
    }

    // if (fusionInit() != HAL_OK) {
    //     Error_Handler();
    // }

    // if (odometryInit() != HAL_OK) {
    //     Error_Handler();
    // }
    if (encodersInit() != HAL_OK) {
        Error_Handler();
    }

    return HAL_OK;
}