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
    IMUData_T prevImuData;
    ICM_Read(&imuData);
    ICM_CalibrateGyro();
    double hGyro = 180 / PI * atan2(imuData.mag.x, imuData.mag.y);
    double k = 0.95;
    while (1) {
        prevImuData = imuData;
        ICM_Read(&imuData);
        if (fabs(imuData.gyro.x) < 0.5) imuData.gyro.x = 0;
        if (fabs(imuData.gyro.y) < 0.5) imuData.gyro.y = 0;
        if (fabs(imuData.gyro.z) < 0.5) imuData.gyro.z = 0;
        double hMag = 180 / PI * atan2(imuData.mag.x, imuData.mag.y);
        hGyro = fmod(hGyro + imuData.gyro.z * (imuData.timestamp - prevImuData.timestamp) / 1000., 360);
        hGyro = k * hGyro + (1 - k) * hMag;

        uprintf("%.3f %.3f %.3f\n",hMag, hGyro, imuData.gyro.z);

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