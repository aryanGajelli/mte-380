#include <stdio.h>

#include "FreeRTOS.h"
#include "ICM20948_register.h"
#include "ICM20948_spi.h"
#include "arm_math.h"
#include <math.h>
#include "bsp.h"
#include "color_sensor.h"
#include "debug.h"
#include "demo.h"
#include "encoders.h"
#include "fusion.h"
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
    MFX_input_t data_in;
    MFX_output_t data_out;
    IMUData_T imuData;
    IMUData_T prevImuData;
    ICM_Read(&imuData);
    float dT;
    float *q;
    float *g;
    while (1) {
        prevImuData = imuData;
        ICM_Read(&imuData);
        
        data_in.gyro[0] = imuData.gyro.x;
        data_in.gyro[1] = imuData.gyro.y;
        data_in.gyro[2] = imuData.gyro.z;
        data_in.acc[0] = imuData.accel.x / 9.81;
        data_in.acc[1] = imuData.accel.y / 9.81;
        data_in.acc[2] = imuData.accel.z / 9.81;
        data_in.mag[0] = imuData.mag.x/50;
        data_in.mag[1] = imuData.mag.y/50;
        data_in.mag[2] = imuData.mag.z/50;

        dT = (imuData.timestamp - prevImuData.timestamp) / 1000.0f;
        MotionFX_propagate(&data_out, &data_in, &dT);
        MotionFX_update(&data_out, &data_in, &dT, NULL);
        q = data_out.quaternion_9X;
        g = data_out.rotation_9X;
        uprintf("g: %.3f, %.3f, %.3f\n", g[0], g[1], g[2]);
        // print q
        // uprintf("q: %f, %f, %f, %f\n", q[0], q[1], q[2], q[3]);
        // uprintf("h: %f\n", data_out.heading_9X);
        // uprintf("dT: %f\n", dT);
        // print acc gyro mag
        // uprintf("acc: %.3f, %.3f, %.3f\n", data_in.acc[0], data_in.acc[1], data_in.acc[2]);
        // uprintf("gyro: %.3f, %.3f, %.3f\n", imuData.gyro.x, imuData.gyro.y, imuData.gyro.z);
        // uprintf("mag: %.3f, %.3f, %.3f\n", imuData.mag.x, imuData.mag.y, imuData.mag.z); 
        // vTaskDelay(5);
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

    if (fusionInit() != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}