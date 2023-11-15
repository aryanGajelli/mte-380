#include <math.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "ICM20948_register.h"
#include "ICM20948_spi.h"
#include "arm_math.h"
#include "bsp.h"
#include "color_sensor.h"
#include "debug.h"
#include "demo.h"
#include "encoders.h"
#include "fusion.h"
#include "imu.h"
#include "main.h"
#include "mathUtils.h"
#include "motion_fx.h"
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
    MFX_output_t data_out;
    while (1) {
        prevImuData = imuData;
        ICM_Read(&imuData);
        fusionGetOutputs(&data_out, imuData, prevImuData);
        if (isSDemoStarted) break;
    }

    Encoder_T *encLeft = encoder_getInstance(ENCODER_LEFT);
    Encoder_T *encRight = encoder_getInstance(ENCODER_RIGHT);
    // p-controlloer for angle
    double target = 90;  // deg
    double kp = 0.75, kd = 1;
    float *q;
    double prevError = target;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        prevImuData = imuData;
        ICM_Read(&imuData);
        fusionGetOutputs(&data_out, imuData, prevImuData);
        encoderUpdate(encLeft);
        encoderUpdate(encRight);

        double h = data_out.rotation_6X[0];
        double error = target - h;
        if (fabs(error) < 1) error = 0;


        double dc = kp * error + kd * (error - prevError);
        if (dc > 100) dc = 100;
        if (dc < -100) dc = -100;
        // dc *= 0.6;
        motorSetSpeed(MOTOR_LEFT, dc);
        // motorSetSpeed(MOTOR_RIGHT, dc);
        prevError = error;

        uprintf("%.3f %.3f %d %d\n", h, dc, encLeft->ticks, encRight->ticks);
        vTaskDelayUntil(&xLastWakeTime, 25);
    }
}

HAL_StatusTypeDef mainTaskInit() {
    if (ICMInit() != HAL_OK) {
        Error_Handler();
    }

    if (colorSensorInit() != HAL_OK) {
        Error_Handler();
    }

    if (motorsInit() != HAL_OK) {
        Error_Handler();
    }

    if (servoInit() != HAL_OK) {
        Error_Handler();
    }

    if (encodersInit() != HAL_OK) {
        Error_Handler();
    }

    if (fusionInit() != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}