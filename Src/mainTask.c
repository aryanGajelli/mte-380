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

    Encoder_T *encLeft = encoder_getInstance(ENCODER_LEFT);
    Encoder_T *encRight = encoder_getInstance(ENCODER_RIGHT);
    uint32_t start = HAL_GetTick();
    while (!isSDemoStarted) {
        if (HAL_GetTick() - start > 7000) {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        }
        vTaskDelay(100);
    }

    if (controlInit() != HAL_OK) {
        Error_Handler();
    }

    uint32_t dT = HAL_GetTick();
    Pose_T pose = {.x = 0, .y = 0, .theta = 0};
    Pose_T prevPose = {.x = encLeft->dist, .y = encRight->dist, .theta = odometryGetHeading()};
    TickType_t xLastWakeTime = xTaskGetTickCount();
    IMUData_T imuData;
    ICM_ReadAccelGyro(&imuData);
    double k = 0.1;
    while (1) {
        dT = HAL_GetTick() - imuData.timestamp;
        encoderUpdate(encLeft);
        encoderUpdate(encRight);
        ICM_ReadAccelGyro(&imuData);
        if (fabs(imuData.gyro.z) < 0.1)
            imuData.gyro.z = 0;
        // pose.theta = odometryGetHeading();

        double dL = encLeft->dist - prevPose.x;
        double dR = encRight->dist - prevPose.y;
        double LR = (encLeft->dist + encRight->dist) / 2;
        double d = (dL + dR) / 2;

        double dTheta = k * RAD_TO_DEG((dR - dL)/ WHEEL_TO_WHEEL_DISTANCE) + (1-k) * imuData.gyro.z * dT / 1000.;
        if (dTheta > 180) {
            dTheta -= 360;
        } else if (dTheta < -180) {
            dTheta += 360;
        }

        pose.theta += dTheta;

        pose.x += d * sin(DEG_TO_RAD(pose.theta + dTheta / 2));
        pose.y += d * cos(DEG_TO_RAD(pose.theta + dTheta / 2));

        prevPose.x = encLeft->dist;
        prevPose.y = encRight->dist;
        prevPose.theta = pose.theta;
        odometrySetPose(pose);

        // uprintf("%.3f %.3f %.2f %.3f\n", pose.x, pose.y, pose.theta, dTheta);
        vTaskDelayUntil(&xLastWakeTime, 4);
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

    if (fusionInit() != HAL_OK) {
        Error_Handler();
    }

    if (odometryInit() != HAL_OK) {
        Error_Handler();
    }
    if (encodersInit() != HAL_OK) {
        Error_Handler();
    }

    return HAL_OK;
}