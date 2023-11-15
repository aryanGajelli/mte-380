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

    while (!isSDemoStarted) {
        vTaskDelay(10);
    }

    if (controlInit() != HAL_OK) {
        Error_Handler();
    }

    uint32_t dT = HAL_GetTick();
    Pose_T pose = {.x = 0, .y = 0, .theta = 0};
    double prevDistL = encLeft->dist, prevDistR = encRight->dist;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        dT = HAL_GetTick() - dT;
        encoderUpdate(encLeft);
        encoderUpdate(encRight);

        double dTheta = odometryGetDeltaHeading();
        double dL = encLeft->dist - prevDistL;
        double dR = encRight->dist - prevDistR;
        double LR = (encLeft->dist + encRight->dist) / 2;
        double d = (dL + dR) / 2;
        pose.theta = odometryGetHeading();
        pose.x += d * sin((pose.theta + dTheta / 2) * PI / 180);
        pose.y += -d * cos((pose.theta + dTheta / 2) * PI / 180);

        prevDistL = encLeft->dist;
        prevDistR = encRight->dist;
        odometrySetPoseXY(pose);
        vTaskDelayUntil(&xLastWakeTime, 10);
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