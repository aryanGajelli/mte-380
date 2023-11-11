#include <stdio.h>

#include "FreeRTOS.h"
#include "bsp.h"
#include "color_sensor.h"
#include "debug.h"
#include "demo.h"
#include "imu.h"
#include "main.h"
#include "mathUtils.h"
#include "motors.h"
#include "sensors.h"
#include "servo.h"
#include "encoders.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "task.h"
#include <math.h>

void mainTask(void *pvParameters) {
    uprintf("mainTask\n");
    servoInit();

    motorsInit();
    encodersInit();
    colorSensorInit();
    if (ICMInit() != HAL_OK) {
        uprintf("ICMInit failed\n");
        Error_Handler();

    }
    // motorSetSpeed(MOTOR_LEFT, 100);
    // motorSetSpeed(MOTOR_RIGHT, 100);
    // uint32_t start = HAL_GetTick();
    // uint32_t deltaT_ms = HAL_GetTick();
    // // motorSetSpeed(MOTOR_RIGHT, -100);
    // // HAL_Delay(1000);
    // // motorSoftStop(MOTOR_LEFT);
    Encoder_T *encoderLeft = encoder_getInstance(ENCODER_LEFT);
    Encoder_T *encoderRight = encoder_getInstance(ENCODER_RIGHT);

    // p controller
    // float Kp = -0.1;
    // float Kd = 2.85;
    // int32_t target = -25000;
    // float error = 0;
    // float prevError = 0;

    while (1){
        // ICM_Read(&imuData);
        // uprintf("accel: %f %f %f\n", imuData.accel.x, imuData.accel.y, imuData.accel.z);
        // uprintf("gyro: %f %f %f\n", imuData.gyro.x, imuData.gyro.y, imuData.gyro.z);
        // uprintf("mag: %f %f %f\n", imuData.mag.x, imuData.mag.y, imuData.mag.z);
        // error = target - encoderLeft->position;
        // float dutyCycle = Kp * error + Kd * (error - prevError);
        // if (dutyCycle > 100) dutyCycle = 100;
        // if (dutyCycle < -100) dutyCycle = -100;
        // motorSetSpeed(MOTOR_RIGHT, dutyCycle);
        // prevError = error;
        encoderUpdate(encoderLeft);
        encoderUpdate(encoderRight);
        uprintf("l: %ld r: %ld\n",encoderLeft->position, encoderRight->position);
        vTaskDelay(10);
    }
}