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

    Encoder_T *encoderLeft = encoder_getInstance(ENCODER_LEFT);
    while (1) {
        // uprintf("%lu\n", __HAL_TIM_GET_COUNTER(&ENCODER_TIMER_LEFT_HANDLE));
        uprintf("ev1: %ld ev2: %ld ep: %ld ec: %lu\n",encoderLeftVelocity, encoderLeft->velocity,encoderLeft->position,encoderLeft->lastCount);

        vTaskDelay(30);
    }
}