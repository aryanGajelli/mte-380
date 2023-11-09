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
    // getDemoState();
    // demoStateMachine();
    // selectColorSensor(COLOR_1);
    // while (!isSDemoStarted) vTaskDelay(1);

    // servoSetAngle(70);
    // demoRight();
    // HAL_Delay(1000);
    // motorSoftStop();
    // HAL_Delay(500);
    // demoDistSense();
    // servoSetAngle(CLAW_OPEN_ANGLE);
    // HAL_Delay(200);
    // servoSetAngle(CLAW_CLOSED_ANGLE);
    
    // motorSetSpeed(MOTOR_RIGHT, 10);
    // motorSetDir(MOTOR_RIGHT, MOTOR_FWD);
    Encoder_T *encoderLeft = encoder_getInstance(ENCODER_LEFT);
    float x = 0;
    while (1) {
        // motorSetSpeed(MOTOR_LEFT, sinf(x) * 100);
        // motorSetSpeed(MOTOR_RIGHT, cosf(x) * 100);
        // print encoder values
        uprintf("left: %ld, %ld, %lu\n", encoderLeft->position, encoderLeft->velocity, encoderLeft->lastCount);

        // print dist value
        // uprintf("dist: %lf\n", ADC_to_Volt(adcBuf[0]));
        // uprintf("color: %d\n", getFreq());

        vTaskDelay(10);
    }
}