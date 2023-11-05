#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "debug.h"
#include "bsp.h"
#include "sensors.h"
#include "imu.h"
#include "mathUtils.h"
#include "color_sensor.h"
#include "servo.h"
#include "motors.h"
#include <stdio.h>


void mainTask(void *pvParameters){
    uprintf("mainTask\n");
    servoInit();
    motorsInit();
    float dc = 0;
    float inc = 0.1;
    MotorDirection_E dir = MOTOR_FWD;
    while (1){
        setMotorDutyCycle(MOTOR_LEFT, dc);
        setMotorDutyCycle(MOTOR_RIGHT, dc);
        dc += inc;
        if (dc >= 99.8){
            inc = -0.1;
        }
        if (dc <= 0.1){
            inc = 0.1;
            if (dir == MOTOR_FWD){
                dir = MOTOR_BWD;
            } else {
                dir = MOTOR_FWD;
            }
            setMotorDir(MOTOR_LEFT, dir);
            setMotorDir(MOTOR_RIGHT, dir);
        }
        vTaskDelay(10);
    }
}