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
#include "demo.h"
#include <stdio.h>


void mainTask(void *pvParameters){
    uprintf("mainTask\n");
    servoInit();
    motorsInit();
    getDemoState();
    demoStateMachine();
    while (1){
        vTaskDelay(10);
    }
}