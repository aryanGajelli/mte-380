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
#include <stdio.h>


void mainTask(void *pvParameters){
    uprintf("mainTask\n");
    servoInit();
    
    while (1){    
        if (getServoAngle() == CLAW_OPEN_ANGLE){
            setServoAngle(CLAW_CLOSED_ANGLE);
            vTaskDelay(700);
        }
        else{
            setServoAngle(CLAW_OPEN_ANGLE);
            vTaskDelay(700);
        }
        vTaskDelay(10);
    }
}