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
    colorSensorInit();
    // getDemoState();
    // demoStateMachine();
    // selectColorSensor(COLOR_1);
    demoFwd();
    vTaskDelay(1000);
    demoBwd();
    vTaskDelay(1000);
    demoLeft();
    vTaskDelay(500);
    demoRight();
    vTaskDelay(500);
    demoDistSense();
    while (1){
        // print dist value
        uprintf("dist: %lf\n", ADC_to_Volt(adcBuf[0]));
        // uprintf("color: %d\n", getFreq());
        vTaskDelay(10);
    }
}