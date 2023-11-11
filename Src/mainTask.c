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
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "task.h"

void mainTask(void *pvParameters) {
    uprintf("mainTask\n");
    servoInit();

    motorsInit();
    colorSensorInit();
    // getDemoState();
    // demoStateMachine();
    // selectColorSensor(COLOR_1);
    while (!isSDemoStarted) vTaskDelay(1);
    setServoAngle(CLAW_OPEN_ANGLE);
    HAL_Delay(500);
    demoFwd();
    HAL_Delay(1000);
    motorSoftStop();
    HAL_Delay(500);
    demoBwd();
    HAL_Delay(1000);
    motorSoftStop();
    HAL_Delay(500);
    demoLeft();
    HAL_Delay(1000);
    motorSoftStop();
    HAL_Delay(500);
    demoRight();
    HAL_Delay(1000);
    motorSoftStop();
    HAL_Delay(500);
    demoDistSense();
    HAL_Delay(200);
    setServoAngle(CLAW_CLOSED_ANGLE);

    while (1) {
        // print dist value
        // uprintf("dist: %lf\n", ADC_to_Volt(adcBuf[0]));
        // uprintf("color: %d\n", getFreq());
        vTaskDelay(10);
    }
}