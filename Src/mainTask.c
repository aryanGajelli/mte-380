#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "debug.h"
#include "bsp.h"
#include "sensors.h"
#include "imu.h"
#include <stdio.h>


void mainTask(void *pvParameters){
    // sensors_t *sensors = getSensors_Handle();
    // uprintf("%ld\n",  HAL_RCC_GetSysClockFreq());
    vTaskDelay(200);
    uprintf("0x%lx\n",  ICM_test());
    while (1){
        // print adc values
        vTaskDelay(20);
    }
}