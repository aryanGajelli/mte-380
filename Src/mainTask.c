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
    uprintf("mainTask\n");
    if (ICMInit() != HAL_OK) {
        Error_Handler();
    }

    while (1){
        // print adc values
        vTaskDelay(20);
    }
}