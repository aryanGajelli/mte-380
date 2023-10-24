#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "debug.h"
#include "bsp.h"
#include "sensors.h"
#include <stdio.h>


void mainTask(void *pvParameters){
    sensors_t *sensors = getSensors_Handle();
    uprintf("%ld\n",  HAL_RCC_GetSysClockFreq());
    while (1){
        // print adc values
        // uprintf("IR: %lf %lf %lf %lf %lf %lf\n", sensors->ir1, sensors->ir2, sensors->ir3, sensors->ir4, sensors->ir5, sensors->dist);
        // HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        vTaskDelay(20);
    }
}