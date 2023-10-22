#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "debug.h"
#include "bsp.h"
#include "sensors.h"


void mainTask(void *pvParameters){
    uprintf("Starting up\n");
    sensors_t *sensors = getSensors_Handle();
    while (1){
        // print adc values

        uprintf("IR: %lu %lu %lu %lu %lu %lu\n", sensors->ir1, sensors->ir2, sensors->ir3, sensors->ir4, sensors->ir5, sensors->dist);
        // HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        // vTaskDelay(20);
    }
}