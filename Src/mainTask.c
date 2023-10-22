#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"



void mainTask(void *pvParameters){
    TickType_t i = 0;
    while (1){
        // blink led
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        vTaskDelay((i+=50)%1000);
    }
}