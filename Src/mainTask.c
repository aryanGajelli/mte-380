#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "debug.h"
#include "bsp.h"


void mainTask(void *pvParameters){
    uprintf("Starting up\n");
    uint32_t adcVals[6];
    HAL_ADC_Start_DMA(&ADC_HANDLE, adcVals, 6);
    while (1){
        // read adc values
        uprintf("ADC values: %lu, %lu, %lu, %lu, %lu, %lu\n", adcVals[0], adcVals[1], adcVals[2], adcVals[3], adcVals[4], adcVals[5]);
        // blink led
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        vTaskDelay(20);
    }
}