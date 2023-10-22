#include "debug.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "bsp.h"
#include "queue.h"
#include "stm32f4xx_hal.h"

QueueHandle_t printQueue;
HAL_StatusTypeDef debugInit(void) {
    printQueue = xQueueCreate(PRINT_QUEUE_LENGTH, PRINT_QUEUE_STRING_SIZE);
    if (!printQueue) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void printTask(void *pvParameters) {
    char buffer[PRINT_QUEUE_STRING_SIZE] = {0};

    while (1) {
        if (xQueueReceive(printQueue, (uint8_t *)buffer, portMAX_DELAY) == pdTRUE) {
            size_t len = strlen(buffer);
            HAL_UART_Transmit(&DEBUG_UART_HANDLE, (uint8_t *)buffer, len, UART_PRINT_TIMEOUT);
        }
    }
}