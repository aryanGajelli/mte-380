#include "debug.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "bsp.h"
#include "queue.h"
#include "stm32f4xx_hal.h"

// Note: printf should only be used for printing before RTOS starts, and error
// cases where rtos has probably failed. (If used after rtos starts, it may
// cause errors in calling non-reentrant hal functions)
int _write(int file, char* data, int len) {
    HAL_UART_Transmit(&DEBUG_UART_HANDLE, (uint8_t*)data, len, UART_PRINT_TIMEOUT);
    return len;
}

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
            buffer[PRINT_QUEUE_STRING_SIZE] = '\0';
            HAL_UART_Transmit_DMA(&DEBUG_UART_HANDLE, (uint8_t *)buffer, len);
        }
    }
}