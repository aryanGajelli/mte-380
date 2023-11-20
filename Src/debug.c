#include "debug.h"

#include <backtrace.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "bsp.h"
#include "queue.h"
#include "stm32f4xx_hal.h"

// Note: printf should only be used for printing before RTOS starts, and error
// cases where rtos has probably failed. (If used after rtos starts, it may
// cause errors in calling non-reentrant hal functions)
int _write(int file, char *data, int len) {
    HAL_UART_Transmit(&DEBUG_UART_HANDLE, (uint8_t *)data, len, UART_PRINT_TIMEOUT);
    return len;
}

QueueHandle_t printQueue;
char isDebugInitialized = 0;
HAL_StatusTypeDef debugInit(void) {
    isDebugInitialized = 1;
    printQueue = xQueueCreate(PRINT_QUEUE_LENGTH, PRINT_QUEUE_STRING_SIZE);
    if (!printQueue) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void printTask(void *pvParameters) {
    char buffer[PRINT_QUEUE_STRING_SIZE] = {0};

    while (1) {
        if (HAL_UART_GetState(&DEBUG_UART_HANDLE) == HAL_UART_STATE_READY && xQueueReceive(printQueue, (uint8_t *)buffer, portMAX_DELAY) == pdTRUE) {
            buffer[PRINT_QUEUE_STRING_SIZE] = '\0';
            size_t len = strlen(buffer);
            HAL_UART_Transmit_DMA(&DEBUG_UART_HANDLE, (uint8_t *)buffer, len);
        }
    }
}

// Reset the debug uart
// This is done to clear the UART in case it is being used by the debug task,
// that way we can send an error message
HAL_StatusTypeDef resetUART() {
    if (HAL_UART_DeInit(&DEBUG_UART_HANDLE) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_UART_Init(&DEBUG_UART_HANDLE) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void _handleError(char *file, int line) {
    const char errorStringFile[] = "Error!: File ";
    const char errorStringLine[] = " line ";
    char lineNumberString[10];

    const int BACKTRACE_SIZE = 3;
    backtrace_t backtrace[BACKTRACE_SIZE];
    backtrace_unwind(backtrace, BACKTRACE_SIZE);
    char backtraceString[20];
    
    taskDISABLE_INTERRUPTS();

    while (resetUART() != HAL_OK)
        ;
    
    HAL_UART_Transmit(&DEBUG_UART_HANDLE, ((uint8_t *)errorStringFile), strlen(errorStringFile), 1000);
    HAL_UART_Transmit(&DEBUG_UART_HANDLE, ((uint8_t *)file), strlen(file), 1000);
    HAL_UART_Transmit(&DEBUG_UART_HANDLE, ((uint8_t *)errorStringLine), strlen(errorStringLine), 1000);
    snprintf(lineNumberString, sizeof(lineNumberString), "%d", line);
    HAL_UART_Transmit(&DEBUG_UART_HANDLE, ((uint8_t *)lineNumberString), strlen(lineNumberString), 1000);

    for (int i = 0; i < BACKTRACE_SIZE; ++i){
		sprintf(backtraceString, "\n%p - %s@ %p", backtrace[i].function, backtrace[i].name, backtrace[i].address);
        HAL_UART_Transmit(&DEBUG_UART_HANDLE, ((uint8_t *)backtraceString), strlen(backtraceString), 1000);
    }

}