#include "FreeRTOS.h"
#include "task.h"

void printTask(void *pvParameters){
    while (1){
        taskYIELD();
    }
}