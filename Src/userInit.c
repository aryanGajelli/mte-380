#include "userInit.h"
#include "debug.h"
#include "sensors.h"
#include "bsp.h"

void userInit(void) {
   
    if (debugInit() != HAL_OK) {
        Error_Handler();
    }
    
    if (sensorsInit() != HAL_OK) {
        Error_Handler();
    }


    printf("----------------------------------\nFinished User Init\n");
    
}

// void vApplicationStackOverflowHook( TaskHandle_t xTask,
//                                     signed char *pcTaskName )
// {
//     _handleError(pcTaskName, __LINE__, __builtin_return_address(0));
// }