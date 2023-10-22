#include "userInit.h"
#include "debug.h"
#include "sensors.h"

void userInit(void) {
    if (debugInit() != HAL_OK) {
        Error_Handler();
    }

    if (sensorsInit() != HAL_OK) {
        Error_Handler();
    }
}