#include "userInit.h"
#include "debug.h"

void userInit(void) {
    if (debugInit() != HAL_OK) {
        Error_Handler();
    }
}