#include "control.h"

#include "FreeRTOS.h"
#include "debug.h"
#include "odometry.h"
#include "stm32f4xx_hal.h"
#include "task.h"

char isControlInit = 0;
HAL_StatusTypeDef controlInit() {
    isControlInit = 1;
    return HAL_OK;
}

void controlTask(void *pvParameters) {
    while (!isControlInit) {
        vTaskDelay(10);
    }
    uprintf("controlTask\n");

    while (1) {
        Pose_T pose = odometryGetPose();
        uprintf("%.3f %.3f %.3f\n", pose.x, pose.y, pose.theta);
        vTaskDelay(10);
    }
}