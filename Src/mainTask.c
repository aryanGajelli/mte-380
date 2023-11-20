#include <math.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "ICM20948_register.h"
#include "ICM20948_spi.h"
#include "arm_math.h"
#include "bsp.h"
#include "color_sensor.h"
#include "control.h"
#include "debug.h"
#include "demo.h"
#include "dmp.h"
#include "encoders.h"
#include "fusion.h"
#include "imu.h"
#include "imu2.h"
#include "main.h"
#include "mathUtils.h"
#include "motion_fx.h"
#include "motors.h"
#include "odometry.h"
#include "sensors.h"
#include "servo.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "task.h"

HAL_StatusTypeDef mainTaskInit();

void mainTask(void *pvParameters) {
    uprintf("mainTask\n");

    if (mainTaskInit() != HAL_OK) {
        Error_Handler();
    }
    // uint32_t start = HAL_GetTick();
    // while (!isSDemoStarted) {
    //     if (HAL_GetTick() - start > 5000) {
    //         HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    //     } else {
    //         HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    //     }
    //     vTaskDelay(100);
    // }

    if (controlInit() != HAL_OK) {
        Error_Handler();
    }

    while (1) {
        // print line deviation
        
        vTaskDelay(100);
    }
}

HAL_StatusTypeDef mainTaskInit() {
    if (imuInit() != HAL_OK) {
        Error_Handler();
    }

    if (colorSensorInit() != HAL_OK) {
        Error_Handler();
    }

    if (motorsInit() != HAL_OK) {
        Error_Handler();
    }

    if (servoInit() != HAL_OK) {
        Error_Handler();
    }

    if (odometryInit() != HAL_OK) {
        Error_Handler();
    }

    if (encodersInit() != HAL_OK) {
        Error_Handler();
    }

    return HAL_OK;
}