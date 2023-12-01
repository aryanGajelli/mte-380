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
#include "imu.h"
#include "imu2.h"
#include "main.h"
#include "mathUtils.h"
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
    uint32_t start = HAL_GetTick();
    while (!isSDemoStarted) {
        if (HAL_GetTick() - start > 5000) {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        }
        vTaskDelay(150);
    }
    vTaskDelay(1000);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    if (controlInit() != HAL_OK) {
        Error_Handler();
    }

    while (1) {
        // colorUpdate();
        // // color readings already has delays of 5ms each
        // // moving average of samples
        // uint32_t val = colorGetFreq(COLOR_SENSOR_1);
        // sum[0] = sum[0] + val - c1[counter[0]];
        // colorSensors.freq[0] = sum[0] / NUM_SAMPLES;
        // c1[counter[0]] = val;
        // counter[0] = (counter[0] + 1) % NUM_SAMPLES;

        // val = colorGetFreq(COLOR_SENSOR_2);
        // sum[1] = sum[1] + val - c2[counter[1]];
        // colorSensors.freq[1] = sum[1] / NUM_SAMPLES;
        // c2[counter[1]] = val;
        // counter[1] = (counter[1] + 1) % NUM_SAMPLES;

        // val = colorGetFreq(COLOR_SENSOR_3);
        // sum[2] = sum[2] + val - c3[counter[2]];
        // colorSensors.freq[2] = sum[2] / NUM_SAMPLES;
        // c3[counter[2]] = val;
        // counter[2] = (counter[2] + 1) % NUM_SAMPLES;

        // // taskENTER_CRITICAL();
        // colorSensors.normalizedOut.x = colorGetNormalizedOut(COLOR_SENSOR_1);
        // colorSensors.normalizedOut.y = colorGetNormalizedOut(COLOR_SENSOR_2);
        // colorSensors.normalizedOut.z = colorGetNormalizedOut(COLOR_SENSOR_3);
        // colorSensors.surface = colorGetLineDeviation(&colorSensors.lineDeviation);
        // taskEXIT_CRITICAL();
        vTaskDelay(5);
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