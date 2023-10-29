#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "debug.h"
#include "bsp.h"
#include "sensors.h"
#include "imu.h"
#include "mathUtils.h"
#include <stdio.h>


void mainTask(void *pvParameters){
    uprintf("mainTask\n");
    uint32_t status = ICMInit();
    if (status != HAL_OK) {
        uprintf("ICMInit failed on line: %lu\n", status);
        vTaskDelay(1000);
        Error_Handler();
    }
    uprintf("passed ICM init\n");

    IMUData_t imuData;
    vector3_t magn;
    while (1){
        ICM_ReadMag(&magn);
        uprintf("mag: %0.3f %0.3f %0.3f\n", magn.x, magn.y, magn.z);
        // if (ICM_ReadAccelGyro(&imuData.accel, &imuData.gyro) != HAL_OK) {
        //     Error_Handler();
        // }
        // print out the data
        // uprintf("accel: %0.3f %0.3f %0.3f\t", imuData.accel.x, imuData.accel.y, imuData.accel.z);
        // uprintf("gyro: %0.3f %0.3f %0.3f\n", imuData.gyro.x, imuData.gyro.y, imuData.gyro.z);
        
        vTaskDelay(20);
    }
}