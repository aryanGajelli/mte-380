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
#include "color_sensor.h"
#include "servo.h"
#include <stdio.h>


void mainTask(void *pvParameters){
    uprintf("mainTask\n");
    colorSensorInit();
    setColor(BLUE);
    servoInit();
    // uint32_t status = ICMInit();
    // if (status != HAL_OK) {
    //     uprintf("ICMInit failed on line: %lu\n", status);
    //     vTaskDelay(1000);
    //     Error_Handler();
    // }
    // uprintf("passed ICM init\n");

    // IMUData_t imuData;
    float angle = 5;
    float inc = 0.1;
    // setServoDutyCycle(pwm);

    // setServoAngle(0);
    while (1){
        // uprintf("%lu\n", __HAL_TIM_GET_COMPARE(&SERVO_TIMER_HANDLE, TIM_CHANNEL_1));
        angle += inc;
        if (angle > 180){
            inc = -inc;
        }
        if (angle < 0){
            inc = -inc;
        }

        // setServoAngle(angle);
        
        // uprintf("%lf\n", ADC_to_Volt(adcBuf[0]));
        // selectColorSensor(COLOR_3);
        // uprintf("%lu ", getFreq());
        // vTaskDelay(10);
        // selectColorSensor(COLOR_1);
        // uprintf("%lu ", getFreq());
        // vTaskDelay(10);
        // selectColorSensor(COLOR_2);
        // uprintf("%lu\n", getFreq());
        // print the color and freq
        // uprintf("%c %lu %c %lu %c %lu %c %lu\n", 'R', colorFreqs[0].freq, 'B', colorFreqs[1].freq, 'C', colorFreqs[2].freq, 'G', colorFreqs[3].freq);
        // uprintf("Test\n");
        // ICM_Read(&imuData);
        // uprintf("mag: %0.3f %0.3f %0.3f\t", imuData.mag.x, imuData.mag.y, imuData.mag.z);
        // // if (ICM_ReadAccelGyro(&imuData.accel, &imuData.gyro) != HAL_OK) {
        // //     Error_Handler();
        // // }
        // // print out the data
        // uprintf("accel: %0.3f %0.3f %0.3f\t", imuData.accel.x, imuData.accel.y, imuData.accel.z);
        // uprintf("gyro: %0.3f %0.3f %0.3f\n", imuData.gyro.x, imuData.gyro.y, imuData.gyro.z);
        
        vTaskDelay(10);
    }
}