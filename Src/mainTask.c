#include <math.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "ICM20948_register.h"
#include "ICM20948_spi.h"
#include "arm_math.h"
#include "bsp.h"
#include "color_sensor.h"
#include "debug.h"
#include "demo.h"
#include "encoders.h"
#include "fusion.h"
#include "imu.h"
#include "main.h"
#include "mathUtils.h"
#include "motors.h"
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
    while (!isSDemoStarted) {
        vTaskDelay(10);
    }
    servoSetAngle(CLAW_OPEN_ANGLE);
    Encoder_T *encL = encoder_getInstance(ENCODER_LEFT);
    // Encoder_T *encR = encoder_getInstance(ENCODER_RIGHT);
    // float dc = -100;
    // motorSetSpeed(MOTOR_LEFT, dc);
    // motorSetSpeed(MOTOR_RIGHT, dc);
    // MFX_input_t data_in;
    // MFX_output_t data_out;

    // IMUData_T imuData;
    // IMUData_T prevImuData;
    // ICM_Read(&imuData);

    // MFX_MagCal_output_t magOut;
    // MotionFX_MagCal_getParams(&magOut);
    // float dT;
    // float *q;
    // float *g;
    // float *r;
    // double hGyro = 180 / PI * atan2(imuData.mag.x, imuData.mag.y);
    // double k = 0.95;
    // p controller for motor ticks
    double kp = -2;
    double kd = 5;
    double targetL = 0;
    // float targetR = 100;
    double dcL = 0;
    // float dcR = 0;
    double lineDeviation = 0;
    colorGetLineDeviation(&lineDeviation);
    double prevErrorL = targetL - lineDeviation;
    // float prevErrorR = targetR - encR->ticks;
    float errorL;
    double constSpeed = 80;
    while (1) {
        SurfaceType_E surf = colorGetLineDeviation(&lineDeviation);
        uprintf("%lu %.3f %.3f %.3f\n", surf, lineDeviation, dcL, errorL);
        if (surf == SURFACE_WOOD) {
            motorHardStop(MOTOR_LEFT);
            motorHardStop(MOTOR_RIGHT);
            break;
        }
        // colorGetLineDeviation(&lineDeviation);
        // encoderUpdate(encL);
        // if (encL > 5000) break;
        // encoderUpdate(encR);

        errorL = targetL - lineDeviation;
        // float errorR = targetR - encR->ticks;
        dcL = kp * errorL + kd * (errorL - prevErrorL);
        // dcR = kp * errorR + kd * (errorR - prevErrorR);
        if (dcL > 100) dcL = 100;
        if (dcL < -100) dcL = -100;
        // if (dcR > 100) dcR = 100;
        // if (dcR < -100) dcR = -100;

        motorSetSpeed(MOTOR_LEFT, dcL - constSpeed);
        motorSetSpeed(MOTOR_RIGHT, -dcL - constSpeed);

        prevErrorL = errorL;
        // prevErrorR = errorR;
        // uprintf("enc: %d, %d dc:%.2f, %.2f\n", encL->ticks, encR->ticks, dcL, dcR);
        // prevImuData = imuData;
        // ICM_Read(&imuData);

        // data_in.gyro[0] = imuData.gyro.x;
        // data_in.gyro[1] = imuData.gyro.y;
        // data_in.gyro[2] = imuData.gyro.z;
        // data_in.acc[0] = imuData.accel.x / 9.81;
        // data_in.acc[1] = imuData.accel.y / 9.81;
        // data_in.acc[2] = imuData.accel.z / 9.81;
        // data_in.mag[0] = imuData.mag.x / 50 - magOut.hi_bias[0];
        // data_in.mag[1] = imuData.mag.y / 50 - magOut.hi_bias[1];
        // data_in.mag[2] = imuData.mag.z / 50 - magOut.hi_bias[2];

        // dT = (imuData.timestamp - prevImuData.timestamp) / 1000.0f;
        // MotionFX_propagate(&data_out, &data_in, &dT);
        // MotionFX_update(&data_out, &data_in, &dT, NULL);
        // q = data_out.quaternion_9X;
        // g = data_out.gravity_9X;
        // r = data_out.rotation_6X;

        // if (fabs(imuData.gyro.x) < 0.5) imuData.gyro.x = 0;
        // if (fabs(imuData.gyro.y) < 0.5) imuData.gyro.y = 0;
        // if (fabs(imuData.gyro.z) < 0.5) imuData.gyro.z = 0;
        // double hMag = 180 / PI * atan2(imuData.mag.x, imuData.mag.y);
        // hGyro = fmod(hGyro + imuData.gyro.z * (imuData.timestamp - prevImuData.timestamp) / 1000., 360);
        // hGyro = k * hGyro + (1 - k) * hMag;
        // // print r
        // uprintf("%.3f %.3f %.3f\n", hMag, hGyro, r[0]);
        // uprintf("g: %.3f, %.3f, %.3f\n", g[0], g[1], g[2]);
        // print q
        // uprintf("q: %f, %f, %f, %f\n", q[0], q[1], q[2], q[3]);
        // uprintf("h: %f\n", data_out.heading_9X);
        // uprintf("dT: %f\n", dT);
        // print acc gyro mag
        // uprintf("acc: %.3f, %.3f, %.3f\n", data_in.acc[0], data_in.acc[1], data_in.acc[2]);
        // uprintf("gyro: %.3f, %.3f, %.3f\n", imuData.gyro.x, imuData.gyro.y, imuData.gyro.z);
        // uprintf("mag: %.3f, %.3f, %.3f\n", imuData.mag.x, imuData.mag.y, imuData.mag.z);
        // uprintf("enc: %d, %d\n", encoderGetCount(ENCODER_LEFT), encoderGetCount(ENCODER_RIGHT));
        // vTaskDelay(5);
    }
    while (1) {
        uprintf("Out of loop\n");
        vTaskDelay(1000);
    }
}

HAL_StatusTypeDef mainTaskInit() {
    // if (ICMInit() != HAL_OK) {
    //     Error_Handler();
    // }

    if (colorSensorInit() != HAL_OK) {
        Error_Handler();
    }

    if (motorsInit() != HAL_OK) {
        Error_Handler();
    }

    if (servoInit() != HAL_OK) {
        Error_Handler();
    }

    if (encodersInit() != HAL_OK) {
        Error_Handler();
    }

    // if (fusionInit() != HAL_OK) {
    //     return HAL_ERROR;
    // }

    return HAL_OK;
}