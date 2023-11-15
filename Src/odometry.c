#include "odometry.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "encoders.h"
#include "fusion.h"
#include "imu.h"
#include "motion_fx.h"
#include "stm32f4xx_hal.h"
#include "task.h"

Encoder_T *encLeft;
Encoder_T *encRight;

Pose_T pose;
Pose_T prevPose;
char isOdometryInit = 0;
HAL_StatusTypeDef odometryInit() {
    Encoder_T *encLeft = encoder_getInstance(ENCODER_LEFT);
    Encoder_T *encRight = encoder_getInstance(ENCODER_RIGHT);
    pose = (Pose_T){0, 0, 0};
    isOdometryInit = 1;
    return HAL_OK;
}

Pose_T odometryGetPose() {
    return pose;
}

float odometryGetDeltaHeading() {
    double dTheta = pose.theta - prevPose.theta;
    if (dTheta > 180) {
        dTheta -= 360;
    } else if (dTheta < -180) {
        dTheta += 360;
    }
    return dTheta;
}

double odometryGetHeading() {
    return pose.theta;
}

MFX_output_t data_out;
void odometryUpdate(IMUData_T imuData, IMUData_T prevImuData) {
    prevPose = pose;
    fusionGetOutputs(&data_out, imuData, prevImuData);
    // taskENTER_CRITICAL();

    // taskEXIT_CRITICAL();
    pose.theta = data_out.heading_6X;
}

void poseTask(void *pvParameters) {
    while (!isOdometryInit) {
        vTaskDelay(10);
    }

    IMUData_T imuData;
    IMUData_T prevImuData;
    ICM_ReadAccelGyro(&imuData);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        prevImuData = imuData;
        ICM_ReadAccelGyro(&imuData);
        // encoderUpdate(encLeft);
        // encoderUpdate(encRight);

        odometryUpdate(imuData, prevImuData);
        // vTaskDelay(10);
        vTaskDelayUntil(&xLastWakeTime, 35);
    }
}