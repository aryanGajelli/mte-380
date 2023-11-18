#include "odometry.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "debug.h"
#include "encoders.h"
#include "fusion.h"
#include "imu.h"
#include "motion_fx.h"
#include "stm32f4xx_hal.h"
#include "task.h"

Pose_T pose;
Pose_T prevPose;
char isOdometryInit = 0;
HAL_StatusTypeDef odometryInit() {
    pose = (Pose_T){0, 0, 0};
    isOdometryInit = 1;
    return HAL_OK;
}

Pose_T odometryGetPose() {
    return pose;
}

double odometryGet2DDist(Pose_T a, Pose_T b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return sqrt(dx * dx + dy * dy);
}

double odometryGet2DAngle(Pose_T a, Pose_T b) {
    return atan2(a.x - b.x, a.y - b.y) * 180 / PI;
}

double odometryDot(Pose_T a, Pose_T b) {
    return a.x * b.x + a.y * b.y;
}

double odometryCross(Pose_T a, Pose_T b) {
    return a.x * b.y - a.y * b.x;
}

double odometryOriginAngle(Pose_T a) {
    return atan2(a.y, a.x) * 180 / PI;
}

double odometryOriginAngleDiff(Pose_T a, Pose_T b) {
    double angleA = odometryOriginAngle(a);
    double angleB = odometryOriginAngle(b);
    double angleDiff = angleA - angleB;
    // if (angleDiff > 180) {
    //     angleDiff -= 360;
    // } else if (angleDiff < -180) {
    //     angleDiff += 360;
    // }
    return angleDiff;
}

/**
 * @brief Computes a sort of distance error while providing negative distance
*/
double odometryDotError(Pose_T a, Pose_T b) {
    b = (Pose_T){a.x - b.x, a.y - b.y, 0};
    double error = odometryDot(a, b);
    error = sign(error) * sqrt(sign(error) * error);
    return error;
}
void odometrySetPoseXY(Pose_T newPose) {
    xSemaphoreTake(poseMutexHandle, portMAX_DELAY);
    pose.x = newPose.x;
    pose.y = newPose.y;
    xSemaphoreGive(poseMutexHandle);
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
    xSemaphoreTake(poseMutexHandle, portMAX_DELAY);
    pose.theta = data_out.heading_6X;
    xSemaphoreGive(poseMutexHandle);
}

void poseTask(void *pvParameters) {
    while (!isOdometryInit) {
        vTaskDelay(10);
    }
    uprintf("poseTask\n");

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
        vTaskDelayUntil(&xLastWakeTime, 40);
    }
}