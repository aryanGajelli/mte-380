#include "odometry.h"

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "debug.h"
#include "encoders.h"
#include "imu.h"
#include "imu2.h"
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

Pose_T *odometryGetPose() {
    return &pose;
}

void odometrySetPose(Pose_T newPose) {
    xSemaphoreTake(poseMutexHandle, portMAX_DELAY);
    pose = newPose;
    xSemaphoreGive(poseMutexHandle);
}

void odometrySetPoseXY(Pose_T newPose) {
    xSemaphoreTake(poseMutexHandle, portMAX_DELAY);
    pose.x = newPose.x;
    pose.y = newPose.y;
    xSemaphoreGive(poseMutexHandle);
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

void odometryUpdate() {
    static double yaw, dTheta = 0;
    static double prevDistL = 0, prevDistR = 0;
    static icm_20948_DMP_data_t dmpData;
    ICM_20948_Status_e status = dmpReadDataFromFIFO(&dmpData);
    // Was valid data available?
    if (status == ICM_20948_Stat_Ok || status == ICM_20948_Stat_FIFOMoreDataAvail) {
        // Check for GRV data (Quat6)
        if ((dmpData.header & DMP_header_bitmap_Quat6) > 0) {
            // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
            // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
            // The quaternion data is scaled by 2^30.
            // Scale to +/- 1
            double q1 = ((double)dmpData.Quat6.Data.Q1) / 1073741824.0;  // Convert to double. Divide by 2^30
            double q2 = ((double)dmpData.Quat6.Data.Q2) / 1073741824.0;  // Convert to double. Divide by 2^30
            double q3 = ((double)dmpData.Quat6.Data.Q3) / 1073741824.0;  // Convert to double. Divide by 2^30

            // Convert the quaternions to Euler angles (roll, pitch, yaw)
            // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2
            double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

            // yaw (z-axis rotation)
            double t3 = 2.0 * (q0 * q3 + q1 * q2);
            double t4 = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
            yaw = RAD_TO_DEG(atan2(t3, t4));
        }
    }

    encoderUpdate(encLeft);
    encoderUpdate(encRight);
    xSemaphoreTake(poseMutexHandle, portMAX_DELAY);
    dTheta = yaw - pose.theta;
    pose.theta = yaw;

    double dL = encLeft->dist - prevDistL;
    double dR = encRight->dist - prevDistR;
    double LR = (encLeft->dist + encRight->dist) / 2;
    double d = (dL + dR) / 2;

    pose.x += d * sin((pose.theta + dTheta / 2) * PI / 180);
    pose.y += d * cos((pose.theta + dTheta / 2) * PI / 180);
    xSemaphoreGive(poseMutexHandle);

    prevDistL = encLeft->dist;
    prevDistR = encRight->dist;
}

void poseTask(void *pvParameters) {
    while (!isOdometryInit) {
        vTaskDelay(10);
    }
    uprintf("poseTask\n");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        odometryUpdate();
        // uprintf("x: %.3f, y: %.3f, theta: %.3f\n", pose.x, pose.y, pose.theta);
        vTaskDelayUntil(&xLastWakeTime, 3);
    }
}