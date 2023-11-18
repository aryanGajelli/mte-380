#ifndef __ODOMETRY_H__
#define __ODOMETRY_H__

#include "cmsis_os.h"
#include "imu.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"
typedef struct Pose_T {
    float x;
    float y;
    float theta;
    float dTheta;
} Pose_T;

#define WHEEL_DIAMETER 42
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * PI)
#define TICKS_PER_REVOLUTION 540
#define ENCODER_DISTANCE_PER_TICK (WHEEL_CIRCUMFERENCE / TICKS_PER_REVOLUTION)
#define WHEEL_TO_WHEEL_DISTANCE 105

extern osMutexId poseMutexHandle;
HAL_StatusTypeDef odometryInit(void);
Pose_T odometryGetPose();
void odometrySetPose(Pose_T newPose);
double odometryGet2DDist(Pose_T a, Pose_T b);
double odometryGet2DAngle(Pose_T a, Pose_T b);
double odometryDot(Pose_T a, Pose_T b);
double odometryOriginAngleDiff(Pose_T a, Pose_T b);
double odometryDotError(Pose_T a, Pose_T b);
void odometrySetPoseXY(Pose_T newPose);
float odometryGetDeltaHeading();
double odometryGetHeading();
void odometryUpdate(IMUData_T imuData, IMUData_T prevImuData);
#endif  // __ODOMETRY_H__