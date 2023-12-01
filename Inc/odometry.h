#ifndef __ODOMETRY_H__
#define __ODOMETRY_H__

#include "arm_math.h"
#include "cmsis_os.h"
#include "imu.h"
#include "stm32f4xx_hal.h"
typedef union Pose_T {
    vector3_t v;
    struct {
        double x;
        double y;
        double theta;
    };
} Pose_T;

#define WHEEL_DIAMETER 42.
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER * PI)
#define TICKS_PER_REVOLUTION 540.
#define ENCODER_DISTANCE_PER_TICK (WHEEL_CIRCUMFERENCE / TICKS_PER_REVOLUTION)
#define WHEEL_TO_WHEEL_DISTANCE 105.

#define WHEEL_DIAMETER_M (WHEEL_DIAMETER / 1000.)
#define WHEEL_CIRCUMFERENCE_M (WHEEL_DIAMETER_M * PI)
#define WHEEL_TO_WHEEL_DISTANCE_M (WHEEL_TO_WHEEL_DISTANCE / 1000.)
#define MAX_WHEEL_RPM 450.

#define MAX_WHEEL_RAD_PER_S (MAX_WHEEL_RPM * PI / 30.)
#define MAX_LINEAR_VELOCITY_M_PER_S (WHEEL_DIAMETER_M * MAX_WHEEL_RAD_PER_S)
#define MAX_ANGULAR_VELOCITY_RAD_PER_S (MAX_LINEAR_VELOCITY_M_PER_S / WHEEL_TO_WHEEL_DISTANCE_M)
#define MAX_ANGULAR_VELOCITY_DEG_PER_S RAD_TO_DEG(MAX_ANGULAR_VELOCITY_RAD_PER_S)

extern osMutexId poseMutexHandle;
HAL_StatusTypeDef odometryInit(void);
Pose_T *odometryGetPose();
void odometrySetPose(Pose_T newPose);
double odometryGet2DDist(Pose_T a, Pose_T b);
double odometryGet2DAngle(Pose_T a, Pose_T b);
double odometryDot(Pose_T a, Pose_T b);
double odometryOriginAngleDiff(Pose_T a, Pose_T b);
double odometryDotError(Pose_T a, Pose_T b);
void odometrySetPoseXY(Pose_T newPose);
float odometryGetDeltaHeading();
double odometryGetHeading();
void odometryUpdate();
#endif  // __ODOMETRY_H__