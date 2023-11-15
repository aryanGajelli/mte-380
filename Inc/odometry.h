#ifndef __ODOMETRY_H__
#define __ODOMETRY_H__

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "imu.h"
typedef struct Pose_T {
    float x;
    float y;
    float theta;
} Pose_T;

extern osMutexId poseMutexHandle;
HAL_StatusTypeDef odometryInit(void);
Pose_T odometryGetPose();
float odometryGetDeltaHeading();
double odometryGetHeading();
void odometryUpdate(IMUData_T imuData, IMUData_T prevImuData);
#endif  // __ODOMETRY_H__