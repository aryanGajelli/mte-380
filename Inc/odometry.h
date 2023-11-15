#ifndef __ODOMETRY_H__
#define __ODOMETRY_H__

typedef struct Pose_T {
    float x;
    float y;
    float theta;
} Pose_T;

void odometryInit(void);
#endif // __ODOMETRY_H__