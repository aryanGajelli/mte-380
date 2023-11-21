#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "stm32f4xx_hal.h"
#include "odometry.h"
#include "mathUtils.h"

typedef struct PurePursuitOutput_T {
    vector3_t targetPoint;
    size_t lastFoundIndex;
    double linError;
    double angError;
} PurePursuitOutput_T;

HAL_StatusTypeDef controlInit();
void controlTurnToHeading(double targetDeg);
void controlMoveForward(double targetDist, double speedMultiplier);
void controlGoToPoint(Pose_T targetPose);
void controlApprochLego();
PurePursuitOutput_T controlPurePursuitStep(vector3_t *path, size_t pathLen, double lookAheadRadius, size_t lastFoundIndex);
#endif  // __CONTROL_H__