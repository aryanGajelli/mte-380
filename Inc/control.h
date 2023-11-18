#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "stm32f4xx_hal.h"
#include "odometry.h"
HAL_StatusTypeDef controlInit();
void controlTurnToHeading(double targetDeg);
void controlMoveForward(double targetDist, double speedMultiplier);
void controlGoToPoint(Pose_T targetPose);
void controlApprochLego();
#endif  // __CONTROL_H__