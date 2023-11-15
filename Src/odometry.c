#include "odometry.h"

#include "encoders.h"

Encoder_T *encLeft;
Encoder_T *encRight;

Pose_T pose;
void odometryInit() {
    Encoder_T *encLeft = encoder_getInstance(ENCODER_LEFT);
    Encoder_T *encRight = encoder_getInstance(ENCODER_RIGHT);
    pose = (Pose_T){0, 0, 0};
}

void odometryUpdate() {
    encoderUpdate(encLeft);
    encoderUpdate(encRight);
}