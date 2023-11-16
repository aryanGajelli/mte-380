#include "control.h"

#include "FreeRTOS.h"
#include "arm_math.h"
#include "debug.h"
#include "motors.h"
#include "odometry.h"
#include "stm32f4xx_hal.h"
#include "task.h"

char isControlInit = 0;
HAL_StatusTypeDef controlInit() {
    isControlInit = 1;
    return HAL_OK;
}

void controlTask(void *pvParameters) {
    while (!isControlInit) {
        vTaskDelay(10);
    }
    uprintf("controlTask\n");

    // controlTurnToHeading(90);
    // controlTurnToHeading(0);
    // controlTurnToHeading(95);
    controlGoToPose((Pose_T){.x = 0, .y = 100, .theta = 0});
    while (1) {
        Pose_T pose = odometryGetPose();
        uprintf("%.3f %.3f %.3f\n", pose.x, pose.y, pose.theta);
    }
}

void controlTurnToHeading(double targetDeg) {
    static const double ACCEPTABLE_ERROR_DEG = 0.5;
    double kp = 0.5, kd = 0.5, ki = 0.01;
    double error = targetDeg - odometryGetPose().theta;
    if (error > 180)
        error -= 360;
    else if (error < -180)
        error += 360;

    if (fabs(error) < 1) return;

    double prevError = error;
    int dir = error > 180 ? -1 : 1;
    double integral = 0;
    while (fabs(error) > ACCEPTABLE_ERROR_DEG) {
        Pose_T pose = odometryGetPose();
        error = targetDeg - pose.theta;
        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }
        if (fabs(error) < 10) {
            integral += error;
        } else {
            integral = 0;
        }
        double turnSpeed = kp * error + kd * (error - prevError) + ki * integral;

        prevError = error;
        if (turnSpeed > 100) turnSpeed = 100;
        if (turnSpeed < -100) turnSpeed = -100;

        motorSetSpeed(MOTOR_LEFT, dir * turnSpeed);
        motorSetSpeed(MOTOR_RIGHT, dir * -turnSpeed);
        uprintf("t: %.3f %.3f %.3f\n", error, turnSpeed, pose.theta);
    }

    motorHardStop(MOTOR_LEFT);
    motorHardStop(MOTOR_RIGHT);
}

void controlGoToPose(Pose_T targetPose) {
    Pose_T pose = odometryGetPose();

    static const double ACCEPTABLE_ERROR = 1;

    double error = dist(targetPose.x, targetPose.y, pose.x, pose.y);
    double prevError = error;
    double integral = 0;

    double targetHeading = 90 - atan2(targetPose.y - pose.y, targetPose.x - pose.x) * 180 / PI;
    double kp = 0.5, kd = 0.5, ki = 0.00;
    controlTurnToHeading(targetHeading);
    do {
        pose = odometryGetPose();
        error = dist(targetPose.x, targetPose.y, pose.x, pose.y);
        prevError = error;
        if (error < 10)
            integral += error;
        else
            integral = 0;

        double linSpeed = kp * error + kd * (error - prevError) + ki * error;
        if (linSpeed > 100) linSpeed = 100;
        if (linSpeed < -100) linSpeed = -100;
        linSpeed *= 0.4;
        // motorSetSpeed(MOTOR_LEFT, linSpeed);
        // motorSetSpeed(MOTOR_RIGHT, linSpeed);
        uprintf("g: %.3f %.2f %.2f %.1f\n", error, pose.x, pose.y, pose.theta);

    } while (fabs(error) > ACCEPTABLE_ERROR);

    motorHardStop(MOTOR_LEFT);
    motorHardStop(MOTOR_RIGHT);
}