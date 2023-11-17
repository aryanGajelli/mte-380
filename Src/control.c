#include "control.h"

#include "FreeRTOS.h"
#include "arm_math.h"
#include "debug.h"
#include "motors.h"
#include "odometry.h"
#include "stm32f4xx_hal.h"
#include "task.h"
#include "encoders.h"

char isControlInit = 0;
Encoder_T *encLeft, *encRight;
HAL_StatusTypeDef controlInit() {
    encLeft = encoder_getInstance(ENCODER_LEFT);
    encRight = encoder_getInstance(ENCODER_RIGHT);
    isControlInit = 1;
    return HAL_OK;
}

void controlTask(void *pvParameters) {
    while (!isControlInit) {
        vTaskDelay(10);
    }
    uprintf("controlTask\n");

    // controlTurnToHeading(90);
    controlTurnToHeading(45);
    // controlTurnToHeading(95);
    // controlGoToPose((Pose_T){.x = 0, .y = 100, .theta = 0});

    controlMoveForward(100);

    while (1) {
        Pose_T pose = odometryGetPose();
        uprintf("%.3f %.3f %.3f\n", pose.x, pose.y, pose.theta);
    }
}

void controlTurnToHeading(double targetDeg) {
    targetDeg = -targetDeg;
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

void controlMoveForward(double targetDist){
    static const double ACCEPTABLE_ERROR = 10;
    double kp = 0.75, kd = 0.5, ki = 0.00;
    double dist = (encLeft->dist + encRight->dist) / 2;
    double error = targetDist - dist;
    if (fabs(error) < ACCEPTABLE_ERROR) return;
    double prevError = error;
    double integral = 0;
    while (fabs(error) > ACCEPTABLE_ERROR) {
        dist = (encLeft->dist + encRight->dist) / 2;
        error = targetDist - dist;
        if (fabs(error) < 10) {
            integral += error;
        } else {
            integral = 0;
        }
        // double turnSpeed = kp * error + kd * (error - prevError) + ki * integral;
        double speed = kp * error + kd * (error - prevError) + ki * integral;
        prevError = error;
        if (speed > 100) speed = 100;
        if (speed < -100) speed = -100;

        motorSetSpeed(MOTOR_LEFT, -speed);
        motorSetSpeed(MOTOR_RIGHT, -speed);
        Pose_T pose = odometryGetPose();
        uprintf("t: %.3f %.3f %.3f %.3f %.3f\n", error, speed, pose.x, pose.y, pose.theta);
    }

    motorHardStop(MOTOR_LEFT);
    motorHardStop(MOTOR_RIGHT);

}

void controlGoToPose(Pose_T targetPose) {
    static const double ACCEPTABLE_ERROR = 1;
    Pose_T pose = odometryGetPose();
    double thetaError = targetPose.theta - pose.theta;
    if (thetaError > 180) {
        thetaError -= 360;
    } else if (thetaError < -180) {
        thetaError += 360;
    }
    arm_matrix_instance_f32 dP = {
        .numRows = 3,
        .numCols = 1,
        .pData = (float32_t[3]){targetPose.x - pose.x,
                                targetPose.y - pose.y,
                                thetaError}};
    arm_matrix_instance_f32 transform = {
        .numRows = 3,
        .numCols = 3,
        .pData = (float32_t[9]){cos(DEG_TO_RAD(pose.theta)), sin(DEG_TO_RAD(pose.theta)), 0,
                                -sin(DEG_TO_RAD(pose.theta)), cos(DEG_TO_RAD(pose.theta)), 0,
                                0, 0, 1}};
    arm_matrix_instance_f32 e = {
        .numRows = 3,
        .numCols = 1,
        .pData = (float32_t[3]){0, 0, 0}};

    arm_mat_mult_f32(&transform, &dP, &e);
    printMatrix(e);

    double smallScalar = 0.01;
    double b = 0.5, c = 0.5, vD = smallScalar * sqrt(e.pData[0] * e.pData[0] + e.pData[1] * e.pData[1]), wD = smallScalar * DEG_TO_RAD(e.pData[2]);
    double k = 2 * c * sqrt(wD * wD + b * vD * vD);
    // double kLin = 0.5, kAng = 0.5;//, vD = 0.5, wD = 0.5;
    while (1) {
        pose = odometryGetPose();
        thetaError = targetPose.theta - pose.theta;
        if (thetaError > 180) {
            thetaError -= 360;
        } else if (thetaError < -180) {
            thetaError += 360;
        }
        dP.pData[0] = targetPose.x - pose.x;
        dP.pData[1] = targetPose.y - pose.y;
        dP.pData[2] = thetaError;
        arm_mat_mult_f32(&transform, &dP, &e);

        // double linSpeed = kLin * sqrt(e.pData[0] * e.pData[0] + e.pData[1] * e.pData[1]);
        // double angSpeed = kAng * e.pData[2];
        double linSpeed = (vD * cos(DEG_TO_RAD(e.pData[2])) + k * dist(e.pData[0], e.pData[1], 0, 0));
        double angSpeed = wD + k * DEG_TO_RAD(e.pData[2]) + (b * vD * sin(DEG_TO_RAD(e.pData[2])) * e.pData[1]) / DEG_TO_RAD(e.pData[2]);
        
        double speedL = linSpeed - angSpeed;
        double speedR = linSpeed + angSpeed;
        if (speedL > 100) speedL = 100;
        if (speedL < -100) speedL = -100;
        if (speedL > 100) speedL = 100;
        if (speedR < -100) speedR = -100;
        // linSpeed *= 0.4;
        // motorSetSpeed(MOTOR_LEFT, -speedL);
        // motorSetSpeed(MOTOR_RIGHT, -speedR);
        // uprintf("g: %.3f %.2f %.2f %.1f\n", error, pose.x, pose.y, pose.theta);
        uprintf("g: %.3f %.2f %.2f %.2f %.2f\n", e.pData[0], e.pData[1], e.pData[2], speedL, speedL);
        vTaskDelay(1);
    }

    motorHardStop(MOTOR_LEFT);
    motorHardStop(MOTOR_RIGHT);
}