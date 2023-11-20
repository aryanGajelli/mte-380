#include "control.h"

#include "FreeRTOS.h"
#include "arm_math.h"
#include "color_sensor.h"
#include "debug.h"
#include "encoders.h"
#include "mathUtils.h"
#include "motors.h"
#include "odometry.h"
#include "sensors.h"
#include "servo.h"
#include "stm32f4xx_hal.h"
#include "task.h"

char isControlInit = 0;
HAL_StatusTypeDef controlInit() {
    isControlInit = 1;
    return HAL_OK;
}

void controlTestSquare();
void controlTestPickup();
void controlTestSquareAbsolute();
void controlTask(void *pvParameters) {
    while (!isControlInit) {
        vTaskDelay(10);
    }
    uprintf("controlTask\n");

    // controlTestSquareAbsolute();
    double line;
    // Pose_T *pose = odometryGetPose();
    while (1) {
        uprintf("%d %d %d\n",colorGetFreq(COLOR_SENSOR_1), colorGetFreq(COLOR_SENSOR_2), colorGetFreq(COLOR_SENSOR_3));
        // uprintf("%.3f %.3f %.3f\n", pose->x, pose->y, pose->theta);
        vTaskDelay(10);
    }
}

void controlTestSquareAbsolute() {
    controlGoToPoint((Pose_T){.x = 0, .y = 150, .theta = 0});
    controlGoToPoint((Pose_T){.x = 150, .y = 150, .theta = 0});
    controlGoToPoint((Pose_T){.x = 150, .y = 0, .theta = 0});
    controlGoToPoint((Pose_T){.x = 0, .y = 0, .theta = 0});
}

void controlTestPickup() {
    controlMoveForward(350, 1);
    HAL_Delay(200);
    controlTurnToHeading(90);
    HAL_Delay(200);
    controlMoveForward(150, 0.7);
    controlMoveForward(100, 0.5);
    HAL_Delay(200);
    servoSetAngle(CLAW_CLOSED_ANGLE);
    HAL_Delay(150);
    controlMoveForward(-100, 0.7);
}

void controlTestSquare() {
    controlMoveForward(150, 1);
    HAL_Delay(200);
    controlTurnToHeading(90);
    HAL_Delay(200);
    controlMoveForward(150, 1);
    HAL_Delay(200);
    controlTurnToHeading(180);
    HAL_Delay(200);
    controlMoveForward(150, 1);
    HAL_Delay(200);
    controlTurnToHeading(270);
    HAL_Delay(200);
    controlMoveForward(150, 1);
    HAL_Delay(200);
    controlTurnToHeading(0);
}

void controlApprochLego() {
    double kp = 0.5, kd = 0, ki = 0;
    double target = .5;
    double error = target, prevError = 0;

    while (fabs(error) > 0.1) {
        error = target - ADC_to_Volt(adcBuf[0]);
        double speed = kp * error + kd * (error - prevError) + ki * error;
        if (speed > 100) speed = 100;
        if (speed < -100) speed = -100;
        prevError = error;
        motorSetSpeed(MOTOR_LEFT, -speed);
        motorSetSpeed(MOTOR_RIGHT, speed);
        uprintf("%.3f\n", error);
    }

    target = 1;
    error = target - ADC_to_Volt(adcBuf[0]);
    double kp2 = 5;
    while (fabs(error) > 0.1) {
        error = target - ADC_to_Volt(adcBuf[0]);
        double speed = kp2 * error;
        if (speed > 100) speed = 100;
        if (speed < -100) speed = -100;
        prevError = error;
        motorSetSpeed(MOTOR_LEFT, -speed);
        motorSetSpeed(MOTOR_RIGHT, -speed);
        uprintf("%.3f\n", error);
    }
}

void controlLineFollowing() {
    double lineDeviation = 0;
    colorGetLineDeviation(&lineDeviation);
    double target = 0;
    double prevError = target - lineDeviation;
    double error;
    double constSpeed = 40;
    double kp = 0.5, kd = 0.0, ki = 0.00;

    while (1) {
        // Pose_T pose = odometryGetPose();
        SurfaceType_E surf = colorGetLineDeviation(&lineDeviation);
        if (surf != SURFACE_TAPE) {
            break;
        }
        error = target - lineDeviation;
        double speed = kp * error + kd * (error - prevError) + ki * error;
        prevError = error;
        if (speed > 100) speed = 100;
        if (speed < -100) speed = -100;
        motorSetSpeed(MOTOR_LEFT, constSpeed + speed);
        motorSetSpeed(MOTOR_RIGHT, constSpeed - speed);
        uprintf("l: %.3f %.3f %.3f\n", error, speed, lineDeviation);
        // if (dist(pose->x, pose->y, 0, 0) > 300)
        //     break;
    }
    motorHardStop(MOTOR_LEFT);
    motorHardStop(MOTOR_RIGHT);
}

void controlTurnToHeading(double targetDeg) {
    // targetDeg = -targetDeg;
    static const double ACCEPTABLE_ERROR_DEG = 0.7;
    Pose_T *pose = odometryGetPose();
    Pose_T startPose = *pose;
    double kp = 0.7, kd = 0.5, ki = 0.05;
    double error = adjustTurn(targetDeg - startPose.theta);

    if (fabs(error) < 1) return;

    double prevError = error;
    int dir = error > 180 ? -1 : 1;
    double integral = 0;
    while (fabs(error) > ACCEPTABLE_ERROR_DEG) {
        error = adjustTurn(targetDeg - pose->theta);
        if (fabs(error) < 10) {
            integral += error;
        } else {
            integral = 0;
        }
        double turnSpeed = kp * error + kd * (error - prevError) + ki * integral;

        prevError = error;
        if (turnSpeed > 100) turnSpeed = 100;
        if (turnSpeed < -100) turnSpeed = -100;

        motorSetSpeed(MOTOR_LEFT, dir * -turnSpeed);
        motorSetSpeed(MOTOR_RIGHT, dir * turnSpeed);
        uprintf("t: %.3f %.3f %.3f\n", error, turnSpeed, pose->theta);
        vTaskDelay(1);
    }

    motorHardStop(MOTOR_LEFT);
    motorHardStop(MOTOR_RIGHT);
}

void controlMoveForward(double targetDist, double speedMultiplier) {
    static const double ACCEPTABLE_ERROR = 1;
    Pose_T *pose = odometryGetPose();
    Pose_T startPose = *pose;
    double kp = 0.75, kd = 0.5, ki = 0.00;
    double kpT = 1.5, kdT = 0.5, kiT = 0.00;
    double targetTheta = startPose.theta;
    double dist = (encLeft->dist + encRight->dist) / 2;
    targetDist += dist;
    double error = targetDist - dist;
    double errorAng = targetTheta - startPose.theta;
    if (errorAng > 180) {
        errorAng -= 360;
    } else if (errorAng < -180) {
        errorAng += 360;
    }
    double prevErrorAng = errorAng;
    if (fabs(error) < ACCEPTABLE_ERROR) return;
    double prevError = error;
    double integral = 0;
    while (fabs(error) > ACCEPTABLE_ERROR) {
        dist = (encLeft->dist + encRight->dist) / 2;
        error = targetDist - dist;

        errorAng = targetTheta - pose->theta;
        if (errorAng > 180) {
            errorAng -= 360;
        } else if (errorAng < -180) {
            errorAng += 360;
        }

        if (fabs(error) < 10) {
            integral += error;
        } else {
            integral = 0;
        }
        // double turnSpeed = kp * error + kd * (error - prevError) + ki * integral;
        double speedLin = kp * error + kd * (error - prevError) + ki * integral;
        double angSpeed = kpT * errorAng + kdT * (errorAng - prevErrorAng);
        prevError = error;
        prevErrorAng = errorAng;

        double speedL = speedLin - angSpeed;
        double speedR = speedLin + angSpeed;
        if (speedR > 100) speedR = 100;
        if (speedR < -100) speedR = -100;

        if (speedL > 100) speedL = 100;
        if (speedL < -100) speedL = -100;

        motorSetSpeed(MOTOR_LEFT, speedMultiplier * speedL);
        motorSetSpeed(MOTOR_RIGHT, speedMultiplier * speedR);

        uprintf("t: %.3f %.3f %.3f %.3f %.3f\n", error, speedLin, pose->x, pose->y, pose->theta);
    }

    motorHardStop(MOTOR_LEFT);
    motorHardStop(MOTOR_RIGHT);
}

void controlGoToPoint(Pose_T targetPose) {
    static const double ACCEPTABLE_ERROR = 5;
    Pose_T *pose = odometryGetPose();
    Pose_T startPose = *pose;

    double targetTheta = odometryGet2DAngle(targetPose, startPose);
    controlTurnToHeading(targetTheta);
    // recenters the targetPose to the robot's current position
    targetPose.x -= startPose.x;
    targetPose.y -= startPose.y;
    double error = odometryDotError(targetPose, startPose);
    double prevError = error;

    double errorT = adjustTurn(targetTheta - pose->theta);
    double prevErrorT = errorT;

    double kp = 0.7, kd = 0.5, ki = 0.00;
    double kpT = 2, kdT = 0.5, kiT = 0.00;

    double integral = 0;
    double integralT = 0;
    while (fabs(error) > ACCEPTABLE_ERROR) {
        Pose_T adjustedPose = *pose;
        adjustedPose.x -= startPose.x;
        adjustedPose.y -= startPose.y;
        error = odometryDotError(targetPose, adjustedPose);
        errorT = adjustTurn(targetTheta - pose->theta);

        if (fabs(error) < 10)
            integral += error;
        else
            integral = 0;

        if (fabs(errorT) < 10)
            integralT += errorT;
        else
            integralT = 0;
    
        double speed = kp * error + kd * (error - prevError) + ki * integral;
        double speedT = kpT * errorT + kdT * (errorT - prevErrorT) + kiT * integralT;
        
        prevError = error;
        prevErrorT = errorT;

        motorSetSpeed(MOTOR_LEFT, clamp(speed - speedT, -100, 100));
        motorSetSpeed(MOTOR_RIGHT, clamp(speed + speedT, -100, 100));

        uprintf("%.3f %.3f %.3f %.3f\n", error, pose->x, pose->y, pose->theta);
        vTaskDelay(1);
    }

    motorHardStop(MOTOR_LEFT);
    motorHardStop(MOTOR_RIGHT);
}

void controlGoToPoseRASETE(Pose_T targetPose) {
    static const double ACCEPTABLE_ERROR = 1;
    Pose_T *pose = odometryGetPose();
    double thetaError = targetPose.theta - pose->theta;
    if (thetaError > 180) {
        thetaError -= 360;
    } else if (thetaError < -180) {
        thetaError += 360;
    }
    arm_matrix_instance_f32 dP = {
        .numRows = 3,
        .numCols = 1,
        .pData = (float32_t[3]){targetPose.x - pose->x,
                                targetPose.y - pose->y,
                                thetaError}};
    arm_matrix_instance_f32 transform = {
        .numRows = 3,
        .numCols = 3,
        .pData = (float32_t[9]){cos(DEG_TO_RAD(pose->theta)), sin(DEG_TO_RAD(pose->theta)), 0,
                                -sin(DEG_TO_RAD(pose->theta)), cos(DEG_TO_RAD(pose->theta)), 0,
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
        thetaError = targetPose.theta - pose->theta;
        if (thetaError > 180) {
            thetaError -= 360;
        } else if (thetaError < -180) {
            thetaError += 360;
        }
        dP.pData[0] = targetPose.x - pose->x;
        dP.pData[1] = targetPose.y - pose->y;
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
        // uprintf("g: %.3f %.2f %.2f %.1f\n", error, pose->x, pose->y, pose->theta);
        uprintf("g: %.3f %.2f %.2f %.2f %.2f\n", e.pData[0], e.pData[1], e.pData[2], speedL, speedL);
        vTaskDelay(1);
    }

    motorHardStop(MOTOR_LEFT);
    motorHardStop(MOTOR_RIGHT);
}