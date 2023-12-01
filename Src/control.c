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
void controlTurnToLego();
void controlPurePursuit(vector3_t *path, size_t pathLen, MotorDirection_E dir);
void controlFullSequence();
void controlAlignWithLine();

void controlTask(void *pvParameters) {
    while (!isControlInit) {
        vTaskDelay(10);
    }
    uprintf("controlTask\n");

    double line;
    Pose_T *pose = odometryGetPose();
    // controlApprochLego();
    // controlLineFollowing();
    controlFullSequence();
    // controlTurnToLego();
    // controlMoveForward(110, 0.7);
    // servoSetAngle(CLAW_CLOSED_ANGLE);
    // controlAlignWithLine();

    // controlGoToPoint((Pose_T){.x = 0, .y = -100, .theta = 0}, -1);
    // controlGoToPoint((Pose_T){.x = 0, .y = -100, .theta = 0}, MOTOR_BWD);
    while (1) {
        // IntersectionType_E type = circleSegmentIntersection(pose->v, lookAheadRadius, path[i], path[i+1], &out1, &out2);
        // vector3_t targetPoint = pickClosestIntersection(path[i+1], out1, out2);
        // uprintf("t:%d %.2f %.2f p:%.2f %.2f\n", type, targetPoint.x, targetPoint.y, pose->x, pose->y);
        // colorUpdate();
        // uprintf("%d %d %d\n", colorSensors.freq[0], colorSensors.freq[1], colorSensors.freq[2]);
        // uprintf("%.3f %.3f %.3f %d %f\n", colorSensors.normalizedOut.x, colorSensors.normalizedOut.y, colorSensors.normalizedOut.z, colorSensors.surface, colorSensors.lineDeviation);
        uprintf("%.3f %.3f %.3f\n", pose->x, pose->y, pose->theta);
        vTaskDelay(10);
    }
}

char legoTURN = 0;
void controlFullSequence() {
    vector3_t path[] = {
        {0.0, 0.0},
        {-39.624, 167.64000000000001},
        {-152.4, 360.52},
        {-304.8, 502.92},
        {-457.20000000000005, 594.36},
        {-609.6, 655.32},
        {-762.0, 688.848},
        {-914.4000000000001, 670.5600000000001},
        {-1066.8, 624.8399999999999},
        {-1203.96, 518.16},
        {-1249.6799999999998, 472.44000000000005},
        {-1351.6000000000001, 391.0}
        };

    size_t pathLen = sizeof(path) / sizeof(path[0]);

    motorHardStop(MOTOR_LEFT);
    motorHardStop(MOTOR_RIGHT);

    Pose_T *pose = odometryGetPose();
    controlPurePursuit(path, pathLen, MOTOR_FWD);

    motorSetSpeed(MOTOR_LEFT, -1);
    motorSetSpeed(MOTOR_RIGHT, -1);
    vTaskDelay(100);
    motorHardStop(MOTOR_LEFT);
    motorHardStop(MOTOR_RIGHT);

    Pose_T legoPose = {-1513.0, 370.8};
    Pose_T endPose = {-1350, 370};

    double heading = odometryGet2DAngle(legoPose, *pose);
    controlTurnToHeading(heading, 0);
    motorHardStop(MOTOR_LEFT);
    motorHardStop(MOTOR_RIGHT);

    controlTurnToLego();
    controlMoveForward(125, 0.17);
    servoSetAngle(CLAW_CLOSED_ANGLE);
    motorSetSpeed(MOTOR_LEFT, -1);
    motorSetSpeed(MOTOR_RIGHT, -1);
    vTaskDelay(300);
    motorHardStop(MOTOR_LEFT);
    motorHardStop(MOTOR_RIGHT);
    
    Pose_T backupPoint = {.x = -1250, .y = pose->y, .theta = 0};
    heading = odometryGet2DAngle(backupPoint, *pose);

    controlTurnToHeading(heading + 180, 1);
    controlGoToPoint(backupPoint, -0.3);
    controlTurnToHeading(-90, 0);
    controlMoveForward(150, 0.5);
    // controlGoToPoint((Pose_T){.x = -1200, .y = 200, .theta = 0}, 0.5);

    servoSetAngle(CLAW_OPEN_ANGLE);
    // vTaskDelay(300);
    controlMoveForward(50, 0.8);
    vTaskDelay(600);
    controlMoveForward(-300, 0.3);
    // controlTurnToHeading(-90, 0);
    controlMoveForward(-250, 0.3);
    controlTurnToHeading(-10, 0);

    controlMoveForward(800, 0.3);
    controlMoveForward(500, 0.3);
    motorHardStop(MOTOR_LEFT);
    motorHardStop(MOTOR_RIGHT);

    controlGoToPoint((Pose_T){.x = 90, .y = 150, .theta = 0}, 0.3);
}

void controlAlignWithLine() {
    double kp = 0.5, kd = 0, ki = 0;
    double target = 5;
    double error = target - colorSensors.lineDeviation, prevError = 0;
    while (fabs(error) > 0.1) {
        colorUpdate();
        error = target - colorSensors.lineDeviation;
        double speed = kp * error + kd * (error - prevError);
        prevError = error;
        speed = clamp(speed, -100, 100);

        motorSetSpeed(MOTOR_LEFT, -speed);
        motorSetSpeed(MOTOR_RIGHT, speed);
        uprintf("c: %.3f %.3f %.3f\n", error, speed, colorSensors.lineDeviation);
        vTaskDelay(1);
    }
    motorHardStop(MOTOR_LEFT);
    motorHardStop(MOTOR_RIGHT);
}

void controlTurnToLego() {
    Pose_T *pose = odometryGetPose();
    Pose_T startPose = *pose;
    vector3_t maxVal = {.x = 0, .y = pose->theta};
    int side = 0;
    double arcAngle = 20;
    double maxSpeed = 60;
    double maxDistVal = 1;
    double target = startPose.theta + arcAngle;
    double kp = 1, kd = 0.5;
    double error = adjustTurn(target - pose->theta);
    double prevError = error;
    double val = ADC_to_Volt(adcBuf[3]);
    double prevVal;
    while (fabs(error) > 1) {
        if (fabs(error) < 10 && side == 0) {
            side = 1;
            motorHardStop(MOTOR_LEFT);
            motorHardStop(MOTOR_RIGHT);
            vTaskDelay(200);
            target = startPose.theta - arcAngle;
        }

        error = adjustTurn(target - pose->theta);
        double angVel = kp * error + kd * (error - prevError);
        prevVal = val;
        val = ADC_to_Volt(adcBuf[3]);

        if (fabs(val - prevVal) > 0.2) {
            continue;
        }
        
        if (val < maxDistVal && val > maxVal.x && fabs(maxVal.y - pose->theta) > 2) {
            maxVal.x = val;
            maxVal.y = pose->theta;
        }
        prevError = error;
        motorSetSpeed(MOTOR_LEFT, clamp(-angVel, -maxSpeed, maxSpeed));
        motorSetSpeed(MOTOR_RIGHT, clamp(angVel, -maxSpeed, maxSpeed));
        uprintf("t1: %.3f %.3f c: %.3f %.3f m:  %.3f %.3f\n", target, error, val, pose->theta, maxVal.x, maxVal.y);
        vTaskDelay(2);
    }

    // motorSetSpeed(MOTOR_LEFT, sign(error));
    // motorSetSpeed(MOTOR_RIGHT, -sign(error));

    motorHardStop(MOTOR_LEFT);
    motorHardStop(MOTOR_RIGHT);
    vTaskDelay(300);
    // servoSetAngle(CLAW_OPEN_ANGLE);
    controlTurnToHeading(maxVal.y, 1);
}

void controlPurePursuit(vector3_t *path, size_t pathLen, MotorDirection_E dir) {
    int direction = dir == MOTOR_FWD ? 1 : -1;
    double linVel;
    double lookAheadRadius = 50;
    size_t lastFoundIndex = 0;

    double kp = 2, kd = 0.0;
    double kpL = 0.2, kdL = 0.5;
    PurePursuitOutput_T prevPP;
    PurePursuitOutput_T PP = {0, 0, 0, 0, 0};
    Pose_T *pose = odometryGetPose();
    double angDirOffset = dir == MOTOR_FWD ? 0 : 180;

    double errorLin = odometryDotError((Pose_T)path[pathLen - 1], *pose), prevErrorLin = 0;
    while (lastFoundIndex < pathLen) {
        prevPP = PP;
        PP = controlPurePursuitStep(path, pathLen, lookAheadRadius, lastFoundIndex);
        lastFoundIndex = PP.lastFoundIndex;
        errorLin = odometryDotError((Pose_T)path[pathLen - 1], *pose);

        // double angVel = kp * PP.angError + kd * (PP.angError - prevPP.angError);

        prevErrorLin = errorLin;

        linVel = kpL * errorLin + kdL * (errorLin - prevErrorLin);

        if (lastFoundIndex < 2) {
            linVel = clamp(linVel, -70, 70);
        }
        double angVel = WHEEL_TO_WHEEL_DISTANCE * sin(DEG_TO_RAD(PP.angError + angDirOffset)) * linVel / lookAheadRadius;

        motorSetSpeed(MOTOR_LEFT, clamp(direction * linVel - angVel, -100, 100));
        motorSetSpeed(MOTOR_RIGHT, clamp(direction * linVel + angVel, -100, 100));
        uprintf("p: %.3f %.3f %.3f %d %.3f\n", PP.targetPoint.x, PP.targetPoint.y, PP.angError, PP.lastFoundIndex, angVel);
        vTaskDelay(1);
    }

    // controlGoToPoint((Pose_T)path[pathLen - 1]);
    motorHardStop(MOTOR_LEFT);
    motorHardStop(MOTOR_RIGHT);
}

PurePursuitOutput_T controlPurePursuitStep(vector3_t *path, size_t pathLen, double lookAheadRadius, size_t lastFoundIndex) {
    vector3_t targetPoint = {NAN, NAN, NAN};
    IntersectionType_E type;
    Pose_T *pose = odometryGetPose();
    size_t i;
    uint32_t start = HAL_GetTick();
    for (i = lastFoundIndex; i < pathLen; i++) {
        vector3_t out1, out2;
        type = circleSegmentIntersection(pose->v, lookAheadRadius, path[i], path[i + 1], &out1, &out2);
        if (type == NO_INTERSECTION) {
            targetPoint = path[lastFoundIndex];
            break;
        }
        if (type == ONE_INTERSECTION) {
            targetPoint = out1;
        }
        if (type == TWO_INTERSECTIONS) {
            targetPoint = pickClosestIntersection(path[i + 1], out1, out2);
        }
        if (dist(targetPoint.x, targetPoint.y, path[i + 1].x, path[i + 1].y) < dist(pose->x, pose->y, path[i + 1].x, path[i + 1].y)) {
            lastFoundIndex = i + 1;
            break;
        } else {
            lastFoundIndex = i;
        }
    }

    // we now have target point
    double targetAng = RAD_TO_DEG(atan2(targetPoint.y - pose->y, targetPoint.x - pose->x));
    double angError = adjustTurn(targetAng - pose->theta);

    // uprintf("ps: %d %.2f %.2f i: %d %d p: %.2f %.2f %.2f %.2f\n", type, targetPoint.x, targetPoint.y, i, lastFoundIndex, pose->x, pose->y, pose->theta, angError);
    return (PurePursuitOutput_T){.targetPoint = targetPoint, .lastFoundIndex = lastFoundIndex, .angError = angError};
}

void controlTestSquareAbsolute() {
    controlGoToPoint((Pose_T){.x = 0, .y = 300, .theta = 0}, 1);
    controlGoToPoint((Pose_T){.x = 300, .y = 300, .theta = 0}, 1);
    controlGoToPoint((Pose_T){.x = 300, .y = 0, .theta = 0}, 1);
    controlGoToPoint((Pose_T){.x = 0, .y = 0, .theta = 0}, 1);
}

void controlTestPickup() {
    controlMoveForward(350, 1);
    HAL_Delay(200);
    controlTurnToHeading(90, 0);
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
    controlTurnToHeading(90, 0);
    HAL_Delay(200);
    controlMoveForward(150, 1);
    HAL_Delay(200);
    controlTurnToHeading(180, 0);
    HAL_Delay(200);
    controlMoveForward(150, 1);
    HAL_Delay(200);
    controlTurnToHeading(270, 0);
    HAL_Delay(200);
    controlMoveForward(150, 1);
    HAL_Delay(200);
    controlTurnToHeading(0, 0);
}

void controlApprochLego() {
    double kp = 20, kd = 0.5, ki = 0;
    double target = 0.7;
    double error = target - ADC_to_Volt(adcBuf[3]), prevError = error;

    while (fabs(error) > 0.1) {
        double val  = ADC_to_Volt(adcBuf[3]);
        error = target - val;
        double speed = kp * error + kd * (error - prevError);
        prevError = error;
        motorSetSpeed(MOTOR_LEFT, clamp(speed, -100, 100));
        motorSetSpeed(MOTOR_RIGHT, clamp(speed, -100, 100));
        uprintf("%.3f %.3f\n", error, val);
    }
    motorHardStop(MOTOR_LEFT);
    motorHardStop(MOTOR_RIGHT);
}

void controlLineFollowing() {
    // controlGoToPoint((Pose_T){.x = -30, .y = 150, .theta = 0});
    // controlMoveForward(120, 0.6);
    double target = 0;
    double prevError = target - colorSensors.lineDeviation;
    double error;
    double constSpeed = 60;
    double kp = 2, kd = 1, ki = 0.00;
    Pose_T *pose = odometryGetPose();
    double dotError;
    double angError;
    do {
        // colorUpdate();
        dotError = odometryDotError((Pose_T){.x = -1400, .y = 300}, *pose);
        error = target - colorSensors.lineDeviation;
        double speed = kp * error + kd * (error - prevError) + ki * error;
        // angError =
        // speed = clamp(speed, -100, 100);
        // if (fabs(error) < 1) {
        //     speed = 0;
        // }
        prevError = error;
        // if (colorSensors.surface != SURFACE_TAPE) {
        //     break;
        // }
        motorSetSpeed(MOTOR_LEFT, clamp(constSpeed - speed, -100, 100));
        motorSetSpeed(MOTOR_RIGHT, clamp(constSpeed + speed, -100, 100));
        uprintf("l: %.3f %.3f %.3f %d %.3f\n", pose->x, pose->y, speed, colorSensors.surface, colorSensors.lineDeviation);
        // vTaskDelay(1);
        // if (dist(pose->x, pose->y, 0, 0) > 300)
        //     break;
    } while (fabs(dotError) > 50);
    motorHardStop(MOTOR_LEFT);
    motorHardStop(MOTOR_RIGHT);

    double heading = RAD_TO_DEG(atan2(300 - pose->y, -1800 - pose->x));
    controlTurnToHeading(heading, 0);
}

void controlTurnToHeading(double targetDeg, char legoTurn) {
    // targetDeg = -targetDeg;
    static const double ACCEPTABLE_ERROR_DEG = 1.5;
    Pose_T *pose = odometryGetPose();
    Pose_T startPose = *pose;
    double kp = 0.7, kd = 0.5, ki = 0.09;
    double error = adjustTurn(targetDeg - startPose.theta);

    if (fabs(error) < 1) return;
    if (legoTurn) {
        kp = 1.9;
    }
    double prevError = error;
    int dir = error > 180 ? -1 : 1;
    double integral = 0;
    double angVel, angAcc, prevAngVel = 0;
    do {
        prevError = error;
        error = adjustTurn(targetDeg - pose->theta);
        if (fabs(error) < 5) {
            integral = 0.99 * integral + error;
        } else {
            integral = 0;
        }
        angVel = kp * error + kd * (error - prevError) + ki * integral;
        angVel = clamp(angVel, -100, 100);
        angAcc = angVel - prevAngVel;
        prevAngVel = angVel;
        motorSetSpeed(MOTOR_LEFT, dir * -angVel);
        motorSetSpeed(MOTOR_RIGHT, dir * angVel);
        uprintf("t:%.3f %.3f %.3f %.3f\n", error, angVel, angAcc, pose->theta);
        // vTaskDelay(1);
    } while (fabs(error) > ACCEPTABLE_ERROR_DEG || fabs(angAcc) > 0.4);

    motorHardStop(MOTOR_LEFT);
    motorHardStop(MOTOR_RIGHT);
}

void controlMoveForward(double targetDist, double speedMultiplier) {
    static const double ACCEPTABLE_ERROR = 10;
    Pose_T *pose = odometryGetPose();
    Pose_T startPose = *pose;
    double kp = 0.75, kd = 0.5, ki = 0.00;
    double kpT = 1.5, kdT = 0.5, kiT = 0.00;
    double targetTheta = startPose.theta;
    // targetTheta += speedMultiplier > 0 ? 0 : 180;
    double dist = (encLeft->dist + encRight->dist) / 2;
    targetDist += dist;
    double error = targetDist - dist;
    double errorAng = adjustTurn(targetTheta - startPose.theta);
    double prevErrorAng = errorAng;
    if (fabs(error) < ACCEPTABLE_ERROR) return;
    double prevError = error;
    double integral = 0;
    double angSpeed = 0;
    while (fabs(error) > ACCEPTABLE_ERROR) {
        dist = (encLeft->dist + encRight->dist) / 2;
        error = targetDist - dist;
        error *= sign(speedMultiplier);

        errorAng = adjustTurn(targetTheta - pose->theta);

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

        motorSetSpeed(MOTOR_LEFT, clamp(speedMultiplier * speedLin - angSpeed, -100, 100));
        motorSetSpeed(MOTOR_RIGHT, clamp(speedMultiplier * speedLin + angSpeed, -100, 100));

        uprintf("m: %.3f %.3f %.3f %.3f %.3f\n", error, speedLin, pose->x, pose->y, pose->theta);
    }

    motorHardStop(MOTOR_LEFT);
    motorHardStop(MOTOR_RIGHT);
}

void controlGoToPoint(Pose_T targetPose, double speedMultiplier) {
    static const double ACCEPTABLE_ERROR = 5;
    Pose_T *pose = odometryGetPose();
    Pose_T startPose = *pose;

    double targetTheta = odometryGet2DAngle(targetPose, startPose);
    targetTheta += speedMultiplier > 0 ? 0 : 180;
    controlTurnToHeading(targetTheta, legoTURN);
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

        motorSetSpeed(MOTOR_LEFT, clamp(speedMultiplier * speed - speedT, -100, 100));
        motorSetSpeed(MOTOR_RIGHT, clamp(speedMultiplier * speed + speedT, -100, 100));

        uprintf("g: %.3f %.3f %.3f %.3f\n", error, pose->x, pose->y, pose->theta);
        vTaskDelay(1);
    }

    motorHardStop(MOTOR_LEFT);
    motorHardStop(MOTOR_RIGHT);
}

void controlGoToPoseRAMSETE(Pose_T targetPose) {
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