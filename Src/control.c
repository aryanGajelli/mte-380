#include "control.h"

#include "FreeRTOS.h"
#include "debug.h"
#include "odometry.h"
#include "stm32f4xx_hal.h"
#include "task.h"
#include "motors.h"
#include "arm_math.h"

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

    controlTurnToHeading(90);
    controlTurnToHeading(-45);
    while (1){
        Pose_T pose = odometryGetPose();
        uprintf("%.3f %.3f %.3f\n", pose.x, pose.y, pose.theta);
    }
}

void controlTurnToHeading(double targetDeg) {
    static const double ACCEPTABLE_ERROR_DEG = 1;
    double kp = 0.5, kd = 0.5;
    double error = targetDeg - odometryGetPose().theta;
    double prevError = error;
    int dir = error > 180 ? -1 : 1;
    do {
        Pose_T pose = odometryGetPose();
        error = targetDeg - pose.theta;
        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }
        double turnSpeed = kp * error + kd * (error - prevError);

        prevError = error;
        if (turnSpeed > 100) turnSpeed = 100;
        if (turnSpeed < -100) turnSpeed = -100;

        // turnSpeed *= 0.5;
        motorSetSpeed(MOTOR_LEFT, dir * turnSpeed);
        motorSetSpeed(MOTOR_RIGHT, dir * -turnSpeed);
        uprintf("%.3f %.3f %.3f %.3f\n", error, pose.x, pose.y, pose.theta);
    } while (fabs(error) > ACCEPTABLE_ERROR_DEG);

    motorHardStop(MOTOR_LEFT);
    motorHardStop(MOTOR_RIGHT);
}
void controlGoToPose(Pose_T pose) {
    
}