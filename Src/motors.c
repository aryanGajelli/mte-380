#include "motors.h"

#include "bsp.h"
#include "debug.h"
#include "main.h"
#include "mathUtils.h"

uint32_t motorARR;
HAL_StatusTypeDef setMotorDutyCycle(Motor_E motor, float dutyCycle) {
    if (dutyCycle < 0 || dutyCycle > 100) {
        uprintf("Invalid duty cycle: %f\n", dutyCycle);
        return HAL_ERROR;
    }

    uint32_t CRR = dutyCycle * motorARR / 100.;

    if (motor == MOTOR_LEFT)
        __HAL_TIM_SET_COMPARE(&MOTORS_TIMER_HANDLE, TIM_CHANNEL_1, CRR);
    else if (motor == MOTOR_RIGHT)
        __HAL_TIM_SET_COMPARE(&MOTORS_TIMER_HANDLE, TIM_CHANNEL_2, CRR);
    else
        return HAL_ERROR;
    return HAL_OK;
}

void motorSetDir(Motor_E motor, MotorDirection_E dir) {
    if (motor == MOTOR_LEFT) {
        HAL_GPIO_WritePin(MOTOR_L_IN1_GPIO_Port, MOTOR_L_IN1_Pin, (dir & 0b10) >> 1);
        HAL_GPIO_WritePin(MOTOR_L_IN2_GPIO_Port, MOTOR_L_IN2_Pin, dir & 0b1);
        return;
    } else {
        HAL_GPIO_WritePin(MOTOR_R_IN3_GPIO_Port, MOTOR_R_IN3_Pin, (dir & 0b10) >> 1);
        HAL_GPIO_WritePin(MOTOR_R_IN4_GPIO_Port, MOTOR_R_IN4_Pin, dir & 0b1);
        return;
    }
}

void motorSetSpeed(Motor_E motor, float speed) {
    if (speed < -100 || speed > 100) {
        uprintf("Invalid speed: %f\n", speed);
        return;
    }
    
    if (speed < 0) {
        motorSetDir(motor, MOTOR_BWD);
        speed = -speed;
    } else if (speed > 0){
        motorSetDir(motor, MOTOR_FWD);
    } else {
        motorSetDir(motor, MOTOR_STOP);
    }

    // motor only moves at 15% duty cycle
    #define MIN_MOTOR_MOVE_DC 14.5
    speed = map(speed, 0, 100, MIN_MOTOR_MOVE_DC, 100);
    setMotorDutyCycle(motor, speed);
}

void motorSoftStop(){
    motorSetDir(MOTOR_LEFT, MOTOR_STOP);
    motorSetDir(MOTOR_RIGHT, MOTOR_STOP);
}

void motorHardStop(){
    motorSetDir(MOTOR_LEFT, 0);
    motorSetDir(MOTOR_RIGHT, 0);
}

HAL_StatusTypeDef motorsInit(void) {
    motorARR = __HAL_TIM_GET_AUTORELOAD(&MOTORS_TIMER_HANDLE);
    HAL_TIM_PWM_Start(&MOTORS_TIMER_HANDLE, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&MOTORS_TIMER_HANDLE, TIM_CHANNEL_2);
    motorSetDir(MOTOR_LEFT, MOTOR_STOP);
    motorSetDir(MOTOR_RIGHT, MOTOR_STOP);
    setMotorDutyCycle(MOTOR_LEFT, 0);
    setMotorDutyCycle(MOTOR_RIGHT, 0);
    return HAL_OK;
}
