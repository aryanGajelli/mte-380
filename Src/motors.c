#include "motors.h"

#include "bsp.h"
#include "debug.h"
#include "main.h"

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

void setMotorDir(Motor_E motor, MotorDirection_E dir) {
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

HAL_StatusTypeDef motorsInit(void) {
    motorARR = __HAL_TIM_GET_AUTORELOAD(&MOTORS_TIMER_HANDLE);
    HAL_TIM_PWM_Start(&MOTORS_TIMER_HANDLE, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&MOTORS_TIMER_HANDLE, TIM_CHANNEL_2);
    setMotorDir(MOTOR_LEFT, MOTOR_STOP);
    setMotorDir(MOTOR_RIGHT, MOTOR_STOP);
    setMotorDutyCycle(MOTOR_LEFT, 0);
    setMotorDutyCycle(MOTOR_RIGHT, 0);
    return HAL_OK;
}
