#ifndef __MOTORS_H__
#define __MOTORS_H__

#include "stm32f4xx_hal.h"

typedef enum {
    MOTOR_LEFT,
    MOTOR_RIGHT
} Motor_E;

typedef enum {
    MOTOR_FWD = 0b10,
    MOTOR_BWD = 0b01,
    MOTOR_STOP = 0b00,
} MotorDirection_E;
HAL_StatusTypeDef setMotorDutyCycle(Motor_E motor, float dutyCycle);
void setMotorDir(Motor_E motor, MotorDirection_E dir);
HAL_StatusTypeDef motorsInit(void);
#endif // __MOTORS_H__