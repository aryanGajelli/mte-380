#include "servo.h"
#include "mathUtils.h"

#include "bsp.h"
#include "debug.h"
#include "stm32f4xx_hal.h"

#define MIN_SERVO_PERIOD_MS 0.5
#define MAX_SERVO_PERIOD_MS 2.5

uint32_t ARR;
uint32_t PSC;
float minDutyCycle;
float maxDutyCycle;
float currentAngle = CLAW_OPEN_ANGLE;

HAL_StatusTypeDef servoSetAngle(float angle) {
    if (angle < 0 || angle > 180) {
        uprintf("SERVO angle out of range (0-180), got: %f\n", angle);
        return HAL_ERROR;
    }
    currentAngle = angle;
    return servoSetDutyCycle(map(angle, 0, 180, minDutyCycle, maxDutyCycle));
}

HAL_StatusTypeDef servoSetDutyCycle(float dutyCycle) {
    if (dutyCycle < 0 || dutyCycle > 100) {
        uprintf("Invalid servo duty cycle: %f\n", dutyCycle);
        return HAL_ERROR;
    }

    uint32_t CRR = dutyCycle * ARR / 100.;
    __HAL_TIM_SET_COMPARE(&SERVO_TIMER_HANDLE, TIM_CHANNEL_1, CRR);
    return HAL_OK;
}

float servoGetAngle() {
    return currentAngle;
}

HAL_StatusTypeDef servoInit() {
    ARR = __HAL_TIM_GET_AUTORELOAD(&SERVO_TIMER_HANDLE);
    PSC = SERVO_TIMER_INSTANCE->PSC;
    uint32_t pwmPeriod_ms = 1000 * ((ARR + 1) * (PSC + 1)) / HAL_RCC_GetSysClockFreq();
    minDutyCycle = MIN_SERVO_PERIOD_MS / pwmPeriod_ms * 100;
    maxDutyCycle = MAX_SERVO_PERIOD_MS / pwmPeriod_ms * 100;

    HAL_TIM_PWM_Start(&SERVO_TIMER_HANDLE, TIM_CHANNEL_1);
    servoSetAngle(CLAW_OPEN_ANGLE);
    return HAL_OK;
    // return servoSetAngle(CLAW_OPEN_ANGLE);
}