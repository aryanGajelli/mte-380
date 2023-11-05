#include "servo.h"
#include "mathUtils.h"

#include "bsp.h"
#include "debug.h"
#include "stm32f4xx_hal.h"


uint32_t ARR;
uint32_t PSC;
uint32_t pwmPeriod_ms;
float currentAngle = CLAW_OPEN_ANGLE;

HAL_StatusTypeDef setServoAngle(float angle) {
    if (angle < 0 || angle > 180) {
        uprintf("SERVO angle out of range (0-180), got: %f\n", angle);
        return HAL_ERROR;
    }
    static const double minPeriod_ms = 0.5;
    static const double maxPeriod_ms = 2.5;
    double minDutyCycle = minPeriod_ms / pwmPeriod_ms * 100;
    double maxDutyCycle = maxPeriod_ms / pwmPeriod_ms * 100;
    // uprintf("mapped: %f\n", map(angle, 0, 180, minDutyCycle, maxDutyCycle));
    currentAngle = angle;
    return setServoDutyCycle(map(angle, 0, 180, minDutyCycle, maxDutyCycle));
}

HAL_StatusTypeDef setServoDutyCycle(float dutyCycle) {
    if (dutyCycle < 0 || dutyCycle > 100) {
        uprintf("Invalid duty cycle: %f\n", dutyCycle);
        return HAL_ERROR;
    }

    uint32_t CRR = dutyCycle * ARR / 100.;
    __HAL_TIM_SET_COMPARE(&SERVO_TIMER_HANDLE, TIM_CHANNEL_1, CRR);
    return HAL_OK;
}

float getServoAngle() {
    return currentAngle;
}

HAL_StatusTypeDef servoInit() {
    ARR = __HAL_TIM_GET_AUTORELOAD(&SERVO_TIMER_HANDLE);
    PSC = SERVO_TIMER_INSTANCE->PSC;
    pwmPeriod_ms = 1000 * ((ARR + 1) * (PSC + 1)) / HAL_RCC_GetSysClockFreq();

    HAL_TIM_PWM_Start(&SERVO_TIMER_HANDLE, TIM_CHANNEL_1);
    return setServoAngle(CLAW_OPEN_ANGLE);
}