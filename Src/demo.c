#include "demo.h"

#include <math.h>

#include "FreeRTOS.h"
#include "bsp.h"
#include "color_sensor.h"
#include "debug.h"
#include "motors.h"
#include "sensors.h"
#include "stm32f4xx_hal.h"
#include "task.h"

DemoState_T demoStates[] = {
    {DEMO_INIT, demoInit},
    {DEMO_FWD, demoFwd},
    {DEMO_BWD, demoBwd},
    {DEMO_LEFT, demoLeft},
    {DEMO_RIGHT, demoRight},
    {DEMO_DIST_SENSE, demoDistSense},
    {DEMO_LINE_SENSE, demoLineSense},
    {DEMO_END, demoEnd},
};

DemoStates_E demoState = DEMO_INIT;

uint8_t isSDemoStarted = 0;
#define DEBOUNCE_TIME 150
volatile uint32_t buttonPressTime = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == B1_Pin && !isSDemoStarted) {
        if (HAL_GetTick() - buttonPressTime < DEBOUNCE_TIME) return;
        buttonPressTime = HAL_GetTick();
        isSDemoStarted = 1;
        // nextDemoState();
    }
}

DemoStates_E nextDemoState() {
    uprintfISR("demoState: %d -> ", demoState);
    demoState = (DemoStates_E)((demoState + 1) % (DEMO_END + 1));
    uprintfISR("%d\n", demoState);
    return demoState;
}

DemoStates_E getDemoState() {
    uprintf("demoState: %d\n", demoState);
    return demoState;
}

void demoStateMachine() {
    DemoStates_E prevState = demoState;
    while (1) {
        if (demoState != prevState) {
            uprintf("calling from: %d -> %d\n", prevState, demoState);
            prevState = demoState;
            demoStates[demoState].demoFunc();
        }
        vTaskDelay(10);
    }
}

void demoInit() {
    uprintf("demoInit\n");
    motorSetDir(MOTOR_LEFT, MOTOR_STOP);
    motorSetDir(MOTOR_RIGHT, MOTOR_STOP);
    setMotorDutyCycle(MOTOR_LEFT, 0);
    setMotorDutyCycle(MOTOR_RIGHT, 0);
}

void demoEnd() {
    uprintf("demoEnd\n");
    motorSetDir(MOTOR_LEFT, MOTOR_STOP);
    motorSetDir(MOTOR_RIGHT, MOTOR_STOP);
    setMotorDutyCycle(MOTOR_LEFT, 0);
    setMotorDutyCycle(MOTOR_RIGHT, 0);
}

void demoFwd() {
    uprintf("demoFwd\n");
    motorSetDir(MOTOR_LEFT, MOTOR_FWD);
    motorSetDir(MOTOR_RIGHT, MOTOR_FWD);
    setMotorDutyCycle(MOTOR_LEFT, 100);
    setMotorDutyCycle(MOTOR_RIGHT, 100);
}

void demoBwd() {
    uprintf("demoBwd\n");
    motorSetDir(MOTOR_LEFT, MOTOR_BWD);
    motorSetDir(MOTOR_RIGHT, MOTOR_BWD);
    setMotorDutyCycle(MOTOR_LEFT, 100);
    setMotorDutyCycle(MOTOR_RIGHT, 100);
}

void demoLeft() {
    uprintf("demoLeft\n");
    motorSetDir(MOTOR_LEFT, MOTOR_BWD);
    motorSetDir(MOTOR_RIGHT, MOTOR_FWD);
    setMotorDutyCycle(MOTOR_LEFT, 100);
    setMotorDutyCycle(MOTOR_RIGHT, 100);
}

void demoRight() {
    uprintf("demoRight\n");
    motorSetDir(MOTOR_LEFT, MOTOR_FWD);
    motorSetDir(MOTOR_RIGHT, MOTOR_BWD);
    setMotorDutyCycle(MOTOR_LEFT, 100);
    setMotorDutyCycle(MOTOR_RIGHT, 100);
}

void demoDistSense() {
    // drive forward until distance sensor detects something
    uprintf("demoDistSense\n");
    motorSetDir(MOTOR_LEFT, MOTOR_FWD);
    motorSetDir(MOTOR_RIGHT, MOTOR_FWD);
    setMotorDutyCycle(MOTOR_LEFT, 100);
    setMotorDutyCycle(MOTOR_RIGHT, 100);
    while (ADC_to_Volt(adcBuf[0]) < .6) {
        uprintf("dist l: %lf\n", ADC_to_Volt(adcBuf[0]));
        vTaskDelay(50);
    }
    motorSoftStop();
    uprintf("dist: %lf\n", ADC_to_Volt(adcBuf[0]));
}

void demoLineSense() {
    // pid loop to follow line
    uprintf("demoLineSense\n");
    vTaskDelay(1000);
    float Kp = 0.5;
    float Kd = 0.1;
    float prevError = 0;
    float error;
    while (1) {
        // error = colorGetLineDeviation();
        error = 2;
        float dutyCycle = Kp * error + Kd * (error - prevError);
        prevError = error;

        motorSetDir(MOTOR_LEFT, dutyCycle > 0 ? MOTOR_FWD : MOTOR_BWD);
        motorSetDir(MOTOR_RIGHT, dutyCycle > 0 ? MOTOR_BWD : MOTOR_FWD);
        setMotorDutyCycle(MOTOR_LEFT, fabs(dutyCycle));
        setMotorDutyCycle(MOTOR_RIGHT, fabs(dutyCycle));
        vTaskDelay(1);
    }
}