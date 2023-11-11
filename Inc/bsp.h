#ifndef __BSP_H__
#define __BSP_H__

#include "main.h"
#include "usart.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"

// define all the peripherals here
#define DEBUG_UART_HANDLE huart2
#define ADC_HANDLE hadc1
#define IMU_SPI_HANDLE hspi2
#define COLOR_TIMER_HANDLE htim4
#define COLOR_TIMER_INSTANCE (COLOR_TIMER_HANDLE.Instance)
#define SERVO_TIMER_HANDLE htim11
#define SERVO_TIMER_INSTANCE (SERVO_TIMER_HANDLE.Instance)
#define MOTORS_TIMER_HANDLE htim2
#define MOTORS_TIMER_INSTANCE (MOTORS_TIMER_HANDLE.Instance)
#define ENCODER_TIMER_LEFT_HANDLE htim1
#define ENCODER_TIMER_LEFT_INSTANCE (ENCODER_TIMER_LEFT_HANDLE.Instance)
#define ENCODER_TIMER_RIGHT_HANDLE htim3
#define ENCODER_TIMER_RIGHT_INSTANCE (ENCODER_TIMER_RIGHT_HANDLE.Instance)

#endif // __BSP_H__