#ifndef __BSP_H__
#define __BSP_H__

#include "main.h"
#include "usart.h"
#include "adc.h"
#include "spi.h"

// define all the peripherals here
#define DEBUG_UART_HANDLE huart2
#define ADC_HANDLE hadc1
#define IMU_SPI_HANDLE hspi2
#endif // __BSP_H__