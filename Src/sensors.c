#include "sensors.h"

#include <stdbool.h>

#include "bsp.h"
#include "debug.h"
#include "stm32f4xx_hal.h"

bool ADC_CALIBRATED = false;
bool ADC_Complete = false;
double VDDA = 3.31;
float VDDA_DIV_4095;
void ADC_Calibrate() {
    uint16_t VREFINT_CAL = (*((uint16_t*)VREFINT_CAL_ADDR));
    // set channel to VREFINT
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_VREFINT;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    HAL_ADC_ConfigChannel(&ADC_HANDLE, &sConfig);

    // start DMA ADC
    const size_t bufLen = 100;
    uint32_t buf[6 * bufLen];

    HAL_ADC_Start_DMA(&ADC_HANDLE, buf, sizeof(buf) / sizeof(buf[0]));
    HAL_Delay(10);
    HAL_ADC_Stop_DMA(&ADC_HANDLE);

    // average VREFINT_DATA located at buf[n%6 == 0]
    float VREFINT_DATA = 0;
    for (int i = 0; i < bufLen; i++) {
        VREFINT_DATA += buf[i * 6];
    }
    VREFINT_DATA /= bufLen;
    // calculate VDDA
    VDDA = 3.3 * VREFINT_CAL / VREFINT_DATA;
    VDDA_DIV_4095 = VDDA / 4095;
    ADC_CALIBRATED = true;
    MX_ADC1_Init();
}

sensors_t sensors;
uint32_t adcBuf[NUM_ADC_CHANNELS];
HAL_StatusTypeDef sensorsInit(void) {
    // ADC_Calibrate();
    return HAL_ADC_Start_DMA(&ADC_HANDLE, adcBuf, NUM_ADC_CHANNELS);
}

sensors_t* getSensors_Handle(void) {
    return &sensors;
}

double ADC_to_Volt(uint32_t adc_val) {
    double offset = 0.00;
    return VDDA * adc_val / 4095 + offset;
}