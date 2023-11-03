#include "color_sensor.h"

#include "bsp.h"
#include "debug.h"
#include "stm32f4xx_hal.h"

#define IDLE 0
#define DONE 1
#define F_CLK (84500000/64)

volatile uint16_t gu16_TIM2_OVC = 0;

volatile ColorFreq_T colorFreqs[4];
volatile Color_E currentColor = RED;

volatile uint32_t redLow = 2200;
volatile uint32_t redHigh = 80500;

volatile uint32_t greenLow = 1000;
volatile uint32_t greenHigh = 47500;

volatile uint32_t blueLow = 700;
volatile uint32_t blueHigh = 47500;

volatile uint32_t clearLow = 10000;
volatile uint32_t clearHigh = 10000;

volatile uint32_t gu32_Freq = 0;

#define COLOR_1_EN() HAL_GPIO_WritePin(COLOR_1_CS_GPIO_Port, COLOR_1_CS_Pin, GPIO_PIN_RESET)
#define COLOR_1_DIS() HAL_GPIO_WritePin(COLOR_1_CS_GPIO_Port, COLOR_1_CS_Pin, GPIO_PIN_SET)

volatile uint8_t gu8_State = IDLE;
volatile uint32_t gu32_T1 = 0;
volatile uint32_t gu32_T2 = 0;
volatile uint32_t gu32_Ticks = 0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == COLOR_TIMER_INSTANCE) {
        if (gu8_State == IDLE) {
            gu32_T1 = COLOR_TIMER_INSTANCE->CCR1;
            gu16_TIM2_OVC = 0;
            gu8_State = DONE;

        } else if (gu8_State == DONE) {
            gu32_T2 = COLOR_TIMER_INSTANCE->CCR1;
            gu32_Ticks = (gu32_T2 + (gu16_TIM2_OVC * 65536)) - gu32_T1;
            gu32_Freq = F_CLK / gu32_Ticks;
                // colorFreqs[currentColor].color = currentColor;
                // colorFreqs[currentColor].freq = gu32_Freq;
                // currentColor = (currentColor + 1);
                // if (currentColor > 3) {
                //     currentColor = RED;
                // }
                // setColor(currentColor);
                // if (HAL_GetTick() > 2000) {
                //     if (gu32_Freq < clearLow)
                //         clearLow = gu32_Freq;
                //     if (gu32_Freq > clearHigh)
                //         clearHigh = gu32_Freq;
                // }
            gu8_State = IDLE;
        }
    }
}

void setColorSensorFreqScaling(FreqScale_E freqScale) {
    HAL_GPIO_WritePin(COLOR_S0_GPIO_Port, COLOR_S0_GPIO_Port, (freqScale & 0b10) >> 1);
    HAL_GPIO_WritePin(COLOR_S1_GPIO_Port, COLOR_S1_GPIO_Port, freqScale & 0b1);
}

char* colorToStr(Color_E color) {
    switch (color) {
        case RED:
            return "RED";
            break;
        case GREEN:
            return "GRN";
            break;
        case BLUE:
            return "BLU";
            break;
        case CLEAR:
            return "CLR";
            break;
    }
}
void setColor(Color_E color) {
    COLOR_1_DIS();
    HAL_GPIO_WritePin(COLOR_S2_GPIO_Port, COLOR_S2_Pin, (color & 0b10) >> 1);
    HAL_GPIO_WritePin(COLOR_S3_GPIO_Port, COLOR_S3_Pin, color & 0b1);
    COLOR_1_EN();
}


void colorSensorInit() {
    HAL_TIM_Base_Start_IT(&COLOR_TIMER_HANDLE);
    HAL_TIM_IC_Start_IT(&COLOR_TIMER_HANDLE, TIM_CHANNEL_1);
    setColorSensorFreqScaling(FREQ_SCALE_100);

    setColor(CLEAR);
    // uint32_t currentTick = HAL_GetTick();
    // uint32_t prevTick = currentTick;
    // uint8_t enableCCInt = 0;
    while (1) {
        // currentTick = HAL_GetTick();
        // if (currentTick - prevTick >= 10){
        //     if (enableCCInt){
        //         HAL_TIM_IC_Start_IT(&COLOR_TIMER_HANDLE, TIM_CHANNEL_1);
        //         enableCCInt = 0;
        //     } else {
        //         HAL_TIM_IC_Stop_IT(&COLOR_TIMER_HANDLE, TIM_CHANNEL_1);
        //         enableCCInt = 1;
        //     }
        //     prevTick = currentTick;
        // }
        
        uprintf("%lu %lu\n",gu32_Ticks, gu32_Freq);
    }
}
