#include "fusion.h"

#include "debug.h"
#include "motion_fx.h"

#define MFX_STR_LENG 35
#define ENABLE_9X 1

HAL_StatusTypeDef fusionInit(void) {
    __CRC_CLK_ENABLE();
    char lib_version[MFX_STR_LENG];
    MFX_knobs_t iKnobs;
    float LastTime;

    MotionFX_initialize();

    MotionFX_GetLibVersion(lib_version);
    uprintf("MotionFX version: %s\n", lib_version);

    MotionFX_getKnobs(&iKnobs);
    iKnobs.LMode = 0;
    iKnobs.modx = 1;
    MotionFX_setKnobs(&iKnobs);
    printKnobs(&iKnobs);
    MotionFX_enable_6X(MFX_ENGINE_DISABLE);
    MotionFX_enable_9X(MFX_ENGINE_DISABLE);

    if (ENABLE_9X){
        MotionFX_enable_9X(MFX_ENGINE_ENABLE);
    }
    else{
        MotionFX_enable_6X(MFX_ENGINE_ENABLE);
    }
    uprintf("9X state: %d\n", MotionFX_getStatus_9X());
    return HAL_OK;
}

void printKnobs(MFX_knobs_t* iKnobs) {
    uprintf("Default knobs values:\n");
    uprintf("ATime: %f\n", iKnobs->ATime);
    uprintf("MTime: %f\n", iKnobs->MTime);
    uprintf("FrTime: %f\n", iKnobs->FrTime);
    uprintf("LMode: %d\n", iKnobs->LMode);
    uprintf("gbias_mag_th_sc_6X: %f\n", iKnobs->gbias_mag_th_sc_6X);
    uprintf("gbias_acc_th_sc_6X: %f\n", iKnobs->gbias_acc_th_sc_6X);
    uprintf("gbias_gyro_th_sc_6X: %f\n", iKnobs->gbias_gyro_th_sc_6X);
    uprintf("gbias_mag_th_sc_9X: %f\n", iKnobs->gbias_mag_th_sc_9X);
    uprintf("gbias_acc_th_sc_9X: %f\n", iKnobs->gbias_acc_th_sc_9X);
    uprintf("gbias_gyro_th_sc_9X: %f\n", iKnobs->gbias_gyro_th_sc_9X);
    uprintf("modx: %d\n", iKnobs->modx);
    uprintf("acc_orientation: %s\n", iKnobs->acc_orientation);
    uprintf("gyro_orientation: %s\n", iKnobs->gyro_orientation);
    uprintf("mag_orientation: %s\n", iKnobs->mag_orientation);
    uprintf("output_type: %d\n", iKnobs->output_type);
    uprintf("start_automatic_gbias_calculation: %d\n", iKnobs->start_automatic_gbias_calculation);
}