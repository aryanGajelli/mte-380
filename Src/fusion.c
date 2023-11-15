#include "fusion.h"

#include <string.h>

#include "debug.h"
#include "imu.h"
#include "motion_fx.h"
#define MFX_STR_LENG 35
#define ENABLE_9X 0

void runMagCal();

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
    memcpy(iKnobs.mag_orientation, "ned", 4);
    iKnobs.output_type = MFX_ENGINE_OUTPUT_ENU;
    printKnobs(&iKnobs);
    MotionFX_setKnobs(&iKnobs);
    MotionFX_enable_6X(MFX_ENGINE_DISABLE);
    MotionFX_enable_9X(MFX_ENGINE_DISABLE);
    // runMagCal();

    if (ENABLE_9X) {
        MotionFX_enable_9X(MFX_ENGINE_ENABLE);
    } else {
        MotionFX_enable_6X(MFX_ENGINE_ENABLE);
    }
    return HAL_OK;
}

void runMagCal() {
    MotionFX_MagCal_init(10, MFX_ENGINE_ENABLE);
#define CAL_SIZE 1000
    MFX_MagCal_input_t magCal;
    IMUData_T imuData;
    for (int i = 0; i < CAL_SIZE; i++) {
        ICM_Read(&imuData);
        magCal.mag[0] = imuData.mag.x / 50;
        magCal.mag[1] = imuData.mag.y / 50;
        magCal.mag[2] = imuData.mag.z / 50;
        magCal.time_stamp = imuData.timestamp;
        // uprintf("mag: %.3f, %.3f, %.3f\n", magCal.mag[0], magCal.mag[1], magCal.mag[2]);
        MotionFX_MagCal_run(&magCal);
        MFX_MagCal_output_t mag_cal_out;
        MotionFX_MagCal_getParams(&mag_cal_out);
        if (mag_cal_out.cal_quality == MFX_MAGCALGOOD) {
            uprintf("MagCal quality: %d\n", mag_cal_out.cal_quality);
            uprintf("Mag HI Bias: %.3f, %.3f, %.3f\n", mag_cal_out.hi_bias[0], mag_cal_out.hi_bias[1], mag_cal_out.hi_bias[2]);
            break;
        }
        HAL_Delay(10);
    }
}

void printKnobs(MFX_knobs_t *iKnobs) {
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

// following added to prevent linker errors. idk why the lib doesn't have these

/**
 * @brief  Load calibration parameter from memory
 * @param  dataSize length ot the data
 * @param  data pointer to the data
 * @retval (1) fail, (0) success
 */
unsigned int *fx_buf = NULL;
char MotionFX_LoadMagCalFromNVM(unsigned short int dataSize, unsigned int *data) {
    uprintf("MotionFX_LoadMagCalFromNVM data size %d\n", dataSize);
    if (fx_buf == NULL) {
        return (char)0;
    }
    memcpy(data, fx_buf, dataSize);
    return (char)1;
}

/**
 * @brief  Save calibration parameter to memory
 * @param  dataSize length ot the data
 * @param  data pointer to the data
 * @retval (1) fail, (0) success
 */
char MotionFX_SaveMagCalInNVM(unsigned short int dataSize, unsigned int *data) {
    uprintf("MotionFX_SaveMagCalInNVM data size %d\n", dataSize);
    fx_buf = (unsigned int *)malloc(dataSize);
    if (fx_buf == NULL) {
        return (char)0;
    }
    memcpy(fx_buf, data, dataSize);
    return (char)1;
}

void fusionGetOutputs(MFX_output_t *data_out, IMUData_T imuData, IMUData_T prevImuData) {
    static MFX_input_t data_in;
    data_in.gyro[0] = imuData.gyro.x;
    data_in.gyro[1] = imuData.gyro.y;
    data_in.gyro[2] = imuData.gyro.z;
    data_in.acc[0] = imuData.accel.x / 9.81;
    data_in.acc[1] = imuData.accel.y / 9.81;
    data_in.acc[2] = imuData.accel.z / 9.81;
    // data_in.mag[0] = imuData.mag.x / 50;
    // data_in.mag[1] = imuData.mag.y / 50;
    // data_in.mag[2] = imuData.mag.z / 50;

    float dT = (imuData.timestamp - prevImuData.timestamp) / 1000.;

    MotionFX_propagate(data_out, &data_in, &dT);
    MotionFX_update(data_out, &data_in, &dT, NULL);
}