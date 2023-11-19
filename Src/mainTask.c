#include <math.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "ICM20948_register.h"
#include "ICM20948_spi.h"
#include "arm_math.h"
#include "bsp.h"
#include "color_sensor.h"
#include "control.h"
#include "debug.h"
#include "demo.h"
#include "dmp.h"
#include "encoders.h"
#include "fusion.h"
#include "imu.h"
#include "imu2.h"
#include "main.h"
#include "mathUtils.h"
#include "motion_fx.h"
#include "motors.h"
#include "odometry.h"
#include "sensors.h"
#include "servo.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "task.h"

HAL_StatusTypeDef mainTaskInit();

void mainTask(void *pvParameters) {
    uprintf("mainTask\n");

    if (mainTaskInit() != HAL_OK) {
        Error_Handler();
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    IMUData_T imuData;
    // ICM_ReadAccelGyro(&imuData);
    double k = 0.1;
    ICM_20948_AGMT_t agmt;
    icm_20948_DMP_data_t dmpData;
    double yaw, prevYaw = 0.0;
    while (1) {
        ICM_20948_Status_e status = dmpReadDataFromFIFO(&dmpData);
        // Was valid data available?
        if (status == ICM_20948_Stat_Ok || status == ICM_20948_Stat_FIFOMoreDataAvail) {
            // Check for GRV data (Quat6)
            if ((dmpData.header & DMP_header_bitmap_Quat6) > 0) {
                // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
                // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
                // The quaternion data is scaled by 2^30.
                // Scale to +/- 1
                double q1 = ((double)dmpData.Quat6.Data.Q1) / 1073741824.0;  // Convert to double. Divide by 2^30
                double q2 = ((double)dmpData.Quat6.Data.Q2) / 1073741824.0;  // Convert to double. Divide by 2^30
                double q3 = ((double)dmpData.Quat6.Data.Q3) / 1073741824.0;  // Convert to double. Divide by 2^30

                // Convert the quaternions to Euler angles (roll, pitch, yaw)
                // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2
                double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

                double q2sqr = q2 * q2;

                // yaw (z-axis rotation)
                double t3 = +2.0 * (q0 * q3 + q1 * q2);
                double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
                yaw = RAD_TO_DEG(atan2(t3, t4));
                double dYaw = yaw - prevYaw;
                prevYaw = yaw;
                if (!isnan(yaw))
                    uprintf("%.3f, %.3f\n", yaw, dYaw);
            }
        }
        // If more data is available then we should read it right away - and not delay
        if (status != ICM_20948_Stat_FIFOMoreDataAvail)
            vTaskDelay(1);  // Keep this short!

        vTaskDelayUntil(&xLastWakeTime, 10);
    }
}

HAL_StatusTypeDef mainTaskInit() {
    if (imuInit() != HAL_OK) {
        Error_Handler();
    }
    // if (ICMInit() != HAL_OK) {
    //     Error_Handler();
    // }

    if (colorSensorInit() != HAL_OK) {
        Error_Handler();
    }

    if (motorsInit() != HAL_OK) {
        Error_Handler();
    }

    if (servoInit() != HAL_OK) {
        Error_Handler();
    }

    // if (fusionInit() != HAL_OK) {
    //     Error_Handler();
    // }

    // if (odometryInit() != HAL_OK) {
    //     Error_Handler();
    // }
    if (encodersInit() != HAL_OK) {
        Error_Handler();
    }

    return HAL_OK;
}