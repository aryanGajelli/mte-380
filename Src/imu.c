#include "bsp.h"
#include "imu.h"

#include "stm32f4xx_hal.h"

#define WHO_AM_I_REG 0x00
#define READ_EN(reg) ((reg) |= 0x80)
#define WRITE_EN(reg) ((reg))

#define HSPI_TIMEOUT 15
uint32_t ICM_test() // ***
{
	uint8_t reg = WHO_AM_I_REG;

    READ_EN(reg);
    uint8_t data = 0;
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&IMU_SPI_HANDLE, &reg, 1, HSPI_TIMEOUT);
    HAL_SPI_Receive(&IMU_SPI_HANDLE, &data, 1, HSPI_TIMEOUT);
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

    return (uint32_t) (data);
}
