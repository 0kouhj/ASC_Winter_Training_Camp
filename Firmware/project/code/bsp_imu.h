#ifndef BSP_IMU_H
#define BSP_IMU_H

#include "zf_common_typedef.h"
#include "zf_common_headfile.h"
#include "Kalman.h"

// --- I2C 配置 ---
#define ICM_I2C_ADDR    0x68    // AD0 接地为 0x68, 接高电平为 0x69
#define ICM_SCL_PIN     B13
#define ICM_SDA_PIN     B14

// --- 寄存器地址 (与 SPI 模式相同) ---
#define ICM42688_REG_DEVICE_CONFIG      0x11
#define ICM42688_REG_PWR_MGMT0          0x4E
#define ICM42688_REG_GYRO_CONFIG0       0x4F
#define ICM42688_REG_ACCEL_CONFIG0      0x50
#define ICM42688_REG_TEMP_DATA1         0x1D
#define ICM42688_REG_WHO_AM_I           0x75

// --- 函数声明 ---
uint8 ICM42688_I2C_Init(void);
void ICM42688_I2C_Read_Data(ICM42688_t *dev);
void ICM_Update(void);
#endif