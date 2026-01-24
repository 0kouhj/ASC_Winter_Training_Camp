#include "param_config.h"

// 实例化全局变量
SYSTEM_CONFIG_T Config;
SYSTEM_STATE_T State;
ICM42688_t Icm;
/**
 * @brief 初始化ICM42688的值
 * @note  开机时调用
 */
void Icm42688_Init(void)
{
    // 原始数据清零
    Icm.accel_x_raw = 0;
    Icm.accel_y_raw = 0;
    Icm.accel_z_raw = 0;
    Icm.gyro_x_raw = 0;
    Icm.gyro_y_raw = 0;
    Icm.gyro_z_raw = 0;
    
    // 物理量清零
    Icm.accel_x_g = 0.0f;
    Icm.accel_y_g = 0.0f;
    Icm.accel_z_g = 0.0f;
    Icm.gyro_x_dps = 0.0f;
    Icm.gyro_y_dps = 0.0f;
    Icm.gyro_z_dps = 0.0f;
}

/**
 * @brief 实时状态初始化
 * @note  每次开机时调用，将所有状态清零
 */
void State_Init(void) {
    State.pitch = 0.0f;
    State.roll = 0.0f;
    State.yaw = 0.0f;
    State.gyro_x = 0.0f;
    State.gyro_y = 0.0f;
    State.encoder_left = 0;
    State.encoder_right = 0;
    State.is_stop = 1;      // 开机默认锁定电机，保护安全
    State.run_stage = 0;
    State.loop_count = 0;
    State.menu_index = 0;
}

/**
 * @brief 参数默认初始化
 * @note  当检测到Flash无数据时，加载这些默认值
 */
void Param_Init(void) {
    // 默认模式
    Config.boot.boot_mode = MODE_1;
    Config.boot.boot_flag = 0;

    // 姿态环默认值 (根据实际调试修改)
    Config.angle.angle_kp = 10.0f;
    Config.angle.angle_kd = 0.5f;
    Config.angle.mech_angle = 0.0f;

    // 运动参数
    Config.path.move_speed = 50;
    Config.path.target_dist_AB = 1000.0f;

    // 硬件配置
    Config.hw.battery_min = 10.5f;
    Config.hw.motor_deadzone = 50;
    
    Config.checksum = 0x12345678; // 用于标记Flash已被初始化
}

/**
 * @brief 将当前Config结构体所有内容存入Flash
 */
void Param_SaveToFlash(void) {
    // 伪代码：调用你库中的Flash擦除和写入函数
    // flash_erase_sector(PARAM_FLASH_SECTOR);
    // flash_write(PARAM_FLASH_ADDR, (uint32 *)&Config, sizeof(Config)/4);
}

/**
 * @brief 从Flash读取参数
 */
void Param_ReadFromFlash(void) {
    // 如果读取后的checksum不对，说明是新片子，需要初始化默认值
    if(Config.checksum != 0x12345678) {
        Param_Init();
        Param_SaveToFlash();
    }
}