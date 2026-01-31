#include "param_config.h"
#include "Control.h"
// 实例化全局变量
SYSTEM_CONFIG_T Config;
SYSTEM_STATE_T State;
ICM42688_t Icm;
/**
 * @brief 初始化ICM42688的值
 * @note  开机时调用
 */
static void Icm42688_Init(void)
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
static void State_Init(void)
{
    State.pitch = 0.0f;
    State.roll = 0.0f;
    State.yaw = 0.0f;
    State.gyro_x = 0.0f;
    State.gyro_y = 0.0f;
    State.encoder_left = 0;
    State.encoder_right = 0;
    State.motor_target_speed_left = 0;
    State.motor_target_speed_right = 0;
    State.motor_actual_speed_left = 0;
    State.motor_actual_speed_right = 0;
    State.is_stop = 0; // 开机默认锁定电机，保护安全
    State.run_stage = 0;
    State.loop_count = 0;
    State.menu_index = 0;
    State.battery_v = 0.0f;
    State.system_tick = 0;
}

/**
 * @brief 参数默认初始化
 * @note  当检测到Flash无数据时，加载这些默认值
 */
void Param_Init(void)
{
    State_Init();
    Icm42688_Init();

    // 1. 默认模式
    Config.boot.boot_mode = MODE_1;

    // 2. 姿态环 (直立环)
    Config.angle.angle_kp = 0.0f; // 需调试，单位: m/s per degree
    Config.angle.angle_kd = 0.0f;
    Config.angle.mech_angle = 0.0f; // 机械中值

    // 3. 速度环限幅修改 (外环输出是角度修正量)
    Config.speed.Kp = 0.0f;
    Config.speed.Ki = 0.0f;
    Config.speed.Kd = 0.0f;
    Config.speed.integral = 0.0f;
    Config.speed.i_max = 5.0f; // 积分限幅，防止大幅度漂移累积
    Config.speed.err_last = 0.0f;
    Config.speed.out_max = 10.0f; // 速度环最大允许倾斜角度补偿为 10度

    // 4. 电机环默认值 (内环：m/s 转换为 PWM 0-10000)
    Config.motor_l.Kp = 50000.0f;
    Config.motor_l.Ki = 4000.0f; // 适量积分消除稳态误差
    Config.motor_l.Kd = 10000.0f;
    Config.motor_l.integral = 0.0f;
    Config.motor_l.i_max = 2000.0f; // 积分限幅，设为输出上限的一半
    Config.motor_l.err_last = 3000.0f;
    Config.motor_l.out_max = MOTOR_MAX_PWM; // 必须设为 10000.0f

    Config.motor_r.Kp = 15000.0f;
    Config.motor_r.Ki = 800.0f;
    Config.motor_r.Kd = 0.0f;
    Config.motor_r.integral = 0.0f;
    Config.motor_r.i_max = 2000.0f;
    Config.motor_r.err_last = 3000.0f;
    Config.motor_r.out_max = MOTOR_MAX_PWM; // 必须设为 10000.0f

    // 5. 位置环默认值 (可选)
    Config.position.Kp = 0.0f;
    Config.position.Ki = 0.0f;
    Config.position.Kd = 0.0f;
    Config.position.integral = 0.0f;
    Config.position.i_max = 1.0f;
    Config.position.err_last = 0.0f;
    Config.position.out_max = 0.5f; // 输出限幅通常设得很小，用于缓慢位置修正

    // 6. 转向环限幅修改
    Config.yaw.Kp = 0.0f;
    Config.yaw.Ki = 0.0f;
    Config.yaw.Kd = 0.0f;
    Config.yaw.integral = 0.0f;
    Config.yaw.i_max = 0.5f; // 转向积分限幅 (m/s)
    Config.yaw.err_last = 0.0f;
    Config.yaw.out_max = 1.0f; // 转向输出最大差速限制在 1.0m/s

    // 7. 运动参数
    Config.path.move_speed = 50;
    Config.path.target_dist_AB = 1000.0f;

    Config.checksum = 0x12345678;
}