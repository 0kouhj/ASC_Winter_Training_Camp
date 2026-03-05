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

    // 2. 电机环默认值
    Config.motor_l.Kp = 80.0f;
    Config.motor_l.Ki = 2.0f; // 适量积分消除稳态误差
    Config.motor_l.Kd = 0.5f;
    Config.motor_l.integral = 0.0f;
    Config.motor_l.i_max = 8000.0f; // 积分限幅，设为输出上限的一半
    Config.motor_l.err_last = 0.0f;
    Config.motor_l.out_max = MOTOR_MAX_PWM; // 必须设为 10000.0f

    Config.motor_r.Kp = 80.0f;
    Config.motor_r.Ki = 2.0f;
    Config.motor_r.Kd = 0.5f;
    Config.motor_r.integral = 0.0f;
    Config.motor_r.i_max = 8000.0f;
    Config.motor_r.err_last = 0.0f;
    Config.motor_r.out_max = MOTOR_MAX_PWM; // 必须设为 10000.0f

    // 3. 角速度环 (Gyro Loop: 平衡控制内环，抑制抖动)
    Config.gyro_loop.Kp = -1.05f; // 初始设为0，调参建议范围
    Config.gyro_loop.Ki = 0.0f; // 角速度环通常不需要积分
    Config.gyro_loop.Kd = 0.0f;
    Config.gyro_loop.integral = 0.0f;
    Config.gyro_loop.i_max = 9000.0f;
    Config.gyro_loop.err_last = 0.0f;
    Config.gyro_loop.out_max = MOTOR_MAX_PWM;

    // 4. 角度环 (Angle Loop: 平衡控制中环，维持直立)
    Config.angle_loop.Kp = 4.4f; // 初始设为0，调参建议范围: 200 - 600
    Config.angle_loop.Ki = 0.0f; // 角度环不建议使用积分，易导致低频震荡
    Config.angle_loop.Kd = 0.0f; // 角度环的D项通常作用于角速度
    Config.angle_loop.integral = 0.0f;
    Config.angle_loop.i_max = 9000.0f;
    Config.angle_loop.err_last = 0.0f;
    Config.angle_loop.out_max = 99000.0f; // 限制角度环输出的幅度

    // 5. 速度环 (Speed Loop: 平衡控制外环，维持静止/位移)
    Config.speed_loop.Kp = 0.0f; // 初始设为0，调参建议范围: 50 - 200
    Config.speed_loop.Ki = 0.0f; // 速度环必须有积分，用于消除静差（如坡道停车）
    Config.speed_loop.Kd = 0.0f;
    Config.speed_loop.integral = 0.0f;
    Config.speed_loop.i_max = 3000.0f;
    Config.speed_loop.err_last = 0.0f;
    Config.speed_loop.out_max = 20.0f; // 速度环输出的是目标角度，不宜过大

    // 6. 转向环 (Yaw Loop)
    Config.yaw_loop.Kp = 0.0f;
    Config.yaw_loop.Ki = 0.0f;
    Config.yaw_loop.Kd = 0.0f;
    Config.yaw_loop.integral = 0.0f;
    Config.yaw_loop.i_max = 1000.0f;
    Config.yaw_loop.err_last = 0.0f;
    Config.yaw_loop.out_max = MOTOR_MAX_PWM;

    // 7. 运动参数
    Config.path.move_speed = 50;
    Config.path.target_dist_AB = 1000.0f;

    Config.checksum = 0x12345678;
}

volatile uint8_t sys_ready;
volatile uint16_t fuck_imu;