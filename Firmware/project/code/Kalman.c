//
// Created by 0kouhj on 2025/12/12.
//
#include "Kalman.h"

/**
 * @brief 初始化卡尔曼滤波器参数
 */
void Kalman_Init(Kalman_t *kalman) {
    kalman->Q_angle = 0.01f;   // 角度数据置信度 (增加以减少响应过大)
    kalman->Q_bias = 0.003f;    // 偏差数据置信度
    kalman->R_measure = 0.01f;  // 测量值置信度 (减少以更相信加速度计)

    kalman->angle = 0.0f;
    kalman->bias = 0.0f;

    kalman->P[0][0] = 0.0f;
    kalman->P[0][1] = 0.0f;
    kalman->P[1][0] = 0.0f;
    kalman->P[1][1] = 0.0f;
}

/**
 * @brief 卡尔曼滤波计算
 * @param newAngle 加速度计计算出的角度 (测量值)
 * @param newRate  陀螺仪的角速度
 * @param dt       采样周期 (秒)
 * @return 融合后的角度
 */
float Kalman_GetAngle(Kalman_t *kalman, float newAngle, float newRate, float dt) {
    // 1. 预测更新 (Prediction)
    // 根据上一时刻的角度和角速度预测当前角度
    float rate = newRate - kalman->bias;
    kalman->angle += dt * rate;

    // 更新误差协方差矩阵 P
    kalman->P[0][0] += dt * (dt * kalman->Q_bias - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += kalman->Q_bias * dt;

    // 2. 测量更新 (Update)
    // 计算卡尔曼增益 K
    float S = kalman->P[0][0] + kalman->R_measure;
    float K[2];
    K[0] = kalman->P[0][0] / S;
    K[1] = kalman->P[1][0] / S;

    // 计算角度偏差 (Innovation)
    float y = newAngle - kalman->angle;

    // 更新最优估计值
    kalman->angle += K[0] * y;
    kalman->bias += K[1] * y;

    // 更新误差协方差矩阵 P
    float P00_temp = kalman->P[0][0];
    float P01_temp = kalman->P[0][1];

    kalman->P[0][0] -= K[0] * P00_temp;
    kalman->P[0][1] -= K[0] * P01_temp;
    kalman->P[1][0] -= K[1] * P00_temp;
    kalman->P[1][1] -= K[1] * P01_temp;

    return kalman->angle;
}

// 姿态解算相关
#include "param_config.h"
#include <math.h>

static Kalman_t kalman_pitch;
static Kalman_t kalman_roll;
static Kalman_t kalman_yaw; // 可选，用于yaw

/**
 * @brief 初始化姿态解算
 */
void Attitude_Init(void) {
    Kalman_Init(&kalman_pitch);
    Kalman_Init(&kalman_roll);
    Kalman_Init(&kalman_yaw);
}

/**
 * @brief 更新姿态解算，使用ICM42688数据
 * @param dt 时间间隔 (秒)
 */
void Attitude_Update(float dt) {
    // 从Icm读取数据
    float ax = Icm.accel_x_g;
    float ay = Icm.accel_y_g;
    float az = Icm.accel_z_g;
    float gx = Icm.gyro_x_dps * PI / 180.0f; // 转换为rad/s
    float gy = Icm.gyro_y_dps * PI / 180.0f;
    float gz = Icm.gyro_z_dps * PI / 180.0f;

    // 计算加速度计角度 (测量值)
    float accel_pitch = atan2f(ay, az) * 180.0f / PI;
    float accel_roll = atan2f(ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;

    // 使用卡尔曼滤波
    State.pitch = Kalman_GetAngle(&kalman_pitch, accel_pitch, gx, dt);
    State.roll = Kalman_GetAngle(&kalman_roll, accel_roll, gy, dt);

    // Yaw 使用陀螺仪积分 (无磁力计)
    State.yaw += gz * dt * 180.0f / PI * 4.5; // 转换为度

    // Gyro 赋值
    State.gyro_x = Icm.gyro_x_dps;
    State.gyro_y = Icm.gyro_y_dps;
}