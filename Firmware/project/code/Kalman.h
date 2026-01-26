#ifndef KALMAN_H
#define KALMAN_H

#define dt (0.01f) // 采样周期 (秒)
typedef struct {
    float Q_angle;   // 过程噪声协方差 (角度)
    float Q_bias;    // 过程噪声协方差 (偏差)
    float R_measure; // 测量噪声协方差
    float angle;     // 计算出的角度 (这是你想要的结果)
    float bias;      // 计算出的陀螺仪零漂
    float P[2][2];   // 误差协方差矩阵
} Kalman_t;

void Kalman_Init(Kalman_t *kalman);
float Kalman_GetAngle(Kalman_t *kalman, float newAngle, float newRate);
void Attitude_Init(void);
void Attitude_Update(void);

#endif