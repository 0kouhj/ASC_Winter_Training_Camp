#ifndef CONTROL_H
#define CONTROL_H

#include "zf_common_headfile.h"

// --- 轮式机器人参数 ---
#define WHEEL_DIAMETER 0.068f
#define WHEEL_PERIMETER (PI * WHEEL_DIAMETER)

// --- 编码器与减速比参数 ---
#define ENCODER_RESOLUTION 500.0f // 请根据实际线数修改
#define ENCODER_MULTI 4.0f
#define GEAR_RATIO 30.0f
#define SAMPLE_TIME_S 0.005f // 采样周期，如果是10ms请改为0.01f

// --- 电机硬件参数 ---
#define MOTOR_MAX_PWM 10000.0f
#define MOTOR_MIN_PWM 1000.0f //死区pwm值，根据实际电机调试修改
#define MOTOR_MAX_SPEED_MPS 2.5f // 对应原本的 MOTOR_MAX_SPEED

// --- 计算系数 ---
// 脉冲转速度系数
#define PULSE_TO_MPS_FACTOR (WHEEL_PERIMETER / (ENCODER_RESOLUTION * ENCODER_MULTI * GEAR_RATIO * SAMPLE_TIME_S))
// 速度转PWM系数
#define MPS_TO_PWM_FACTOR (MOTOR_MAX_PWM / MOTOR_MAX_SPEED_MPS)

// 函数声明
float Motion_Get_Speed(int32_t pulse_count);
int16_t Motion_Speed_To_PWM(float target_speed);

#endif