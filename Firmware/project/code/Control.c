#include "Control.h"

/**
 * @brief 计算物理速度 (m/s)
 * @param pulse_count 当前采样周期内的脉冲增量
 */
float Motion_Get_Speed(int32_t pulse_count)
{
    // 使用统一命名的宏 PULSE_TO_MPS_FACTOR
    return (float)pulse_count * PULSE_TO_MPS_FACTOR;
}

/**
 * @brief 将物理速度目标转换为电机 PWM 占空比
 * @param target_speed 目标速度 (m/s)
 */
int16_t Motion_Speed_To_PWM(float target_speed)
{
    float pwm_out;

    // 1. 限幅：防止输入速度超过物理极限 MOTOR_MAX_SPEED_MPS
    if (target_speed > MOTOR_MAX_SPEED_MPS)
        target_speed = MOTOR_MAX_SPEED_MPS;
    if (target_speed < -MOTOR_MAX_SPEED_MPS)
        target_speed = -MOTOR_MAX_SPEED_MPS;

    // 2. 线性映射计算
    pwm_out = target_speed * MPS_TO_PWM_FACTOR;

    // 3. 死区补偿
    if (pwm_out > 0.1f)
    {
        pwm_out += MOTOR_MIN_PWM;
    }
    else if (pwm_out < -0.1f)
    {
        pwm_out -= MOTOR_MIN_PWM;
    }

    // 4. 最终PWM限幅
    if (pwm_out > MOTOR_MAX_PWM)
        pwm_out = MOTOR_MAX_PWM;
    if (pwm_out < -MOTOR_MAX_PWM)
        pwm_out = -MOTOR_MAX_PWM;

    return (int16_t)pwm_out;
}