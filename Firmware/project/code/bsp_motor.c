#include "zf_common_headfile.h"
#include "bsp_motor.h"
#include "Control.h"

void motor_init(void)
{
    // 初始化左电机方向引脚
    gpio_init(Motor_L_DIR1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(Motor_L_DIR2, GPO, GPIO_LOW, GPO_PUSH_PULL);

    // 初始化右电机方向引脚
    gpio_init(Motor_R_DIR1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(Motor_R_DIR2, GPO, GPIO_LOW, GPO_PUSH_PULL);

    // 初始化PWM
    pwm_init(PWM_CH1, MOTOR_PWM_FREQ, 0);
    pwm_init(PWM_CH2, MOTOR_PWM_FREQ, 0);
}

void motor_set_left_speed(int16 speed)
{
    State.motor_target_speed_left = speed;
}

void motor_set_right_speed(int16 speed)
{
    State.motor_target_speed_right = speed;
}

void motor_stop(void)
{
    State.motor_target_speed_left = 0;
    State.motor_target_speed_right = 0;
}

void motor_test_50_50(void)
{
    State.motor_target_speed_left = 5000;
    State.motor_target_speed_right = 5000;
}

void motor_test_neg50_neg50(void)
{
    State.motor_target_speed_left = -5000;
    State.motor_target_speed_right = -5000;
}

void motor_test_100_100(void)
{
    State.motor_target_speed_left = 10000;
    State.motor_target_speed_right = 10000;
}

void motor_test_neg100_neg100(void)
{
    State.motor_target_speed_left = -10000;
    State.motor_target_speed_right = -10000;
}

void motor_test_deadzone(void)
{
    State.motor_target_speed_left = 1000;
    State.motor_target_speed_right = 1000;
}

void motor_set_speed(int16_t left_speed, int16_t right_speed)
{
    State.motor_target_speed_left = left_speed;
    State.motor_target_speed_right = right_speed;
}

/**
 * @brief 电机控制更新（闭环速度控制）
 * @note  5ms调用一次
 */
void motor_update(void)
{
    // 1. 计算左电机 PID 输出
    // error = 目标速度 (m/s) - 实际速度 (m/s)
    float error_l = State.motor_target_speed_left - State.motor_actual_speed_left;
    float pwm_out_l = PID_Compute(&Config.motor_l, error_l);

    // 2. 计算右电机 PID 输出
    float error_r = State.motor_target_speed_right - State.motor_actual_speed_right;
    float pwm_out_r = PID_Compute(&Config.motor_r, error_r);

    // 3. 执行左电机硬件输出
    if (pwm_out_l > 0)
    {
        gpio_set_level(Motor_L_DIR1, 0);
        gpio_set_level(Motor_L_DIR2, 1);
        pwm_set_duty(PWM_CH1, (uint32)pwm_out_l);
    }
    else
    {
        gpio_set_level(Motor_L_DIR1, 1);
        gpio_set_level(Motor_L_DIR2, 0);
        pwm_set_duty(PWM_CH1, (uint32)-pwm_out_l);
    }

    // 4. 执行右电机硬件输出
    if (pwm_out_r > 0)
    {
        gpio_set_level(Motor_R_DIR1, 1);
        gpio_set_level(Motor_R_DIR2, 0);
        pwm_set_duty(PWM_CH2, (uint32)pwm_out_r);
    }
    else
    {
        gpio_set_level(Motor_R_DIR1, 0);
        gpio_set_level(Motor_R_DIR2, 1);
        pwm_set_duty(PWM_CH2, (uint32)-pwm_out_r);
    }
}