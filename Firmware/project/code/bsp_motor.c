#include "zf_common_headfile.h"
#include "bsp_motor.h"
#include "pin_config.h"

// 电机PWM频率
#define MOTOR_PWM_FREQ 17000

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

void motor_set_speed(int16_t left_speed, int16_t right_speed)
{
    State.motor_target_speed_left = left_speed;
    State.motor_target_speed_right = right_speed;
}

void motor_update(void)
{
    // 左电机
    if (State.motor_target_speed_left > 0)
    {
        // 前进
        gpio_set_level(Motor_L_DIR1, 0);
        gpio_set_level(Motor_L_DIR2, 1);
        pwm_set_duty(PWM_CH1, State.motor_target_speed_left);
    }
    else if (State.motor_target_speed_left < 0)
    {
        // 后退
        gpio_set_level(Motor_L_DIR1, 1);
        gpio_set_level(Motor_L_DIR2, 0);
        pwm_set_duty(PWM_CH1, -State.motor_target_speed_left);
    }
    else
    {
        // 停止
        pwm_set_duty(PWM_CH1, 0);
    }

    // 右电机
    if (State.motor_target_speed_right > 0)
    {
        // 前进
        gpio_set_level(Motor_R_DIR1, 0);
        gpio_set_level(Motor_R_DIR2, 1);
        pwm_set_duty(PWM_CH2, State.motor_target_speed_right);
    }
    else if (State.motor_target_speed_right < 0)
    {
        // 后退
        gpio_set_level(Motor_R_DIR1, 1);
        gpio_set_level(Motor_R_DIR2, 0);
        pwm_set_duty(PWM_CH2, -State.motor_target_speed_right);
    }
    else
    {
        // 停止
        pwm_set_duty(PWM_CH2, 0);
    }
}