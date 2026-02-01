#include "zf_common_headfile.h"
#include "bsp_motor.h"
#include "Control.h"
#include "math.h"
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
    Config.motor_l.integral = 0;
    Config.motor_r.integral = 0;
    pwm_set_duty(PWM_CH1, 0);
    pwm_set_duty(PWM_CH2, 0);
}

void motor_test_50_50(void)
{
    State.motor_target_speed_left = MOTOR_MAX_SPEED / 2;
    State.motor_target_speed_right = MOTOR_MAX_SPEED / 2;
}

void motor_test_neg50_neg50(void)
{
    State.motor_target_speed_left = -MOTOR_MAX_SPEED / 2;
    State.motor_target_speed_right = -MOTOR_MAX_SPEED / 2;
}

void motor_test_100_100(void)
{
    State.motor_target_speed_left = MOTOR_MAX_SPEED;
    State.motor_target_speed_right = MOTOR_MAX_SPEED;
}

void motor_test_neg100_neg100(void)
{
    State.motor_target_speed_left = -MOTOR_MAX_SPEED;
    State.motor_target_speed_right = -MOTOR_MAX_SPEED;
}

void motor_test_deadzone(void)
{
    State.motor_target_speed_left = 20;
    State.motor_target_speed_right = 20;
}

void motor_set_speed(int16_t left_speed, int16_t right_speed)
{
    State.motor_target_speed_left = left_speed;
    State.motor_target_speed_right = right_speed;
}

/**
 * @brief 电机控制更新（闭环速度控制）
 * @note  当前设定为 1ms 调用一次
 */
void motor_update(void)
{
    // 如果处于停机状态，直接切断动力
    if (State.is_stop)
    {
        motor_stop();
        return;
    }

    if (State.is_stop || (State.motor_target_speed_left == 0 && fabs(State.motor_actual_speed_left) < 1.0f))
    {
        motor_stop();
        return;
    }

    if (State.is_stop || (State.motor_target_speed_right == 0 && fabs(State.motor_actual_speed_right) < 1.0f))
    {
        motor_stop();
        return;
    }

    /* ================= 左电机控制 ================= */
    // 1. 调用 PID_Simple 计算基础输出
    float pwm_l = PID_Simple(&Config.motor_l, State.motor_target_speed_left, State.motor_actual_speed_left);

    // 2. 添加死区补偿 (MOTOR_MIN_PWM = 1000)
    // 只要 PID 觉得该动，我们就给它起步的保底力
    if (pwm_l > 0.1f)
        pwm_l += MOTOR_MIN_PWM;
    else if (pwm_l < -0.1f)
        pwm_l -= MOTOR_MIN_PWM;

    // 3. 检查是否超过最大限制 (MOTOR_MAX_PWM = 10000)
    // 必须在输出前做这步，否则底层驱动会因为 duty > 10000 报 Assert Error
    if (pwm_l > MOTOR_MAX_PWM)
        pwm_l = MOTOR_MAX_PWM;
    if (pwm_l < -MOTOR_MAX_PWM)
        pwm_l = -MOTOR_MAX_PWM;

    // 4. 驱动硬件
    if (pwm_l >= 0)
    {
        gpio_set_level(Motor_L_DIR1, 0);
        gpio_set_level(Motor_L_DIR2, 1);
        pwm_set_duty(PWM_CH1, (uint32)pwm_l);
    }
    else
    {
        gpio_set_level(Motor_L_DIR1, 1);
        gpio_set_level(Motor_L_DIR2, 0);
        pwm_set_duty(PWM_CH1, (uint32)(-pwm_l)); // 取绝对值输出
    }

    /* ================= 右电机控制 (完全对称) ================= */
    float pwm_r = PID_Simple(&Config.motor_r, State.motor_target_speed_right, State.motor_actual_speed_right);

    if (pwm_r > 0.1f)
        pwm_r += MOTOR_MIN_PWM;
    else if (pwm_r < -0.1f)
        pwm_r -= MOTOR_MIN_PWM;

    if (pwm_r > MOTOR_MAX_PWM)
        pwm_r = MOTOR_MAX_PWM;
    if (pwm_r < -MOTOR_MAX_PWM)
        pwm_r = -MOTOR_MAX_PWM;

    if (pwm_r >= 0)
    {
        gpio_set_level(Motor_R_DIR1, 1); // 注意：右电机方向通常与左边相反
        gpio_set_level(Motor_R_DIR2, 0);
        pwm_set_duty(PWM_CH2, (uint32)pwm_r);
    }
    else
    {
        gpio_set_level(Motor_R_DIR1, 0);
        gpio_set_level(Motor_R_DIR2, 1);
        pwm_set_duty(PWM_CH2, (uint32)(-pwm_r));
    }
}