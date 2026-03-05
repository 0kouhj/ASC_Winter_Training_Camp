#include "zf_common_headfile.h"
#include "bsp_motor.h"
#include "Control.h"
#include "math.h"
void motor_init(void)
{
    // 初始化左电机方向引脚
    gpio_init(Motor_L_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);

    // 初始化右电机方向引脚
    gpio_init(Motor_R_DIR, GPO, GPIO_LOW, GPO_PUSH_PULL);

    gpio_set_level(Motor_L_DIR, 0);
    gpio_set_level(Motor_R_DIR, 0);

    // 初始化PWM
    pwm_init(PWM_L, MOTOR_PWM_FREQ, 0); //L
    pwm_init(PWM_R, MOTOR_PWM_FREQ, 0);
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
    pwm_set_duty(PWM_L, 0);
    pwm_set_duty(PWM_R, 0);
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
    State.motor_target_speed_left = -MOTOR_MAX_SPEED / 10;
    State.motor_target_speed_right = -MOTOR_MAX_SPEED / 10;
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
    if (fabs(State.pitch)>=45.0f)
    {
        motor_stop();
        State.is_stop = 1;
        return;
    }
    // 如果处于停机状态，直接切断动力
    if (State.is_stop)
    {
        motor_stop();
        State.is_stop = 1;
        return;
    }
    

    /* ================= 左电机控制 ================= */
    // 1. 调用 PID_Compute 计算基础输出
    float pwm_l = PID_Compute_Motor(&Config.motor_l, State.motor_target_speed_left, State.motor_actual_speed_left);

    // 2. 添加死区补偿 (MOTOR_MIN_PWM = 1000)
    // 只要 PID 觉得该动，我们就给它起步的保底力
    if (pwm_l > 0.1f)
        pwm_l += MOTOR_MIN_PWM_L;
    else if (pwm_l < -0.1f)
        pwm_l -= MOTOR_MIN_PWM_L;

    // 3. 检查是否超过最大限制 (MOTOR_MAX_PWM = 10000)
    // 必须在输出前做这步，否则底层驱动会因为 duty > 10000 报 Assert Error
    if (pwm_l > MOTOR_MAX_PWM)
        pwm_l = MOTOR_MAX_PWM;
    if (pwm_l < -MOTOR_MAX_PWM)
        pwm_l = -MOTOR_MAX_PWM;

    // 4. 驱动硬件
    if (pwm_l >= 0)
    {
        gpio_set_level(Motor_L_DIR, 1);
        pwm_set_duty(PWM_L, (uint32)pwm_l);
    }
    else
    {
        gpio_set_level(Motor_L_DIR, 0);
        pwm_set_duty(PWM_L, (uint32)(-pwm_l)); // 取绝对值输出
    }

    /* ================= 右电机控制 (完全对称) ================= */
    float pwm_r = PID_Compute_Motor(&Config.motor_r, State.motor_target_speed_right, State.motor_actual_speed_right);

    if (pwm_r > 0.1f)
        pwm_r += MOTOR_MIN_PWM_R;
    else if (pwm_r < -0.1f)
        pwm_r -= MOTOR_MIN_PWM_R;

    if (pwm_r > MOTOR_MAX_PWM)
        pwm_r = MOTOR_MAX_PWM;
    if (pwm_r < -MOTOR_MAX_PWM)
        pwm_r = -MOTOR_MAX_PWM;

    if (pwm_r >= 0)
    {
        gpio_set_level(Motor_R_DIR, 1);
        pwm_set_duty(PWM_R, (uint32)pwm_r);
    }
    else
    {
        gpio_set_level(Motor_R_DIR, 0);
        pwm_set_duty(PWM_R, (uint32)(-pwm_r));
    }
}