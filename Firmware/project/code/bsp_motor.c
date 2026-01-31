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
    State.motor_target_speed_left = MOTOR_MAX_SPEED_MPS / 2;
    State.motor_target_speed_right = MOTOR_MAX_SPEED_MPS / 2;
}

void motor_test_neg50_neg50(void)
{
    State.motor_target_speed_left = -MOTOR_MAX_SPEED_MPS / 2;
    State.motor_target_speed_right = -MOTOR_MAX_SPEED_MPS / 2;
}

void motor_test_100_100(void)
{
    State.motor_target_speed_left = MOTOR_MAX_SPEED_MPS;
    State.motor_target_speed_right = MOTOR_MAX_SPEED_MPS;
}

void motor_test_neg100_neg100(void)
{
    State.motor_target_speed_left = -MOTOR_MAX_SPEED_MPS;
    State.motor_target_speed_right = -MOTOR_MAX_SPEED_MPS;
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

float Motor_PID_Update(
    MotorPID_t *pid,
    float target_speed, // m/s
    float actual_speed  // m/s
)
{
    float error = target_speed - actual_speed;
    float p_out, i_out = 0.0f;
    float output;

    /* ---------- P 控制 ---------- */
    p_out = pid->Kp * error;

    /* ---------- 低速才启用积分 ---------- */
    if (fabsf(target_speed) < LOW_SPEED_THRESHOLD)
    {
        /* 抗饱和积分 */
        pid->integral += error;

        if (pid->integral > pid->i_max)
            pid->integral = pid->i_max;
        else if (pid->integral < -pid->i_max)
            pid->integral = -pid->i_max;

        i_out = pid->Ki * pid->integral;
    }
    else
    {
        /* 高速段：清积分，防止拖慢 */
        pid->integral = 0.0f;
        i_out = 0.0f;
    }

    /* ---------- 合成 ---------- */
    output = p_out + i_out;

    /* ---------- 输出限幅 ---------- */
    if (output > pid->out_max)
        output = pid->out_max;
    else if (output < -pid->out_max)
        output = -pid->out_max;

    return output;
}

/**
 * @brief 电机控制更新（闭环速度控制）
 * @note  5ms调用一次
 */
void motor_update(void)
{
    float pwm_l = 0.0f;
    float pwm_r = 0.0f;

    if (State.is_stop)
    {
        pwm_set_duty(PWM_CH1, 0);
        pwm_set_duty(PWM_CH2, 0);
        return;
    }

    /**************** 左电机 ****************/
    {
        float tar = State.motor_target_speed_left;
        float act = State.motor_actual_speed_left;

        /* ===== 1. 零速阻尼模式（防起飞） ===== */
        if (fabsf(tar) < SPEED_ZERO_TH)
        {
            pwm_l = -MOTOR_DAMP_K * act;

            if (pwm_l > Config.motor_l.out_max)
                pwm_l = Config.motor_l.out_max;
            if (pwm_l < -Config.motor_l.out_max)
                pwm_l = -Config.motor_l.out_max;

            // 清积分，防止切回时爆
            Config.motor_l.integral = 0.0f;
            Config.motor_l.err_last = 0.0f;
        }
        else
        {
            /* ===== 2. PI 控制（抗积分饱和） ===== */
            float error = tar - act;

            /* --- 低速 Kp 加权 --- */
            float kp_eff = Config.motor_l.Kp;
            if (fabsf(act) < SPEED_LOW_TH)
            {
                kp_eff *= KP_LOW_SPEED_GAIN;
            }

            /* --- P 项 --- */
            float p_out = kp_eff * error;

            /* --- 先假算一次（用于判断是否饱和）--- */
            float i_out = Config.motor_l.Ki * Config.motor_l.integral;
            float pwm_tmp = p_out + i_out;

            /* --- 抗积分饱和核心 --- */
            if (fabsf(pwm_tmp) < Config.motor_l.out_max)
            {
                Config.motor_l.integral += error;

                if (Config.motor_l.integral > Config.motor_l.i_max)
                    Config.motor_l.integral = Config.motor_l.i_max;
                if (Config.motor_l.integral < -Config.motor_l.i_max)
                    Config.motor_l.integral = -Config.motor_l.i_max;
            }

            /* --- 最终输出 --- */
            i_out = Config.motor_l.Ki * Config.motor_l.integral;
            pwm_l = p_out + i_out;

            if (pwm_l > Config.motor_l.out_max)
                pwm_l = Config.motor_l.out_max;
            if (pwm_l < -Config.motor_l.out_max)
                pwm_l = -Config.motor_l.out_max;
        }

        /* --- 硬件输出 --- */
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
            pwm_set_duty(PWM_CH1, (uint32)(-pwm_l));
        }
    }

    /**************** 右电机（完全对称） ****************/
    {
        float tar = State.motor_target_speed_right;
        float act = State.motor_actual_speed_right;

        if (fabsf(tar) < SPEED_ZERO_TH)
        {
            pwm_r = -MOTOR_DAMP_K * act;

            if (pwm_r > Config.motor_r.out_max)
                pwm_r = Config.motor_r.out_max;
            if (pwm_r < -Config.motor_r.out_max)
                pwm_r = -Config.motor_r.out_max;

            Config.motor_r.integral = 0.0f;
            Config.motor_r.err_last = 0.0f;
        }
        else
        {
            float error = tar - act;

            float kp_eff = Config.motor_r.Kp;
            if (fabsf(act) < SPEED_LOW_TH)
            {
                kp_eff *= KP_LOW_SPEED_GAIN;
            }

            float p_out = kp_eff * error;

            float i_out = Config.motor_r.Ki * Config.motor_r.integral;
            float pwm_tmp = p_out + i_out;

            if (fabsf(pwm_tmp) < Config.motor_r.out_max)
            {
                Config.motor_r.integral += error;

                if (Config.motor_r.integral > Config.motor_r.i_max)
                    Config.motor_r.integral = Config.motor_r.i_max;
                if (Config.motor_r.integral < -Config.motor_r.i_max)
                    Config.motor_r.integral = -Config.motor_r.i_max;
            }

            i_out = Config.motor_r.Ki * Config.motor_r.integral;
            pwm_r = p_out + i_out;

            if (pwm_r > Config.motor_r.out_max)
                pwm_r = Config.motor_r.out_max;
            if (pwm_r < -Config.motor_r.out_max)
                pwm_r = -Config.motor_r.out_max;
        }

        if (pwm_r >= 0)
        {
            gpio_set_level(Motor_R_DIR1, 1);
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
}
