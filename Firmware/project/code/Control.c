#include "Control.h"
#include "zf_common_headfile.h"
#include "bsp_imu.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"

/**
 * @brief 简化后的速度反馈（直接返回每毫秒脉冲数）
 */
float Motion_Get_Speed_L(int32_t pulse_count)
{
    static float filtered_pulses_l = 0.0f; // 必须使用独立的静态变量
    const float alpha = 0.3f;              // 1ms下兼顾响应与平滑

    // 根据实测值 200，设置 250 为物理极限限幅，超过即判定为干扰
    if (pulse_count > 210)
        pulse_count = 210;
    if (pulse_count < -210)
        pulse_count = -210;

    filtered_pulses_l = alpha * (float)pulse_count + (1.0f - alpha) * filtered_pulses_l;
    return filtered_pulses_l;
}

float Motion_Get_Speed_R(int32_t pulse_count)
{
    static float filtered_pulses_r = 0.0f; // 必须使用独立的静态变量
    const float alpha = 0.3f;

    if (pulse_count > 210)
        pulse_count = 210;
    if (pulse_count < -210)
        pulse_count = -210;

    filtered_pulses_r = alpha * (float)pulse_count + (1.0f - alpha) * filtered_pulses_r;
    return filtered_pulses_r;
}

/**
 * @brief 通用 PID 计算函数
 * @param pid  PID 结构体指针
 * @param error 当前误差 (目标值 - 实际值)
 * @return float 计算得出的 PWM 或 修正量
 */
float PID_Compute(STRUCT_PID *pid, float error)
{
    float p_out = pid->Kp * error;

    pid->integral += error;
    if (pid->integral > pid->i_max)
        pid->integral = pid->i_max;
    if (pid->integral < -pid->i_max)
        pid->integral = -pid->i_max;

    float i_out = pid->Ki * pid->integral;

    float d_out = pid->Kd * (error - pid->err_last);
    pid->err_last = error;

    float out = p_out + i_out + d_out;

    if (out > pid->out_max)
        out = pid->out_max;
    if (out < -pid->out_max)
        out = -pid->out_max;

    return out;
}

float PID_Simple(STRUCT_PID *pid, float target, float actual)
{
    float error = target - actual;

    // P
    float p_out = pid->Kp * error;

    // I
    pid->integral += error;
    if (pid->integral > pid->i_max)
        pid->integral = pid->i_max;
    if (pid->integral < -pid->i_max)
        pid->integral = -pid->i_max;

    float i_out = pid->Ki * pid->integral;

    // 输出
    float out = p_out + i_out;

    if (out > pid->out_max)
        out = pid->out_max;
    if (out < -pid->out_max)
        out = -pid->out_max;

    return out;
}

/**
 * @brief 平衡车核心控制循环 (5ms)
 * @note  所有输入数据（角度、角速度、速度）均由外部函数自动更新至 State 和 Icm 结构体
 */
void Balance_Control_Loop(void)
{
    static uint8 velocity_cnt = 0;
    static float speed_angle_offset = 0; // 速度环输出：期望角度补偿量

    // --- 1. 数据准备 (从 State 结构体直接读取) ---
    float current_pitch = State.pitch;     // 姿态角度
    float current_gyro_x = Icm.gyro_x_dps; // 绕X轴角速度

    // 使用 State 中存储的电机实际速度反馈计算平均速度
    float current_velocity = (State.motor_actual_speed_left + State.motor_actual_speed_right) / 2.0f;

    // --- 2. 速度环 (外环 - 每 20ms 执行一次) ---
    velocity_cnt++;
    if (velocity_cnt >= 10)
    {
        velocity_cnt = 0;

        // 目标速度为 0 (原地平衡)
        float speed_error = 0.0f - current_velocity;

        // 使用 Config 结构体中的 Speed_PID 进行计算
        speed_angle_offset = 0; // PID_Compute(&Config.speed, speed_error);
    }

    // --- 3. 直立环 (内环 - 每 5ms 执行一次) ---
    // 目标角度 = 机械中值 + 速度环输出的角度修正
    float target_pitch = Config.angle.mech_angle + speed_angle_offset;
    float pitch_error = target_pitch - current_pitch;

    // 计算直立控制量
    // 使用 Config 结构体中的 angle_PID
    // D项直接使用硬件滤波后的 gyro_x
    float balance_control_out = Config.angle.angle_kp * pitch_error + Config.angle.angle_kd * current_gyro_x;

    // --- 4. 转向环 (独立运行) ---
    // 维持左右轮差速为 0
    float turn_error = State.motor_actual_speed_left - State.motor_actual_speed_right;

    // 使用 Config 结构体中的 Turn_PID
    float turn_control_out = 0; // PID_Compute(&Config.yaw, turn_error);

    // --- 5. 更新电机目标速度 ---
    // 将计算结果写入 State 结构体中的目标速度变量
    State.motor_target_speed_left = balance_control_out + turn_control_out;
    State.motor_target_speed_right = balance_control_out - turn_control_out;

    // --- 6. 倾倒安全保护 ---
    if (current_pitch > 45.0f || current_pitch < -45.0f)
    {
        State.motor_target_speed_left = 0;
        State.motor_target_speed_right = 0;
        State.is_stop = 1; // 停机保护
        pwm_set_duty(PWM_CH1, 0);
        pwm_set_duty(PWM_CH2, 0);
        gpio_set_level(Motor_L_DIR1, 0);
        gpio_set_level(Motor_L_DIR2, 1);
        gpio_set_level(Motor_R_DIR1, 0);
        gpio_set_level(Motor_R_DIR2, 1);
    }
}

void All_Update(void)
{
    static uint8_t ms2_cnt = 0;
    static uint8_t debug_cnt = 0;
    ms2_cnt++;
    if (ms2_cnt >= 2)
    {
        ms2_cnt = 0;
        Balance_Control_Loop();
        Attitude_Update();
    }
    ICM_Update();
    if (debug_cnt++ >= 10)
    {
        debug_cnt = 0;
        #ifndef DEBUG
        #else
                char str[128];
                // sprintf(str, "%f,%f,%f\n", State.pitch, State.roll, State.yaw);
                sprintf(str, "%f,%f,%f,%f\n", State.motor_actual_speed_left, State.motor_actual_speed_right, State.motor_target_speed_left, State.motor_target_speed_right);
                uart_write_string(DEBUG_UART_INDEX, str);
        #endif
    }
}