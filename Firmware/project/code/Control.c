#include "Control.h"
#include "zf_common_headfile.h"
#include "bsp_imu.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "math.h"

/**
 * @brief 简化后的速度反馈（直接返回每毫秒脉冲数）
 */
float Motion_Get_Speed_L(int32_t pulse_count)
{
    static float filtered_pulses_l = 0.0f; // 必须使用独立的静态变量
    const float alpha = 0.3f;              // 1ms下兼顾响应与平滑

    filtered_pulses_l = alpha * (float)pulse_count + (1.0f - alpha) * filtered_pulses_l;
    if (fabsf(filtered_pulses_l) < 1.0f)
        filtered_pulses_l = 0.0f;
    return -filtered_pulses_l;
}

float Motion_Get_Speed_R(int32_t pulse_count)
{
    static float filtered_pulses_r = 0.0f; // 必须使用独立的静态变量
    const float alpha = 0.3f;

    filtered_pulses_r = alpha * (float)pulse_count + (1.0f - alpha) * filtered_pulses_r;

    if (fabsf(filtered_pulses_r) < 1.0f)
        filtered_pulses_r = 0.0f;

    return -filtered_pulses_r;
}

/**
 * @brief Motor PID 计算函数
 * @param pid  PID 结构体指针
 * @param error 当前误差 (目标值 - 实际值)
 * @return float 计算得出的 PWM 或 修正量
 */
float PID_Compute_Motor(STRUCT_PID *pid, float target, float actual)
{
    float error = target - actual;
    float current_kp = pid->Kp;

    // --- 分段 Kp 逻辑：未达到目标速度 2/3 时，Kp 翻倍 ---
    // 使用 fabsf 保证正反转逻辑一致
    if (fabsf(actual) < fabsf(target) * 0.8f)
    {
        current_kp = pid->Kp * 2.5f; // 起步增益，可根据实际效果在 1.5~2.5 之间调整
    }

    // 1. P 输出
    float p_out = current_kp * error;

    // 2. I 输出 (累加误差)
    pid->integral += error;
    if (pid->integral > pid->i_max)
        pid->integral = pid->i_max;
    if (pid->integral < -pid->i_max)
        pid->integral = -pid->i_max;

    float i_out = pid->Ki * pid->integral;

    // 3. D 输出 (误差变化率)
    float d_out = pid->Kd * (error - pid->err_last);
    pid->err_last = error;

    // 4. 合并输出并进行最终限幅
    float out = p_out + i_out + d_out;

    if (out > pid->out_max)
        out = pid->out_max;
    if (out < -pid->out_max)
        out = -pid->out_max;

    return out;
}
/**
 * @brief  位置式 PID 计算函数
 * @param  pid:    指向 STRUCT_PID 结构体的指针
 * @param  target: 目标值
 * @param  real:   当前实际测量值
 * @return float:  PID 计算后的输出值
 */
float pid_control(STRUCT_PID *pid, float target, float real)
{
    float err = 0;  // 当前偏差
    float diff = 0; // 微分项
    float out = 0;  // 临时输出结果

    // 1. 计算当前偏差 (Error)
    err = target - real;

    // 2. 积分项累加 (Integral)
    pid->integral += err;

    // 3. 积分抗饱和 (Anti-windup)
    // 根据结构体定义中的 i_max 进行限幅
    if (pid->integral > pid->i_max)
        pid->integral = pid->i_max;
    if (pid->integral < -pid->i_max)
        pid->integral = -pid->i_max;

    // 4. 微分项计算 (Derivative)
    // 使用当前偏差减去结构体中存储的 err_last
    diff = err - pid->err_last;

    // 5. PID 公式拟合
    out = (pid->Kp * err) + (pid->Ki * pid->integral) + (pid->Kd * diff);

    // 6. 输出限幅
    if (out > pid->out_max)
        out = pid->out_max;
    if (out < -pid->out_max)
        out = -pid->out_max;

    // 7. 更新历史偏差，供下次微分计算使用
    pid->err_last = err;

    return out;
}

/**
 * @brief  平衡小车 1ms 核心控制函数
 * @param  target_speed: 期望速度
 * @param  target_yaw:   期望转向角
 */
void Control_Loop_1ms(float target_speed, float target_yaw_rate)
{
    static uint8_t speed_count = 0;
    static uint8_t speed_count_1 = 0;
    static float speed_target_angle = 0.0f; // 速度环输出：期望倾角

    float target_gyro_y = 0.0f;

        // 0. 检查停机状态
        if (State.is_stop)
    {
        // 停机时清空所有积分项，防止复位瞬间产生巨大脉冲
        Config.speed_loop.integral = 0;
        Config.angle_loop.integral = 0;
        Config.gyro_loop.integral = 0;
        Config.motor_l.integral = 0;
        Config.motor_r.integral = 0;

        // 输出清零
        State.motor_target_speed_left = 0;
        State.motor_target_speed_right = 0;
        return;
    }

    // --- 第一层：速度环 (Speed Loop) ---
    // 20ms运行一次。速度环必须慢，否则会干扰平衡环的快速响应
    speed_count++;
    if (speed_count >= 20)
    {
        float current_speed = (State.motor_actual_speed_left + State.motor_actual_speed_right) / 2.0f;

        // 速度环输出给角度环：为了往前走，车必须先往前倾
        // 注意：此处输出通常需要限制在安全角度内（如 -15° 到 15°）
        speed_target_angle = 0;//pid_control(&Config.speed_loop, target_speed, current_speed);
        speed_count = 0;
    }

    // --- 第二层：角度环 (Angle Loop) ---
    // 5ms运行一次。输入：期望倾角，反馈：当前加速度计计算出的 Pitch
    speed_count_1++;
    // 角度环输出 = 目标角速度
    if (speed_count_1 >=5)
    {
        speed_count_1 = 0;
        target_gyro_y = 0;//pid_control(&Config.angle_loop, speed_target_angle, State.pitch);
    }


    // --- 第三层：角速度环 (Gyro Loop) ---
    // 1ms运行一次。这是最关键的内环，直接决定车的稳定性
    // 输入：角度环给出的目标角速度，反馈：陀螺仪实时角速度 (State.gyro_y)
    float balance_output = pid_control(&Config.gyro_loop, target_gyro_y, State.gyro_y);

    // --- 转向环 (Yaw Loop) ---
    // 转向通常作为差分量叠加。输入：目标转向速度，反馈：陀螺仪 Z 轴角速度
    float yaw_output = 0; // pid_control(&Config.yaw_loop, target_yaw_rate, State.gyro_z);

    // --- 最终输出叠加与映射 ---
    // 1. 叠加平衡量与转向差分量
    float out_l = balance_output + yaw_output;
    float out_r = balance_output - yaw_output;

    // 2. 这里的输出通常直接作为底层“电机电流环”或“电机速度环”的输入
    // 如果没有底层环，则直接映射为 PWM 占空比。
    // 注意：需根据实际电机性能进行限幅（例如 PWM 最大值为 10000）
    State.motor_target_speed_left = out_l;
    State.motor_target_speed_right = out_r;
}

void All_Update(void)
{
    static uint8_t speed_cnt = 0;

    // 1ms 执行一次 IMU 更新
    ICM_Update();
    Attitude_Update();

    // 2. 输出姿态传感器数据
    char str[128];
    // sprintf(str, "%f,%f,%f,%f\n", State.pitch, State.roll, State.yaw,State.gyro_x);
    // uart_write_string(DEBUG_UART_INDEX, str);

    // 3. 输出电机速度控制逻辑
    sprintf(str, "%f,%f,%f,%f\n", State.motor_target_speed_left, State.motor_target_speed_right, State.motor_actual_speed_left, State.motor_actual_speed_right);
    uart_write_string(DEBUG_UART_INDEX, str);
}