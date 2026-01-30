#include "Control.h"
#include "Kalman.h"
#include "bsp_encoder.h"
#include "bsp_imu.h"
#include "param_config.h"
#include "bsp_motor.h"
#include "Obverser.h"
#include "math.h"

// PWM限幅，比17000小一些
#define Act_Motor_PWM_Freq_Max 4000
#define Act_Motor_PWM_Freq_Min -4000
//时间轮调用频率，与姿态解算频率保持一致
#define CONTROL_PERIOD_S 0.005f

// 三环比例系数
float Vertical_Kp=-2050, Vertical_Kd=-25;
float Velocity_Kp=0, Velocity_Ki=0;
float Turn_Kp, Turn_Kd;
uint8_t stop;  // 用于蓝牙遥控直接停止

// Control()中需要读取的传感器数据
float pitch, roll, yaw;      // 欧拉角
float gyrox, gyroy, gyroz;   // 三轴角速度

// 闭环控制中间变量
int Vertical_out, Turn_out;
float Velocity_out_angle;    // 速度环输出的角度偏移量(度)
float Target_Speed_mps=0.5f;      // 目标速度 (m/s)
int Target_Turn;             // 目标转向量
float mech_med=-0.2;//-2.5f;              // 机械中值（平衡时角度的偏移量）
static int16_t current_control_pwm = 0;
static int16_t pwm_filtered_l = 0, pwm_filtered_r = 0;

// 添加死区补偿函数
static int16_t apply_motor_deadzone_adaptive(int16_t pwm_value) {
    static uint16_t actual_deadzone = MOTOR_DEADZONE_THRESHOLD;
    static int deadzone_learning = 0;
    
    if (deadzone_learning < 1000) {
        deadzone_learning++;
        // 学习阶段：逐步调整死区
        if (pwm_value > actual_deadzone && pwm_value < actual_deadzone + 200) {
            // 在这个区间，如果电机响应不好，增加死区
            actual_deadzone += 10;
        }
    }
    
    if (pwm_value > actual_deadzone) {
        return pwm_value + MOTOR_MIN_PWM;
    } else if (pwm_value < -actual_deadzone) {
        return pwm_value - MOTOR_MIN_PWM;
    } else {
        // 死区内：根据pwm值给出比例输出（不是全0）
        if (pwm_value != 0) {
            // 比例映射：死区内也给出小输出
            float ratio = (float)pwm_value / actual_deadzone;
            return (int16_t)(ratio * (actual_deadzone + MOTOR_MIN_PWM));
        }
        return 0;
    }
}

// 添加输出平滑函数
static int16_t smooth_pwm_output(int16_t new_pwm, int16_t *filtered_pwm)
{
    // 一阶低通滤波平滑输出
    *filtered_pwm = (int16_t)((1.0f - OUTPUT_SMOOTHING_ALPHA) * (*filtered_pwm) + 
                               OUTPUT_SMOOTHING_ALPHA * new_pwm);
    
    // 去除微小抖动
    if (abs(*filtered_pwm) < MIN_EFFECTIVE_PWM) {
        *filtered_pwm = 0;
    }
    
    return *filtered_pwm;
}


// 直立环PD控制器
// 输入：期望角度、真实角度、角速度
int Vertical(float anticipation, float Angle, float gyro_Y)
{
    static float gyro_filtered = 0.0f;
    const float gyro_alpha = 0.15f;
    
    int16 temp;
    
    // 1. 角速度滤波
    gyro_filtered = (1.0f - gyro_alpha) * gyro_filtered + gyro_alpha * gyro_Y;
    
    // 2. 计算原始输出
    float angle_error = Angle - anticipation;
    temp = (int16)(Vertical_Kp * angle_error + Vertical_Kd * gyro_filtered);
    
    // 3. 输出限幅（保护电机）
    if (temp > Act_Motor_PWM_Freq_Max * 0.8f) 
        temp = Act_Motor_PWM_Freq_Max * 0.8f;
    if (temp < Act_Motor_PWM_Freq_Min * 0.8f) 
        temp = Act_Motor_PWM_Freq_Min * 0.8f;
    
    return temp;
}

// 速度环PI控制器(使用物理速度)
// 输入：目标速度、当前速度
// 添加抗饱和和输出滤波
float Velocity_PID_Physical(float target_speed_mps, float current_speed_mps)
{
    static float err_lowpass_last = 0.0f;
    static float integral_sum = 0.0f;
    static float output_last = 0.0f;
    static float alpha = 0.7f;
    
    float err, err_lowpass, output_raw, output;
    
    // 1. 计算速度误差
    err = current_speed_mps - target_speed_mps;
    
    // 2. 低通滤波
    err_lowpass = (1.0f - alpha) * err + alpha * err_lowpass_last;
    err_lowpass_last = err_lowpass;
    
    // 3. 积分项（带抗饱和）
    integral_sum += err_lowpass * CONTROL_PERIOD_S;
    
    // 积分限幅（根据你的VELOCITY_INTEGRAL_LIMIT）
    if (integral_sum > VELOCITY_INTEGRAL_LIMIT)
        integral_sum = VELOCITY_INTEGRAL_LIMIT;
    if (integral_sum < -VELOCITY_INTEGRAL_LIMIT)
        integral_sum = -VELOCITY_INTEGRAL_LIMIT;
    
    // 蓝牙停止时清零积分
    if (stop == 1) {
        integral_sum = 0.0f;
        stop = 0;
    }
    
    // 4. 计算原始输出
    output_raw = Velocity_Kp * err_lowpass + Velocity_Ki * integral_sum;
    
    // 5. 输出变化率限制（避免突变）
    float max_change = 0.1f * CONTROL_PERIOD_S; // 最大变化率
    if (fabs(output_raw - output_last) > max_change) {
        if (output_raw > output_last) {
            output = output_last + max_change;
        } else {
            output = output_last - max_change;
        }
    } else {
        output = output_raw;
    }
    output_last = output;
    
    // 6. 输出限幅
    if (output > VELOCITY_MAX_ANGLE_OFFSET)
        output = VELOCITY_MAX_ANGLE_OFFSET;
    if (output < -VELOCITY_MAX_ANGLE_OFFSET)
        output = -VELOCITY_MAX_ANGLE_OFFSET;
    
    return output;
}

// 转向环PD控制器
// 输入：角速度、遥控时期望转过的角度值
int Turn(float gyro_Z, int Target_turn)
{
    int temp;
    temp = Turn_Kp * Target_turn + Turn_Kd * gyro_Z;
    return temp;
}

void Control(void)
{
    int16_t PWM_L, PWM_R;
    
    // ===== 0. 初始化观测器（在系统启动时调用一次）=====
    static uint8_t observer_initialized = 0;
    if (!observer_initialized) {
        speed_observer_init();
        observer_initialized = 1;
    }
    
    // ===== 1. 更新传感器数据 =====
    // a. 读取IMU原始数据（用于观测器）
    float imu_accel_x_g = get_imu_accel_x_g();
    
    // b. 读取IMU姿态解算数据
    pitch = State.pitch;     // 俯仰角（度）
    roll = State.roll;       // 横滚角（度）
    yaw = State.yaw;         // 偏航角（度）
    gyrox = State.gyro_x;    // X轴角速度（°/s）
    gyroy = State.gyro_y;    // Y轴角速度（平衡控制核心）（°/s）
    gyroz = State.gyro_z;    // Z轴角速度（°/s）
    
    // ===== 2. 使用观测器估计速度 =====
    // 注意：current_control_pwm是上一次的PWM输出
    float estimated_speed_mps = speed_observer_update(imu_accel_x_g, pitch, current_control_pwm);
    
    // ===== 3. 三环PID计算 =====
    // a. 速度环：使用观测器估计的速度
    float current_speed_mps = estimated_speed_mps;  // 使用观测器速度
    
    // 可选：如果需要，也可以尝试使用State.motor_actual_speed_right（如果还有残存信号）
    // 但主要依赖观测器
    
    // 调用物理速度PID
    Velocity_out_angle = Velocity_PID_Physical(Target_Speed_mps, current_speed_mps);
    
    // 速度环输出限幅（防止角度偏移过大）
    if (Velocity_out_angle > VELOCITY_MAX_ANGLE_OFFSET)
        Velocity_out_angle = VELOCITY_MAX_ANGLE_OFFSET;
    if (Velocity_out_angle < -VELOCITY_MAX_ANGLE_OFFSET)
        Velocity_out_angle = -VELOCITY_MAX_ANGLE_OFFSET;
    
    // b. 直立环：期望角度 = 机械中值 + 速度环输出的角度偏移
    float desired_angle = mech_med + Velocity_out_angle;
    Vertical_out = Vertical(desired_angle, pitch, gyrox);
    
    // c. 转向环
    Turn_out = 0;
    
    // ===== 4. 三环合成输出（直立±转向） =====
    PWM_L = Vertical_out + Turn_out;
    PWM_R = Vertical_out - Turn_out;  // 差速转向
    
    // ===== 5. 保存当前PWM供下个周期观测器使用 =====
    current_control_pwm = (PWM_L + PWM_R) / 2;
    
    // ===== 6. PWM限幅 =====
    if (PWM_L > Act_Motor_PWM_Freq_Max) PWM_L = Act_Motor_PWM_Freq_Max;
    if (PWM_R > Act_Motor_PWM_Freq_Max) PWM_R = Act_Motor_PWM_Freq_Max;
    if (PWM_L < Act_Motor_PWM_Freq_Min) PWM_L = Act_Motor_PWM_Freq_Min;
    if (PWM_R < Act_Motor_PWM_Freq_Min) PWM_R = Act_Motor_PWM_Freq_Min;
    
		// ===== 6.5 死区补偿与输出平滑 =====
    // 应用死区补偿
    PWM_L = apply_motor_deadzone_adaptive(PWM_L);
    PWM_R = apply_motor_deadzone_adaptive(PWM_R);
    
    // 输出平滑（避免突变）
    PWM_L = smooth_pwm_output(PWM_L, &pwm_filtered_l);
    PWM_R = smooth_pwm_output(PWM_R, &pwm_filtered_r);
		
		// 重新限幅（平滑后可能超出范围）
    if (PWM_L > Act_Motor_PWM_Freq_Max) PWM_L = Act_Motor_PWM_Freq_Max;
    if (PWM_R > Act_Motor_PWM_Freq_Max) PWM_R = Act_Motor_PWM_Freq_Max;
    if (PWM_L < Act_Motor_PWM_Freq_Min) PWM_L = Act_Motor_PWM_Freq_Min;
    if (PWM_R < Act_Motor_PWM_Freq_Min) PWM_R = Act_Motor_PWM_Freq_Min;
		
    // ===== 7. 更新State中的速度（用于显示） =====
    State.motor_actual_speed_left = estimated_speed_mps;
    State.motor_actual_speed_right = estimated_speed_mps;
    
    // ===== 8. 电机运行 =====
    motor_set_left_speed(PWM_L);
    motor_set_right_speed(PWM_R);
}

// 原有的转换函数保持不变（供其他模块使用）
float Motion_Get_Speed(int32_t pulse_count)
{
    return (float)pulse_count * PULSE_TO_MPS_FACTOR;
}

// 创建最简单的测试程序
void minimal_test(void) {
    // 1. 读取IMU数据
    pitch = State.pitch;
    gyroy = State.gyro_y;
    
    // 2. 最简化的PD控制
    float desired_angle = 0;  // 期望0度
    float error = pitch - desired_angle;
    
    // 非常小的增益
    float Kp = -200.0f;
    float Kd = -5.0f;
    
    int16_t pwm = (int16_t)(Kp * error + Kd * gyroy);
    
    // 直接输出，无任何滤波、死区
    motor_set_left_speed(pwm);
    motor_set_right_speed(pwm);
}



int16_t Motion_Speed_To_PWM(float target_speed)
{
    float pwm_out;
    
    // 1. 限幅
    if (target_speed > MOTOR_MAX_SPEED_MPS)
        target_speed = MOTOR_MAX_SPEED_MPS;
    if (target_speed < -MOTOR_MAX_SPEED_MPS)
        target_speed = -MOTOR_MAX_SPEED_MPS;
    
    // 2. 线性映射计算
    pwm_out = target_speed * MPS_TO_PWM_FACTOR;
    
    // 3. 死区补偿
    if (pwm_out > 0.1f) {
        pwm_out += MOTOR_MIN_PWM;
    } else if (pwm_out < -0.1f) {
        pwm_out -= MOTOR_MIN_PWM;
    }
    
    // 4. 最终PWM限幅
    if (pwm_out > MOTOR_MAX_PWM)
        pwm_out = MOTOR_MAX_PWM;
    if (pwm_out < -MOTOR_MAX_PWM)
        pwm_out = -MOTOR_MAX_PWM;
    
    return (int16_t)pwm_out;
}