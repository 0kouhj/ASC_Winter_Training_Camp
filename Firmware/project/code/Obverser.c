#include "Obverser.h"
#include "param_config.h"
#include "Control.h"
#include "Kalman.h"
#include "math.h"

static SpeedObserver speed_observer = {0};

// ===== 2. 初始化观测器 =====
void speed_observer_init(void)
{
    speed_observer.speed_estimate = 0.0f;
    speed_observer.position_estimate = 0.0f;
    speed_observer.accel_bias = 0.0f;
    speed_observer.last_speed = 0.0f;
    speed_observer.last_pwm = 0;
//    speed_observer.last_update_time = systick_get_ms();
}

// ===== 3. 观测器更新函数 =====
float speed_observer_update(float accel_x_g, float pitch_deg, int16_t current_pwm)
{
    // 获取时间间隔 (秒)
//    uint32_t current_time = systick_get_ms();
    float dt = 0.005;
    
    // === 1. 从加速度计获取线性加速度 ===
    // 将g转换为m/s²
    float accel_mss = accel_x_g * 9.80665f;
    
    // 去除重力分量 (关键步骤！)
    float pitch_rad = pitch_deg * 3.1415926535f / 180.0f;
    float gravity_component = 9.80665f * sinf(pitch_rad);
    float linear_accel = accel_mss - gravity_component - speed_observer.accel_bias;
    
    // 对加速度进行低通滤波 (减少振动噪声)
    static float accel_filtered = 0.0f;
    const float accel_alpha = 0.2f;
    accel_filtered = (1.0f - accel_alpha) * linear_accel + accel_alpha * accel_filtered;
    
    // === 2. 从电机PWM获取速度估计 ===
    // 简单模型：PWM与速度成正比 (需要标定)
    float speed_from_pwm = 0.0f;
    
    // 模型参数 (需要根据实际电机调整)
    #define PWM_TO_SPEED_GAIN 0.00015f  // PWM值到速度的转换系数
    #define MOTOR_TIME_CONSTANT 0.05f   // 电机时间常数
    
    // 使用一阶惯性模型
    speed_from_pwm = (float)current_pwm * PWM_TO_SPEED_GAIN;
    
    // 低通滤波模拟电机惯性
    static float pwm_speed_filtered = 0.0f;
    const float pwm_alpha = dt / (MOTOR_TIME_CONSTANT + dt);
    pwm_speed_filtered = (1.0f - pwm_alpha) * pwm_speed_filtered + pwm_alpha * speed_from_pwm;
    
    // === 3. 互补滤波融合 ===
    // 预测：基于加速度积分
    float speed_predicted = speed_observer.speed_estimate + accel_filtered * dt;
    
    // 测量：来自PWM模型
    float speed_measured = pwm_speed_filtered;
    
    // 互补滤波系数 (可调)
    const float K_accel = 0.7f;  // 信任加速度计的程度
    const float K_pwm = 0.3f;    // 信任PWM模型的程度
    const float K_bias = 0.001f; // 零偏学习速率
    
    // 融合
    float new_speed_estimate = K_accel * speed_predicted + K_pwm * speed_measured;
    
    // === 4. 零速检测与归零 ===
    // 当小车直立且角速度很小时，认为速度接近0
    float gyro_y_dps = State.gyro_y;  // 前后方向的角速度
    
    if (fabs(pitch_deg) < 2.0f && fabs(gyro_y_dps) < 5.0f) {
        // 缓慢归零
        new_speed_estimate *= 0.98f;
        // 更新零偏估计
        speed_observer.accel_bias += K_bias * (linear_accel - (new_speed_estimate - speed_observer.speed_estimate) / dt);
    }
    
    // 死区处理 (消除微小速度)
    if (fabs(new_speed_estimate) < 0.02f) {
        new_speed_estimate = 0.0f;
    }
    
    // === 5. 限幅 ===
    const float MAX_ESTIMATED_SPEED = 1.5f;
    if (new_speed_estimate > MAX_ESTIMATED_SPEED) new_speed_estimate = MAX_ESTIMATED_SPEED;
    if (new_speed_estimate < -MAX_ESTIMATED_SPEED) new_speed_estimate = -MAX_ESTIMATED_SPEED;
    
    // === 6. 更新观测器状态 ===
    speed_observer.last_speed = speed_observer.speed_estimate;
    speed_observer.speed_estimate = new_speed_estimate;
    speed_observer.position_estimate += new_speed_estimate * dt;
    speed_observer.last_pwm = current_pwm;
    
    return new_speed_estimate;
}

// ===== 4. 获取IMU原始加速度的函数 =====
float get_imu_accel_x_g(void)
{
    // 从Icm结构体获取原始加速度数据
    return Icm.accel_x_g;
}
