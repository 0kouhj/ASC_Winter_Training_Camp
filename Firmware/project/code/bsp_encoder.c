#include "zf_common_headfile.h"
#include "bsp_encoder.h"
#include "pin_config.h"
#include "Control.h"

void encoder_init(void)
{
    // 初始化左编码器，使用TIM3，通道1 B4，通道2 B5
    encoder_quad_init(TIM3_ENCODER, ENCODER_LEFT_A, ENCODER_LEFT_B);

    // 初始化右编码器，使用TIM4，通道1 B6，通道2 B7
    encoder_quad_init(TIM4_ENCODER, ENCODER_RIGHT_A, ENCODER_RIGHT_B);
}

void encoder_get_left(void)
{
    State.encoder_left = encoder_get_count(TIM3_ENCODER);
    encoder_clear_count(TIM3_ENCODER);
}

void encoder_get_right(void)
{
    State.encoder_right = encoder_get_count(TIM4_ENCODER);
    encoder_clear_count(TIM4_ENCODER);
}

#define SPEED_LIMIT_MPS 0.44f // 定义物理速度限幅值
#define MAX_REASONABLE_PULSE 500 // 5ms内电机不可能超过500脉冲，超过即为毛刺

void encoder_update(void)
{
    encoder_get_left();
    encoder_get_right();

    // 1. 强制类型转换处理回绕
    int16_t pulse_l = (int16_t)((uint16_t)State.encoder_left);
    int16_t pulse_r = (int16_t)((uint16_t)State.encoder_right);

    // 2. 核心补丁：剔除毛刺数据
    // 如果脉冲数大得离谱（即你图中的绿线尖峰），直接丢弃本次采样，维持上次速度
    if (abs(pulse_l) > MAX_REASONABLE_PULSE || abs(pulse_r) > MAX_REASONABLE_PULSE)
    {
        // 保持 motor_actual_speed 不变，不更新
        return;
    }

    // 3. 计算物理速度并配合低通滤波
    float speed_l = Motion_Get_Speed(pulse_l);
    float speed_r = Motion_Get_Speed(pulse_r);

    // 4. 限幅并平滑滤波
    State.motor_actual_speed_left = State.motor_actual_speed_left * 0.8f + speed_l * 0.2f;
    State.motor_actual_speed_right = State.motor_actual_speed_right * 0.8f + speed_r * 0.2f;
}