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

/**
 * @brief 获取左电机原始脉冲并清零
 */
void encoder_get_left(void)
{
    // 1. 获取原始脉冲计数（由定时器硬件计数器提供）
    int32_t raw_count = (int32_t)encoder_get_count(TIM3_ENCODER);
    encoder_clear_count(TIM3_ENCODER);

    // 2. 通过中间层函数进行“信号调理”（滤波、限幅）
    // 这样 State 里的数据就是平滑的 float，PID 算起来才不会有毛刺
    State.motor_actual_speed_left = Motion_Get_Speed_L(raw_count);
}

/**
 * @brief 获取右电机原始脉冲并清零
 */
void encoder_get_right(void)
{
    int32_t raw_count = (int32_t)encoder_get_count(TIM4_ENCODER);
    encoder_clear_count(TIM4_ENCODER);

    State.motor_actual_speed_right = Motion_Get_Speed_R(raw_count);
}

#define MAX_REASONABLE_PULSE 500 // 5ms内电机不可能超过500脉冲，超过即为毛刺

void encoder_update(void)
{
    encoder_get_left();
    encoder_get_right();
}