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

void encoder_update(void)
{
    encoder_get_left();
    encoder_get_right();
    State.motor_actual_speed_left = Motion_Get_Speed(State.encoder_left);
    State.motor_actual_speed_right = Motion_Get_Speed(State.encoder_right);
}