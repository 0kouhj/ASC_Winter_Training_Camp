#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H
#include "zf_common_headfile.h"

void motor_init(void);
void motor_set_left_speed(int16 speed);
void motor_set_right_speed(int16 speed);
void motor_set_speed(int16_t left_speed, int16_t right_speed);
void motor_stop(void);
void Limit(int PwmLeft,int PwmRight);


// PID 控制更新
void motor_update(void);

// 测试函数
void motor_test_50_50(void);
void motor_test_neg50_neg50(void);
void motor_test_100_100(void);
void motor_test_neg100_neg100(void);

#endif