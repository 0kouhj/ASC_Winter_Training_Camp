#ifndef OBVERSER_H
#define OBVERSER_H

#include "zf_common_headfile.h"

// 函数声明
void speed_observer_init(void);
float speed_observer_update(float accel_x_g, float pitch_deg, int16_t current_pwm);
float get_imu_accel_x_g(void);

#endif
