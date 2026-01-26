#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "zf_common_headfile.h"

/**
 * @brief 初始化编码器
 */
void encoder_init(void);

/**
 * @brief 获取左编码器脉冲数
 * @return 左编码器脉冲计数
 */
void encoder_get_left(void);

/**
 * @brief 获取右编码器脉冲数
 * @return 右编码器脉冲计数
 */
void encoder_get_right(void);

void encoder_update(void);

#endif