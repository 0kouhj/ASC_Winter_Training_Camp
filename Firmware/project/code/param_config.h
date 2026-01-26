#ifndef PARAM_CONFIG_H
#define PARAM_CONFIG_H

#include "zf_common_headfile.h"

typedef enum{
    LED1,
    LED2,
    ALL
}LED_LIST;

/******* 以下为非FLASH储存的实时状态变量 *******/

typedef struct {
    // 原始数据
    int16 accel_x_raw, accel_y_raw, accel_z_raw;
    int16 gyro_x_raw, gyro_y_raw, gyro_z_raw;
    
    // 物理量
    float accel_x_g, accel_y_g, accel_z_g;    // 单位: g
    float gyro_x_dps, gyro_y_dps, gyro_z_dps; // 单位: deg/s
    
} ICM42688_t;

typedef struct {
    // 传感器原始/融合数据
    float pitch;            // 当前俯仰角
    float roll;             // 当前横滚角
    float yaw;              // 当前航向角
    float gyro_x;           // X轴角速度 (前进方向)
    float gyro_y;           // Y轴角速度（平衡控制核心）
    float gyro_z;           // Z轴角速度
		float Kp;
		float Ki;
		float Kd;

    // 红外传感器
    uint8_t infrared[6];
    
    // 动力系统状态
    int16_t encoder_left;   // 左电机总脉冲
    int16_t encoder_right;  // 右电机总脉冲
    float motor_target_speed_left;   // 左电机目标速度
    float motor_target_speed_right;  // 右电机目标速度
    float motor_actual_speed_left;   // 左电机实际速度
    float motor_actual_speed_right;  // 右电机实际速度
    float displacement;     // 当前累计位移
    
    // 电池与系统
    float battery_v;        // 实时电池电压
    uint32_t system_tick;   // 系统运行时间 (ms)
    
    // 逻辑控制
    uint8_t is_stop;        // 停机保护标志 (1: 停机, 0: 运行)
    uint8_t run_stage;      // 模式2/3中的具体阶段 (如: 0-直行, 1-弧线)
    uint16_t loop_count;    // 模式3中的圈数计数
    
    // 菜单交互
    int8_t menu_index;      // 菜单当前选中的行
    uint8_t menu_level;     // 菜单层级
} SYSTEM_STATE_T;


/*********以下为flash需要存储的函数*************/
// 运行模式枚举
typedef enum {
    MODE_1 = 1,
    MODE_2,
    MODE_3,
    MODE_4,
    MODE_5
} BOOT_MODE_E;

// --- 结构体定义 (UPER Case) ---

typedef struct {
    uint8_t boot_flag;      // 是否首次启动标志
    BOOT_MODE_E boot_mode;  // 当前选择的模式
} STRUCT_BOOT;

typedef struct {
    float angle_kp;
    float angle_ki;
    float angle_kd;
    float mech_angle;       // 机械中值
} STRUCT_ANGLE_PID;

typedef struct {
    float speed_kp;
    float speed_ki;
    float speed_kd;
} STRUCT_SPEED_PID;

typedef struct {
    float pos_kp;
    float pos_ki;
    float pos_kd;
} STRUCT_POS_PID;

typedef struct {
    float yaw_kp;
    float yaw_ki;
    float yaw_kd;
} STRUCT_YAW_PID;

typedef struct {
    uint8_t move_speed;
    uint8_t corner_speed;
    uint8_t arc_radius;
    float target_dist_AB;   // A到B的预设距离
} STRUCT_PATH_MOTION;

typedef struct {
    uint32_t path_record_len;
    uint32_t path_data_addr;
} STRUCT_MODE_4;

typedef struct {
    STRUCT_BOOT        boot;
    STRUCT_ANGLE_PID   angle;
    STRUCT_SPEED_PID   speed;
    STRUCT_POS_PID     position;
    STRUCT_YAW_PID     yaw;
    STRUCT_PATH_MOTION path;
    STRUCT_MODE_4      mode4;
    uint32_t           checksum; // 校验码，判断Flash数据是否合法
} SYSTEM_CONFIG_T;

// 声明全局变量
extern SYSTEM_CONFIG_T Config;
extern SYSTEM_STATE_T  State;
extern ICM42688_t Icm;
// 函数声明
void Param_Init(void);

#endif