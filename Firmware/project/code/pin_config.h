#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

//驱动定义
#define LED_1           (H2)
#define LED_2           (B8)

#define DOUBLE_KEY_1    (D3)
#define DOUBLE_KEY_2    (D4)

#define BUZZER          (D7)

#define IMU_CS          (B12)
#define IMU_SCLK        (B13)
#define IMU_MOSI        (B14)
#define IMU_MISO        (B15)

#define OLED_SCL        (A7)
#define OLED_SDA        (A5)

//电机
#define PWM_CH1         (TIM5_PWM_CH1_A0)
#define PWM_CH2         (TIM5_PWM_CH2_A1)

#define ENCODER_LEFT_A  (B4)
#define ENCODER_LEFT_B  (B5)

#define ENCODER_RIGHT_A (B6)
#define ENCODER_RIGHT_B (B7)

#define Motor_L_DIR1      (A2)
#define Motor_L_DIR2      (A3)

#define Motor_R_DIR1      (D5)
#define Motor_R_DIR2      (D6)
//软件定义

#define KEY_TIM         (TIM6_PIT)
#define KEY_PRIORITY    (TIM6_IRQn)

#endif