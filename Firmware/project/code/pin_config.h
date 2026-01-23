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

//软件定义

#define KEY_TIM         (TIM6_PIT)
#define KEY_PRIORITY    (TIM6_IRQn)

#endif