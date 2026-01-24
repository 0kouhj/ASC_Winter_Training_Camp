#include "bsp_imu.h"
#include "zf_driver_gpio.h"
#include "zf_common_debug.h"
#include <math.h>

// --- 软件 I2C 底层时序 ---

#define SCL_H   gpio_set_level(ICM_SCL_PIN, 1)
#define SCL_L   gpio_set_level(ICM_SCL_PIN, 0)
#define SDA_H   gpio_set_level(ICM_SDA_PIN, 1)
#define SDA_L   gpio_set_level(ICM_SDA_PIN, 0)
#define SDA_IN  gpio_get_level(ICM_SDA_PIN)

// 简单的延时，用于控制 I2C 速率 (约 400kHz)
static void i2c_delay(void) {
    for(volatile uint8 i=0; i<5; i++); 
}

// SDA 切换为输出
static void sda_out_mode(void) {
    gpio_init(ICM_SDA_PIN, GPO, 1, GPO_PUSH_PULL);
}

// SDA 切换为输入
static void sda_in_mode(void) {
    gpio_init(ICM_SDA_PIN, GPI, 0, GPI_FLOATING_IN); 
}

static void i2c_start(void) {
    sda_out_mode();
    SDA_H; SCL_H; i2c_delay();
    SDA_L; i2c_delay();
    SCL_L; i2c_delay();
}

static void i2c_stop(void) {
    sda_out_mode();
    SDA_L; SCL_H; i2c_delay();
    SDA_H; i2c_delay();
}

static uint8 i2c_wait_ack(void) {
    uint8 retry = 0;
    sda_in_mode();
    SDA_H; i2c_delay();
    SCL_H; i2c_delay();
    while(SDA_IN) {
        if(++retry > 250) { i2c_stop(); return 1; }
    }
    SCL_L; i2c_delay();
    return 0;
}

static void i2c_send_byte(uint8 data) {
    sda_out_mode();
    for(uint8 i=0; i<8; i++) {
        if(data & 0x80) SDA_H; else SDA_L;
        data <<= 1;
        i2c_delay();
        SCL_H; i2c_delay();
        SCL_L; i2c_delay();
    }
}

static uint8 i2c_read_byte(uint8 ack) {
    uint8 res = 0;
    sda_in_mode();
    for(uint8 i=0; i<8; i++) {
        SCL_H; i2c_delay();
        res <<= 1;
        if(SDA_IN) res++;
        SCL_L; i2c_delay();
    }
    sda_out_mode();
    if(ack) SDA_L; else SDA_H; // 发送 ACK 或 NACK
    SCL_H; i2c_delay();
    SCL_L; i2c_delay();
    return res;
}

// --- ICM42688 寄存器操作 ---

void icm_i2c_write_reg(uint8 reg, uint8 data) {
    i2c_start();
    i2c_send_byte((ICM_I2C_ADDR << 1) | 0);
    i2c_wait_ack();
    i2c_send_byte(reg);
    i2c_wait_ack();
    i2c_send_byte(data);
    i2c_wait_ack();
    i2c_stop();
}

void icm_i2c_read_regs(uint8 reg, uint8 *buf, uint8 len) {
    i2c_start();
    i2c_send_byte((ICM_I2C_ADDR << 1) | 0);
    i2c_wait_ack();
    i2c_send_byte(reg);
    i2c_wait_ack();
    
    i2c_start();
    i2c_send_byte((ICM_I2C_ADDR << 1) | 1);
    i2c_wait_ack();
    while(len--) {
        *buf++ = i2c_read_byte(len > 0); // 最后一位发 NACK
    }
    i2c_stop();
}

// --- 核心逻辑 ---

uint8 ICM42688_I2C_Init(void) {
    gpio_init(ICM_SCL_PIN, GPO, 1, GPO_PUSH_PULL);
    gpio_init(ICM_SDA_PIN, GPO, 1, GPO_PUSH_PULL);
    
    system_delay_ms(10);
    
    // 检查 ID
    uint8 id;
    icm_i2c_read_regs(ICM42688_REG_WHO_AM_I, &id, 1);
    if(id != 0x47) return 1;

    // 复位与配置
    icm_i2c_write_reg(ICM42688_REG_DEVICE_CONFIG, 0x01);
    system_delay_ms(10);
    icm_i2c_write_reg(ICM42688_REG_PWR_MGMT0, 0x0F);     // 开启陀螺仪和加速度计 LN 模式
    icm_i2c_write_reg(ICM42688_REG_GYRO_CONFIG0, 0x06);  // 2000dps, 1kHz
    icm_i2c_write_reg(ICM42688_REG_ACCEL_CONFIG0, 0x06); // 16g, 1kHz
    
    return 0;
}

void ICM42688_I2C_Read_Data(ICM42688_t *dev) {
    uint8 data[14];
    icm_i2c_read_regs(ICM42688_REG_TEMP_DATA1, data, 14);

    dev->accel_x_raw = (int16)((data[2] << 8) | data[3]);
    dev->accel_y_raw = (int16)((data[4] << 8) | data[5]);
    dev->accel_z_raw = (int16)((data[6] << 8) | data[7]);

    dev->gyro_x_raw = (int16)((data[8] << 8) | data[9]);
    dev->gyro_y_raw = (int16)((data[10] << 8) | data[11]);
    dev->gyro_z_raw = (int16)((data[12] << 8) | data[13]);

    // 灵敏度转换 (16g -> 2048 LSB/g, 2000dps -> 16.4 LSB/dps)
    dev->accel_x_g = (float)dev->accel_x_raw / 2048.0f;
    dev->accel_y_g = (float)dev->accel_y_raw / 2048.0f;
    dev->accel_z_g = (float)dev->accel_z_raw / 2048.0f;
    dev->gyro_x_dps = (float)dev->gyro_x_raw / 16.4f;
    dev->gyro_y_dps = (float)dev->gyro_y_raw / 16.4f;
    dev->gyro_z_dps = (float)dev->gyro_z_raw / 16.4f;
}