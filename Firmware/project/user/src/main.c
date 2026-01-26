/*********************************************************************************************************************
 * MM32F327X-G8P Opensourec Library 即（MM32F327X-G8P 开源库）是一个基于官方 SDK 接口的第三方开源库
 * Copyright (c) 2022 SEEKFREE 逐飞科技
 *
 * 本文件是 MM32F327X-G8P 开源库的一部分
 *
 * MM32F327X-G8P 开源库 是免费软件
 * 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
 * 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
 *
 * 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
 * 甚至没有隐含的适销性或适合特定用途的保证
 * 更多细节请参见 GPL
 *
 * 您应该在收到本开源库的同时收到一份 GPL 的副本
 * 如果没有，请参阅<https://www.gnu.org/licenses/>
 *
 * 额外注明：
 * 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
 * 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
 * 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
 * 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
 *
 * 文件名称          main
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          IAR 8.32.4 or MDK 5.37
 * 适用平台          MM32F327X_G8P
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2022-08-10        Teternal            first version
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库移植用空工程

// **************************** 代码区域 ****************************
// *************************用户自定义包含****************************
#include "bsp_led.h"
#include "bsp_buzzer.h"
#include "bsp_oled.h"
#include "bsp_imu.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_battery.h"
#include "bsp_bluetooth.h"

#include "kalman.h"

#include "Menu.h"

#include "test.h"
#include "param_config.h"
#include "Simple_Timewheel.h"
// *************************用户自定义包含****************************

// 函数声明
static void OLED_Start(void);
// 函数声明

// DEBUG

// DEBUG

// Timewheel
extern volatile uint8_t tick_flag; // 时间轮滴答标志
// Timewheel
int main(void)
{
    clock_init(SYSTEM_CLOCK_120M); // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                  // 初始化默认 Debug UART
    // TIM 与 Encoder
    pit_ms_init(TIME_TIM, 1); // 使用TIM6进行按键扫描
    interrupt_set_priority(TIME_PRIORITY, 0);
    // TIM 与 Encoder

    // 数据初始化
    Param_Init();
    // 数据初始化

    // 电机
    motor_init();
    encoder_init();
    // 电机

    // 此处编写用户代码 例如外设初始化代码等
    key_init(10);
    LED_Init();
    Buzzer_Init();
    battery_init();
    OLED_Init();
    Menu_Init();
    Attitude_Init();
    bluetooth_init();

    // 此处编写用户代码 例如外设初始化代码等

    // 自检
    LED_On(ALL);
    //Buzzer_On();
    OLED_Start();
    if (ICM42688_I2C_Init() != 0)
    {
        OLED_Clear();
        OLED_ShowString(0, 0, "IMU ERROR!", OLED_8X16);
        OLED_Update();
        system_delay_ms(1000);
    }
    else
    {
        OLED_Clear();
        OLED_ShowString(0, 0, "IMU OK!", OLED_8X16);
        OLED_Update();
        system_delay_ms(1000);
    }
    LED_Off(ALL);
    Buzzer_Off();
    // 自检

    OLED_Clear();

    add_task(5, Attitude_Update);  // 每5ms更新姿态
    add_task(30, Menu_Process);    // 每30ms处理菜单
    add_task(5, motor_update);    // 每5ms更新电机控制
    add_task(100, battery_update); // 每100ms更新电池电压
    add_task(20, encoder_update);  // 每20ms更新编码器读取
    add_task(5, ICM_Update);       // 每5ms读取IMU数据
    add_task(30, key_scanner);     // 每25ms按键扫描
    add_task(10, bluetooth_update); // 每10ms更新蓝牙数据

    while (1)
    {
        // Debug
        // OLED_Clear();
        // test_key();
        // Debug
        if (tick_flag)
        {
            time_wheel_run();
            tick_flag = 0;
        }
    }
}
// **************************** 代码区域 ****************************
static void OLED_Start(void)
{
    OLED_ShowString(0, 0, "---------------------", OLED_6X8);
    OLED_ShowString(0, 24, "---------------------", OLED_6X8);
    OLED_ShowString(0, 6, "->SYSTEM START<-", OLED_8X16);
    OLED_ShowString(0, 32, "Team Member:", OLED_8X16);
    OLED_ShowString(0, 48, "aaa", OLED_6X8);
    OLED_ShowString(0, 56, "bbb", OLED_6X8);
    OLED_ShowString(90, 32, "11", OLED_8X16);
    OLED_Update();
}