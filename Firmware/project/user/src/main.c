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

#include "Menu.h"

#include "test.h"
// *************************用户自定义包含****************************
int main(void)
{
    clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 Debug UART
    
    // DEBUG
    int32_t debug_count = 0;
    // DEBUG

    // TIM 与 Encoder
    pit_ms_init(KEY_TIM,30);
    interrupt_set_priority(KEY_PRIORITY, 0);
    // TIM 与 Encoder

    // 此处编写用户代码 例如外 设初始化代码等
    key_init(10);
    LED_Init();
    Buzzer_Init();
    OLED_Init();
    Menu_Init();

    // 此处编写用户代码 例如外设初始化代码等

    // 自检
    LED_On(ALL);
    //Buzzer_On();

    OLED_ShowString(0,0, "---------------------", OLED_6X8);
	OLED_ShowString(0,24, "---------------------", OLED_6X8);
    OLED_ShowString(0,6, "->SYSTEM START<-", OLED_8X16);
	OLED_ShowString(0,32,"Team Member:",OLED_8X16);
    OLED_ShowString(0,48,"aaa",OLED_6X8);
    OLED_ShowString(0,56,"bbb",OLED_6X8);
    OLED_ShowString(90,32,"11",OLED_8X16);
    OLED_Update();

    system_delay_ms(1000);
    LED_Off(ALL);
    Buzzer_Off();
    // 自检

    OLED_Clear();

    while(1)
    {
        // Debug
        
        debug_count++;
        //test_key();

        //Debug
        // 此处编写需要循环执行的代码
        Menu_Process();
        // 此处编写需要循环执行的代码
    }
}
// **************************** 代码区域 ****************************
