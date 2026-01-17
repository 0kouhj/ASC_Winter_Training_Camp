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


// *************************** 例程硬件连接说明 ***************************
// 使用逐飞科技 CMSIS-DAP | ARM 调试下载器连接
//      直接将下载器正确连接在核心板的调试下载接口即可
// 
// 使用 USB-TTL 模块连接
//      模块管脚            单片机管脚
//      USB-TTL-RX          查看 zf_common_debug.h 文件中 DEBUG_UART_TX_PIN 宏定义的引脚 默认 A9
//      USB-TTL-TX          查看 zf_common_debug.h 文件中 DEBUG_UART_RX_PIN 宏定义的引脚 默认 A10
//      USB-TTL-GND         核心板电源地 GND
//      USB-TTL-3V3         核心板 3V3 电源


// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程，完成上电
// 
// 2.接好串口，插上SD卡
// 
// 3.按下复位按键，可以从串口看到对应输出信息
// 
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查


// **************************** 代码区域 ****************************
uint8 get_char;
uint8 data_seed = 0xA5;

uint32 read_index = 16;
uint32 write_index = 16;

uint8 read_buffer[SDCARD_BLOCK_SIZE * 4];
uint8 write_buffer[SDCARD_BLOCK_SIZE * 4];

sdcard_error_code_enum return_state = SDCARD_ERROR_STATE_NO_ERROR;

int main (void)
{
    clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 Debug UART

    // 此处编写用户代码 例如外设初始化代码等
    uint32 i = 0u;

    memset(write_buffer, data_seed, SDCARD_BLOCK_SIZE * 4);
    printf("\r\n sdcard_init() start.");
    if(SDCARD_ERROR_STATE_NO_ERROR !=  sdcard_init())
    {
        zf_log(0, "\r\n sdcard_init() error.");
        while(1);
    }
    printf("\r\n sdcard_init() done.");

    printf("\r\n sdcard_info.");
    switch(sdcard_info.card_type)
    {
        case SDCARD_TYPE_SDSC:  printf("\r\n sdcard_info->card_type : SDSC.");  break;
        case SDCARD_TYPE_SDHC:  printf("\r\n sdcard_info->card_type : SDHC.");  break;
        case SDCARD_TYPE_SDXC:  printf("\r\n sdcard_info->card_type : SDXC.");  break;
    }
    printf("\r\n sdcard_info->block_count : %d.", sdcard_info.block_count);
    printf("\r\n sdcard_info->block_len : %d.", sdcard_info.block_len);
    printf("\r\n sdcard_info->rca : %x.", sdcard_info.rca);

    printf("\r\n sdcard test command :");
    printf("\r\n Enter '1' : at read_index read 1 block data.");
    printf("\r\n Enter '2' : at read_index read 4 block data.");
    printf("\r\n Enter '3' : at write_index write 1 block data, data = data_seed.");
    printf("\r\n Enter '4' : at write_index write 4 block data, data = data_seed.");
    printf("\r\n Enter 'q' : read_index add 1.");
    printf("\r\n Enter 'e' : read_index minus 1.");
    printf("\r\n Enter 'r' : write_index 1 add data.");
    printf("\r\n Enter 'f' : write_index 4 minus data.");
    printf("\r\n Enter 'w' : data_seed 1 add data.");
    printf("\r\n Enter 's' : data_seed 4 minus data.");
    printf("\r\n ");
    // 此处编写用户代码 例如外设初始化代码等

    while(1)
    {
        // 此处编写需要循环执行的代码
        get_char = uart_read_byte(DEBUG_UART_INDEX);
        
        switch(get_char)
        {
            case '1':
            {
                printf("\r\n read 1 block num: %u\r\n", read_index);
                return_state = sdcard_read_blocks(read_index, 1, read_buffer);
                if(SDCARD_ERROR_STATE_NO_ERROR ==  return_state)
                {
                    for(i = 0u; SDCARD_BLOCK_SIZE > i; i ++)
                    {
                        if((i % 32) == 0u)
                        {
                            printf("\r\n");
                        }
                        printf("0x%2X, ", read_buffer[i]);
                    }
                    read_index ++;
                }
                else
                {
                    zf_log(0, "\r\n sdcard_read_blocks() error.");
                    while(1);
                }
            }break;
            case '2':
            {
                printf("\r\n read 4 blocks num: %u-%u\r\n", read_index, read_index + 3);
                return_state = sdcard_read_blocks(read_index, 4, read_buffer);
                if(SDCARD_ERROR_STATE_NO_ERROR ==  return_state)
                {
                    for(i = 0u; SDCARD_BLOCK_SIZE * 4 > i; i ++)
                    {
                        if((i % 32) == 0u)
                        {
                            printf("\r\n");
                        }
                        printf("0x%2X, ", read_buffer[i]);
                    }
                    read_index += 4;
                }
                else
                {
                    zf_log(0, "\r\n sdcard_read_blocks() error.");
                    while(1);
                }
            }break;

            case '3':
            {
                printf("\r\n write 1 block num: %u", write_index);
                return_state = sdcard_write_blocks(write_index, 1, write_buffer);
                if(SDCARD_ERROR_STATE_NO_ERROR ==  return_state)
                {
                    printf("\r\n write 1 blocks num success");
                    write_index ++;
                }
                else
                {
                    zf_log(0, "\r\n sdcard_write_blocks() error.");
                    while(1);
                }
            }break;
            case '4':
            {
                printf("\r\n write 4 blocks num: %u-%u", write_index, write_index + 3);
                return_state = sdcard_write_blocks(write_index, 4, write_buffer);
                if(SDCARD_ERROR_STATE_NO_ERROR ==  return_state)
                {
                    printf("\r\n write 4 blocks num success");
                    write_index += 4;
                }
                else
                {
                    zf_log(0, "\r\n sdcard_write_blocks() error.");
                    while(1);
                }
            }break;

            case 'q':
            {
                read_index ++;
                printf("\r\n read_index: %u", read_index);
            }break;
            case 'e':
            {
                read_index --;
                printf("\r\n read_index: %u", read_index);
            }break;
            case 'r':
            {
                write_index ++;
                printf("\r\n write_index: %u", write_index);
            }break;
            case 'f':
            {
                write_index --;
                printf("\r\n write_index: %u", write_index);
                break;
            case 'w':
            {
                data_seed ++;
                memset(write_buffer, data_seed, SDCARD_BLOCK_SIZE * 4);
                printf("\r\n data_seed: %u", data_seed);
            }break;
            case 's':
            {
                data_seed --;
                memset(write_buffer, data_seed, SDCARD_BLOCK_SIZE * 4);
                printf("\r\n data_seed: %u", data_seed);
            }break;

            default:
            {
                printf("\r\n sdcard test command :");
                printf("\r\n Enter '1' : at read_index read 1 block data.");
                printf("\r\n Enter '2' : at read_index read 4 block data.");
                printf("\r\n Enter '3' : at write_index write 1 block data, data = data_seed.");
                printf("\r\n Enter '4' : at write_index write 4 block data, data = data_seed.");
                printf("\r\n Enter 'q' : read_index add 1.");
                printf("\r\n Enter 'e' : read_index minus 1.");
                printf("\r\n Enter 'r' : write_index 1 add data.");
                printf("\r\n Enter 'f' : write_index 4 minus data.");
                printf("\r\n Enter 'w' : data_seed 1 add data.");
                printf("\r\n Enter 's' : data_seed 4 minus data.");
                printf("\r\n ");
            }break;
        }
        // 此处编写需要循环执行的代码
    }
}
// **************************** 代码区域 ****************************

// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 
// 问题1：串口没有数据
//      查看串口助手打开的是否是正确的串口，检查打开的 COM 口是否对应的是调试下载器或者 USB-TTL 模块的 COM 口
//      如果是使用逐飞科技 CMSIS-DAP | ARM 调试下载器连接，那么检查下载器线是否松动，检查核心板串口跳线是否已经焊接，串口跳线查看核心板原理图即可找到
//      如果是使用 USB-TTL 模块连接，那么检查连线是否正常是否松动，模块 TX 是否连接的核心板的 RX，模块 RX 是否连接的核心板的 TX
// 
// 问题2：串口数据乱码
//      查看串口助手设置的波特率是否与程序设置一致，程序中 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义为 debug uart 使用的串口波特率
