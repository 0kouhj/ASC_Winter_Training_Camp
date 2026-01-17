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
uint8 sdcard_write_buffer[SDCARD_BLOCK_SIZE];
uint8 sdcard_read_buffer[SDCARD_BLOCK_SIZE];

int main (void)
{
    clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 Debug UART

    // 此处编写用户代码 例如外设初始化代码等

    char ch = '0';
    volatile bool failedFlag = false;

    UINT bytesWritten;
    UINT bytesRead;

    FATFS fs;                                                                   // 文件系统结构体
    const TCHAR fs_drv[] = "1:/";                                               // 文件系统根目录 默认是两个部分 SD 卡在 1 部分

    FRESULT error;                                                              // 文件系统操作状态
    DIR directory;                                                              // 目录结构体变量
    static FIL g_fileObject;                                                    // 文件结构体变量
    FILINFO fileInformation;                                                    // 文件信息结构体

    printf("\r\n Log -> fatfs_mkfs_test() example.");
    printf("\r\n Log -> f_mount() start.");
    while(1)
    {
        if(!f_mount(&fs, fs_drv ,1))                                            // 挂载文件系统
        {
            break;
        }
        printf("\r\n Log -> f_mount() error, try again after one second.");     // 如果失败了就输出错误信息
        system_delay_ms(1000);                                                  // 延时一秒后再试
    }
    printf("\r\n Log -> f_mount() success.");

#if (FF_FS_RPATH >= 2U)                                                         // 默认两个部分
    error = f_chdrive((char const *)&fs_drv[0U]);                               // 切换目录
    if(error)
    {
        printf("\r\n ERROR -> Failed to change drive.");
        while(1);
    }
#endif

    printf("\r\n Log -> Creating a directory.");
    error = f_mkdir("/dir_1");                                                  // 创建目录 dir_1
    if(error)
    {
        if(FR_EXIST == error)
        {
            printf("\r\n Log -> Directory already exist.");
        }
        else
        {
            printf("\r\n ERROR -> Directory creation failed.");
            while(1);
        }
    }

    printf("\r\n Log -> Create a file in the current directory.");
    error = f_open(&g_fileObject, "/dir_1/f_1.dat", (FA_WRITE | FA_READ | FA_CREATE_ALWAYS));   // 读写权限创建文件
    if(error)
    {
        if(FR_EXIST == error)
        {
            printf("\r\n Warning -> File already exist.");                      // 文件已存在 直接打开
        }
        else
        {
            printf("\r\n ERROR -> File creation failed.");
            while(1);
        }
    }

    printf("\r\n Log -> In the current directory, create another directory.");
    error = f_mkdir("/dir_1/dir_2");                                            // 创建目录 dir_2
    if(error)
    {
        if(FR_EXIST == error)
        {
            printf("\r\n Warning -> Directory already exist.");
        }
        else
        {
            printf("\r\n ERROR -> Directory creation failed.");
            while(1);
        }
    }

    printf("\r\n Log -> Lists the files in this directory.");
    if(f_opendir(&directory, "/dir_1"))                                         // 返回 dir_1 目录
    {
        printf("\r\n ERROR -> Directory opening failure.");
        while(1);
    }

    for(;;)
    {
        error = f_readdir(&directory, &fileInformation);                        // 读取当前的目录信息

        /* To the end. */
        if((FR_OK != error) || (fileInformation.fname[0U] == 0U))
        {
            break;
        }
        if('.' == fileInformation.fname[0])
        {
            continue;
        }
        if(fileInformation.fattrib & AM_DIR)
        {
            printf("\r\n Log -> Directory: %s.", fileInformation.fname);
        }
        else
        {
            printf("\r\n Log -> File     : %s.", fileInformation.fname);
        }
    }

    memset(sdcard_write_buffer, 'a', sizeof(sdcard_write_buffer));              // 填充写入 buffer
    sdcard_write_buffer[SDCARD_BLOCK_SIZE - 2U] = '\r';
    sdcard_write_buffer[SDCARD_BLOCK_SIZE - 1U] = '\n';

    printf("\r\n Log -> Write/read the file until an error is encountered.");
    while(true)
    {
        if(failedFlag || ('q' == ch))
        {
            break;
        }

        printf("\r\n Log -> Write data to the file created above.");
        error = f_write(&g_fileObject, sdcard_write_buffer, sizeof(sdcard_write_buffer), &bytesWritten);    // 写入缓冲区数据到文件中
        if((error) || (bytesWritten != sizeof(sdcard_write_buffer)))            // 检索是否出错 写入大小是否一致
        {
            printf("\r\n Warning -> Failed to write file.");
            failedFlag = true;
            continue;
        }

        /* Move the file pointer */
        if(f_lseek(&g_fileObject, 0U))                                          // 文件同步 将指针指向文件头部
        {
            printf("\r\n Warning -> Failed to set the file pointer position.");
            failedFlag = true;
            continue;
        }

        printf("\r\n Log -> Read the file created above.");
        memset(sdcard_read_buffer, 0U, sizeof(sdcard_read_buffer));
        error = f_read(&g_fileObject, sdcard_read_buffer, sizeof(sdcard_read_buffer), &bytesRead);          // 从文件中读取出数据
        if((error) || (bytesRead != sizeof(sdcard_read_buffer)))                // 确认是否出错 读取大小是否跟预期大小一致
        {
            printf("\r\n Warning -> Failed to read file.");
            failedFlag = true;
            continue;
        }

        printf("\r\n Log -> Start comparing write and read data for consistency.");
        if(memcmp(sdcard_write_buffer, sdcard_read_buffer, sizeof(sdcard_write_buffer)))
        {
            printf("\r\n Warning -> The write and read data are inconsistent.");
            failedFlag = true;
            continue;
        }
        printf("\r\n Log -> Write and read data are consistent.");

        printf("\r\n Log -> Enter q to exit the program and enter other repeat writes and reads.");
        printf("\r\n Get -> ");
        ch = uart_read_byte(DEBUG_UART_INDEX);
        uart_write_byte(DEBUG_UART_INDEX, ch);
    }
    printf("\r\n Log -> The sample program does not read any more.");

    if(f_close(&g_fileObject))                                                  // 关闭文件
    {
        printf("\r\n ERROR -> Failed to close file.");
    }
    // 此处编写用户代码 例如外设初始化代码等

    while(1)
    {
        // 此处编写需要循环执行的代码
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
