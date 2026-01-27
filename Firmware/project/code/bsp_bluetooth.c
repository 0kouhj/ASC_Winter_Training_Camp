#include "zf_common_headfile.h"
#include "bsp_motor.h"

#define BT_RX_BUF_SIZE 64
// 创建缓冲区
static uint8_t bt_rx_buffer[BT_RX_BUF_SIZE];
static uint8_t bt_command_ready = 0;
static uint8_t bt_rx_index = 0;

uint8_t fifo_rx_buffer[64];
fifo_struct fifo_rx;

void bluetooth_init(void)
{
    // 蓝牙初始化代码
    fifo_init(&fifo_rx, FIFO_DATA_8BIT, fifo_rx_buffer, 64);

    uart_init(BLUETOOTH_UART_INDEX, BLUETOOTH_BAUD, BLUETOOTH_TX, BLUETOOTH_RX);
    uart_rx_interrupt(BLUETOOTH_UART_INDEX, ZF_ENABLE);
    interrupt_set_priority(BLUETOOTH_UART_PRIORITY, 1);

    bt_rx_index = 0;
    bt_command_ready = 0;
    memset(bt_rx_buffer, 0, BT_RX_BUF_SIZE);

    uart_write_string(BLUETOOTH_UART_INDEX, "Bluetooth Initialized\r\n");
}

void Bluetooth_Rx_Handler(uint8_t data)
{
    static uint8_t rx_state = 0; // 0:等待起始符 [, 1:接收内容

    if (rx_state == 0)
    {
        if (data == '[')
        {
            rx_state = 1;
            bt_rx_index = 0;
            bt_rx_buffer[bt_rx_index++] = data;
        }
    }
    else if (rx_state == 1)
    {
        if (bt_rx_index < BT_RX_BUF_SIZE - 1)
        {
            bt_rx_buffer[bt_rx_index++] = data;
            if (data == ']')
            {
                bt_rx_buffer[bt_rx_index] = '\0'; // 封口
                bt_command_ready = 1;
                rx_state = 0;
            }
        }
        else
        {
            rx_state = 0; // 溢出重置
        }
    }
}

void Bluetooth_Command_Process(void)
{
    if (!bt_command_ready)
        return;

    char *cmd = (char *)bt_rx_buffer;
    char response[128];

    // 参照 Communication.c 的分发逻辑
    if (strncmp(cmd, "[joystick", 9) == 0)
    {
        // 解析摇杆: [joystick,x,y,0,0]
        int x, y, d1, d2;
        if (sscanf(cmd, "[joystick,%d,%d,%d,%d]", &x, &y, &d1, &d2) == 4)
        {
            // 这里调用你的电机控制逻辑
            // Calculate_Differential_Speed(x, y);
            uart_write_string(BLUETOOTH_UART_INDEX, "[ACK JOYSTICK]\r\n");
        }
    }
    else if (strcmp(cmd, "[stop]") == 0)
    {
        motor_stop();
        uart_write_string(BLUETOOTH_UART_INDEX, "[OK STOP]\r\n");
    }
    else if (sscanf(cmd, "[motor]") == 0)
    {
        int x,y;
        if (sscanf(cmd, "[motor,%d,%d]", &x, &y) == 2)
        {
            switch (x)
            {
                case 0:
                    motor_stop();
                    break;
                case 1:
                    State.motor_target_speed_left = y;
                    break;
                case 2:
                    State.motor_target_speed_right = y;
                    break;
                case 3:
                    State.motor_target_speed_left = y;
                    State.motor_target_speed_right = y;
                    break;
                default:
                    break;
            }   
        }
        sprintf(response, "[motor %d %d]\r\n", x, y);
        uart_write_string(BLUETOOTH_UART_INDEX, response);
    }
    else if (strcmp(cmd, "[reset]") == 0)
    {
        Param_Init();
        uart_write_string(BLUETOOTH_UART_INDEX, "[OK RESET]\r\n");
    }
    else
    {
        sprintf(response, "[ERROR UNKNOWN: %s]\r\n", cmd);
        uart_write_string(BLUETOOTH_UART_INDEX, response);
    }

    bt_command_ready = 0; // 处理完毕，清除标志
    memset(bt_rx_buffer, 0, BT_RX_BUF_SIZE);
}