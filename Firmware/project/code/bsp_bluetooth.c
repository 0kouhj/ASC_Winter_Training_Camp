#include "zf_common_headfile.h"

// 创建缓冲区
uint8_t bluetooth_rx_buffer[64];
uint8_t bluetooth_tx_buffer[64];

uint8_t fifo_rx_buffer[64];
uint8_t fifo_tx_buffer[64];

// 创建标质量
uint8_t get_data = 0;
uint32_t fifo_data_count = 0;

fifo_struct fifo_rx;
fifo_struct fifo_tx;

void bluetooth_init(void)
{
    // 蓝牙初始化代码
    fifo_init(&fifo_rx, FIFO_DATA_8BIT, fifo_rx_buffer, 64);
    fifo_init(&fifo_tx, FIFO_DATA_8BIT, fifo_tx_buffer, 64);

    uart_init(BLUETOOTH_UART_INDEX,BLUETOOTH_BAUD,BLUETOOTH_TX,BLUETOOTH_RX);
    uart_rx_interrupt(BLUETOOTH_UART_INDEX,ZF_ENABLE);
    interrupt_set_priority(BLUETOOTH_UART_PRIORITY,1);

    uart_write_string(BLUETOOTH_UART_INDEX,"Bluetooth Initialized\r\n");
    uart_write_byte(BLUETOOTH_UART_INDEX,'\r');
    uart_write_byte(BLUETOOTH_UART_INDEX,'\n');
}

void bluetooth_update(void)
{
    fifo_data_count = fifo_used(&fifo_rx);
    if(fifo_data_count > 0){
        fifo_read_buffer(&fifo_rx,fifo_rx_buffer,&fifo_data_count,FIFO_READ_AND_CLEAN);
        uart_write_string(BLUETOOTH_UART_INDEX, "Received Data:");
        uart_write_buffer(BLUETOOTH_UART_INDEX, fifo_rx_buffer, fifo_data_count);
        uart_write_byte(BLUETOOTH_UART_INDEX,'\r');
        uart_write_byte(BLUETOOTH_UART_INDEX,'\n');
    }
}

void uart_rx_interrupt_handler(void)
{
    uart_query_byte(BLUETOOTH_UART_INDEX, &get_data);
    fifo_write_buffer(&fifo_rx, &get_data, 1);
}