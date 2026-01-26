#ifndef BSP_BLUETOOTH_H
#define BSP_BLUETOOTH_H

void bluetooth_init(void);
void Bluetooth_Command_Process(void);
void Bluetooth_Rx_Handler(uint8_t data);

#endif