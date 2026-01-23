#ifndef BSP_LED_H
#define BSP_LED_H

#include "zf_common_headfile.h"

void LED_Init(void);
void LED_On(LED_LIST led);
void LED_Off(LED_LIST led);

#endif