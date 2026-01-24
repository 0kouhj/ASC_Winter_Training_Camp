#ifndef BSP_BATTERY_H
#define BSP_BATTERY_H

#include "zf_common_headfile.h"

void battery_init(void);
float battery_get_voltage(void);

#endif