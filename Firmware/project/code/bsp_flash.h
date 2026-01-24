#ifndef BSP_FLASH_H
#define BSP_FLASH_H
#include "zf_common_headfile.h"
void flash_read_params_from_flash(void);
void flash_write_params_to_flash(void);
void flash_erase_params_flash_sector(void);
#endif