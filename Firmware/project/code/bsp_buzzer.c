#include "zf_common_headfile.h"

void Buzzer_Init(void)
{
    gpio_init(BUZZER,GPO,GPIO_LOW,GPO_PUSH_PULL);
}

void Buzzer_On(void)
{
    gpio_set_level(BUZZER,1);
}

void Buzzer_Off(void)
{
    gpio_set_level(BUZZER,0);
}