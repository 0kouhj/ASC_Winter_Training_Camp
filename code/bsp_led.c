#include "zf_common_headfile.h"

void LED_Init(void)
{
    gpio_init(LED_1,GPO,GPIO_LOW,GPO_PUSH_PULL);
    gpio_init(LED_2,GPO,GPIO_LOW,GPO_PUSH_PULL);
}

void LED_On(LED_LIST led)
{
    switch(led){
        case LED1:
            gpio_set_level(LED_1,0);
            break;
        case LED2:
            gpio_set_level(LED_2,0);
            break;
        default:
            gpio_set_level(LED_1,0);
            gpio_set_level(LED_2,0);
    }
}

void LED_Off(LED_LIST led)
{
    switch(led){
        case LED1:
            gpio_set_level(LED_1,1);
            break;
        case LED2:
            gpio_set_level(LED_2,1);
            break;
        default:
            gpio_set_level(LED_1,1);
            gpio_set_level(LED_2,1);
    }
}