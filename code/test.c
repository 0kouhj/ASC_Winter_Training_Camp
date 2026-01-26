#include "zf_common_headfile.h"
#include "bsp_oled.h"
#include "zf_device_key.h"

void test_key(void)
{
    OLED_ShowString(0,0,"KEY1",OLED_8X16);
    OLED_ShowString(0,16,"KEY2",OLED_8X16);
    OLED_ShowString(0,32,"KEY3",OLED_8X16);
    if(key_get_state(KEY_1) == KEY_SHORT_PRESS){
        OLED_ShowString(40,0,"y",OLED_8X16);
        
        OLED_Update();
    }
    if(key_get_state(KEY_2) == KEY_SHORT_PRESS){
        OLED_ShowString(40,16,"y",OLED_8X16);
        OLED_Update();
    }
    if(key_get_state(KEY_3) == KEY_SHORT_PRESS){
        OLED_ShowString(40,32,"y",OLED_8X16);
        OLED_Update();
    }
    if(key_get_state(KEY_4) == KEY_SHORT_PRESS){
        OLED_Clear();
    }


    OLED_Update();
}