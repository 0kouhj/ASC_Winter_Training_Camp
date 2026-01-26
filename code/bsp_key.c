#include "zf_common_headfile.h"
#include "bsp_key.h"

#define KEY_PRESSED             1
#define KEY_UNPRESSED           0

#define KEY_TIME_DOUBLE         10
#define KEY_TIME_LONG           50
#define KEY_TIME_REPEAT         10
#define KEY_COUNT               6

// 按键引脚映射表
static const gpio_pin_enum key_pins[KEY_COUNT] = {E5, E2, E3, E4, D3, D4};

uint8_t Key_Flag[KEY_COUNT];

// 按键初始化
void KEY_Init(void)
{
    for (uint8_t i = 0; i < KEY_COUNT; i++)
    {
        // MM32 逐飞库初始化：输入模式，默认高电平，上拉
        gpio_init(key_pins[i], GPI, GPIO_HIGH, GPI_PULL_UP);
    }
}

// 获取物理引脚状态
uint8_t Key_GetState(uint8_t n)
{
    if (n >= KEY_COUNT) return KEY_UNPRESSED;

    if (gpio_get_level(key_pins[n]) == 0)
    {
        return KEY_PRESSED;
    }
    
    return KEY_UNPRESSED;
}

// 检查按键逻辑状态（供应用层调用）
uint8_t Key_Check(uint8_t n, uint8_t Flag)
{
    if (Key_Flag[n] & Flag)
    {
        if (Flag != KEY_HOLD)
        {
            Key_Flag[n] &= ~Flag;
        }
        return 1;
    }
    return 0;
}

// 按键扫描核心算法（放置在 10ms 或 1ms 中断中）
void Key_Tick(void)
{
    static uint8_t Count = 0;
    static uint8_t CurrState[KEY_COUNT], PrevState[KEY_COUNT];
    static uint8_t S[KEY_COUNT];
    static uint16_t Time[KEY_COUNT];
    
    // 递减时间计数器
    for (uint8_t i = 0; i < KEY_COUNT; i++)
    {
        if (Time[i] > 0) Time[i]--;
    }
    
    // 5次 Tick 执行一次逻辑判断（如果是 1ms 调用一次，则此处为 5ms 周期）
    Count++;
    if (Count >= 5)
    {
        Count = 0;
        
        for (uint8_t i = 0; i < KEY_COUNT; i++)
        {
            PrevState[i] = CurrState[i];
            CurrState[i] = Key_GetState(i);
            
            // 基础状态更新
            if (CurrState[i] == KEY_PRESSED) Key_Flag[i] |= KEY_HOLD;
            else                             Key_Flag[i] &= ~KEY_HOLD;
            
            if (CurrState[i] == KEY_PRESSED && PrevState[i] == KEY_UNPRESSED) Key_Flag[i] |= KEY_DOWN;
            if (CurrState[i] == KEY_UNPRESSED && PrevState[i] == KEY_PRESSED) Key_Flag[i] |= KEY_UP;
            
            // 状态机判断：单击、双击、长按
            if (S[i] == 0) {
                if (CurrState[i] == KEY_PRESSED) { Time[i] = KEY_TIME_LONG; S[i] = 1; }
            }
            else if (S[i] == 1) {
                if (CurrState[i] == KEY_UNPRESSED) { Time[i] = KEY_TIME_DOUBLE; S[i] = 2; }
                else if (Time[i] == 0) { Time[i] = KEY_TIME_REPEAT; Key_Flag[i] |= KEY_LONG; S[i] = 4; }
            }
            else if (S[i] == 2) {
                if (CurrState[i] == KEY_PRESSED) { Key_Flag[i] |= KEY_DOUBLE; S[i] = 3; }
                else if (Time[i] == 0) { Key_Flag[i] |= KEY_SINGLE; S[i] = 0; }
            }
            else if (S[i] == 3) {
                if (CurrState[i] == KEY_UNPRESSED) S[i] = 0;
            }
            else if (S[i] == 4) {
                if (CurrState[i] == KEY_UNPRESSED) S[i] = 0;
                else if (Time[i] == 0) { Time[i] = KEY_TIME_REPEAT; Key_Flag[i] |= KEY_REPEAT; S[i] = 4; }
            }
        }
    }
}
