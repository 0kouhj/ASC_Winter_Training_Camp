#include "menu.h"
#include "bsp_oled.h"
#include "zf_device_key.h"
#include <string.h>
#include <stdlib.h>

// 实时显示状态
static RealtimeDisplayState realtime_state = {0};
static uint8_t in_realtime_mode = 0;  // 是否处于实时显示模式

// 当前显示的菜单
static Menu *current_menu = NULL;
uint8_t need_refresh = 1;  // 需要刷新显示标志

// 警告模式状态
static uint8_t warning_mode = 0;  // 是否处于警告模式
static uint32_t warning_start_time = 0;  // 警告开始时间

// 显示配置
#define DISPLAY_LINES 6           // 显示总行数
#define MENU_ITEMS_PER_PAGE 5     // 每页显示的菜单项数量
#define REAL_TIME_PAGE 2
//辅助函数
static void IntToStr(uint16_t num, char *str)
{
    char temp[6];
    uint8_t i = 0, j = 0;
    // 处理0的情况
    if (num == 0) {
        str[0] = '0';
        str[1] = '\0';
        return;
    }
    // 提取各位数字
    while (num > 0) {
        temp[i++] = '0' + (num % 10);
        num /= 10;
    }
    // 反转字符串
    while (i > 0) {
        str[j++] = temp[--i];
    }
    str[j] = '\0';
}

// 新增实时参数显示函数
void Menu_DisplayRealtimeParams(void)
{
    OLED_Clear();
    
    // 显示标题
    OLED_ShowString(0, 0, "Debug", OLED_8X16);
    
    // 显示电池信息
    Menu_DisplayBatteryInfo();
    
    // 分割线
    OLED_ShowString(0, 16, "-----------------------------", OLED_6X8);
    
    uint8_t line = 3;  // 从第3行开始显示
    
    // 提前声明所有局部变量
    char bat_str[16];
    char ir_str[4];
    char pid_str[8];
    char page_str[4];
    uint16_t volt_int, volt_frac;
    
    // 根据当前页面显示不同参数
    switch(realtime_state.current_page) {
        case 0:  // 第1页：电池和电机状态
        {
            // 电池状态
            OLED_ShowString(0, line*8, "Bat:", OLED_6X8);
            //IntToStr(system_status_packet.battery_level, bat_str);
            OLED_ShowString(24, line*8, bat_str, OLED_6X8);
            // 显示电压（整数部分.小数部分）
            //volt_int = (uint16_t)system_status_packet.battery_voltage;
            //volt_frac = (uint16_t)((system_status_packet.battery_voltage - volt_int) * 100);
            IntToStr(volt_int, bat_str);
            OLED_ShowString(54, line*8, bat_str, OLED_6X8);
            OLED_ShowString(66, line*8, ".", OLED_6X8);
            IntToStr(volt_frac, bat_str);
            OLED_ShowString(72, line*8, bat_str, OLED_6X8);
            OLED_ShowString(78, line*8, "V)", OLED_6X8);
            line++;
            // 左电机目标速度
            OLED_ShowString(0, line*8, "L-Target:", OLED_6X8);
            //OLED_ShowSignedNum(54, line*8,motor_speed_data.left_target_speed,3,OLED_6X8);
            line++;
            // 左电机实际速度
            OLED_ShowString(0, line*8, "L-Actual:", OLED_6X8);
            //OLED_ShowSignedNum(54, line*8,motor_speed_data.left_actual_speed,3,OLED_6X8);
            line++;
            // 右电机目标速度
            OLED_ShowString(0, line*8, "R-Target:", OLED_6X8);
            //OLED_ShowSignedNum(54, line*8,motor_speed_data.right_target_speed,3,OLED_6X8);
            line++;
            // 右电机实际速度
            OLED_ShowString(0, line*8, "R-Target:", OLED_6X8);
            //OLED_ShowSignedNum(54, line*8,motor_speed_data.right_actual_speed,3,OLED_6X8);
            line++;
            // 系统控制模式
            OLED_ShowString(0, line*8, "Mode:", OLED_6X8);
            /*switch(system_status_packet.system_control_mode) {
                case MODE_MANUAL: OLED_ShowString(30, line*8, "Manual", OLED_6X8); break;
                case MODE_AUTO: OLED_ShowString(30, line*8, "Auto", OLED_6X8); break;
                case MODE_BLUETOOTH: OLED_ShowString(30, line*8, "BT", OLED_6X8); break;
                default: OLED_ShowString(30, line*8, "Unknown", OLED_6X8); break;
            }
            line++;*/
            break;
        }
            
        case 1:  // 第2页：传感器数据
        {
            // 红外传感器
            OLED_ShowString(0, line*8, "o", OLED_6X8);
            for(int i = 0; i < 4; i++) {
                //IntToStr(sensor_packet.ir_sensor_raw[i], ir_str);
                OLED_ShowString(18 + i*24, line*8, ir_str, OLED_6X8);
				if(i==1) OLED_ShowString(30,line*8,"L",OLED_6X8);
				if(i==2) OLED_ShowString(54,line*8,"R",OLED_6X8);
				if(i==3) OLED_ShowString(78,line*8,"V",OLED_6X8);
				
            }
            line++;
            break;
		}
    }
    
    // 显示页码指示器
    OLED_ShowString(110, 56, "P", OLED_6X8);
    IntToStr(realtime_state.current_page + 1, page_str);
    OLED_ShowString(116, 56, page_str, OLED_6X8);
    OLED_ShowString(122, 56, "/2", OLED_6X8);
    
    OLED_Update();
}

// 菜单初始化
void Menu_Init(void)
{
    // 创建菜单
    Menu *main_menu = Menu_Create("Main Menu");
    Menu *test_menu = Menu_Create("Test Mode");
    
    // 主菜单
    Menu_AddItem(main_menu, "DEBUG", MENU_ITEM_REALTIME_PARAMS, NULL, NULL);
    Menu_AddItem(main_menu, "Test", MENU_ITEM_SUBMENU, test_menu, NULL);
    Menu_AddItem(main_menu, "RESET", MENU_ITEM_SUBMENU, NULL, NULL);
    
	//Test菜单
	Menu_AddItem(test_menu, "Motor Test",MENU_ITEM_SUBMENU,NULL,NULL);
	
    current_menu = main_menu;
    need_refresh = 1;
    
    // 初始化实时显示状态
    realtime_state.current_page = 0;
    realtime_state.total_pages = REAL_TIME_PAGE;
    realtime_state.last_refresh = 0;
    in_realtime_mode = 0;
}

// 处理菜单逻辑（在主循环中调用）
void Menu_Process(void)
{
    
    if (current_menu == NULL) return;
    
    // 检查是否处于实时参数显示模式
    if (in_realtime_mode) {
        // 实时模式下的按键处理
        if (key_get_state(KEY_1) == KEY_SHORT_PRESS) {
            // 上键切换页面
            if (realtime_state.current_page > 0) {
                realtime_state.current_page--;
            }
        }
        else if (key_get_state(KEY_2) == KEY_SHORT_PRESS) {
            // 下键切换页面
            if (realtime_state.current_page < realtime_state.total_pages - 1) {
                realtime_state.current_page++;
            }
        }
        else if (key_get_state(KEY_4) == KEY_SHORT_PRESS) {
            in_realtime_mode = 0;
            need_refresh = 1;
            return;
        }
        Menu_DisplayRealtimeParams();
        return;
    }
    
    // 原有的菜单处理逻辑
    // 检查按键事件
    if (key_get_state(KEY_1) == KEY_SHORT_PRESS) {
        // 上移
        if (current_menu->current_item->prev) {
            current_menu->current_item = current_menu->current_item->prev;
            need_refresh = 1;
        }
    }
    else if (key_get_state(KEY_2) == KEY_SHORT_PRESS) {
        // 下移
        if (current_menu->current_item->next) {
            current_menu->current_item = current_menu->current_item->next;
            need_refresh = 1;
        }
    }
    else if (key_get_state(KEY_3) == KEY_SHORT_PRESS) {
        // 确认键
        switch (current_menu->current_item->type) {
            case MENU_ITEM_SUBMENU:
                if (current_menu->current_item->submenu) {
                    current_menu->current_item->submenu->parent = current_menu;
                    current_menu = current_menu->current_item->submenu;
                    need_refresh = 1;
                }
                break;
                
            case MENU_ITEM_BACK:
                if (current_menu->parent) {
                    current_menu = current_menu->parent;
                    need_refresh = 1;
                }
                break;
                
            case MENU_ITEM_ACTION:
                if (current_menu->current_item->action) {
                    current_menu->current_item->action();
                    need_refresh = 1;  // 动作执行后可能需要刷新
                }
                break;
                
            case MENU_ITEM_REALTIME_PARAMS:
                in_realtime_mode = 1;
                realtime_state.current_page = 0;
                need_refresh = 1;
                break;
                
            default:
                break;
        }
    }
    else if (key_get_state(KEY_4) == KEY_SHORT_PRESS) {
        // 返回键
        if (current_menu->parent) {
            current_menu = current_menu->parent;
            need_refresh = 1;
        }
    }
    
    // 如果需要刷新显示
    if (need_refresh) {
        Menu_RefreshDisplay();
        need_refresh = 0;
    }
}

// 显示电池信息和系统控制模式在右上角
void Menu_DisplayBatteryInfo(void)
{
    
}

// 刷新显示
void Menu_RefreshDisplay(void)
{
    if (current_menu == NULL) return;
    
    // 如果处于警告模式，显示警告页面
    if (warning_mode) {
        //Menu_DisplayWarningPage();
        return;
    }
    
    // 清屏
    OLED_Clear();
    
    // 显示菜单标题（第一行）
    OLED_ShowString(0, 0, current_menu->title, OLED_8X16);
    
    // 在右上角显示电池信息
    Menu_DisplayBatteryInfo();
    
    // ============ 添加字符分割线 ============
	// 在标题下方显示一行连续的横线字符
	OLED_ShowString(0, 16, "-----------------------------", OLED_6X8);
	// ============ 分割线结束 ============
    
    // 显示菜单项
    MenuItem *item = current_menu->items;
    uint8_t line = 3;  // 从第3行开始显示（因为标题占2行+分割线占1行）
    uint8_t displayed_count = 0;
    
    // 计算显示起始位置（实现滚动效果）
    uint8_t start_index = 0;
    MenuItem *temp = current_menu->items;
    uint8_t current_index = 0;
    
    // 找到当前选中项的位置
    while (temp && temp != current_menu->current_item) {
        temp = temp->next;
        current_index++;
    }
    
    // 计算起始显示位置
    if (current_index >= MENU_ITEMS_PER_PAGE) {
        start_index = current_index - MENU_ITEMS_PER_PAGE + 1;
    }
    
    // 跳转到起始显示位置
    item = current_menu->items;
    for (uint8_t i = 0; i < start_index && item; i++) {
        item = item->next;
    }
    
    // 显示菜单项
    while (item && displayed_count < MENU_ITEMS_PER_PAGE) {
        uint8_t y_pos = line * 8;  // 每行8像素
        
        // 判断是否为当前选中项
        if (item == current_menu->current_item) {
            OLED_ShowString(0, y_pos, ">", OLED_6X8);
            OLED_ShowString(6, y_pos, item->name, OLED_6X8);
            OLED_ShowString(114,y_pos,"*",OLED_6X8);
        } else {
            OLED_ShowString(0, y_pos, " ", OLED_6X8);
            OLED_ShowString(6, y_pos, item->name, OLED_6X8);
        }
        
        item = item->next;
        line++;
        displayed_count++;
    }
    OLED_Update();
}

// 创建新菜单
Menu* Menu_Create(char *title)
{
    Menu *menu = (Menu*)malloc(sizeof(Menu));
    if (menu) {
        menu->title = title;
        menu->items = NULL;
        menu->current_item = NULL;
        menu->parent = NULL;
        menu->item_count = 0;
        menu->display_start = 0;
    }
    return menu;
}

// 添加菜单项
void Menu_AddItem(Menu *menu, char *name, MenuItemType type, 
                 Menu *submenu, void (*action)(void))
{
    if (menu == NULL) return;
    
    MenuItem *new_item = (MenuItem*)malloc(sizeof(MenuItem));
    if (new_item) {
        new_item->name = name;
        new_item->type = type;
        new_item->submenu = submenu;
        new_item->action = action;
        new_item->next = NULL;
        new_item->prev = NULL;
        
        // 添加到链表尾部
        if (menu->items == NULL) {
            menu->items = new_item;
            menu->current_item = new_item;  // 第一个项设为当前选中
        } else {
            MenuItem *last = menu->items;
            while (last->next) {
                last = last->next;
            }
            last->next = new_item;
            new_item->prev = last;
        }
        
        menu->item_count++;
    }
}

// 获取当前菜单
Menu* Menu_GetCurrentMenu(void)
{
    return current_menu;
}

// 获取当前菜单项
MenuItem* Menu_GetCurrentItem(void)
{
    return current_menu ? current_menu->current_item : NULL;
}

// 强制刷新菜单显示
void Menu_ForceRefresh(void)
{
    need_refresh = 1;
}
