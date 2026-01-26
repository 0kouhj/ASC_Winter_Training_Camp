#ifndef __MENU_H
#define __MENU_H

#include "zf_common_headfile.h"
#include <stdint.h>

typedef struct Menu Menu;
typedef struct MenuItem MenuItem;

// 菜单项类型定义
typedef enum {
    MENU_ITEM_ACTION,
    MENU_ITEM_SUBMENU, 
    MENU_ITEM_BACK,
    MENU_ITEM_REALTIME_PARAMS,  // 实时参数显示
		MENU_ITEM_VALUE_EDIT       // 新增：PID数据编辑项
} MenuItemType;

// 菜单项结构
struct MenuItem {
    char *name;
    MenuItemType type;
    Menu *submenu;
    void (*action)(void);
    MenuItem *next;
    MenuItem *prev;
};

// 菜单结构
struct Menu {
    char *title;
    MenuItem *items;
    MenuItem *current_item;
    Menu *parent;
    uint8_t item_count;
    uint8_t display_start;
};

// 按键定义映射到您的KEY驱动
typedef enum {
    MENU_KEY_UP = 0,    // KEY_1 - 上键
    MENU_KEY_DOWN = 1,  // KEY_2 - 下键  
    MENU_KEY_ENTER = 2, // KEY_3 - 确认键
    MENU_KEY_BACK = 3   // KEY_4 - 返回键
} MenuKey;


// 实时参数显示配置
typedef struct {
    uint8_t current_page;     // 当前显示页
    uint8_t total_pages;      // 总页数
    uint32_t last_refresh;    // 上次刷新时间
} RealtimeDisplayState;

// 添加PID参数编辑状态结构体
typedef struct {
    uint8_t is_editing;      // 是否正在编辑
    uint8_t edit_item;       // 当前编辑的项目(0:Kp, 1:Ki, 2:Kd)
    float edit_step;         // 编辑步长
    float *kp_ptr;           // Kp指针
    float *ki_ptr;           // Ki指针
    float *kd_ptr;           // Kd指针
} PIDEditState;


// 函数声明
void Menu_Init(void);
void Menu_Process(void);
void Menu_RefreshDisplay(void);
Menu* Menu_Create(char *title);
void Menu_AddItem(Menu *menu, char *name, MenuItemType type, 
                 Menu *submenu, void (*action)(void));
Menu* Menu_GetCurrentMenu(void);
MenuItem* Menu_GetCurrentItem(void);
void Menu_ForceRefresh(void);
void Menu_DisplayBatteryInfo(void);
uint8_t Menu_IsInWarningMode(void);
void Emergency_Stop_Execute(void);
void Show_Debug_Menu(void);

#endif
