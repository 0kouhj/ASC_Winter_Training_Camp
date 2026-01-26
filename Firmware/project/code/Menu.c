#include "menu.h"
#include "bsp_oled.h"
#include "bsp_motor.h"
#include "zf_device_key.h"
#include <string.h>
#include <stdlib.h>

// 实时显示状态
static RealtimeDisplayState realtime_state = {0};
static uint8_t in_realtime_mode = 0;  // 是否处于实时显示模式
static PIDEditState pid_edit_state = {0};
static float original_kp, original_ki, original_kd;  // 保存原始值

// 当前显示的菜单
static Menu *current_menu = NULL;
uint8_t need_refresh = 1;  // 需要刷新显示标志

// 警告模式状态
static uint8_t warning_mode = 0;  // 是否处于警告模式

// 显示配置
#define DISPLAY_LINES 6           // 显示总行数
#define MENU_ITEMS_PER_PAGE 5     // 每页显示的菜单项数量
#define REAL_TIME_PAGE 5
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

// 增加PID编辑辅助函数
static void PIDEdit_Init(void)
{
    pid_edit_state.is_editing = 0;
    pid_edit_state.edit_item = 0;  // 默认编辑Kp
    pid_edit_state.edit_step = 0.05f;
    
    // 根据我的实际变量修改
     pid_edit_state.kp_ptr = &State.Kp;
     pid_edit_state.ki_ptr = &State.Ki;
     pid_edit_state.kd_ptr = &State.Kd;
}

static void PIDEdit_Start(void)
{
    // 保存原始值
    if (pid_edit_state.kp_ptr) original_kp = *pid_edit_state.kp_ptr;
    if (pid_edit_state.ki_ptr) original_ki = *pid_edit_state.ki_ptr;
    if (pid_edit_state.kd_ptr) original_kd = *pid_edit_state.kd_ptr;
    
    pid_edit_state.is_editing = 1;
    pid_edit_state.edit_item = 0;  // 从Kp开始
}

static void PIDEdit_Stop(uint8_t save_changes)
{
    if (!save_changes) {
        // 恢复原始值
        if (pid_edit_state.kp_ptr) *pid_edit_state.kp_ptr = original_kp;
        if (pid_edit_state.ki_ptr) *pid_edit_state.ki_ptr = original_ki;
        if (pid_edit_state.kd_ptr) *pid_edit_state.kd_ptr = original_kd;
    }
    
    pid_edit_state.is_editing = 0;
}

static void PIDEdit_AdjustValue(float delta)
{
    float *target = NULL;
    
    switch(pid_edit_state.edit_item) {
        case 0: target = pid_edit_state.kp_ptr; break;
        case 1: target = pid_edit_state.ki_ptr; break;
        case 2: target = pid_edit_state.kd_ptr; break;
    }
    
    if (target) {
        *target += delta * pid_edit_state.edit_step;
        
        // 限制范围（必要时开启）
        // if (*target < 0.0f) *target = 0.0f;
        // if (*target > 10.0f) *target = 10.0f;
    }
}

static void PIDEdit_NextItem(void)
{
    pid_edit_state.edit_item = (pid_edit_state.edit_item + 1) % 3;
}


// 实时参数显示函数
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
    char page_str[4];
    uint16_t volt_int, volt_frac;
    
    // 根据当前页面显示不同参数
    switch(realtime_state.current_page) {
        case 0:  // 第1页：电池和电机状态
        {
            // 电池状态
            OLED_ShowString(0, line*8, "Bat:", OLED_6X8);
            OLED_ShowFloatNum(30, line*8, State.battery_v, 2, 2, OLED_6X8);
            OLED_ShowString(68, line*8, "V", OLED_6X8);
            line++;
            // 左电机目标速度
            OLED_ShowString(0, line*8, "L-Target:", OLED_6X8);
            OLED_ShowSignedNum(60, line*8, State.motor_target_speed_left, 5, OLED_6X8);
            line++;
            // 左电机实际速度
            OLED_ShowString(0, line*8, "L-Actual:", OLED_6X8);
            OLED_ShowSignedNum(60, line*8, State.motor_actual_speed_left, 5, OLED_6X8);
            line++;
            // 右电机目标速度
            OLED_ShowString(0, line*8, "R-Target:", OLED_6X8);
            OLED_ShowSignedNum(60, line*8, State.motor_target_speed_right, 5, OLED_6X8);
            line++;
            // 右电机实际速度
            OLED_ShowString(0, line*8, "R-Actual:", OLED_6X8);
            OLED_ShowSignedNum(60, line*8, State.motor_actual_speed_right, 5, OLED_6X8);
            line++;
            break;
        }
            
        case 1:  // 第2页：传感器数据
        {
            // 显示原始数据
            OLED_ShowString(0,line*8,"axr:",OLED_6X8);
            OLED_ShowSignedNum(24,line*8,Icm.accel_x_raw,4,OLED_6X8);
            OLED_ShowString(67,line*8,"ayr:",OLED_6X8);
            OLED_ShowSignedNum(91,line*8,Icm.accel_y_raw,4,OLED_6X8);
            line++;
            OLED_ShowString(0,line*8,"azr:",OLED_6X8);
            OLED_ShowSignedNum(24,line*8,Icm.accel_z_raw,4,OLED_6X8);
            OLED_ShowString(67,line*8,"gxr:",OLED_6X8);
            OLED_ShowSignedNum(91,line*8,Icm.gyro_x_raw,4,OLED_6X8);
            line++;
            OLED_ShowString(0,line*8,"gyr:",OLED_6X8);
            OLED_ShowSignedNum(24,line*8,Icm.gyro_y_raw,4,OLED_6X8);
            OLED_ShowString(67,line*8,"gzr:",OLED_6X8);
            OLED_ShowSignedNum(91,line*8,Icm.gyro_z_raw,4,OLED_6X8);
            line++;
            // 显示物理量
            OLED_ShowString(0,line*8,"axg:",OLED_6X8);
            OLED_ShowFloatNum(24,line*8,Icm.accel_x_g,1,2,OLED_6X8);
            OLED_ShowString(67,line*8,"ayg:",OLED_6X8);
            OLED_ShowFloatNum(91,line*8,Icm.accel_y_g,1,2,OLED_6X8);
            line++;
            OLED_ShowString(0,line*8,"azg:",OLED_6X8);
            OLED_ShowFloatNum(24,line*8,Icm.accel_z_g,1,2,OLED_6X8);
            OLED_ShowString(67,line*8,"gxd:",OLED_6X8);
            OLED_ShowFloatNum(91,line*8,Icm.gyro_x_dps,3,1,OLED_6X8);
            line++;
            OLED_ShowString(0,line*8,"gyd:",OLED_6X8);
            OLED_ShowFloatNum(24,line*8,Icm.gyro_y_dps,3,1,OLED_6X8);
            OLED_ShowString(67,line*8,"gzd:",OLED_6X8);
            OLED_ShowFloatNum(91,line*8,Icm.gyro_z_dps,3,1,OLED_6X8);
            break;
		}
            
        case 2:  // 第3页：解算数据
        {
            OLED_ShowString(0,line*8,"pitch:",OLED_6X8);
            OLED_ShowFloatNum(50,line*8,State.pitch,3,2,OLED_6X8);
            line++;
            OLED_ShowString(0,line*8,"roll:",OLED_6X8);
            OLED_ShowFloatNum(50,line*8,State.roll,3,2,OLED_6X8);
            line++;
            OLED_ShowString(0,line*8,"yaw:",OLED_6X8);
            OLED_ShowFloatNum(50,line*8,State.yaw,3,2,OLED_6X8);
            line++;
            OLED_ShowString(0,line*8,"gyro_x:",OLED_6X8);
            OLED_ShowFloatNum(50,line*8,State.gyro_x,3,2,OLED_6X8);
            break;
		}
				case 3:  // 第4页：可调控PID参数
				{
					// 显示编辑模式提示
					if (pid_edit_state.is_editing) {
						OLED_ShowString(0, line*8, "EDIT MODE", OLED_6X8);
						line++;
					}
				
					// 显示Kp
					OLED_ShowString(0, line*8, "Kp:", OLED_6X8);
					if (pid_edit_state.is_editing && pid_edit_state.edit_item == 0) {
							OLED_ShowString(20, line*8, ">", OLED_6X8);  // 编辑指示符
					}
					OLED_ShowFloatNum(50, line*8, State.Kp, 3, 2, OLED_6X8);
					line++;
					
					// 显示Ki
					OLED_ShowString(0, line*8, "Ki:", OLED_6X8);
					if (pid_edit_state.is_editing && pid_edit_state.edit_item == 1) {
							OLED_ShowString(20, line*8, ">", OLED_6X8);
					}
					OLED_ShowFloatNum(50, line*8, State.Ki, 3, 2, OLED_6X8);
					line++;
					
					// 显示Kd
					OLED_ShowString(0, line*8, "Kd:", OLED_6X8);
					if (pid_edit_state.is_editing && pid_edit_state.edit_item == 2) {
							OLED_ShowString(20, line*8, ">", OLED_6X8);
					}
					OLED_ShowFloatNum(50, line*8, State.Kd, 3, 2, OLED_6X8);
					line++;
					
					// 显示操作提示
					if (pid_edit_state.is_editing) {
							OLED_ShowString(0, line*8, "KEY3:Next  KEY4:Cancel", OLED_6X8);
					} else {
							// 非编辑模式下显示进入编辑的提示
							OLED_ShowString(0, line*8, "KEY3:Edit  KEY4:Back", OLED_6X8);
					}
					break;
		}

            
        case 4:  // 第4页：编码器数据
        {
            OLED_ShowString(0,line*8,"Left Encoder:",OLED_6X8);
            OLED_ShowSignedNum(80,line*8,State.encoder_left,6,OLED_6X8);
            line++;
            OLED_ShowString(0,line*8,"Right Encoder:",OLED_6X8);
            OLED_ShowSignedNum(80,line*8,State.encoder_right,6,OLED_6X8);
            line++;
            // 可以添加更多编码器相关信息，如果需要
            break;
		}
    }
    
    // 显示页码指示器
    OLED_ShowString(104, 8, "P", OLED_6X8);
    IntToStr(realtime_state.current_page + 1, page_str);
    OLED_ShowString(110, 8, page_str, OLED_6X8);
    OLED_ShowString(116, 8, "/", OLED_6X8);
    OLED_ShowNum(122,8,REAL_TIME_PAGE,1,OLED_6X8);
    
    OLED_Update();
}

// 菜单初始化
void Menu_Init(void)
{
    // 创建菜单
    Menu *main_menu = Menu_Create("Main Menu");
    Menu *test_menu = Menu_Create("Test Menu");
    Menu *flash_menu = Menu_Create("Flash Menu");
    Menu *go_menu = Menu_Create("Go!!!");

    // Test子菜单
    Menu *motor_test_menu = Menu_Create("Motor Test");
    

    // 主菜单
    Menu_AddItem(main_menu, "Info", MENU_ITEM_REALTIME_PARAMS, NULL, NULL);
    Menu_AddItem(main_menu, "Test", MENU_ITEM_SUBMENU,test_menu,NULL);
    Menu_AddItem(main_menu, "Flash", MENU_ITEM_SUBMENU,flash_menu,NULL);
    Menu_AddItem(main_menu, "Go Car!", MENU_ITEM_SUBMENU,go_menu,NULL);
    Menu_AddItem(main_menu, "Reset", MENU_ITEM_ACTION, NULL, NULL);

    //测试菜单
    Menu_AddItem(test_menu, "Motor Test", MENU_ITEM_SUBMENU,motor_test_menu,NULL);
    Menu_AddItem(test_menu, "OLED Test", MENU_ITEM_ACTION,NULL,OLED_Test);

    //Flash菜单
    Menu_AddItem(flash_menu, "R Flash", MENU_ITEM_ACTION,NULL,NULL);
    Menu_AddItem(flash_menu, "W Flash", MENU_ITEM_ACTION,NULL,NULL);
    Menu_AddItem(flash_menu, "C Flash", MENU_ITEM_ACTION,NULL,NULL);
	
    //发车菜单
    Menu_AddItem(go_menu, "Mode 1",MENU_ITEM_ACTION,NULL,NULL);
    Menu_AddItem(go_menu, "Mode 2",MENU_ITEM_ACTION,NULL,NULL);
    Menu_AddItem(go_menu, "Mode 3",MENU_ITEM_ACTION,NULL,NULL);
    Menu_AddItem(go_menu, "Mode 4",MENU_ITEM_ACTION,NULL,NULL);
    Menu_AddItem(go_menu, "Mode 5",MENU_ITEM_ACTION,NULL,NULL);
    
    //Motor Test子菜单
    Menu_AddItem(motor_test_menu, "Motor STOP", MENU_ITEM_ACTION,NULL,motor_stop);
    Menu_AddItem(motor_test_menu, "100",MENU_ITEM_ACTION,NULL,motor_test_100_100);
    Menu_AddItem(motor_test_menu,"-100",MENU_ITEM_ACTION,NULL,motor_test_neg100_neg100);
    Menu_AddItem(motor_test_menu, "50",MENU_ITEM_ACTION,NULL,motor_test_50_50);
    Menu_AddItem(motor_test_menu, "-50",MENU_ITEM_ACTION,NULL,motor_test_neg50_neg50);

    current_menu = main_menu;
    need_refresh = 1;
    
    // 初始化实时显示状态
    realtime_state.current_page = 0;
    realtime_state.total_pages = REAL_TIME_PAGE;
    realtime_state.last_refresh = 0;
    in_realtime_mode = 0;
		
		//初始化PID编辑状态
		PIDEdit_Init();
}

// 处理菜单逻辑（在主循环中调用）
void Menu_Process(void)
{
    
    if (current_menu == NULL) return;
    
    if (in_realtime_mode) {
        // 第四页，且处于编辑模式
        if (realtime_state.current_page == 3 && pid_edit_state.is_editing) {
            // 编辑模式下的按键处理
            if (key_get_state(KEY_1) == KEY_SHORT_PRESS) {
                // 上键增加数值 (+0.05)
                PIDEdit_AdjustValue(1.0f);
            }
            else if (key_get_state(KEY_2) == KEY_SHORT_PRESS) {
                // 下键减少数值 (-0.05)
                PIDEdit_AdjustValue(-1.0f);
            }
            else if (key_get_state(KEY_3) == KEY_SHORT_PRESS) {
                // 确认键切换到下一个编辑项
                PIDEdit_NextItem();
            }
            else if (key_get_state(KEY_4) == KEY_SHORT_PRESS) {
                // 取消键退出编辑模式（不保存）
                PIDEdit_Stop(0);
            }
            // 编辑模式下刷新显示
            Menu_DisplayRealtimeParams();
            return;
        }
        else{
            // 普通实时模式下的按键处理
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
            else if (key_get_state(KEY_3) == KEY_SHORT_PRESS) {
                // 确认键：在第四页进入编辑模式
                if (realtime_state.current_page == 3) {
                    PIDEdit_Start();
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
    }
	
    
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
