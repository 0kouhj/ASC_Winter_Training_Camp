#include "Simple_Timewheel.h"

// 实际定义变量
TimeWheel_Task task_list[MAX_TASKS] = {0}; 
volatile uint8_t tick_flag = 0;

void add_task(uint16_t ms, void (*f)(void)) {
    for(int i = 0; i < MAX_TASKS; i++) {
        if(task_list[i].func == NULL) {
            task_list[i].func = f;
            task_list[i].interval = ms;
            task_list[i].timer = ms;
            return;
        }
    }
}

void time_wheel_run(void) {
    if (tick_flag) {
        tick_flag = 0;
        for(int i = 0; i < MAX_TASKS; i++) {
            if(task_list[i].func != NULL) {
                // 递减计数
                if(task_list[i].timer > 0) {
                    task_list[i].timer--;
                }
                
                // 时间到
                if(task_list[i].timer == 0) {
                    task_list[i].func();
                    // 重载时间
                    task_list[i].timer = task_list[i].interval;
                    // 如果 interval 为 0，说明是单次任务，执行完就清除
                    if(task_list[i].interval == 0) {
                        task_list[i].func = NULL;
                    }
                }
            }
        }
    }
}